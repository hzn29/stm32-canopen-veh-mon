/* BH-182 GNSS 模块的 UART DMA 环形接收和 NMEA 解析实现。 */
#include "bh182.h"

/* 引入字符串和整数转换函数。 */
#include <string.h>
#include <stdlib.h>

/* 定义单行 NMEA 语句缓冲区长度。 */
#define BH182_LINE_BUFFER_SIZE 128U

/* 保存绑定的 UART 外设句柄。 */
static UART_HandleTypeDef *bh182Uart;
/* 保存 UART DMA 环形接收缓冲区。 */
static uint8_t bh182RxBuffer[BH182_RX_BUFFER_SIZE];
/* 保存上一次已经处理到的 DMA 缓冲区位置。 */
static volatile uint16_t bh182LastRxPosition;
/* 保存当前正在组装的 NMEA 语句。 */
static char bh182LineBuffer[BH182_LINE_BUFFER_SIZE];
/* 保存当前 NMEA 语句长度。 */
static uint16_t bh182LineLength;
/* 保存驱动内部的最新定位数据。 */
static volatile BH182_Data_t bh182Data;

/* 将一个十六进制字符转换为数值。 */
static uint8_t BH182_HexValue(char value)
{
  /* 将数字字符转换为 0 到 9。 */
  if ((value >= '0') && (value <= '9'))
  {
    /* 返回数字字符对应的数值。 */
    return (uint8_t)(value - '0');
  }
  /* 将大写十六进制字符转换为 10 到 15。 */
  if ((value >= 'A') && (value <= 'F'))
  {
    /* 返回大写十六进制字符对应的数值。 */
    return (uint8_t)(value - 'A' + 10U);
  }
  /* 将小写十六进制字符转换为 10 到 15。 */
  if ((value >= 'a') && (value <= 'f'))
  {
    /* 返回小写十六进制字符对应的数值。 */
    return (uint8_t)(value - 'a' + 10U);
  }
  /* 返回无效十六进制字符标记。 */
  return 0xFFU;
}

/* 检查一行 NMEA 语句的 XOR 校验和。 */
static uint8_t BH182_ChecksumValid(char *line)
{
  /* 保存星号位置和校验计算变量。 */
  char *star;
  uint8_t checksum = 0U;
  uint8_t receivedChecksum;
  char *cursor;
  /* 检查 NMEA 语句是否以美元符号开始。 */
  if (line[0] != '$')
  {
    /* 起始符号错误时返回校验失败。 */
    return 0U;
  }
  /* 查找校验和分隔符。 */
  star = strchr(line, '*');
  /* 检查星号及其后的两个校验字符是否存在。 */
  if ((star == NULL) || (star[1] == '\0') || (star[2] == '\0'))
  {
    /* 校验和字段缺失时返回失败。 */
    return 0U;
  }
  /* 计算美元符号和星号之间所有字符的 XOR 值。 */
  for (cursor = line + 1; cursor < star; cursor++)
  {
    /* 累加当前字符的 XOR 值。 */
    checksum ^= (uint8_t)(*cursor);
  }
  /* 将两个 ASCII 十六进制字符转换为接收校验值。 */
  receivedChecksum = (uint8_t)((BH182_HexValue(star[1]) << 4U) |
                               BH182_HexValue(star[2]));
  /* 检查校验字符是否有效并比较计算结果。 */
  return ((BH182_HexValue(star[1]) != 0xFFU) &&
          (BH182_HexValue(star[2]) != 0xFFU) &&
          (checksum == receivedChecksum));
}

/* 将 NMEA 的 ddmm.mmmm 或 dddmm.mmmm 坐标转换为 1e-7 度。 */
static uint8_t BH182_ParseCoordinate(const char *text, char direction, int32_t *value)
{
  /* 保存度分整数部分、小数部分和小数倍率。 */
  uint32_t whole = 0U;
  uint32_t fraction = 0U;
  uint32_t scale = 1U;
  uint8_t afterDecimal = 0U;
  const char *cursor = text;
  uint32_t degrees;
  uint32_t minutes;
  uint64_t minutesE7;
  uint64_t coordinateE7;
  /* 检查输入字符串和方向是否有效。 */
  if ((text == NULL) || (value == NULL) ||
      ((direction != 'N') && (direction != 'S') &&
       (direction != 'E') && (direction != 'W')))
  {
    /* 参数错误时返回解析失败。 */
    return 0U;
  }
  /* 逐字符解析坐标的整数和小数部分。 */
  while (*cursor != '\0')
  {
    /* 处理坐标小数点。 */
    if (*cursor == '.')
    {
      /* 标记后续字符属于小数部分。 */
      afterDecimal = 1U;
    }
    else if ((*cursor >= '0') && (*cursor <= '9'))
    {
      /* 解析小数点前的数字。 */
      if (afterDecimal == 0U)
      {
        /* 累加度分整数部分。 */
        whole = whole * 10U + (uint32_t)(*cursor - '0');
      }
      else if (scale < 1000000U)
      {
        /* 累加最多六位小数，避免无界整数增长。 */
        fraction = fraction * 10U + (uint32_t)(*cursor - '0');
        scale *= 10U;
      }
    }
    else
    {
      /* 出现非数字字符时返回解析失败。 */
      return 0U;
    }
    /* 移动到下一个坐标字符。 */
    cursor++;
  }
  /* 分离度分格式中的度和分钟整数部分。 */
  degrees = whole / 100U;
  minutes = whole % 100U;
  /* 将分钟换算为 1e-7 度。 */
  minutesE7 = (uint64_t)minutes * 10000000ULL;
  minutesE7 += ((uint64_t)fraction * 10000000ULL) / scale;
  minutesE7 /= 60ULL;
  /* 合并度和分钟得到绝对坐标。 */
  coordinateE7 = (uint64_t)degrees * 10000000ULL + minutesE7;
  /* 南纬和西经使用负值表示。 */
  if ((direction == 'S') || (direction == 'W'))
  {
    /* 写入带符号的坐标结果。 */
    *value = -(int32_t)coordinateE7;
  }
  else
  {
    /* 写入北纬或东经坐标结果。 */
    *value = (int32_t)coordinateE7;
  }
  /* 返回坐标解析成功。 */
  return 1U;
}

/* 将最多三位小数的有符号十进制数转换为千分之一单位。 */
static int32_t BH182_ParseMilli(const char *text)
{
  /* 保存符号、整数、小数和小数位数。 */
  int32_t sign = 1;
  int32_t whole = 0;
  int32_t fraction = 0;
  uint8_t digits = 0U;
  const char *cursor = text;
  /* 处理负号。 */
  if (*cursor == '-')
  {
    /* 保存负数符号。 */
    sign = -1;
    /* 跳过负号。 */
    cursor++;
  }
  /* 解析整数和最多三位小数。 */
  while (*cursor != '\0')
  {
    /* 跳过小数点但不改变数值。 */
    if (*cursor == '.')
    {
      /* 通过 digits 的高位标记小数状态。 */
      digits = 0x80U;
    }
    else if ((*cursor >= '0') && (*cursor <= '9'))
    {
      /* 解析小数点前的整数。 */
      if ((digits & 0x80U) == 0U)
      {
        /* 累加整数部分。 */
        whole = whole * 10 + (int32_t)(*cursor - '0');
      }
      else if ((digits & 0x7FU) < 3U)
      {
        /* 累加小数部分。 */
        fraction = fraction * 10 + (int32_t)(*cursor - '0');
        digits++;
      }
    }
    else
    {
      /* 遇到其他字符时停止解析。 */
      break;
    }
    /* 移动到下一个字符。 */
    cursor++;
  }
  /* 将小数补齐到千分之一单位。 */
  if ((digits & 0x7FU) == 0U)
  {
    /* 没有小数时保持零。 */
    fraction = 0;
  }
  else if ((digits & 0x7FU) == 1U)
  {
    /* 一位小数补两个零。 */
    fraction *= 100;
  }
  else if ((digits & 0x7FU) == 2U)
  {
    /* 两位小数补一个零。 */
    fraction *= 10;
  }
  /* 返回带符号的千分之一数值。 */
  return sign * (whole * 1000 + fraction);
}

/* 解析一条已经完成接收的 NMEA 语句。 */
static void BH182_ParseLine(char *line)
{
  /* 保存字段指针、字段数量和校验分隔符。 */
  char *fields[16] = {0};
  char *cursor;
  char *star;
  uint8_t fieldCount = 0U;
  int32_t coordinate;
  /* 检查 NMEA 校验和。 */
  if (BH182_ChecksumValid(line) == 0U)
  {
    /* 累加校验和错误计数。 */
    bh182Data.checksumErrorCount++;
    /* 校验失败时不解析字段。 */
    return;
  }
  /* 去除校验和字段，便于使用逗号分隔字段。 */
  star = strchr(line, '*');
  *star = '\0';
  /* 将语句按逗号拆分为字段指针。 */
  fields[fieldCount++] = line + 1;
  for (cursor = line + 1; (*cursor != '\0') && (fieldCount < 16U); cursor++)
  {
    /* 将逗号替换为字符串结束符。 */
    if (*cursor == ',')
    {
      /* 结束当前字段并记录下一个字段地址。 */
      *cursor = '\0';
      fields[fieldCount++] = cursor + 1;
    }
  }
  /* 只处理字段数量足够的语句。 */
  if (fieldCount == 0U)
  {
    /* 累加格式解析错误计数。 */
    bh182Data.parseErrorCount++;
    /* 结束当前语句解析。 */
    return;
  }
  /* 解析 RMC 语句中的有效状态、经纬度和速度。 */
  if ((strncmp(fields[0], "GNRMC", 5U) == 0) ||
      (strncmp(fields[0], "GPRMC", 5U) == 0))
  {
    /* 检查 RMC 的字段数量。 */
    if (fieldCount < 8U)
    {
      /* 累加格式解析错误计数。 */
      bh182Data.parseErrorCount++;
      /* 结束 RMC 解析。 */
      return;
    }
    /* 使用状态字段判断定位是否有效。 */
    bh182Data.fixValid = (fields[2][0] == 'A') ? 1U : 0U;
    /* 定位有效时更新经纬度。 */
    if ((bh182Data.fixValid != 0U) &&
        (BH182_ParseCoordinate(fields[3], fields[4][0], &coordinate) != 0U))
    {
      /* 保存纬度。 */
      bh182Data.latitudeE7 = coordinate;
    }
    /* 定位有效时更新经度。 */
    if ((bh182Data.fixValid != 0U) &&
        (BH182_ParseCoordinate(fields[5], fields[6][0], &coordinate) != 0U))
    {
      /* 保存经度。 */
      bh182Data.longitudeE7 = coordinate;
    }
    /* 将节速度换算为毫米每秒。 */
    if (bh182Data.fixValid != 0U)
    {
      /* 一节等于约 514.444 毫米每秒。 */
      bh182Data.speedMmps = (uint32_t)(((int64_t)BH182_ParseMilli(fields[7]) * 514444LL) / 1000000LL);
    }
    /* 累加成功解析的语句计数。 */
    bh182Data.sentenceCount++;
  }
  /* 解析 GGA 语句中的定位质量、卫星数和海拔。 */
  else if ((strncmp(fields[0], "GNGGA", 5U) == 0) ||
           (strncmp(fields[0], "GPGGA", 5U) == 0))
  {
    /* 检查 GGA 的字段数量。 */
    if (fieldCount < 10U)
    {
      /* 累加格式解析错误计数。 */
      bh182Data.parseErrorCount++;
      /* 结束 GGA 解析。 */
      return;
    }
    /* 保存 GGA 定位质量状态。 */
    bh182Data.fixValid = (strtoul(fields[6], NULL, 10) != 0U) ? 1U : 0U;
    /* 保存参与定位的卫星数。 */
    bh182Data.satelliteCount = strtoul(fields[7], NULL, 10);
    /* 将海拔米转换为毫米。 */
    bh182Data.altitudeMm = BH182_ParseMilli(fields[9]) * 1000;
    /* 累加成功解析的语句计数。 */
    bh182Data.sentenceCount++;
  }
}

/* 将一个 DMA 接收到的字节加入 NMEA 行缓冲区。 */
static void BH182_PushByte(uint8_t byte)
{
  /* 收到美元符号时重新开始一行语句。 */
  if (byte == '$')
  {
    /* 清空当前行长度并保存起始符号。 */
    bh182LineLength = 0U;
    bh182LineBuffer[bh182LineLength++] = (char)byte;
    /* 返回等待后续字符。 */
    return;
  }
  /* 忽略尚未开始的无效前导字符。 */
  if (bh182LineLength == 0U)
  {
    /* 丢弃当前字节。 */
    return;
  }
  /* 忽略回车符，换行符用于提交完整语句。 */
  if (byte == '\r')
  {
    /* 返回继续接收。 */
    return;
  }
  /* 收到换行符时结束并解析当前语句。 */
  if (byte == '\n')
  {
    /* 添加字符串结束符。 */
    bh182LineBuffer[bh182LineLength] = '\0';
    /* 解析完整 NMEA 语句。 */
    BH182_ParseLine(bh182LineBuffer);
    /* 清空行缓冲区状态。 */
    bh182LineLength = 0U;
    /* 返回等待下一条语句。 */
    return;
  }
  /* 检查行缓冲区是否还有空间。 */
  if (bh182LineLength < (BH182_LINE_BUFFER_SIZE - 1U))
  {
    /* 保存普通 NMEA 字节。 */
    bh182LineBuffer[bh182LineLength++] = (char)byte;
  }
  else
  {
    /* 超长语句直接丢弃并等待下一行。 */
    bh182LineLength = 0U;
    bh182Data.parseErrorCount++;
  }
}

/* 初始化 BH-182 驱动并保存 UART 句柄。 */
HAL_StatusTypeDef BH182_Init(UART_HandleTypeDef *huart)
{
  /* 检查 UART 句柄是否有效。 */
  if (huart == NULL)
  {
    /* 无效参数返回 HAL 错误。 */
    return HAL_ERROR;
  }
  /* 保存 UART 句柄。 */
  bh182Uart = huart;
  /* 清空 DMA 和行解析状态。 */
  bh182LastRxPosition = 0U;
  bh182LineLength = 0U;
  memset((void *)bh182RxBuffer, 0, sizeof(bh182RxBuffer));
  memset((void *)bh182LineBuffer, 0, sizeof(bh182LineBuffer));
  memset((void *)&bh182Data, 0, sizeof(bh182Data));
  /* 返回初始化成功。 */
  return HAL_OK;
}

/* 启动 UART DMA 循环接收和空闲事件检测。 */
HAL_StatusTypeDef BH182_Start(void)
{
  /* 检查 UART 句柄是否已经初始化。 */
  if (bh182Uart == NULL)
  {
    /* 未初始化时返回 HAL 错误。 */
    return HAL_ERROR;
  }
  /* 启动 UART DMA 接收并检测空闲事件。 */
  if (HAL_UARTEx_ReceiveToIdle_DMA(bh182Uart,
                                   bh182RxBuffer,
                                   BH182_RX_BUFFER_SIZE) != HAL_OK)
  {
    /* DMA 接收启动失败时返回 HAL 错误。 */
    return HAL_ERROR;
  }
  /* 关闭 DMA 半传输中断，减少无完整语句时的回调次数。 */
  if (bh182Uart->hdmarx != NULL)
  {
    /* 禁止 DMA 半传输事件。 */
    __HAL_DMA_DISABLE_IT(bh182Uart->hdmarx, DMA_IT_HT);
  }
  /* 返回启动成功。 */
  return HAL_OK;
}

/* 处理 UART DMA 空闲回调报告的当前写入位置。 */
void BH182_OnRxEvent(uint16_t size)
{
  /* 保存 DMA 当前写指针、软件读指针和待处理字节数。 */
  uint16_t writePosition = size;
  /* 记录软件上一次处理完成的位置。 */
  uint16_t readPosition = bh182LastRxPosition;
  /* 记录本次需要交给 NMEA 解析器的字节数。 */
  uint16_t availableBytes = 0U;
  /* 获取本次回调是 IDLE、满传输还是其他事件。 */
  HAL_UART_RxEventTypeTypeDef eventType;
  /* 检查 UART 句柄，避免异常回调访问无效对象。 */
  if (bh182Uart == NULL)
  {
    /* 未绑定 UART 时直接返回。 */
    return;
  }
  /* 读取 HAL 保存的 UART 接收事件类型。 */
  eventType = HAL_UARTEx_GetRxEventType(bh182Uart);
  /* HAL 的 Size 取值范围为 1 到缓冲区长度，长度值转换为环形写指针 0。 */
  if (writePosition >= BH182_RX_BUFFER_SIZE)
  {
    /* DMA 恰好写满缓冲区后，下一写入位置回到 0。 */
    writePosition = 0U;
  }
  /* 写指针在读指针之后时，新增数据位于连续的前半段。 */
  if (writePosition > readPosition)
  {
    /* 计算未回绕的新增字节数量。 */
    availableBytes = writePosition - readPosition;
  }
  /* 写指针在读指针之前时，新增数据已经跨越缓冲区末尾。 */
  else if (writePosition < readPosition)
  {
    /* 计算从读指针到缓冲区末尾再到写指针的字节数量。 */
    availableBytes = (BH182_RX_BUFFER_SIZE - readPosition) + writePosition;
  }
  /* 写指针与读指针相同，只有满传输事件才表示完整一圈新数据。 */
  else if (eventType == HAL_UART_RXEVENT_TC)
  {
    /* 将整块 DMA 缓冲区标记为待处理数据。 */
    availableBytes = BH182_RX_BUFFER_SIZE;
  }
  /* 处理所有尚未交给 NMEA 解析器的字节。 */
  while (availableBytes > 0U)
  {
    /* 将当前读指针对应的字节送入 NMEA 行组装器。 */
    BH182_PushByte(bh182RxBuffer[readPosition]);
    /* 消费一个字节。 */
    availableBytes--;
    /* 读指针前进一个位置。 */
    readPosition++;
    /* 读指针到达缓冲区末尾时回绕到起点。 */
    if (readPosition >= BH182_RX_BUFFER_SIZE)
    {
      /* 将环形读指针回绕为 0。 */
      readPosition = 0U;
    }
  }
  /* 保存已经处理完成的环形读指针。 */
  bh182LastRxPosition = writePosition;
}

/* 提供 HAL UART 空闲事件回调给 BH-182 驱动。 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  /* 只处理绑定给 BH-182 的 UART1。 */
  if (huart == bh182Uart)
  {
    /* 处理 DMA 缓冲区中的新增字节。 */
    BH182_OnRxEvent(Size);
  }
}

/* 获取 BH-182 定位数据的一致快照。 */
void BH182_GetData(BH182_Data_t *data)
{
  /* 检查输出指针是否有效。 */
  if (data == NULL)
  {
    /* 无效指针时直接返回。 */
    return;
  }
  /* 暂停中断，避免复制过程中被 UART 回调修改。 */
  __disable_irq();
  /* 复制当前定位数据。 */
  *data = bh182Data;
  /* 恢复中断。 */
  __enable_irq();
}
