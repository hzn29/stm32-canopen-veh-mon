/* SSD1306 128x64 I2C OLED 显示驱动实现。 */
#include "ssd1306.h"
/* 引入字符串长度函数。 */
#include <string.h>

/* 保存 OLED 使用的 HAL I2C 句柄。 */
static I2C_HandleTypeDef *ssd1306I2c;
/* 保存 OLED 的 HAL 8 位设备地址。 */
static uint16_t ssd1306Address;
/* 保存 OLED 128x64 显示缓存。 */
static uint8_t ssd1306Buffer[128U * 64U / 8U];
/* 保存当前文本光标像素列。 */
static uint8_t ssd1306CursorColumn;
/* 保存当前文本光标页号。 */
static uint8_t ssd1306CursorPage;

/* 向 SSD1306 写入一条控制命令序列。 */
static HAL_StatusTypeDef SSD1306_WriteCommand(const uint8_t *commands, uint8_t length)
{
  /* 保存带控制字节的命令发送缓存。 */
  uint8_t packet[17U] = {0U};
  /* 检查驱动句柄、命令指针和命令长度。 */
  if ((ssd1306I2c == NULL) || (commands == NULL) || (length > 16U))
  {
    /* 参数无效时返回错误。 */
    return HAL_ERROR;
  }
  /* 选择后续字节为命令。 */
  packet[0] = 0x00U;
  /* 复制命令序列。 */
  (void)memcpy(&packet[1], commands, length);
  /* 通过 I2C 发送命令序列。 */
  return HAL_I2C_Master_Transmit(ssd1306I2c, ssd1306Address,
                                 packet, (uint16_t)length + 1U, 100U);
}

/* 返回一个 5x7 ASCII 字符的列点阵。 */
static void SSD1306_GetGlyph(char character, uint8_t glyph[5U])
{
  /* 默认使用问号，避免未知字符显示为空白。 */
  static const uint8_t question[5U] = {0x02U, 0x01U, 0x51U, 0x09U, 0x06U};
  /* 使用空格点阵初始化输出。 */
  (void)memset(glyph, 0, 5U);
  /* 空格保持为空白并直接返回。 */
  if (character == ' ')
  {
    /* 空格无需写入点阵。 */
    return;
  }
  /* 将小写 ASCII 转换为可复用的大写点阵。 */
  if ((character >= 'a') && (character <= 'z'))
  {
    /* 转换小写字符为大写字符。 */
    character = (char)(character - 'a' + 'A');
  }
  /* 根据字符范围选择数字、字母或标点点阵。 */
  if ((character >= '0') && (character <= '9'))
  {
    /* 定义数字 0 至 9 的 5x7 列点阵。 */
    static const uint8_t digits[10U][5U] = {
      {0x3EU,0x51U,0x49U,0x45U,0x3EU}, {0x00U,0x42U,0x7FU,0x40U,0x00U},
      {0x62U,0x51U,0x49U,0x49U,0x46U}, {0x22U,0x49U,0x49U,0x49U,0x36U},
      {0x18U,0x14U,0x12U,0x7FU,0x10U}, {0x2FU,0x49U,0x49U,0x49U,0x31U},
      {0x3EU,0x49U,0x49U,0x49U,0x32U}, {0x01U,0x71U,0x09U,0x05U,0x03U},
      {0x36U,0x49U,0x49U,0x49U,0x36U}, {0x26U,0x49U,0x49U,0x49U,0x3EU}
    };
    /* 复制对应数字点阵。 */
    (void)memcpy(glyph, digits[(uint8_t)character - (uint8_t)'0'], 5U);
  }
  else if ((character >= 'A') && (character <= 'Z'))
  {
    /* 定义大写字母 A 至 Z 的 5x7 列点阵。 */
    static const uint8_t letters[26U][5U] = {
      {0x7EU,0x11U,0x11U,0x11U,0x7EU}, {0x7FU,0x49U,0x49U,0x49U,0x36U},
      {0x3EU,0x41U,0x41U,0x41U,0x22U}, {0x7FU,0x41U,0x41U,0x22U,0x1CU},
      {0x7FU,0x49U,0x49U,0x49U,0x41U}, {0x7FU,0x09U,0x09U,0x09U,0x01U},
      {0x3EU,0x41U,0x49U,0x49U,0x7AU}, {0x7FU,0x08U,0x08U,0x08U,0x7FU},
      {0x00U,0x41U,0x7FU,0x41U,0x00U}, {0x20U,0x40U,0x41U,0x3FU,0x01U},
      {0x7FU,0x08U,0x14U,0x22U,0x41U}, {0x7FU,0x40U,0x40U,0x40U,0x40U},
      {0x7FU,0x02U,0x04U,0x02U,0x7FU}, {0x7FU,0x04U,0x08U,0x10U,0x7FU},
      {0x3EU,0x41U,0x41U,0x41U,0x3EU}, {0x7FU,0x09U,0x09U,0x09U,0x06U},
      {0x3EU,0x41U,0x51U,0x21U,0x5EU}, {0x7FU,0x09U,0x19U,0x29U,0x46U},
      {0x46U,0x49U,0x49U,0x49U,0x31U}, {0x01U,0x01U,0x7FU,0x01U,0x01U},
      {0x3FU,0x40U,0x40U,0x40U,0x3FU}, {0x1FU,0x20U,0x40U,0x20U,0x1FU},
      {0x3FU,0x40U,0x38U,0x40U,0x3FU}, {0x63U,0x14U,0x08U,0x14U,0x63U},
      {0x07U,0x08U,0x70U,0x08U,0x07U}, {0x61U,0x51U,0x49U,0x45U,0x43U}
    };
    /* 复制对应大写字母点阵。 */
    (void)memcpy(glyph, letters[(uint8_t)character - (uint8_t)'A'], 5U);
  }
  else if (character == ':')
  {
    /* 输出冒号点阵。 */
    glyph[1] = 0x36U;
  }
  else if (character == '-')
  {
    /* 输出减号点阵。 */
    glyph[2] = 0x08U;
  }
  else if (character == '.')
  {
    /* 输出小数点点阵。 */
    glyph[2] = 0x40U;
  }
  else if (character == '%')
  {
    /* 输出百分号点阵。 */
    /* 写入百分号第一列。 */
    glyph[0] = 0x63U;
    /* 写入百分号第二列。 */
    glyph[1] = 0x13U;
    /* 写入百分号第三列。 */
    glyph[2] = 0x08U;
    /* 写入百分号第四列。 */
    glyph[3] = 0x64U;
  }
  else if (character == '/')
  {
    /* 输出斜杠第一列。 */
    glyph[0] = 0x20U;
    /* 输出斜杠第二列。 */
    glyph[1] = 0x10U;
    /* 输出斜杠第三列。 */
    glyph[2] = 0x08U;
    /* 输出斜杠第四列。 */
    glyph[3] = 0x04U;
    /* 输出斜杠第五列。 */
    glyph[4] = 0x02U;
  }
  else
  {
    /* 对未知字符使用问号点阵。 */
    (void)memcpy(glyph, question, 5U);
  }
}

/* 初始化 SSD1306 控制器和显示缓存。 */
HAL_StatusTypeDef SSD1306_Init(I2C_HandleTypeDef *hi2c, uint16_t address)
{
  /* 定义 SSD1306 初始化命令序列。 */
  static const uint8_t initCommands[] = {
    0xAEU, 0xD5U, 0x80U, 0xA8U, 0x3FU, 0xD3U, 0x00U, 0x40U,
    0x8DU, 0x14U, 0x20U, 0x00U, 0xA1U, 0xC8U, 0xDAU, 0x12U,
    0x81U, 0xCFU, 0xD9U, 0xF1U, 0xDBU, 0x40U, 0xA4U, 0xA6U, 0xAFU
  };
  /* 定义带有命令控制字节的初始化发送缓冲区。 */
  uint8_t initPacket[sizeof(initCommands) + 1U] = {0U};
  /* 保存传入的 I2C 句柄。 */
  ssd1306I2c = hi2c;
  /* 保存 HAL 格式的 OLED 地址。 */
  ssd1306Address = address;
  /* 设置 SSD1306 命令流的控制字节。 */
  initPacket[0] = 0x00U;
  /* 将初始化命令复制到控制字节之后。 */
  (void)memcpy(&initPacket[1], initCommands, sizeof(initCommands));
  /* 清空显示缓存和光标。 */
  SSD1306_Clear();
  /* 确认 OLED I2C 地址能够应答。 */
  if (HAL_I2C_IsDeviceReady(ssd1306I2c, ssd1306Address, 2U, 100U) != HAL_OK)
  {
    /* 设备无应答时返回错误。 */
    return HAL_ERROR;
  }
  /* 发送 SSD1306 初始化命令。 */
  /* 发送带有 0x00 命令控制字节的初始化序列。 */
  if (HAL_I2C_Master_Transmit(ssd1306I2c, ssd1306Address,
                              initPacket, (uint16_t)sizeof(initPacket), 100U) != HAL_OK)
  {
    /* 初始化命令发送失败时返回错误。 */
    return HAL_ERROR;
  }
  /* 刷新一次全黑画面。 */
  return SSD1306_UpdateScreen();
}

/* 清空显示缓存。 */
void SSD1306_Clear(void)
{
  /* 将所有像素清零。 */
  (void)memset(ssd1306Buffer, 0, sizeof(ssd1306Buffer));
  /* 将光标复位到左上角。 */
  ssd1306CursorColumn = 0U;
  /* 将光标页复位到第一页。 */
  ssd1306CursorPage = 0U;
}

/* 设置文本光标，column 为像素列，page 为 8 像素页。 */
void SSD1306_SetCursor(uint8_t column, uint8_t page)
{
  /* 保存不超过屏幕范围的像素列。 */
  ssd1306CursorColumn = (column < 128U) ? column : 0U;
  /* 保存不超过八页范围的页号。 */
  ssd1306CursorPage = (page < 8U) ? page : 0U;
}

/* 向当前光标位置写入 ASCII 字符串。 */
void SSD1306_WriteString(const char *text)
{
  /* 保存当前字符的 5 列点阵。 */
  uint8_t glyph[5U] = {0U};
  /* 检查字符串指针。 */
  if (text == NULL)
  {
    /* 无效字符串直接返回。 */
    return;
  }
  /* 逐字符绘制到显示缓存。 */
  while (*text != '\0')
  {
    /* 遇到换行时切换到下一页。 */
    if (*text == '\n')
    {
      /* 换行后从左侧开始。 */
      ssd1306CursorColumn = 0U;
      /* 移动到下一页。 */
      ssd1306CursorPage = (ssd1306CursorPage < 7U) ? (ssd1306CursorPage + 1U) : 0U;
      /* 处理下一个字符。 */
      text++;
      continue;
    }
    /* 屏幕右侧空间不足时自动换行。 */
    if (ssd1306CursorColumn > 122U)
    {
      /* 从左侧开始新行。 */
      ssd1306CursorColumn = 0U;
      /* 移动到下一页。 */
      ssd1306CursorPage = (ssd1306CursorPage < 7U) ? (ssd1306CursorPage + 1U) : 0U;
    }
    /* 获取当前字符点阵。 */
    SSD1306_GetGlyph(*text, glyph);
    /* 将 5 列点阵写入当前显示页。 */
    for (uint8_t column = 0U; column < 5U; column++)
    {
      /* 写入当前字符的一列像素。 */
      ssd1306Buffer[(ssd1306CursorPage * 128U) + ssd1306CursorColumn + column] = glyph[column];
    }
    /* 保留一列字符间距。 */
    ssd1306CursorColumn = (uint8_t)(ssd1306CursorColumn + 6U);
    /* 移动到下一个字符。 */
    text++;
  }
}

/* 将显示缓存刷新到 OLED 面板。 */
HAL_StatusTypeDef SSD1306_UpdateScreen(void)
{
  /* 保存当前页的命令。 */
  uint8_t commands[4U] = {0U, 0U, 0U, 0U};
  /* 保存当前页的数据包，首字节为数据控制字节。 */
  uint8_t packet[129U] = {0U};
  /* 保存当前显示页号。 */
  uint8_t page = 0U;
  /* 检查 I2C 句柄。 */
  if (ssd1306I2c == NULL)
  {
    /* 句柄无效时返回错误。 */
    return HAL_ERROR;
  }
  /* 选择数据控制字节。 */
  packet[0] = 0x40U;
  /* 逐页发送 128 字节显示数据。 */
  for (page = 0U; page < 8U; page++)
  {
    /* 设置当前页地址。 */
    commands[0] = 0xB0U + page;
    /* 设置低列地址。 */
    commands[1] = 0x00U;
    /* 设置高列地址。 */
    commands[2] = 0x10U;
    /* 发送页地址命令。 */
    if (SSD1306_WriteCommand(commands, 3U) != HAL_OK)
    {
      /* 命令发送失败时停止刷新。 */
      return HAL_ERROR;
    }
    /* 复制当前页的 128 字节数据。 */
    (void)memcpy(&packet[1], &ssd1306Buffer[page * 128U], 128U);
    /* 发送当前页像素数据。 */
    if (HAL_I2C_Master_Transmit(ssd1306I2c, ssd1306Address,
                                packet, sizeof(packet), 100U) != HAL_OK)
    {
      /* 数据发送失败时返回错误。 */
      return HAL_ERROR;
    }
  }
  /* 返回刷新成功。 */
  return HAL_OK;
}
