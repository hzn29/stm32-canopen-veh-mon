/* 实现 ICM-42688 六轴惯性传感器的 SPI 驱动。 */
#include "icm42688.h"

/* 保存驱动使用的 SPI 外设句柄。 */
static SPI_HandleTypeDef *icm42688Spi;
/* 保存 ICM-42688 片选 GPIO 端口。 */
static GPIO_TypeDef *icm42688CsPort;
/* 保存 ICM-42688 片选 GPIO 引脚。 */
static uint16_t icm42688CsPin;

/* 拉低片选，开始一次 SPI 事务。 */
static void ICM42688_Select(void)
{
  /* 片选低电平选中 ICM-42688。 */
  HAL_GPIO_WritePin(icm42688CsPort, icm42688CsPin, GPIO_PIN_RESET);
}

/* 拉高片选，结束一次 SPI 事务。 */
static void ICM42688_Deselect(void)
{
  /* 片选高电平释放 ICM-42688。 */
  HAL_GPIO_WritePin(icm42688CsPort, icm42688CsPin, GPIO_PIN_SET);
}

/* 初始化驱动参数并检查传感器设备标识。 */
HAL_StatusTypeDef ICM42688_Init(SPI_HandleTypeDef *hspi, GPIO_TypeDef *csPort, uint16_t csPin)
{
  uint8_t whoAmI = 0U;
  /* 检查传入的 SPI 和片选参数，避免空指针访问。 */
  if ((hspi == NULL) || (csPort == NULL))
  {
    return HAL_ERROR;
  }
  /* 保存 SPI 外设句柄供后续读写使用。 */
  icm42688Spi = hspi;
  /* 保存片选 GPIO 端口供后续读写使用。 */
  icm42688CsPort = csPort;
  /* 保存片选 GPIO 引脚供后续读写使用。 */
  icm42688CsPin = csPin;
  /* 默认释放片选，避免上电时误触发 SPI 事务。 */
  ICM42688_Deselect();
  /* 读取设备标识确认 SPI 线路和芯片响应正常。 */
  if (ICM42688_ReadWhoAmI(&whoAmI) != HAL_OK)
  {
    return HAL_ERROR;
  }
  /* 检查芯片返回的固定设备标识。 */
  if (whoAmI != ICM42688_WHO_AM_I_VALUE)
  {
    return HAL_ERROR;
  }
  /* 写入软件复位位，恢复芯片默认寄存器状态。 */
  if (ICM42688_WriteReg(ICM42688_REG_DEVICE_CONFIG, 0x10U) != HAL_OK)
  {
    return HAL_ERROR;
  }
  /* 等待芯片完成内部复位。 */
  HAL_Delay(2U);
  /* 打开加速度计和陀螺仪低噪声工作模式。 */
  if (ICM42688_WriteReg(ICM42688_REG_PWR_MGMT0, 0x0FU) != HAL_OK)
  {
    return HAL_ERROR;
  }
  /* 配置陀螺仪为默认量程和约 200 Hz 输出数据率。 */
  /* 将陀螺仪设置为 +/-2000 dps、1.6 kHz 输出数据率，满足高速事故采样要求。 */
  if (ICM42688_WriteReg(ICM42688_REG_GYRO_CONFIG0, 0x05U) != HAL_OK)
  {
    return HAL_ERROR;
  }
  /* 配置加速度计为默认量程和约 200 Hz 输出数据率。 */
  /* 将加速度计设置为 +/-16 g、1.6 kHz 输出数据率，避免短时冲击被低速采样漏掉。 */
  if (ICM42688_WriteReg(ICM42688_REG_ACCEL_CONFIG0, 0x05U) != HAL_OK)
  {
    return HAL_ERROR;
  }
  /* 返回初始化成功。 */
  return HAL_OK;
}

/* 读取 ICM-42688 的一个 8 位寄存器。 */
HAL_StatusTypeDef ICM42688_ReadReg(uint8_t reg, uint8_t *value)
{
  uint8_t readAddress = (uint8_t)(reg | 0x80U);
  HAL_StatusTypeDef status;
  /* 检查驱动句柄和输出缓冲区是否有效。 */
  if ((icm42688Spi == NULL) || (value == NULL))
  {
    return HAL_ERROR;
  }
  /* 选中传感器并发送读寄存器地址。 */
  ICM42688_Select();
  status = HAL_SPI_Transmit(icm42688Spi, &readAddress, 1U, 100U);
  /* 地址发送成功后读取寄存器数据。 */
  if (status == HAL_OK)
  {
    status = HAL_SPI_Receive(icm42688Spi, value, 1U, 100U);
  }
  /* 释放传感器片选。 */
  ICM42688_Deselect();
  /* 返回 SPI 操作结果。 */
  return status;
}

/* 写入 ICM-42688 的一个 8 位寄存器。 */
HAL_StatusTypeDef ICM42688_WriteReg(uint8_t reg, uint8_t value)
{
  uint8_t writeData[2] = {(uint8_t)(reg & 0x7FU), value};
  HAL_StatusTypeDef status;
  /* 检查驱动 SPI 句柄是否有效。 */
  if (icm42688Spi == NULL)
  {
    return HAL_ERROR;
  }
  /* 选中传感器并发送写寄存器地址及数据。 */
  ICM42688_Select();
  status = HAL_SPI_Transmit(icm42688Spi, writeData, 2U, 100U);
  /* 释放传感器片选。 */
  ICM42688_Deselect();
  /* 返回 SPI 操作结果。 */
  return status;
}

/* 读取 ICM-42688 的 WHO_AM_I 设备标识。 */
HAL_StatusTypeDef ICM42688_ReadWhoAmI(uint8_t *value)
{
  /* 调用通用寄存器读取接口。 */
  return ICM42688_ReadReg(ICM42688_REG_WHO_AM_I, value);
}

/* 读取 ICM-42688 的连续六轴原始数据。 */
HAL_StatusTypeDef ICM42688_ReadRaw(ICM42688_RawData_t *data)
{
  uint8_t readAddress = (uint8_t)(ICM42688_REG_ACCEL_DATA_X1 | 0x80U);
  uint8_t raw[12] = {0U};
  HAL_StatusTypeDef status;
  /* 检查驱动句柄和输出结构体是否有效。 */
  if ((icm42688Spi == NULL) || (data == NULL))
  {
    return HAL_ERROR;
  }
  /* 选中传感器并发送连续读取起始地址。 */
  ICM42688_Select();
  status = HAL_SPI_Transmit(icm42688Spi, &readAddress, 1U, 100U);
  /* 地址发送成功后连续读取 12 个数据字节。 */
  if (status == HAL_OK)
  {
    status = HAL_SPI_Receive(icm42688Spi, raw, 12U, 100U);
  }
  /* 释放传感器片选。 */
  ICM42688_Deselect();
  /* SPI 失败时不解析无效数据。 */
  if (status != HAL_OK)
  {
    return status;
  }
  /* 将大端格式的加速度数据合成为有符号 16 位数。 */
  data->accelX = (int16_t)(((uint16_t)raw[0] << 8U) | raw[1]);
  /* 将大端格式的加速度数据合成为有符号 16 位数。 */
  data->accelY = (int16_t)(((uint16_t)raw[2] << 8U) | raw[3]);
  /* 将大端格式的加速度数据合成为有符号 16 位数。 */
  data->accelZ = (int16_t)(((uint16_t)raw[4] << 8U) | raw[5]);
  /* 将大端格式的陀螺仪数据合成为有符号 16 位数。 */
  data->gyroX = (int16_t)(((uint16_t)raw[6] << 8U) | raw[7]);
  /* 将大端格式的陀螺仪数据合成为有符号 16 位数。 */
  data->gyroY = (int16_t)(((uint16_t)raw[8] << 8U) | raw[9]);
  /* 将大端格式的陀螺仪数据合成为有符号 16 位数。 */
  data->gyroZ = (int16_t)(((uint16_t)raw[10] << 8U) | raw[11]);
  /* 返回读取成功。 */
  return HAL_OK;
}
