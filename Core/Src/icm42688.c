#include "icm42688.h"

static SPI_HandleTypeDef *icm42688Spi;
static GPIO_TypeDef *icm42688CsPort;
static uint16_t icm42688CsPin;

static void ICM42688_Select(void)
{
  HAL_GPIO_WritePin(icm42688CsPort, icm42688CsPin, GPIO_PIN_RESET);
}

static void ICM42688_Deselect(void)
{
  HAL_GPIO_WritePin(icm42688CsPort, icm42688CsPin, GPIO_PIN_SET);
}

HAL_StatusTypeDef ICM42688_Init(SPI_HandleTypeDef *hspi, GPIO_TypeDef *csPort, uint16_t csPin)
{
  uint8_t whoAmI = 0U;
  if ((hspi == NULL) || (csPort == NULL))
  {
    return HAL_ERROR;
  }
  icm42688Spi = hspi;
  icm42688CsPort = csPort;
  icm42688CsPin = csPin;
  ICM42688_Deselect();
  if (ICM42688_ReadWhoAmI(&whoAmI) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (whoAmI != ICM42688_WHO_AM_I_VALUE)
  {
    return HAL_ERROR;
  }
  if (ICM42688_WriteReg(ICM42688_REG_DEVICE_CONFIG, 0x10U) != HAL_OK)
  {
    return HAL_ERROR;
  }
  HAL_Delay(2U);
  if (ICM42688_WriteReg(ICM42688_REG_PWR_MGMT0, 0x0FU) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (ICM42688_WriteReg(ICM42688_REG_GYRO_CONFIG0, 0x05U) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (ICM42688_WriteReg(ICM42688_REG_ACCEL_CONFIG0, 0x05U) != HAL_OK)
  {
    return HAL_ERROR;
  }
  return HAL_OK;
}

HAL_StatusTypeDef ICM42688_ReadReg(uint8_t reg, uint8_t *value)
{
  uint8_t readAddress = (uint8_t)(reg | 0x80U);
  HAL_StatusTypeDef status;
  if ((icm42688Spi == NULL) || (value == NULL))
  {
    return HAL_ERROR;
  }
  ICM42688_Select();
  status = HAL_SPI_Transmit(icm42688Spi, &readAddress, 1U, 100U);
  if (status == HAL_OK)
  {
    status = HAL_SPI_Receive(icm42688Spi, value, 1U, 100U);
  }
  ICM42688_Deselect();
  return status;
}

HAL_StatusTypeDef ICM42688_WriteReg(uint8_t reg, uint8_t value)
{
  uint8_t writeData[2] = {(uint8_t)(reg & 0x7FU), value};
  HAL_StatusTypeDef status;
  if (icm42688Spi == NULL)
  {
    return HAL_ERROR;
  }
  ICM42688_Select();
  status = HAL_SPI_Transmit(icm42688Spi, writeData, 2U, 100U);
  ICM42688_Deselect();
  return status;
}

HAL_StatusTypeDef ICM42688_ReadWhoAmI(uint8_t *value)
{
  return ICM42688_ReadReg(ICM42688_REG_WHO_AM_I, value);
}

HAL_StatusTypeDef ICM42688_ReadRaw(ICM42688_RawData_t *data)
{
  uint8_t readAddress = (uint8_t)(ICM42688_REG_ACCEL_DATA_X1 | 0x80U);
  uint8_t raw[12] = {0U};
  HAL_StatusTypeDef status;
  if ((icm42688Spi == NULL) || (data == NULL))
  {
    return HAL_ERROR;
  }
  ICM42688_Select();
  status = HAL_SPI_Transmit(icm42688Spi, &readAddress, 1U, 100U);
  if (status == HAL_OK)
  {
    status = HAL_SPI_Receive(icm42688Spi, raw, 12U, 100U);
  }
  ICM42688_Deselect();
  if (status != HAL_OK)
  {
    return status;
  }
  data->accelX = (int16_t)(((uint16_t)raw[0] << 8U) | raw[1]);
  data->accelY = (int16_t)(((uint16_t)raw[2] << 8U) | raw[3]);
  data->accelZ = (int16_t)(((uint16_t)raw[4] << 8U) | raw[5]);
  data->gyroX = (int16_t)(((uint16_t)raw[6] << 8U) | raw[7]);
  data->gyroY = (int16_t)(((uint16_t)raw[8] << 8U) | raw[9]);
  data->gyroZ = (int16_t)(((uint16_t)raw[10] << 8U) | raw[11]);
  return HAL_OK;
}
