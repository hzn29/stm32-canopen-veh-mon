#ifndef ICM42688_H
#define ICM42688_H

#include "main.h"

#define ICM42688_REG_WHO_AM_I 0x75U
#define ICM42688_REG_DEVICE_CONFIG 0x11U
#define ICM42688_REG_PWR_MGMT0 0x4EU
#define ICM42688_REG_GYRO_CONFIG0 0x4FU
#define ICM42688_REG_ACCEL_CONFIG0 0x50U
#define ICM42688_REG_ACCEL_DATA_X1 0x1FU
#define ICM42688_WHO_AM_I_VALUE 0x47U

typedef struct
{
  int16_t accelX; /* 淇濆瓨 X 杞村姞閫熷害鍘熷 ADC 鏁版嵁銆?*/
  int16_t accelY; /* 淇濆瓨 Y 杞村姞閫熷害鍘熷 ADC 鏁版嵁銆?*/
  int16_t accelZ; /* 淇濆瓨 Z 杞村姞閫熷害鍘熷 ADC 鏁版嵁銆?*/
  int16_t gyroX; /* 淇濆瓨 X 杞磋閫熷害鍘熷 ADC 鏁版嵁銆?*/
  int16_t gyroY; /* 淇濆瓨 Y 杞磋閫熷害鍘熷 ADC 鏁版嵁銆?*/
  int16_t gyroZ; /* 淇濆瓨 Z 杞磋閫熷害鍘熷 ADC 鏁版嵁銆?*/
} ICM42688_RawData_t;

HAL_StatusTypeDef ICM42688_Init(SPI_HandleTypeDef *hspi, GPIO_TypeDef *csPort, uint16_t csPin);
HAL_StatusTypeDef ICM42688_ReadReg(uint8_t reg, uint8_t *value);
HAL_StatusTypeDef ICM42688_WriteReg(uint8_t reg, uint8_t value);
HAL_StatusTypeDef ICM42688_ReadWhoAmI(uint8_t *value);
HAL_StatusTypeDef ICM42688_ReadRaw(ICM42688_RawData_t *data);

#endif /* ICM42688_H */
