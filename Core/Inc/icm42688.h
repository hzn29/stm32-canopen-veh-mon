/* 声明 ICM-42688 六轴惯性传感器的 SPI 驱动接口。 */
#ifndef ICM42688_H
#define ICM42688_H

/* 引入 STM32 HAL 类型和 SPI 接口。 */
#include "main.h"

/* 定义 ICM-42688 的设备标识寄存器地址。 */
#define ICM42688_REG_WHO_AM_I 0x75U
/* 定义 ICM-42688 的设备配置寄存器地址。 */
#define ICM42688_REG_DEVICE_CONFIG 0x11U
/* 定义 ICM-42688 的电源管理寄存器地址。 */
#define ICM42688_REG_PWR_MGMT0 0x4EU
/* 定义 ICM-42688 的陀螺仪配置寄存器地址。 */
#define ICM42688_REG_GYRO_CONFIG0 0x4FU
/* 定义 ICM-42688 的加速度计配置寄存器地址。 */
#define ICM42688_REG_ACCEL_CONFIG0 0x50U
/* 定义 ICM-42688 加速度计数据高字节的起始地址。 */
#define ICM42688_REG_ACCEL_DATA_X1 0x1FU
/* 定义 ICM-42688 的预期设备标识。 */
#define ICM42688_WHO_AM_I_VALUE 0x47U

/* 保存 ICM-42688 一次读取到的六轴原始数据。 */
typedef struct
{
  int16_t accelX; /* 保存 X 轴加速度原始 ADC 数据。 */
  int16_t accelY; /* 保存 Y 轴加速度原始 ADC 数据。 */
  int16_t accelZ; /* 保存 Z 轴加速度原始 ADC 数据。 */
  int16_t gyroX; /* 保存 X 轴角速度原始 ADC 数据。 */
  int16_t gyroY; /* 保存 Y 轴角速度原始 ADC 数据。 */
  int16_t gyroZ; /* 保存 Z 轴角速度原始 ADC 数据。 */
} ICM42688_RawData_t;

/* 初始化 ICM-42688 驱动、复位传感器并检查 WHO_AM_I。 */
HAL_StatusTypeDef ICM42688_Init(SPI_HandleTypeDef *hspi, GPIO_TypeDef *csPort, uint16_t csPin);
/* 读取 ICM-42688 的一个 8 位寄存器。 */
HAL_StatusTypeDef ICM42688_ReadReg(uint8_t reg, uint8_t *value);
/* 写入 ICM-42688 的一个 8 位寄存器。 */
HAL_StatusTypeDef ICM42688_WriteReg(uint8_t reg, uint8_t value);
/* 读取 ICM-42688 的 WHO_AM_I 设备标识。 */
HAL_StatusTypeDef ICM42688_ReadWhoAmI(uint8_t *value);
/* 读取 ICM-42688 的六轴原始数据。 */
HAL_StatusTypeDef ICM42688_ReadRaw(ICM42688_RawData_t *data);

#endif /* ICM42688_H */
