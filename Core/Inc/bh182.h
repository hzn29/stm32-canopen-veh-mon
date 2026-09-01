/* BH-182 北斗/GNSS 模块的 UART DMA 空闲中断驱动接口。 */
#ifndef BH182_H
#define BH182_H

/* 引入 STM32 HAL 的 UART 类型定义。 */
#include "main.h"

/* 定义 UART DMA 环形接收缓冲区大小。 */
#define BH182_RX_BUFFER_SIZE 256U

/* 保存 BH-182 最近一次解析出的定位数据。 */
typedef struct
{
  /* 保存定位是否有效，1 表示已经获得有效定位。 */
  uint8_t fixValid;
  /* 保存纬度，单位为 1e-7 度，北为正、南为负。 */
  int32_t latitudeE7;
  /* 保存经度，单位为 1e-7 度，东为正、西为负。 */
  int32_t longitudeE7;
  /* 保存海拔高度，单位为毫米。 */
  int32_t altitudeMm;
  /* 保存地面速度，单位为毫米每秒。 */
  uint32_t speedMmps;
  /* 保存参与定位的卫星数量。 */
  uint32_t satelliteCount;
  /* 保存已经成功解析的 NMEA 语句数量。 */
  uint32_t sentenceCount;
  /* 保存校验和错误数量。 */
  uint32_t checksumErrorCount;
  /* 保存格式解析错误数量。 */
  uint32_t parseErrorCount;
} BH182_Data_t;

/* 初始化 BH-182 驱动并绑定 UART 句柄。 */
HAL_StatusTypeDef BH182_Init(UART_HandleTypeDef *huart);

/* 启动 BH-182 的 UART DMA 空闲中断接收。 */
HAL_StatusTypeDef BH182_Start(void);

/* 接收 UART 空闲事件回调中的 DMA 当前写入位置。 */
void BH182_OnRxEvent(uint16_t size);

/* 获取一份一致的 BH-182 定位数据快照。 */
void BH182_GetData(BH182_Data_t *data);

#endif /* BH182_H */
