/* ESP32 MQTT 运行参数的 STM32G4 Flash 持久化接口。 */
#ifndef ESP32_PERSIST_H
#define ESP32_PERSIST_H

/* 引入 STM32 HAL 基础类型。 */
#include "stm32g4xx_hal.h"

/* 从保留 Flash 页读取发布周期，失败时返回默认值。 */
HAL_StatusTypeDef ESP32_Persist_LoadPeriod(uint32_t defaultPeriodMs, uint32_t *periodMs);
/* 将发布周期写入保留 Flash 页。 */
HAL_StatusTypeDef ESP32_Persist_SavePeriod(uint32_t periodMs);

/* 定义事故事件掉电保存快照，供 B 节点重启后恢复未确认事件。 */
typedef struct
{
  uint32_t lastEventId; /* 保存已经分配过的最大事件编号。 */
  uint32_t pendingEventId; /* 保存尚未收到 A ACK 的事件编号。 */
  uint32_t pendingPeakAccelMg; /* 保存待确认事件的加速度峰值。 */
  uint32_t pendingPeakGyroDps; /* 保存待确认事件的角速度峰值。 */
  uint8_t pendingEventType; /* 保存待确认事件类型。 */
  uint8_t pendingValid; /* 标记是否存在待确认事件。 */
  uint8_t pendingRetryCount; /* 保存已经执行的重发次数。 */
} ESP32_AccidentPersist_t;

/* 从参数页读取事故编号和待确认事件快照。 */
HAL_StatusTypeDef ESP32_Persist_LoadAccident(ESP32_AccidentPersist_t *snapshot);
/* 将事故编号和待确认事件快照写入参数页。 */
HAL_StatusTypeDef ESP32_Persist_SaveAccident(const ESP32_AccidentPersist_t *snapshot);

#endif /* ESP32_PERSIST_H */
