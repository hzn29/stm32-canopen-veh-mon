#ifndef ESP32_PERSIST_H
#define ESP32_PERSIST_H

#include "stm32g4xx_hal.h"

HAL_StatusTypeDef ESP32_Persist_LoadPeriod(uint32_t defaultPeriodMs, uint32_t *periodMs);
HAL_StatusTypeDef ESP32_Persist_SavePeriod(uint32_t periodMs);

typedef struct
{
  uint32_t lastEventId; /* 淇濆瓨宸茬粡鍒嗛厤杩囩殑鏈€澶т簨浠剁紪鍙枫€?*/
  uint32_t pendingEventId; /* 淇濆瓨灏氭湭鏀跺埌 A ACK 鐨勪簨浠剁紪鍙枫€?*/
  uint32_t pendingPeakAccelMg; /* 淇濆瓨寰呯‘璁や簨浠剁殑鍔犻€熷害宄板€笺€?*/
  uint32_t pendingPeakGyroDps; /* 淇濆瓨寰呯‘璁や簨浠剁殑瑙掗€熷害宄板€笺€?*/
  uint8_t pendingEventType; /* 淇濆瓨寰呯‘璁や簨浠剁被鍨嬨€?*/
  uint8_t pendingValid; /* 鏍囪鏄惁瀛樺湪寰呯‘璁や簨浠躲€?*/
  uint8_t pendingRetryCount; /* 淇濆瓨宸茬粡鎵ц鐨勯噸鍙戞鏁般€?*/
} ESP32_AccidentPersist_t;

HAL_StatusTypeDef ESP32_Persist_LoadAccident(ESP32_AccidentPersist_t *snapshot);
HAL_StatusTypeDef ESP32_Persist_SaveAccident(const ESP32_AccidentPersist_t *snapshot);

#endif /* ESP32_PERSIST_H */
