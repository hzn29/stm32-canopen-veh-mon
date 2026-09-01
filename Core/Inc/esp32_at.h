#ifndef ESP32_AT_H
#define ESP32_AT_H

#include "stm32g4xx_hal.h"

typedef enum
{
  ESP32_STATE_OFFLINE = 0U, /* 灏氭湭寮€濮嬩覆鍙ｉ€氫俊銆?*/
  ESP32_STATE_WAIT_CONFIG = 1U, /* 绛夊緟濉啓缃戠粶閰嶇疆銆?*/
  ESP32_STATE_AT_READY = 2U, /* ESP-AT 鍩虹鍛戒护鍙敤銆?*/
  ESP32_STATE_WIFI_CONNECTING = 3U, /* 姝ｅ湪杩炴帴 Wi-Fi銆?*/
  ESP32_STATE_MQTT_CONNECTING = 4U, /* 姝ｅ湪杩炴帴 MQTT 鏈嶅姟绔€?*/
  ESP32_STATE_MQTT_CONNECTED = 5U, /* MQTT 宸插缓绔嬭繛鎺ャ€?*/
  ESP32_STATE_ERROR = 6U /* 鏈€杩戜竴娆℃搷浣滃け璐ャ€?*/
} ESP32_State_t;

typedef enum
{
  ESP32_DOWNLINK_PUBLISH = 1U, /* 璇锋眰绔嬪嵆鍙戝竷涓€娆￠仴娴嬨€?*/
  ESP32_DOWNLINK_STATUS = 2U, /* 璇锋眰绔嬪嵆鍙戝竷鐘舵€侀仴娴嬨€?*/
  ESP32_DOWNLINK_SET_PERIOD = 3U /* 璇锋眰淇敼閬ユ祴鍙戝竷鍛ㄦ湡銆?*/
} ESP32_DownlinkCommandType_t;

typedef struct
{
  uint8_t state; /* 淇濆瓨褰撳墠缃戠粶鐘舵€佹灇涓惧€笺€?*/
  uint32_t commandCount; /* 缁熻宸插彂閫佺殑 AT 鍛戒护鏁伴噺銆?*/
  uint32_t commandErrorCount; /* 缁熻 AT 鍛戒护澶辫触鏁伴噺銆?*/
  uint32_t publishCount; /* 缁熻鎴愬姛鍙戝竷鐨?MQTT 娑堟伅鏁伴噺銆?*/
  uint32_t publishErrorCount; /* 缁熻鍙戝竷澶辫触鏁伴噺銆?*/
  uint32_t reconnectCount; /* 缁熻 MQTT 閲嶈繛娆℃暟銆?*/
  uint32_t downlinkCount; /* 缁熻鏀跺埌鐨?MQTT 涓嬭鍛戒护鏁伴噺銆?*/
  uint32_t downlinkAcceptedCount; /* 缁熻閫氳繃鐧藉悕鍗曟牎楠岀殑鍛戒护鏁伴噺銆?*/
  uint32_t downlinkRejectedCount; /* 缁熻琚嫆缁濈殑鍛戒护鏁伴噺銆?*/
  uint32_t downlinkExecutedCount; /* 缁熻鎵ц鎴愬姛鐨勪笅琛屽懡浠ゆ暟閲忋€?*/
  uint32_t downlinkExecutionErrorCount; /* 缁熻鎵ц澶辫触鐨勪笅琛屽懡浠ゆ暟閲忋€?*/
  uint32_t downlinkAckCount; /* 缁熻鎴愬姛鍙戦€佺殑鍛戒护搴旂瓟鏁伴噺銆?*/
  uint32_t downlinkAckErrorCount; /* 缁熻鍛戒护搴旂瓟鍙戦€佸け璐ユ暟閲忋€?*/
  uint32_t downlinkQueueFullCount; /* 缁熻鍥犲懡浠ら槦鍒楀凡婊¤€屼涪寮冪殑鏁伴噺銆?*/
  uint32_t lastDownlinkCmdId; /* 淇濆瓨鏈€杩戝鐞嗙殑涓嬭鍛戒护缂栧彿銆?*/
  uint32_t lastDownlinkLatencyMs; /* 淇濆瓨鏈€杩戝懡浠や粠鎺ユ敹鍒版墽琛岀殑寤惰繜銆?*/
  uint32_t maxDownlinkLatencyMs; /* 淇濆瓨鍛戒护鎺ユ敹鍒版墽琛岀殑鏈€澶у欢杩熴€?*/
  uint32_t downlinkTestPassCount; /* 缁熻鏈湴鍙潬鎬ф祴璇曢€氳繃娆℃暟銆?*/
  uint32_t downlinkTestFailCount; /* 缁熻鏈湴鍙潬鎬ф祴璇曞け璐ユ鏁般€?*/
  uint32_t parameterLoadErrorCount; /* 缁熻 Flash 鍙傛暟鏃犳晥鎴栬鍙栧け璐ユ鏁般€?*/
  uint32_t parameterSaveCount; /* 缁熻 Flash 鍙傛暟淇濆瓨鎴愬姛娆℃暟銆?*/
  uint32_t parameterSaveErrorCount; /* 缁熻 Flash 鍙傛暟淇濆瓨澶辫触娆℃暟銆?*/
  uint32_t currentPublishPeriodMs; /* 淇濆瓨褰撳墠閬ユ祴鍙戝竷鍛ㄦ湡銆?*/
  uint32_t uartRxOverflowCount; /* 缁熻涓插彛鎺ユ敹鐜舰缂撳啿婧㈠嚭娆℃暟銆?*/
  uint32_t lastCommandTick; /* 淇濆瓨鏈€杩戜竴娆?AT 鍛戒护鏃堕棿銆?*/
  uint32_t lastPublishTick; /* 淇濆瓨鏈€杩戜竴娆″彂甯冨皾璇曟椂闂淬€?*/
  uint32_t lastErrorCode; /* 淇濆瓨鏈€杩戜竴娆¤蒋浠堕敊璇唬鐮併€?*/
  uint32_t accidentEventQueuedCount; /* 缁熻鎴愬姛杩涘叆 MQTT 鍙潬闃熷垪鐨勪簨鏁呬簨浠舵暟銆?*/
  uint32_t accidentEventSentCount; /* 缁熻鎴愬姛鍙戝竷鍒?MQTT 鐨勪簨鏁呬簨浠舵暟銆?*/
  uint32_t accidentEventRetryCount; /* 缁熻 MQTT 绂荤嚎鎴栧彂閫佸け璐ュ悗鐨勯噸璇曟鏁般€?*/
  uint32_t accidentEventDropCount; /* 缁熻 MQTT 浜嬩欢闃熷垪宸叉弧瀵艰嚧鐨勪涪寮冩暟銆?*/
  uint32_t accidentEventLastId; /* 淇濆瓨鏈€杩戜竴娆″叆闃熸垨鍙戝竷鐨勪簨鏁呬簨浠剁紪鍙枫€?*/
} ESP32_Status_t;

HAL_StatusTypeDef ESP32_Init(UART_HandleTypeDef *huart);
HAL_StatusTypeDef ESP32_StartReceive(void);
void ESP32_OnRxByte(uint8_t byte);
void ESP32_OnRxComplete(void);
void ESP32_OnUartError(void);
void ESP32_MqttTask(void *argument);
void ESP32_RequestImmediatePublish(void);
void ESP32_QueueAccidentEvent(uint32_t eventId, uint8_t eventType,
                              uint16_t peakAccelMg, uint16_t peakGyroDps);
void ESP32_GetStatus(volatile ESP32_Status_t *status);
HAL_StatusTypeDef ESP32_RunDownlinkReliabilityTest(void);

#endif /* ESP32_AT_H */
