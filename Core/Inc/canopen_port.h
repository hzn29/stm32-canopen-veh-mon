#ifndef CANOPEN_PORT_H
#define CANOPEN_PORT_H

#include "301/CO_driver.h"
#include "bh182.h"
#include "icm42688.h"
#include "esp32_persist.h"

typedef enum
{
  CANOPEN_HEALTH_UNCONFIGURED = 0U, /* 鐩戞帶椤规湭閰嶇疆銆?*/
  CANOPEN_HEALTH_UNKNOWN = 1U, /* 宸查厤缃絾灏氭湭鏀跺埌鏈夋晥 Heartbeat銆?*/
  CANOPEN_HEALTH_ACTIVE = 2U, /* Heartbeat 鍦ㄨ瀹氭椂闂村唴姝ｅ父鍒拌揪銆?*/
  CANOPEN_HEALTH_TIMEOUT = 3U, /* 鏈 Heartbeat 宸茶秴鏃躲€?*/
  CANOPEN_HEALTH_RECOVERING = 4U, /* 宸查噸鏂版敹鍒?Heartbeat锛屾鍦ㄧ瓑寰呯ǔ瀹氱‘璁ゃ€?*/
  CANOPEN_HEALTH_FAULT = 5U /* 澶氭瓒呮椂鎴栨仮澶嶈秴鏃讹紝杩涘叆鏁呴殰鐘舵€併€?*/
} CanOpenPortHealthState_t;

CO_ReturnError_t CanOpenPort_Init(uint8_t nodeId);
CO_ReturnError_t CanOpenPort_StackInit(uint8_t nodeId);
void CanOpenPort_RxInterrupt(uint16_t identifier, uint8_t dataLength, const uint8_t *data);
void CanOpenPort_Process(uint32_t timeDifferenceUs);
void CanOpenPort_UpdateGnss(const BH182_Data_t *data);
void CanOpenPort_UpdateImu(const ICM42688_RawData_t *data, uint8_t valid);
CO_ReturnError_t CanOpenPort_SendAccidentEvent(uint8_t eventType, uint32_t eventId,
                                               uint32_t peakAccelMg, uint32_t peakGyroDps);
void CanOpenPort_ClearAccidentError(uint8_t eventType);
CO_ReturnError_t CanOpenPort_SendAccidentAck(uint32_t eventId);
void CanOpenPort_RestoreAccidentState(const ESP32_AccidentPersist_t *snapshot);
CO_CANmodule_t* CanOpenPort_GetModule(void);

typedef struct
{
  uint8_t nodeId; /* 淇濆瓨琚洃鎺ц妭鐐圭殑 CANopen Node-ID銆?*/
  uint8_t state; /* 淇濆瓨褰撳墠鐘舵€侊細0 鏈厤缃€? 鏈煡銆? 娲昏穬銆? 瓒呮椂銆?*/
  uint8_t previousState; /* 淇濆瓨涓婁竴娆￠噰鏍峰埌鐨?Heartbeat 鐘舵€併€?*/
  uint8_t faultActive; /* 鏍囪璇ヨ妭鐐规槸鍚﹀瓨鍦ㄥ皻鏈仮澶嶇殑瓒呮椂鏁呴殰銆?*/
  uint8_t healthState; /* 淇濆瓨鍋ュ悍鐘舵€侊細0 鏈厤缃€? 鏈煡銆? 姝ｅ父銆? 瓒呮椂銆? 鎭㈠涓€? 鏁呴殰銆?*/
  uint8_t previousHealthState; /* 淇濆瓨涓婁竴娆″仴搴风姸鎬侊紝鐢ㄤ簬璇嗗埆鐘舵€佽竟娌裤€?*/
  uint32_t lastTransitionTick; /* 淇濆瓨鏈€杩戜竴娆＄姸鎬佸彉鍖栫殑 HAL 鑺傛媿銆?*/
  uint32_t faultStartTick; /* 淇濆瓨鏈€杩戜竴娆¤秴鏃舵晠闅滃紑濮嬬殑 HAL 鑺傛媿銆?*/
  uint32_t recoveryStartTick; /* 淇濆瓨鏈€杩戜竴娆℃敹鍒版仮澶?Heartbeat 鐨?HAL 鑺傛媿銆?*/
  uint32_t lastRecoveryTimeMs; /* 淇濆瓨鏈€杩戜竴娆′粠瓒呮椂鍒版仮澶嶆墍鐢ㄧ殑姣鏁般€?*/
  uint32_t timeoutCount; /* 淇濆瓨璇ヨ妭鐐圭疮璁¤繘鍏ヨ秴鏃剁姸鎬佺殑娆℃暟銆?*/
  uint32_t timeoutEpisodeCount; /* 淇濆瓨璇ヨ妭鐐圭疮璁＄殑鐙珛瓒呮椂浜嬩欢娆℃暟銆?*/
  uint32_t recoveryCount; /* 淇濆瓨璇ヨ妭鐐圭疮璁℃仮澶嶄负娲昏穬鐘舵€佺殑娆℃暟銆?*/
} CanOpenPortHeartbeatStatus_t;

typedef struct
{
  uint8_t frameSeen;
  uint32_t lastRxTick;
  uint32_t rxFrameCount;
  uint32_t timeoutCount;
  uint8_t dataValid;
} CanOpenPortPdoHealth_t;

typedef struct
{
  uint8_t nmtState; /* 淇濆瓨鏈満 NMT 鐘舵€佸€硷細5 涓?Operational銆?27 涓?Pre-operational銆?*/
  uint8_t errorRegister; /* 淇濆瓨 CANopen 瀵硅薄 0x1001 鐨勯敊璇瘎瀛樺櫒鍊笺€?*/
  uint8_t heartbeatErrorActive; /* 鏍囪 Heartbeat Consumer 閿欒鏄惁澶勪簬娲诲姩鐘舵€併€?*/
  uint32_t emcyReportCount; /* 淇濆瓨妫€娴嬪埌鐨?Heartbeat EMCY 鏁呴殰杈规部娆℃暟銆?*/
  uint32_t emcyClearCount; /* 淇濆瓨妫€娴嬪埌鐨?Heartbeat EMCY 娓呴櫎杈规部娆℃暟銆?*/
  uint16_t lastEmcyErrorCode; /* 淇濆瓨鏈€杩戜竴娆?Heartbeat EMCY 閿欒鐮侊紝姝ｅ父涓?0x8130銆?*/
  uint8_t lastEmcyErrorBit; /* 淇濆瓨鏈€杩戜竴娆?EMCY 鐨勯敊璇綅锛孒eartbeat Consumer 涓?0x1B銆?*/
  uint32_t lastEmcyInfoCode; /* 淇濆瓨鏈€杩戜竴娆?EMCY 鐨勪俊鎭储寮曪紝0 琛ㄧず B锛? 琛ㄧず C銆?*/
  uint32_t lastEmcyTick; /* 淇濆瓨鏈€杩戜竴娆?Heartbeat EMCY 浜х敓鐨?HAL 鑺傛媿銆?*/
  uint32_t lastEmcyClearTick; /* 淇濆瓨鏈€杩戜竴娆?Heartbeat EMCY 娓呴櫎鐨?HAL 鑺傛媿銆?*/
  CanOpenPortHeartbeatStatus_t heartbeatB; /* 淇濆瓨 Node-ID 2锛圔锛夌殑鐘舵€佺粺璁°€?*/
  CanOpenPortHeartbeatStatus_t heartbeatC; /* 淇濆瓨 Node-ID 3锛圕锛夌殑鐘舵€佺粺璁°€?*/
  CanOpenPortPdoHealth_t imuAccel;
  CanOpenPortPdoHealth_t imuGyro;
  CanOpenPortPdoHealth_t gnssPosition;
  CanOpenPortPdoHealth_t gnssStatus;
  uint8_t imuDataValid;
  uint8_t gnssDataValid;
  uint8_t nodeBHealthy;
  uint8_t nodeCHealthy;
  uint32_t accidentEventRxCount;
  uint32_t accidentEventDuplicateCount;
  uint32_t accidentEventInvalidCount;
  uint32_t accidentEventTxCount;
  uint32_t accidentEventTxErrorCount;
  uint32_t lastAccidentEventId;
  uint8_t lastAccidentEventType;
  uint32_t accidentAckRxCount; /* 淇濆瓨 B 鑺傜偣鏀跺埌 A ACK 鐨勬鏁般€?*/
  uint32_t accidentAckTxCount; /* 淇濆瓨 A 鑺傜偣鎴愬姛鍙戦€?ACK 鐨勬鏁般€?*/
  uint32_t accidentAckTxErrorCount; /* 淇濆瓨 A 鑺傜偣鍙戦€?ACK 澶辫触鐨勬鏁般€?*/
  uint32_t accidentRetryCount; /* 淇濆瓨 B 鑺傜偣浜嬫晠浜嬩欢閲嶅彂娆℃暟銆?*/
  uint32_t accidentAckTimeoutCount; /* 淇濆瓨 B 鑺傜偣绛夊緟 ACK 瓒呮椂娆℃暟銆?*/
} CanOpenPortDiagnostics_t;

void CanOpenPort_GetDiagnostics(volatile CanOpenPortDiagnostics_t *diagnostics);

#endif /* CANOPEN_PORT_H */
