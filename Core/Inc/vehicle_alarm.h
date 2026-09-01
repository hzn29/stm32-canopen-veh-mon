#ifndef VEHICLE_ALARM_H
#define VEHICLE_ALARM_H

#include "canopen_port.h"

typedef enum
{
  VEHICLE_ALARM_NORMAL = 0U, /* 鎵€鏈夌洃鎺ч」姝ｅ父銆?*/
  VEHICLE_ALARM_WARNING = 1U, /* 瀛樺湪缁忕‘璁ょ殑涓€鑸憡璀︺€?*/
  VEHICLE_ALARM_FAULT = 2U, /* 瀛樺湪涓ラ噸閫氫俊鎴栨€荤嚎鏁呴殰銆?*/
  VEHICLE_ALARM_RECOVERING = 3U /* 鏁呴殰娑堝け锛屾鍦ㄨ繘琛屾仮澶嶇‘璁ゃ€?*/
} VehicleAlarmState_t;

#define VEHICLE_ALARM_IMU_INVALID (1UL << 0U) /* IMU 鏁版嵁鏃犳晥銆?*/
#define VEHICLE_ALARM_GNSS_INVALID (1UL << 1U) /* GNSS 鏁版嵁鏃犳晥銆?*/
#define VEHICLE_ALARM_NODE_B_TIMEOUT (1UL << 2U) /* B 鑺傜偣 Heartbeat 瓒呮椂銆?*/
#define VEHICLE_ALARM_NODE_C_TIMEOUT (1UL << 3U) /* C 鑺傜偣 Heartbeat 瓒呮椂銆?*/
#define VEHICLE_ALARM_MQTT_OFFLINE (1UL << 4U) /* MQTT 鏈繛鎺ャ€?*/
#define VEHICLE_ALARM_CAN_BUS_OFF (1UL << 5U) /* FDCAN 褰撳墠澶勪簬 Bus-Off銆?*/

typedef struct
{
  uint8_t state; /* 淇濆瓨褰撳墠鎬讳綋鍛婅鐘舵€併€?*/
  uint32_t rawAlarmBits; /* 淇濆瓨鏈粡杩炵画鍛ㄦ湡婊ゆ尝鐨勫師濮嬪紓甯镐綅銆?*/
  uint32_t activeAlarmBits; /* 淇濆瓨褰撳墠宸茬‘璁ゆ垨绔嬪嵆鐢熸晥鐨勫憡璀︿綅銆?*/
  uint32_t warningConfirmCount; /* 淇濆瓨杩炵画涓€鑸紓甯稿懆鏈熸暟銆?*/
  uint32_t recoveryConfirmCount; /* 淇濆瓨杩炵画姝ｅ父鎭㈠鍛ㄦ湡鏁般€?*/
  uint32_t faultEnterCount; /* 缁熻杩涘叆 FAULT 鐨勬鏁般€?*/
  uint32_t warningEnterCount; /* 缁熻杩涘叆 WARNING 鐨勬鏁般€?*/
  uint32_t recoveryEnterCount; /* 缁熻杩涘叆 RECOVERING 鐨勬鏁般€?*/
  uint32_t normalEnterCount; /* 缁熻鎭㈠鍒?NORMAL 鐨勬鏁般€?*/
  uint32_t lastTransitionTick; /* 淇濆瓨鏈€杩戜竴娆＄姸鎬佽浆鎹㈢殑 HAL 鑺傛媿銆?*/
} VehicleAlarmStatus_t;

void VehicleAlarm_Init(void);
void VehicleAlarm_Update(const CanOpenPortDiagnostics_t *diagnostics,
                         uint8_t canBusOffActive,
                         uint8_t mqttConnected);
void VehicleAlarm_GetStatus(volatile VehicleAlarmStatus_t *status);
const char *VehicleAlarm_GetStateText(uint8_t state);

#endif /* VEHICLE_ALARM_H */
