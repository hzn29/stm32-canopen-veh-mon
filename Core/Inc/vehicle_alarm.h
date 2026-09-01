/* 车辆仪表盘统一告警状态机接口。 */
#ifndef VEHICLE_ALARM_H
#define VEHICLE_ALARM_H

/* 引入 CANopen 诊断数据类型。 */
#include "canopen_port.h"

/* 定义车辆仪表盘的总体告警状态。 */
typedef enum
{
  VEHICLE_ALARM_NORMAL = 0U, /* 所有监控项正常。 */
  VEHICLE_ALARM_WARNING = 1U, /* 存在经确认的一般告警。 */
  VEHICLE_ALARM_FAULT = 2U, /* 存在严重通信或总线故障。 */
  VEHICLE_ALARM_RECOVERING = 3U /* 故障消失，正在进行恢复确认。 */
} VehicleAlarmState_t;

/* 定义仪表盘使用的统一告警位。 */
#define VEHICLE_ALARM_IMU_INVALID (1UL << 0U) /* IMU 数据无效。 */
#define VEHICLE_ALARM_GNSS_INVALID (1UL << 1U) /* GNSS 数据无效。 */
#define VEHICLE_ALARM_NODE_B_TIMEOUT (1UL << 2U) /* B 节点 Heartbeat 超时。 */
#define VEHICLE_ALARM_NODE_C_TIMEOUT (1UL << 3U) /* C 节点 Heartbeat 超时。 */
#define VEHICLE_ALARM_MQTT_OFFLINE (1UL << 4U) /* MQTT 未连接。 */
#define VEHICLE_ALARM_CAN_BUS_OFF (1UL << 5U) /* FDCAN 当前处于 Bus-Off。 */

/* 保存告警状态机快照，供 OLED、MQTT 和调试器统一读取。 */
typedef struct
{
  uint8_t state; /* 保存当前总体告警状态。 */
  uint32_t rawAlarmBits; /* 保存未经连续周期滤波的原始异常位。 */
  uint32_t activeAlarmBits; /* 保存当前已确认或立即生效的告警位。 */
  uint32_t warningConfirmCount; /* 保存连续一般异常周期数。 */
  uint32_t recoveryConfirmCount; /* 保存连续正常恢复周期数。 */
  uint32_t faultEnterCount; /* 统计进入 FAULT 的次数。 */
  uint32_t warningEnterCount; /* 统计进入 WARNING 的次数。 */
  uint32_t recoveryEnterCount; /* 统计进入 RECOVERING 的次数。 */
  uint32_t normalEnterCount; /* 统计恢复到 NORMAL 的次数。 */
  uint32_t lastTransitionTick; /* 保存最近一次状态转换的 HAL 节拍。 */
} VehicleAlarmStatus_t;

/* 初始化车辆告警状态机。 */
void VehicleAlarm_Init(void);
/* 使用当前诊断、Bus-Off 和 MQTT 状态更新一次状态机。 */
void VehicleAlarm_Update(const CanOpenPortDiagnostics_t *diagnostics,
                         uint8_t canBusOffActive,
                         uint8_t mqttConnected);
/* 复制当前告警状态快照。 */
void VehicleAlarm_GetStatus(volatile VehicleAlarmStatus_t *status);
/* 返回适合 OLED 显示的状态字符串。 */
const char *VehicleAlarm_GetStateText(uint8_t state);

#endif /* VEHICLE_ALARM_H */
