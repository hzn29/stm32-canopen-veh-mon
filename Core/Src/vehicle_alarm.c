/* 车辆仪表盘统一告警状态机实现。 */
#include "vehicle_alarm.h"
/* 引入 HAL_GetTick 接口。 */
#include "stm32g4xx_hal.h"
/* 引入内存清零函数。 */
#include <string.h>

/* 定义一般告警连续异常确认周期数。 */
#define VEHICLE_ALARM_WARNING_CONFIRM_CYCLES 3U
/* 定义故障恢复连续正常确认周期数。 */
#define VEHICLE_ALARM_RECOVERY_CONFIRM_CYCLES 5U
/* 定义严重故障告警位掩码。 */
#define VEHICLE_ALARM_FAULT_MASK (VEHICLE_ALARM_NODE_B_TIMEOUT | VEHICLE_ALARM_NODE_C_TIMEOUT | VEHICLE_ALARM_CAN_BUS_OFF)
/* 定义需要连续确认的一般告警位掩码。 */
#define VEHICLE_ALARM_WARNING_MASK (VEHICLE_ALARM_IMU_INVALID | VEHICLE_ALARM_GNSS_INVALID | VEHICLE_ALARM_MQTT_OFFLINE)

/* 保存内部告警状态。 */
static volatile VehicleAlarmStatus_t vehicleAlarmStatus;

/* 执行状态转换并更新对应统计。 */
static void VehicleAlarm_SetState(uint8_t state)
{
  /* 仅在状态发生变化时更新统计。 */
  if (vehicleAlarmStatus.state == state)
  {
    /* 状态未变化时无需重复记录。 */
    return;
  }
  /* 保存新的状态。 */
  vehicleAlarmStatus.state = state;
  /* 保存本次状态转换时间。 */
  vehicleAlarmStatus.lastTransitionTick = HAL_GetTick();
  /* 按状态增加进入次数。 */
  if (state == VEHICLE_ALARM_WARNING)
  {
    /* 统计进入一般告警状态。 */
    vehicleAlarmStatus.warningEnterCount++;
  }
  else if (state == VEHICLE_ALARM_FAULT)
  {
    /* 统计进入严重故障状态。 */
    vehicleAlarmStatus.faultEnterCount++;
  }
  else if (state == VEHICLE_ALARM_RECOVERING)
  {
    /* 统计进入恢复确认状态。 */
    vehicleAlarmStatus.recoveryEnterCount++;
  }
  else
  {
    /* 统计进入正常状态。 */
    vehicleAlarmStatus.normalEnterCount++;
  }
}

/* 初始化车辆告警状态机。 */
void VehicleAlarm_Init(void)
{
  /* 清零全部状态和统计。 */
  (void)memset((void *)&vehicleAlarmStatus, 0, sizeof(vehicleAlarmStatus));
  /* 系统上电后先进入正常状态，后续由首个采样周期修正。 */
  vehicleAlarmStatus.state = VEHICLE_ALARM_NORMAL;
  /* 记录初始化时刻。 */
  vehicleAlarmStatus.lastTransitionTick = HAL_GetTick();
}

/* 使用当前诊断、Bus-Off 和 MQTT 状态更新一次状态机。 */
void VehicleAlarm_Update(const CanOpenPortDiagnostics_t *diagnostics,
                         uint8_t canBusOffActive,
                         uint8_t mqttConnected)
{
  /* 保存本周期原始告警位。 */
  uint32_t rawAlarmBits = 0U;
  /* 保存当前严重故障位。 */
  uint32_t faultBits = 0U;
  /* 保存当前一般告警位。 */
  uint32_t warningBits = 0U;
  /* 检查诊断快照指针。 */
  if (diagnostics == NULL)
  {
    /* 无诊断数据时不更新状态机。 */
    return;
  }
  /* 根据 IMU 数据健康状态生成原始异常位。 */
  if (diagnostics->imuDataValid == 0U)
  {
    /* 标记 IMU 数据无效。 */
    rawAlarmBits |= VEHICLE_ALARM_IMU_INVALID;
  }
  /* 根据 GNSS 数据健康状态生成原始异常位。 */
  if (diagnostics->gnssDataValid == 0U)
  {
    /* 标记 GNSS 数据无效。 */
    rawAlarmBits |= VEHICLE_ALARM_GNSS_INVALID;
  }
  /* 根据 B 节点健康状态生成立即故障位。 */
  if (diagnostics->nodeBHealthy == 0U)
  {
    /* 标记 B 节点 Heartbeat 或 PDO 超时。 */
    rawAlarmBits |= VEHICLE_ALARM_NODE_B_TIMEOUT;
  }
  /* 根据 C 节点健康状态生成立即故障位。 */
  if (diagnostics->nodeCHealthy == 0U)
  {
    /* 标记 C 节点 Heartbeat 或 PDO 超时。 */
    rawAlarmBits |= VEHICLE_ALARM_NODE_C_TIMEOUT;
  }
  /* 根据 MQTT 连接状态生成原始异常位。 */
  if (mqttConnected == 0U)
  {
    /* 标记 MQTT 离线。 */
    rawAlarmBits |= VEHICLE_ALARM_MQTT_OFFLINE;
  }
  /* 根据 FDCAN Bus-Off 状态生成立即故障位。 */
  if (canBusOffActive != 0U)
  {
    /* 标记 CAN Bus-Off。 */
    rawAlarmBits |= VEHICLE_ALARM_CAN_BUS_OFF;
  }
  /* 保存本周期未经滤波的告警位。 */
  vehicleAlarmStatus.rawAlarmBits = rawAlarmBits;
  /* 提取立即处理的严重故障位。 */
  faultBits = rawAlarmBits & VEHICLE_ALARM_FAULT_MASK;
  /* 提取需要连续确认的一般告警位。 */
  warningBits = rawAlarmBits & VEHICLE_ALARM_WARNING_MASK;
  /* 严重故障无需等待，立即进入 FAULT。 */
  if (faultBits != 0U)
  {
    /* 清除恢复确认计数。 */
    vehicleAlarmStatus.recoveryConfirmCount = 0U;
    /* 清除一般告警确认计数。 */
    vehicleAlarmStatus.warningConfirmCount = 0U;
    /* 保存严重故障与同时存在的一般异常位。 */
    vehicleAlarmStatus.activeAlarmBits = rawAlarmBits;
    /* 立即进入严重故障状态。 */
    VehicleAlarm_SetState(VEHICLE_ALARM_FAULT);
    return;
  }
  /* 一般异常存在时进行连续周期确认。 */
  if (warningBits != 0U)
  {
    /* 清除正常恢复确认计数。 */
    vehicleAlarmStatus.recoveryConfirmCount = 0U;
    /* 累加一般异常确认周期。 */
    vehicleAlarmStatus.warningConfirmCount++;
    /* 达到确认次数后进入 WARNING。 */
    if (vehicleAlarmStatus.warningConfirmCount >= VEHICLE_ALARM_WARNING_CONFIRM_CYCLES)
    {
      /* 保存已确认的一般告警位。 */
      vehicleAlarmStatus.activeAlarmBits = warningBits;
      /* 进入一般告警状态。 */
      VehicleAlarm_SetState(VEHICLE_ALARM_WARNING);
    }
    return;
  }
  /* 本周期所有原始异常均已消失。 */
  vehicleAlarmStatus.warningConfirmCount = 0U;
  /* 正常状态无需再次进入恢复确认。 */
  if (vehicleAlarmStatus.state == VEHICLE_ALARM_NORMAL)
  {
    /* 清除当前活动告警位。 */
    vehicleAlarmStatus.activeAlarmBits = 0U;
    return;
  }
  /* 累加连续正常恢复确认周期。 */
  vehicleAlarmStatus.recoveryConfirmCount++;
  /* 清除当前活动告警位，但保留状态显示恢复中。 */
  vehicleAlarmStatus.activeAlarmBits = 0U;
  /* 进入恢复确认状态。 */
  VehicleAlarm_SetState(VEHICLE_ALARM_RECOVERING);
  /* 达到恢复确认次数后回到正常。 */
  if (vehicleAlarmStatus.recoveryConfirmCount >= VEHICLE_ALARM_RECOVERY_CONFIRM_CYCLES)
  {
    /* 回到正常状态。 */
    VehicleAlarm_SetState(VEHICLE_ALARM_NORMAL);
  }
}

/* 复制当前告警状态快照。 */
void VehicleAlarm_GetStatus(volatile VehicleAlarmStatus_t *status)
{
  /* 检查输出指针。 */
  if (status == NULL)
  {
    /* 无效指针直接返回。 */
    return;
  }
  /* 复制完整状态快照。 */
  *status = vehicleAlarmStatus;
}

/* 返回适合 OLED 显示的状态字符串。 */
const char *VehicleAlarm_GetStateText(uint8_t state)
{
  /* 根据状态返回固定英文文本。 */
  if (state == VEHICLE_ALARM_WARNING)
  {
    /* 返回一般告警文本。 */
    return "WARNING";
  }
  if (state == VEHICLE_ALARM_FAULT)
  {
    /* 返回严重故障文本。 */
    return "FAULT";
  }
  if (state == VEHICLE_ALARM_RECOVERING)
  {
    /* 返回恢复确认文本。 */
    return "RECOVERING";
  }
  /* 返回正常状态文本。 */
  return "NORMAL";
}
