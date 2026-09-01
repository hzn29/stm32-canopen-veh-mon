#include "vehicle_alarm.h"
#include "stm32g4xx_hal.h"
#include <string.h>

#define VEHICLE_ALARM_WARNING_CONFIRM_CYCLES 3U
#define VEHICLE_ALARM_RECOVERY_CONFIRM_CYCLES 5U
#define VEHICLE_ALARM_FAULT_MASK (VEHICLE_ALARM_NODE_B_TIMEOUT | VEHICLE_ALARM_NODE_C_TIMEOUT | VEHICLE_ALARM_CAN_BUS_OFF)
#define VEHICLE_ALARM_WARNING_MASK (VEHICLE_ALARM_IMU_INVALID | VEHICLE_ALARM_GNSS_INVALID | VEHICLE_ALARM_MQTT_OFFLINE)

static volatile VehicleAlarmStatus_t vehicleAlarmStatus;

static void VehicleAlarm_SetState(uint8_t state)
{
  if (vehicleAlarmStatus.state == state)
  {
    return;
  }
  vehicleAlarmStatus.state = state;
  vehicleAlarmStatus.lastTransitionTick = HAL_GetTick();
  if (state == VEHICLE_ALARM_WARNING)
  {
    vehicleAlarmStatus.warningEnterCount++;
  }
  else if (state == VEHICLE_ALARM_FAULT)
  {
    vehicleAlarmStatus.faultEnterCount++;
  }
  else if (state == VEHICLE_ALARM_RECOVERING)
  {
    vehicleAlarmStatus.recoveryEnterCount++;
  }
  else
  {
    vehicleAlarmStatus.normalEnterCount++;
  }
}

void VehicleAlarm_Init(void)
{
  (void)memset((void *)&vehicleAlarmStatus, 0, sizeof(vehicleAlarmStatus));
  vehicleAlarmStatus.state = VEHICLE_ALARM_NORMAL;
  vehicleAlarmStatus.lastTransitionTick = HAL_GetTick();
}

void VehicleAlarm_Update(const CanOpenPortDiagnostics_t *diagnostics,
                         uint8_t canBusOffActive,
                         uint8_t mqttConnected)
{
  uint32_t rawAlarmBits = 0U;
  uint32_t faultBits = 0U;
  uint32_t warningBits = 0U;
  if (diagnostics == NULL)
  {
    return;
  }
  if (diagnostics->imuDataValid == 0U)
  {
    rawAlarmBits |= VEHICLE_ALARM_IMU_INVALID;
  }
  if (diagnostics->gnssDataValid == 0U)
  {
    rawAlarmBits |= VEHICLE_ALARM_GNSS_INVALID;
  }
  if (diagnostics->nodeBHealthy == 0U)
  {
    rawAlarmBits |= VEHICLE_ALARM_NODE_B_TIMEOUT;
  }
  if (diagnostics->nodeCHealthy == 0U)
  {
    rawAlarmBits |= VEHICLE_ALARM_NODE_C_TIMEOUT;
  }
  if (mqttConnected == 0U)
  {
    rawAlarmBits |= VEHICLE_ALARM_MQTT_OFFLINE;
  }
  if (canBusOffActive != 0U)
  {
    rawAlarmBits |= VEHICLE_ALARM_CAN_BUS_OFF;
  }
  vehicleAlarmStatus.rawAlarmBits = rawAlarmBits;
  faultBits = rawAlarmBits & VEHICLE_ALARM_FAULT_MASK;
  warningBits = rawAlarmBits & VEHICLE_ALARM_WARNING_MASK;
  if (faultBits != 0U)
  {
    vehicleAlarmStatus.recoveryConfirmCount = 0U;
    vehicleAlarmStatus.warningConfirmCount = 0U;
    vehicleAlarmStatus.activeAlarmBits = rawAlarmBits;
    VehicleAlarm_SetState(VEHICLE_ALARM_FAULT);
    return;
  }
  if (warningBits != 0U)
  {
    vehicleAlarmStatus.recoveryConfirmCount = 0U;
    vehicleAlarmStatus.warningConfirmCount++;
    if (vehicleAlarmStatus.warningConfirmCount >= VEHICLE_ALARM_WARNING_CONFIRM_CYCLES)
    {
      vehicleAlarmStatus.activeAlarmBits = warningBits;
      VehicleAlarm_SetState(VEHICLE_ALARM_WARNING);
    }
    return;
  }
  vehicleAlarmStatus.warningConfirmCount = 0U;
  if (vehicleAlarmStatus.state == VEHICLE_ALARM_NORMAL)
  {
    vehicleAlarmStatus.activeAlarmBits = 0U;
    return;
  }
  vehicleAlarmStatus.recoveryConfirmCount++;
  vehicleAlarmStatus.activeAlarmBits = 0U;
  VehicleAlarm_SetState(VEHICLE_ALARM_RECOVERING);
  if (vehicleAlarmStatus.recoveryConfirmCount >= VEHICLE_ALARM_RECOVERY_CONFIRM_CYCLES)
  {
    VehicleAlarm_SetState(VEHICLE_ALARM_NORMAL);
  }
}

void VehicleAlarm_GetStatus(volatile VehicleAlarmStatus_t *status)
{
  if (status == NULL)
  {
    return;
  }
  *status = vehicleAlarmStatus;
}

const char *VehicleAlarm_GetStateText(uint8_t state)
{
  if (state == VEHICLE_ALARM_WARNING)
  {
    return "WARNING";
  }
  if (state == VEHICLE_ALARM_FAULT)
  {
    return "FAULT";
  }
  if (state == VEHICLE_ALARM_RECOVERING)
  {
    return "RECOVERING";
  }
  return "NORMAL";
}
