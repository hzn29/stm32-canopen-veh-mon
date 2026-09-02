#include "main.h"
#include "canopen_port.h"
#include "CANopen.h"
#include "OD.h"
#include "canopen_node_config.h"
#include "accident_detector.h"
#include "esp32_at.h"
#include "FreeRTOS.h"
#include "queue.h"
#include <string.h>

extern FDCAN_HandleTypeDef hfdcan1;
static CO_CANrx_t canopenRxArray[32];
static CO_CANtx_t canopenTxArray[40];
static CO_CANmodule_t canopenModule;
static uint8_t canopenNodeId;
static CO_t *canopenObject;
static uint32_t tpdoElapsedUs;
static uint32_t lastReceivedAccidentEventId;
static uint8_t receivedAccidentEventValid;
static ESP32_AccidentPersist_t accidentPending;
static uint32_t accidentNextRetryTick;
typedef struct
{
  uint8_t data[8]; /* 淇濆瓨鍘熷浜嬫晠浜嬩欢鏁版嵁瀛楄妭銆?*/
} CanOpenAccidentRxItem_t;
typedef struct
{
  uint32_t eventId;
} CanOpenAccidentAckRxItem_t;
static QueueHandle_t accidentRxQueue;
static QueueHandle_t accidentAckRxQueue;

static CanOpenPortDiagnostics_t canopenDiagnostics;

static void CanOpenPort_HandleAccidentFrame(uint16_t identifier, uint8_t dataLength,
                                            const uint8_t *data)
{
#if (CAN_NODE_ROLE == CAN_NODE_ROLE_A)
  if ((identifier != 0x382U) || (data == NULL))
  {
    return;
  }
  if (dataLength != 8U)
  {
    canopenDiagnostics.accidentEventInvalidCount++;
    return;
  }
  CanOpenAccidentRxItem_t item = {0};
  (void)memcpy(item.data, data, 8U);
  BaseType_t higherPriorityTaskWoken = pdFALSE;
  if ((accidentRxQueue == NULL) ||
      (xQueueSendFromISR(accidentRxQueue, &item, &higherPriorityTaskWoken) != pdPASS))
  {
    canopenDiagnostics.accidentEventInvalidCount++;
  }
  portYIELD_FROM_ISR(higherPriorityTaskWoken);
#else
  (void)identifier;
  (void)dataLength;
  (void)data;
#endif
}

static void CanOpenPort_HandleAccidentAck(uint16_t identifier, uint8_t dataLength,
                                          const uint8_t *data)
{
#if (CAN_NODE_ROLE == CAN_NODE_ROLE_B)
  if ((identifier != (uint16_t)(0x500U + CANOPEN_NODE_ID)) ||
      (dataLength != 4U) || (data == NULL))
  {
    return;
  }
  CanOpenAccidentAckRxItem_t item = {0};
  item.eventId = (uint32_t)data[0] |
                 ((uint32_t)data[1] << 8U) |
                 ((uint32_t)data[2] << 16U) |
                 ((uint32_t)data[3] << 24U);
  BaseType_t higherPriorityTaskWoken = pdFALSE;
  if ((accidentAckRxQueue == NULL) ||
      (xQueueSendFromISR(accidentAckRxQueue, &item, &higherPriorityTaskWoken) != pdPASS))
  {
    canopenDiagnostics.accidentAckTimeoutCount++;
  }
  portYIELD_FROM_ISR(higherPriorityTaskWoken);
#else
  (void)identifier;
  (void)dataLength;
  (void)data;
#endif
}

static void CanOpenPort_ProcessAccidentAckQueue(void)
{
#if (CAN_NODE_ROLE == CAN_NODE_ROLE_B)
  CanOpenAccidentAckRxItem_t item = {0};
  while ((accidentAckRxQueue != NULL) &&
         (xQueueReceive(accidentAckRxQueue, &item, 0U) == pdPASS))
  {
    if ((accidentPending.pendingValid != 0U) &&
        (accidentPending.pendingEventId == item.eventId))
    {
      accidentPending.pendingValid = 0U;
      accidentPending.pendingRetryCount = 0U;
      canopenDiagnostics.accidentAckRxCount++;
      (void)ESP32_Persist_SaveAccident(&accidentPending);
    }
  }
#endif
}

static void CanOpenPort_ProcessAccidentRetry(void)
{
#if (CAN_NODE_ROLE == CAN_NODE_ROLE_B)
  if (accidentPending.pendingValid == 0U)
  {
    return;
  }
  if ((int32_t)(HAL_GetTick() - accidentNextRetryTick) < 0)
  {
    return;
  }
  if (CanOpenPort_SendAccidentEvent(accidentPending.pendingEventType,
                                    accidentPending.pendingEventId,
                                    accidentPending.pendingPeakAccelMg,
                                    accidentPending.pendingPeakGyroDps) == CO_ERROR_NO)
  {
    if (accidentPending.pendingRetryCount < 0xFFU)
    {
      accidentPending.pendingRetryCount++;
    }
    canopenDiagnostics.accidentRetryCount++;
    accidentNextRetryTick = HAL_GetTick() +
      ((accidentPending.pendingRetryCount >= CANOPEN_ACCIDENT_ACK_RETRY_WARNING_LIMIT) ?
       CANOPEN_ACCIDENT_ACK_DEGRADED_RETRY_MS : CANOPEN_ACCIDENT_ACK_TIMEOUT_MS);
    (void)ESP32_Persist_SaveAccident(&accidentPending);
  }
  else
  {
    canopenDiagnostics.accidentAckTimeoutCount++;
    accidentNextRetryTick = HAL_GetTick() + CANOPEN_ACCIDENT_ACK_TIMEOUT_MS;
  }
#endif
}

static void CanOpenPort_ProcessAccidentEventQueue(void)
{
#if (CAN_NODE_ROLE == CAN_NODE_ROLE_A)
  CanOpenAccidentRxItem_t item = {0};
  while ((accidentRxQueue != NULL) &&
         (xQueueReceive(accidentRxQueue, &item, 0U) == pdPASS))
  {
    canopenDiagnostics.accidentEventRxCount++;
    if ((item.data[0] != ACCIDENT_EVENT_COLLISION) &&
        (item.data[0] != ACCIDENT_EVENT_ROLLOVER))
    {
      canopenDiagnostics.accidentEventInvalidCount++;
      continue;
    }
    uint32_t eventId = (uint32_t)item.data[2] |
                       ((uint32_t)item.data[3] << 8U) |
                       ((uint32_t)item.data[4] << 16U) |
                       ((uint32_t)item.data[5] << 24U);
    (void)CanOpenPort_SendAccidentAck(eventId);
    if ((receivedAccidentEventValid != 0U) &&
        (eventId == lastReceivedAccidentEventId))
    {
      canopenDiagnostics.accidentEventDuplicateCount++;
      continue;
    }
    receivedAccidentEventValid = 1U;
    lastReceivedAccidentEventId = eventId;
    uint32_t peakAccelMg = (uint32_t)item.data[6] * CANOPEN_ACCIDENT_ACCEL_UNIT_MG;
    uint32_t peakGyroDps = (uint32_t)item.data[7] * CANOPEN_ACCIDENT_GYRO_UNIT_DPS;
    AccidentDetector_ApplyRemoteEvent(item.data[0], eventId, peakAccelMg, peakGyroDps);
    ESP32_QueueAccidentEvent(eventId, item.data[0], (uint16_t)peakAccelMg, (uint16_t)peakGyroDps);
    canopenDiagnostics.lastAccidentEventId = eventId;
    canopenDiagnostics.lastAccidentEventType = item.data[0];
  }
#else
#endif
}

static void CanOpenPort_RecordPdoFrame(uint16_t identifier, uint8_t dataLength)
{
#if (CAN_NODE_ROLE == CAN_NODE_ROLE_A)
  CanOpenPortPdoHealth_t *health = NULL;
  uint32_t currentTick = HAL_GetTick();
  if (dataLength != 8U)
  {
    return;
  }
  switch (identifier & 0x07FFU)
  {
    case 0x182U:
      health = &canopenDiagnostics.imuAccel;
      break;
    case 0x282U:
      health = &canopenDiagnostics.imuGyro;
      break;
    case 0x183U:
      health = &canopenDiagnostics.gnssPosition;
      break;
    case 0x283U:
      health = &canopenDiagnostics.gnssStatus;
      break;
    default:
      break;
  }
  if (health != NULL)
  {
    health->frameSeen = 1U;
    health->lastRxTick = currentTick;
    health->rxFrameCount++;
    health->dataValid = 1U;
  }
#else
  (void)identifier;
  (void)dataLength;
#endif
}

static void CanOpenPort_UpdatePdoHealthOne(CanOpenPortPdoHealth_t *health, uint32_t timeoutMs)
{
  uint32_t currentTick = HAL_GetTick();
  if (health == NULL)
  {
    return;
  }
  if (health->frameSeen == 0U)
  {
    health->dataValid = 0U;
    return;
  }
  if ((currentTick - health->lastRxTick) > timeoutMs)
  {
    if (health->dataValid != 0U)
    {
      health->timeoutCount++;
    }
    health->dataValid = 0U;
  }
  else
  {
    health->dataValid = 1U;
  }
}

#define CANOPEN_NMT_CONTROL (CO_NMT_STARTUP_TO_OPERATIONAL | CO_NMT_ERR_ON_ERR_REG | CO_ERR_REG_GENERIC_ERR | CO_ERR_REG_COMMUNICATION)

static void CanOpenPort_ResetDiagnostics(void)
{
  (void)memset(&canopenDiagnostics, 0, sizeof(canopenDiagnostics));
  canopenDiagnostics.heartbeatB.nodeId = 2U;
  canopenDiagnostics.heartbeatC.nodeId = 3U;
  canopenDiagnostics.heartbeatB.state = 0U;
  canopenDiagnostics.heartbeatB.healthState = CANOPEN_HEALTH_UNCONFIGURED;
  canopenDiagnostics.heartbeatC.state = 0U;
  canopenDiagnostics.heartbeatC.healthState = CANOPEN_HEALTH_UNCONFIGURED;
  canopenDiagnostics.nmtState = (uint8_t)CO_NMT_INITIALIZING;
  lastReceivedAccidentEventId = 0U;
  receivedAccidentEventValid = 0U;
}

static void CanOpenPort_UpdateHeartbeatOne(uint8_t index, CanOpenPortHeartbeatStatus_t *status)
{
  uint8_t currentState = 0U;
  uint32_t currentTick = HAL_GetTick();
  uint8_t oldHealthState = status->healthState;
  if ((canopenObject != NULL) && (canopenObject->HBcons != NULL) &&
      (canopenObject->HBconsMonitoredNodes != NULL) &&
      (index < canopenObject->HBcons->numberOfMonitoredNodes))
  {
    currentState = (uint8_t)canopenObject->HBconsMonitoredNodes[index].HBstate;
  }
  if (currentState != status->state)
  {
    status->previousState = status->state;
    status->state = currentState;
    status->lastTransitionTick = currentTick;
    if (currentState == 3U)
    {
      status->faultStartTick = currentTick;
      status->recoveryStartTick = 0U;
      status->faultActive = 1U;
      status->timeoutCount++;
      status->timeoutEpisodeCount++;
    }
    else if ((status->faultActive != 0U) && (currentState == 2U))
    {
      status->lastRecoveryTimeMs = currentTick - status->faultStartTick;
      status->recoveryCount++;
      status->faultStartTick = 0U;
      status->faultActive = 0U;
    }
    else
    {
    }
  }
  if (currentState == 0U)
  {
    status->healthState = CANOPEN_HEALTH_UNCONFIGURED;
  }
  else if (currentState == 1U)
  {
    status->healthState = CANOPEN_HEALTH_UNKNOWN;
  }
  else if (currentState == 3U)
  {
    if ((status->timeoutEpisodeCount >= CANOPEN_HEALTH_TIMEOUT_EPISODE_LIMIT) ||
        ((status->faultActive != 0U) &&
         ((HAL_GetTick() - status->faultStartTick) >= CANOPEN_HEALTH_RECOVERY_TIMEOUT_MS)))
    {
      status->healthState = CANOPEN_HEALTH_FAULT;
    }
    else
    {
      status->healthState = CANOPEN_HEALTH_TIMEOUT;
    }
  }
  else
  {
    if ((status->previousState == 3U) &&
        (status->healthState != CANOPEN_HEALTH_RECOVERING) &&
        (status->recoveryStartTick == 0U))
    {
      status->recoveryStartTick = HAL_GetTick();
      status->healthState = CANOPEN_HEALTH_RECOVERING;
    }
    else if ((status->healthState == CANOPEN_HEALTH_RECOVERING) &&
             ((HAL_GetTick() - status->recoveryStartTick) >= CANOPEN_HEALTH_STABLE_TIME_MS))
    {
      status->healthState = CANOPEN_HEALTH_ACTIVE;
    }
    else if ((status->healthState != CANOPEN_HEALTH_RECOVERING) &&
             (status->healthState != CANOPEN_HEALTH_ACTIVE))
    {
      status->healthState = CANOPEN_HEALTH_ACTIVE;
    }
  }
  if (status->healthState != oldHealthState)
  {
    status->previousHealthState = oldHealthState;
  }
}

static void CanOpenPort_UpdateDiagnostics(void)
{
  bool_t heartbeatError = false;
  uint8_t errorRegister = 0U;
  uint8_t nmtState = (uint8_t)CO_NMT_INITIALIZING;
  uint32_t timeoutInfoCode = 0U;
  if (canopenObject != NULL)
  {
    errorRegister = CO_getErrorRegister(canopenObject->em);
    heartbeatError = CO_isError(canopenObject->em, CO_EM_HEARTBEAT_CONSUMER);
    nmtState = (uint8_t)CO_NMT_getInternalState(canopenObject->NMT);
  }
  CanOpenPort_UpdateHeartbeatOne(0U, &canopenDiagnostics.heartbeatB);
  CanOpenPort_UpdateHeartbeatOne(1U, &canopenDiagnostics.heartbeatC);
  CanOpenPort_UpdatePdoHealthOne(&canopenDiagnostics.imuAccel, CANOPEN_IMU_PDO_TIMEOUT_MS);
  CanOpenPort_UpdatePdoHealthOne(&canopenDiagnostics.imuGyro, CANOPEN_IMU_PDO_TIMEOUT_MS);
  CanOpenPort_UpdatePdoHealthOne(&canopenDiagnostics.gnssPosition, CANOPEN_GNSS_PDO_TIMEOUT_MS);
  CanOpenPort_UpdatePdoHealthOne(&canopenDiagnostics.gnssStatus, CANOPEN_GNSS_PDO_TIMEOUT_MS);
  canopenDiagnostics.imuDataValid =
      (canopenDiagnostics.imuAccel.dataValid != 0U) &&
      (canopenDiagnostics.imuGyro.dataValid != 0U);
  canopenDiagnostics.gnssDataValid =
      (canopenDiagnostics.gnssPosition.dataValid != 0U) &&
      (canopenDiagnostics.gnssStatus.dataValid != 0U);
  canopenDiagnostics.nodeBHealthy =
      (canopenDiagnostics.heartbeatB.healthState == CANOPEN_HEALTH_ACTIVE) &&
      (canopenDiagnostics.imuDataValid != 0U);
  canopenDiagnostics.nodeCHealthy =
      (canopenDiagnostics.heartbeatC.healthState == CANOPEN_HEALTH_ACTIVE) &&
      (canopenDiagnostics.gnssDataValid != 0U);
  if (heartbeatError && (canopenDiagnostics.heartbeatErrorActive == 0U))
  {
    canopenDiagnostics.emcyReportCount++;
    canopenDiagnostics.lastEmcyErrorCode = 0x8130U;
    canopenDiagnostics.lastEmcyErrorBit = CO_EM_HEARTBEAT_CONSUMER;
    if (canopenDiagnostics.heartbeatB.state == 3U)
    {
      timeoutInfoCode = 0U;
    }
    else if (canopenDiagnostics.heartbeatC.state == 3U)
    {
      timeoutInfoCode = 1U;
    }
    else
    {
      timeoutInfoCode = 0U;
    }
    canopenDiagnostics.lastEmcyInfoCode = timeoutInfoCode;
    canopenDiagnostics.lastEmcyTick = HAL_GetTick();
  }
  else if ((!heartbeatError) && (canopenDiagnostics.heartbeatErrorActive != 0U))
  {
    canopenDiagnostics.emcyClearCount++;
    canopenDiagnostics.lastEmcyClearTick = HAL_GetTick();
  }
  else
  {
  }
  canopenDiagnostics.heartbeatErrorActive = heartbeatError ? 1U : 0U;
  canopenDiagnostics.errorRegister = errorRegister;
  canopenDiagnostics.nmtState = nmtState;
}

static void CanOpenPort_ApplyHealthNmtPolicy(void)
{
#if (CAN_NODE_ROLE == CAN_NODE_ROLE_A)
  bool_t holdForB = ((canopenDiagnostics.heartbeatB.healthState == CANOPEN_HEALTH_RECOVERING) ||
                     (canopenDiagnostics.heartbeatB.healthState == CANOPEN_HEALTH_FAULT));
  bool_t holdForC = ((canopenDiagnostics.heartbeatC.healthState == CANOPEN_HEALTH_RECOVERING) ||
                     (canopenDiagnostics.heartbeatC.healthState == CANOPEN_HEALTH_FAULT));
  bool_t holdForBData = ((canopenDiagnostics.heartbeatB.healthState == CANOPEN_HEALTH_ACTIVE) &&
                         (canopenDiagnostics.imuDataValid == 0U));
  bool_t holdForCData = ((canopenDiagnostics.heartbeatC.healthState == CANOPEN_HEALTH_ACTIVE) &&
                         (canopenDiagnostics.gnssDataValid == 0U));
  if ((canopenObject != NULL) && (canopenObject->NMT != NULL) &&
      (holdForB || holdForC || holdForBData || holdForCData))
  {
    CO_NMT_sendInternalCommand(canopenObject->NMT, CO_NMT_ENTER_PRE_OPERATIONAL);
  }
  else if ((canopenObject != NULL) && (canopenObject->NMT != NULL))
  {
    CO_NMT_sendInternalCommand(canopenObject->NMT, CO_NMT_ENTER_OPERATIONAL);
  }
#else
#endif
}

CO_ReturnError_t CanOpenPort_Init(uint8_t nodeId)
{
  canopenNodeId = nodeId;
  (void)canopenNodeId;
  return CO_CANmodule_init(&canopenModule, &hfdcan1, canopenRxArray, 32U,
                           canopenTxArray, 40U, 500U);
}

CO_ReturnError_t CanOpenPort_StackInit(uint8_t nodeId)
{
  uint32_t errorInfo = 0U;
  CanOpenPort_ResetDiagnostics();
  canopenObject = CO_new(NULL, NULL);
  if (canopenObject == NULL)
  {
    return CO_ERROR_OUT_OF_MEMORY;
  }
  if (CO_CANinit(canopenObject, &hfdcan1, 500U) != CO_ERROR_NO)
  {
    return CO_ERROR_ILLEGAL_ARGUMENT;
  }
  if (CO_CANopenInit(canopenObject, NULL, NULL, OD, NULL,
                     CANOPEN_NMT_CONTROL, 500U, 1000U, 500U,
                     false, nodeId, &errorInfo) != CO_ERROR_NO)
  {
    return CO_ERROR_OD_PARAMETERS;
  }
  if (CO_CANopenInitPDO(canopenObject, canopenObject->em, OD, nodeId, &errorInfo) != CO_ERROR_NO)
  {
    return CO_ERROR_OD_PARAMETERS;
  }
#if (CAN_NODE_ROLE == CAN_NODE_ROLE_A)
  accidentRxQueue = xQueueCreate(8U, sizeof(CanOpenAccidentRxItem_t));
  if (accidentRxQueue == NULL)
  {
    return CO_ERROR_OUT_OF_MEMORY;
  }
  accidentAckRxQueue = NULL;
#elif (CAN_NODE_ROLE == CAN_NODE_ROLE_B)
  accidentAckRxQueue = xQueueCreate(8U, sizeof(CanOpenAccidentAckRxItem_t));
  if (accidentAckRxQueue == NULL)
  {
    return CO_ERROR_OUT_OF_MEMORY;
  }
  accidentRxQueue = NULL;
#else
  accidentRxQueue = NULL;
  accidentAckRxQueue = NULL;
#endif
  CO_CANsetNormalMode(canopenObject->CANmodule);
  return CO_ERROR_NO;
}

void CanOpenPort_RxInterrupt(uint16_t identifier, uint8_t dataLength, const uint8_t *data)
{
  if ((canopenObject == NULL) || (data == NULL))
  {
    return;
  }
  uint8_t stackLength = (dataLength > 8U) ? 8U : dataLength;
  CO_CANinterruptMessage(canopenObject->CANmodule, identifier, stackLength, data);
  CanOpenPort_HandleAccidentFrame(identifier & 0x07FFU, dataLength, data);
  CanOpenPort_HandleAccidentAck(identifier & 0x07FFU, dataLength, data);
  CanOpenPort_RecordPdoFrame(identifier & 0x07FFU, dataLength);
}

void CanOpenPort_Process(uint32_t timeDifferenceUs)
{
  bool_t syncWas = false;
  if (canopenObject != NULL)
  {
    tpdoElapsedUs += timeDifferenceUs;
    if (tpdoElapsedUs >= 100000U)
    {
      tpdoElapsedUs -= 100000U;
#if (CAN_NODE_ROLE == CAN_NODE_ROLE_C)
      OD_requestTPDO(OD_ENTRY_H2100, 0U);
      OD_requestTPDO(OD_ENTRY_H2102, 1U);
#else
      OD_RAM.x2000_tpdoSequence++;
      OD_requestTPDO(OD_ENTRY_H2000, 0U);
#endif
    }
    (void)CO_process(canopenObject, false, timeDifferenceUs, NULL);
    syncWas = CO_process_SYNC(canopenObject, timeDifferenceUs, NULL);
    CO_process_RPDO(canopenObject, syncWas, timeDifferenceUs, NULL);
    CO_process_TPDO(canopenObject, syncWas, timeDifferenceUs, NULL);
    CanOpenPort_ProcessAccidentEventQueue();
    CanOpenPort_ProcessAccidentAckQueue();
    CanOpenPort_ProcessAccidentRetry();
    CanOpenPort_UpdateDiagnostics();
    CanOpenPort_ApplyHealthNmtPolicy();
  }
}

void CanOpenPort_UpdateGnss(const BH182_Data_t *data)
{
#if (CAN_NODE_ROLE == CAN_NODE_ROLE_C)
  if (data == NULL)
  {
    return;
  }
  OD_RAM.x2100_latitudeE7 = data->latitudeE7;
  OD_RAM.x2101_longitudeE7 = data->longitudeE7;
  OD_RAM.x2102_altitudeMm = data->altitudeMm;
  OD_RAM.x2103_speedMmps = (data->speedMmps > 0xFFFFU) ? 0xFFFFU : (uint16_t)data->speedMmps;
  OD_RAM.x2104_satelliteCount = (data->satelliteCount > 0xFFU) ? 0xFFU : (uint8_t)data->satelliteCount;
  OD_RAM.x2105_fixValid = data->fixValid;
#else
  (void)data;
#endif
}

void CanOpenPort_UpdateImu(const ICM42688_RawData_t *data, uint8_t valid)
{
#if (CAN_NODE_ROLE == CAN_NODE_ROLE_B)
  if (data == NULL)
  {
    return;
  }
  OD_RAM.x2110_accelX = data->accelX;
  OD_RAM.x2111_accelY = data->accelY;
  OD_RAM.x2112_accelZ = data->accelZ;
  OD_RAM.x2113_gyroX = data->gyroX;
  OD_RAM.x2114_gyroY = data->gyroY;
  OD_RAM.x2115_gyroZ = data->gyroZ;
  OD_RAM.x2116_valid = valid;
  OD_RAM.x2117_tpdo1Sequence++;
  OD_RAM.x2118_tpdo2Sequence++;
#else
  (void)data;
  (void)valid;
#endif
}

CO_ReturnError_t CanOpenPort_SendAccidentEvent(uint8_t eventType, uint32_t eventId,
                                               uint32_t peakAccelMg, uint32_t peakGyroDps)
{
#if (CAN_NODE_ROLE == CAN_NODE_ROLE_B)
  uint32_t infoCode = eventId;
  if ((eventType != ACCIDENT_EVENT_COLLISION) &&
      (eventType != ACCIDENT_EVENT_ROLLOVER))
  {
    return CO_ERROR_ILLEGAL_ARGUMENT;
  }
  if (canopenObject == NULL)
  {
    return CO_ERROR_ILLEGAL_ARGUMENT;
  }
  OD_RAM.x2120_accidentType = eventType;
  OD_RAM.x2121_accidentFlags = 0U;
  uint32_t accelUnit = (peakAccelMg + CANOPEN_ACCIDENT_ACCEL_UNIT_MG - 1U) /
                       CANOPEN_ACCIDENT_ACCEL_UNIT_MG;
  uint32_t gyroUnit = (peakGyroDps + CANOPEN_ACCIDENT_GYRO_UNIT_DPS - 1U) /
                      CANOPEN_ACCIDENT_GYRO_UNIT_DPS;
  OD_RAM.x2122_accidentEventId = eventId;
  OD_RAM.x2123_peakAccelMg = (accelUnit > 0xFFU) ? 0xFFU : (uint8_t)accelUnit;
  OD_RAM.x2124_peakGyroDps = (gyroUnit > 0xFFU) ? 0xFFU : (uint8_t)gyroUnit;
  OD_requestTPDO(OD_ENTRY_H2120, 2U);
  canopenDiagnostics.lastAccidentEventId = eventId;
  canopenDiagnostics.lastAccidentEventType = eventType;
  canopenDiagnostics.accidentEventTxCount++;
  if ((accidentPending.pendingValid == 0U) ||
      (accidentPending.pendingEventId != eventId))
  {
    accidentPending.pendingEventId = eventId;
    accidentPending.lastEventId = eventId;
    accidentPending.pendingEventType = eventType;
    accidentPending.pendingPeakAccelMg = peakAccelMg;
    accidentPending.pendingPeakGyroDps = peakGyroDps;
    accidentPending.pendingRetryCount = 0U;
    accidentPending.pendingValid = 1U;
    accidentNextRetryTick = HAL_GetTick() + CANOPEN_ACCIDENT_ACK_TIMEOUT_MS;
    (void)ESP32_Persist_SaveAccident(&accidentPending);
  }
  CO_errorReport(canopenObject->em, (uint8_t)(CO_EM_MANUFACTURER_START +
                  ((eventType == ACCIDENT_EVENT_ROLLOVER) ? 1U : 0U)),
                 CO_EMC_DEVICE_SPECIFIC, infoCode);
  return CO_ERROR_NO;
#else
  (void)eventType;
  (void)eventId;
  (void)peakAccelMg;
  (void)peakGyroDps;
  return CO_ERROR_ILLEGAL_ARGUMENT;
#endif
}

CO_ReturnError_t CanOpenPort_SendAccidentAck(uint32_t eventId)
{
#if (CAN_NODE_ROLE == CAN_NODE_ROLE_A)
  FDCAN_TxHeaderTypeDef header = {0};
  uint8_t data[4] = {0};
  header.Identifier = 0x502U;
  header.IdType = FDCAN_STANDARD_ID;
  header.TxFrameType = FDCAN_DATA_FRAME;
  header.DataLength = FDCAN_DLC_BYTES_4;
  header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  header.BitRateSwitch = FDCAN_BRS_OFF;
  header.FDFormat = FDCAN_CLASSIC_CAN;
  header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  header.MessageMarker = 0U;
  data[0] = (uint8_t)(eventId & 0xFFU);
  data[1] = (uint8_t)((eventId >> 8U) & 0xFFU);
  data[2] = (uint8_t)((eventId >> 16U) & 0xFFU);
  data[3] = (uint8_t)((eventId >> 24U) & 0xFFU);
  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &header, data) == HAL_OK)
  {
    canopenDiagnostics.accidentAckTxCount++;
    return CO_ERROR_NO;
  }
  canopenDiagnostics.accidentAckTxErrorCount++;
  return CO_ERROR_TX_OVERFLOW;
#else
  (void)eventId;
  return CO_ERROR_ILLEGAL_ARGUMENT;
#endif
}

void CanOpenPort_RestoreAccidentState(const ESP32_AccidentPersist_t *snapshot)
{
#if (CAN_NODE_ROLE == CAN_NODE_ROLE_B)
  if (snapshot == NULL)
  {
    return;
  }
  accidentPending = *snapshot;
  if (accidentPending.pendingValid != 0U)
  {
    accidentNextRetryTick = HAL_GetTick() + 100U;
  }
#else
  (void)snapshot;
#endif
}

void CanOpenPort_ClearAccidentError(uint8_t eventType)
{
#if (CAN_NODE_ROLE == CAN_NODE_ROLE_B)
  if (canopenObject != NULL)
  {
    uint8_t errorBit = (uint8_t)(CO_EM_MANUFACTURER_START +
                         ((eventType == ACCIDENT_EVENT_ROLLOVER) ? 1U : 0U));
    CO_errorReset(canopenObject->em, errorBit, (uint32_t)eventType);
  }
#else
  (void)eventType;
#endif
}

CO_CANmodule_t* CanOpenPort_GetModule(void)
{
  return &canopenModule;
}

void CanOpenPort_GetDiagnostics(volatile CanOpenPortDiagnostics_t *diagnostics)
{
  if (diagnostics == NULL)
  {
    return;
  }
  *diagnostics = canopenDiagnostics;
}
