/* 实现 STM32G474RE 到 CANopenNode 的最小底层适配入口。 */
#include "main.h"
/* 引入 CANopenNode 底层接口。 */
#include "canopen_port.h"
#include "CANopen.h"
#include "OD.h"
#include "canopen_node_config.h"
#include "accident_detector.h"
#include "esp32_at.h"
#include "FreeRTOS.h"
#include "queue.h"
#include <string.h>

/* 声明由 CubeMX 生成的 FDCAN1 句柄。 */
extern FDCAN_HandleTypeDef hfdcan1;
/* 分配 CANopenNode 接收缓冲区。 */
static CO_CANrx_t canopenRxArray[32];
/* 分配 CANopenNode 发送缓冲区。 */
static CO_CANtx_t canopenTxArray[40];
/* 分配 CANopenNode CAN 模块对象。 */
static CO_CANmodule_t canopenModule;
/* 保存当前 CANopen 节点号。 */
static uint8_t canopenNodeId;
static CO_t *canopenObject;
/* 累计 TPDO 序号更新经过的微秒数。 */
static uint32_t tpdoElapsedUs;
/* 保存 A 节点最近一次已经应用的 B 节点事件编号。 */
static uint32_t lastReceivedAccidentEventId;
/* 标记 A 节点是否已经收到过事故事件，用于首次事件去重。 */
static uint8_t receivedAccidentEventValid;
/* 保存 B 节点待确认事故事件的运行时快照。 */
static ESP32_AccidentPersist_t accidentPending;
/* 保存 B 节点下次重发事故事件的本地时间。 */
static uint32_t accidentNextRetryTick;
/* 定义从 CAN 接收回调转交到应用任务的事故事件队列元素。 */
typedef struct
{
  uint8_t data[8]; /* 保存原始事故事件数据字节。 */
} CanOpenAccidentRxItem_t;
/* 定义 ACK 接收队列元素，避免在 FDCAN 中断中直接执行 Flash 操作。 */
typedef struct
{
  uint32_t eventId; /* 保存 A 节点 ACK 的事故事件编号。 */
} CanOpenAccidentAckRxItem_t;
/* 保存 A 节点事故事件接收队列句柄。 */
static QueueHandle_t accidentRxQueue;
/* 保存 B 节点 ACK 接收队列句柄，由 CAN 处理任务消费。 */
static QueueHandle_t accidentAckRxQueue;

/* 保存本适配层维护的 EMCY、NMT 和 Heartbeat 诊断数据。 */
static CanOpenPortDiagnostics_t canopenDiagnostics;

/* 处理 A 节点收到的 B 节点事故事件 TPDO。 */
static void CanOpenPort_HandleAccidentFrame(uint16_t identifier, uint8_t dataLength,
                                            const uint8_t *data)
{
#if (CAN_NODE_ROLE == CAN_NODE_ROLE_A)
  /* 事故事件 TPDO 使用 B 节点 TPDO3 的标准 COB-ID 0x382。 */
  if ((identifier != 0x382U) || (data == NULL))
  {
    /* 其他报文不属于事故事件处理范围。 */
    return;
  }
  /* 事故事件帧固定为 8 字节 Classic CAN 数据。 */
  if (dataLength != 8U)
  {
    /* 长度不正确的帧不能进入事故上报队列。 */
    canopenDiagnostics.accidentEventInvalidCount++;
    return;
  }
  /* 准备事故事件队列元素，避免在接收中断里执行检测和 MQTT 阻塞操作。 */
  CanOpenAccidentRxItem_t item = {0};
  /* 复制八个数据字节到中断安全的转交队列元素。 */
  (void)memcpy(item.data, data, 8U);
  /* 使用 FromISR 接口将事件交给默认任务处理。 */
  BaseType_t higherPriorityTaskWoken = pdFALSE;
  /* 非阻塞入队，队列满时保留丢弃统计。 */
  if ((accidentRxQueue == NULL) ||
      (xQueueSendFromISR(accidentRxQueue, &item, &higherPriorityTaskWoken) != pdPASS))
  {
    /* 记录队列未创建或已满导致的事件丢失。 */
    canopenDiagnostics.accidentEventInvalidCount++;
  }
  /* 请求调度器在退出接收中断后运行默认任务。 */
  portYIELD_FROM_ISR(higherPriorityTaskWoken);
#else
  /* B/C 节点不接收 A 节点的事故事件 TPDO。 */
  (void)identifier;
  (void)dataLength;
  (void)data;
#endif
}

/* B 节点处理 A 返回的事故 ACK 报文。 */
static void CanOpenPort_HandleAccidentAck(uint16_t identifier, uint8_t dataLength,
                                          const uint8_t *data)
{
#if (CAN_NODE_ROLE == CAN_NODE_ROLE_B)
  /* ACK 使用 B 节点 RPDO4 的专用 COB-ID 0x502，载荷为 16 位事件编号。 */
  if ((identifier != (uint16_t)(0x500U + CANOPEN_NODE_ID)) ||
      (dataLength != 2U) || (data == NULL))
  {
    /* 忽略其他报文或格式错误的 ACK。 */
    return;
  }
  /* 将 ACK 编号复制到队列，确认和 Flash 写入统一放到任务上下文执行。 */
  CanOpenAccidentAckRxItem_t item = {0};
  /* 按小端格式解码 ACK 携带的 16 位事件编号。 */
  item.eventId = (uint32_t)data[0] | ((uint32_t)data[1] << 8U);
  /* 使用 FromISR 接口入队，避免中断中访问持久化 Flash。 */
  BaseType_t higherPriorityTaskWoken = pdFALSE;
  if ((accidentAckRxQueue == NULL) ||
      (xQueueSendFromISR(accidentAckRxQueue, &item, &higherPriorityTaskWoken) != pdPASS))
  {
    /* ACK 队列满时记录一次超时，B 节点会按策略继续重发。 */
    canopenDiagnostics.accidentAckTimeoutCount++;
  }
  /* 请求调度器在退出中断后尽快运行 CAN 处理任务。 */
  portYIELD_FROM_ISR(higherPriorityTaskWoken);
#else
  /* A/C 节点不处理 B 节点事故 ACK。 */
  (void)identifier;
  (void)dataLength;
  (void)data;
#endif
}

/* 在任务上下文处理 B 节点收到的 ACK，并安全更新 Flash 持久化状态。 */
static void CanOpenPort_ProcessAccidentAckQueue(void)
{
#if (CAN_NODE_ROLE == CAN_NODE_ROLE_B)
  /* 保存从 ACK 队列取出的事件编号。 */
  CanOpenAccidentAckRxItem_t item = {0};
  /* 一次处理当前队列中的全部 ACK，减少任务切换次数。 */
  while ((accidentAckRxQueue != NULL) &&
         (xQueueReceive(accidentAckRxQueue, &item, 0U) == pdPASS))
  {
    /* 只有编号匹配当前待确认事件时才停止重发。 */
    if ((accidentPending.pendingValid != 0U) &&
        ((uint16_t)accidentPending.pendingEventId == (uint16_t)item.eventId))
    {
      /* 清除待确认标志，后续周期不再重发该事件。 */
      accidentPending.pendingValid = 0U;
      /* 清零重试次数，便于下一次事故从初始策略开始。 */
      accidentPending.pendingRetryCount = 0U;
      /* 累加成功匹配的 ACK 数量。 */
      canopenDiagnostics.accidentAckRxCount++;
      /* 在任务上下文保存已确认状态，掉电后不会重复补发旧事件。 */
      (void)ESP32_Persist_SaveAccident(&accidentPending);
    }
  }
#endif
}

/* 在任务上下文按超时策略重发 B 节点尚未确认的事故事件。 */
static void CanOpenPort_ProcessAccidentRetry(void)
{
#if (CAN_NODE_ROLE == CAN_NODE_ROLE_B)
  /* 没有待确认事件时无需重发。 */
  if (accidentPending.pendingValid == 0U)
  {
    return;
  }
  /* 未到下次重发时刻时直接返回。 */
  if ((int32_t)(HAL_GetTick() - accidentNextRetryTick) < 0)
  {
    return;
  }
  /* 重新写入对象字典并请求 TPDO3 发送同一个事件编号。 */
  if (CanOpenPort_SendAccidentEvent(accidentPending.pendingEventType,
                                    accidentPending.pendingEventId,
                                    accidentPending.pendingPeakAccelMg,
                                    accidentPending.pendingPeakGyroDps) == CO_ERROR_NO)
  {
    /* 增加重试计数并持久化，防止复位丢失重试状态。 */
    if (accidentPending.pendingRetryCount < 0xFFU)
    {
      accidentPending.pendingRetryCount++;
    }
    canopenDiagnostics.accidentRetryCount++;
    /* 超过告警门限后降低重发频率，避免总线长期占用。 */
    accidentNextRetryTick = HAL_GetTick() +
      ((accidentPending.pendingRetryCount >= CANOPEN_ACCIDENT_ACK_RETRY_WARNING_LIMIT) ?
       CANOPEN_ACCIDENT_ACK_DEGRADED_RETRY_MS : CANOPEN_ACCIDENT_ACK_TIMEOUT_MS);
    (void)ESP32_Persist_SaveAccident(&accidentPending);
  }
  else
  {
    /* 记录本次 ACK 等待超时导致的发送失败。 */
    canopenDiagnostics.accidentAckTimeoutCount++;
    accidentNextRetryTick = HAL_GetTick() + CANOPEN_ACCIDENT_ACK_TIMEOUT_MS;
  }
#endif
}

/* 在默认任务上下文完成事故事件校验、去重、状态更新和 MQTT 入队。 */
static void CanOpenPort_ProcessAccidentEventQueue(void)
{
#if (CAN_NODE_ROLE == CAN_NODE_ROLE_A)
  /* 保存从接收队列取出的事故事件元素。 */
  CanOpenAccidentRxItem_t item = {0};
  /* 循环处理本周期积压的全部事件帧。 */
  while ((accidentRxQueue != NULL) &&
         (xQueueReceive(accidentRxQueue, &item, 0U) == pdPASS))
  {
    /* 累计收到的候选事故事件帧数量。 */
    canopenDiagnostics.accidentEventRxCount++;
    /* 只接受碰撞和翻车两种已定义事件。 */
    if ((item.data[0] != ACCIDENT_EVENT_COLLISION) &&
        (item.data[0] != ACCIDENT_EVENT_ROLLOVER))
    {
      /* 未知类型计入格式错误，避免云端收到无法解释的事件。 */
      canopenDiagnostics.accidentEventInvalidCount++;
      continue;
    }
    /* 按小端格式解码 B 节点事件编号。 */
    uint32_t eventId = (uint32_t)item.data[2] | ((uint32_t)item.data[3] << 8U);
    /* 无论是否重复，都回送 ACK，帮助 B 节点停止超时重发。 */
    (void)CanOpenPort_SendAccidentAck(eventId);
    /* 重复事件帧只统计，不重复触发本地状态和 MQTT 消息。 */
    if ((receivedAccidentEventValid != 0U) &&
        (eventId == lastReceivedAccidentEventId))
    {
      /* 记录重复帧数量，便于评估 CAN 重发和云端去重效果。 */
      canopenDiagnostics.accidentEventDuplicateCount++;
      continue;
    }
    /* 保存最近一次有效事件编号，后续重复帧将被过滤。 */
    receivedAccidentEventValid = 1U;
    lastReceivedAccidentEventId = eventId;
    /* 解码事件中的加速度峰值，单位为 mg。 */
    uint32_t peakAccelMg = (uint32_t)item.data[4] | ((uint32_t)item.data[5] << 8U);
    /* 解码事件中的角速度峰值，单位为 dps。 */
    uint32_t peakGyroDps = (uint32_t)item.data[6] | ((uint32_t)item.data[7] << 8U);
    /* 将远端事件复制到 A 节点事故状态机，启动本地保持窗口。 */
    AccidentDetector_ApplyRemoteEvent(item.data[0], eventId, peakAccelMg, peakGyroDps);
    /* 将事件放入 MQTT 可靠队列，实际 UART 操作在 MQTT 任务中执行。 */
    ESP32_QueueAccidentEvent(eventId, item.data[0], (uint16_t)peakAccelMg, (uint16_t)peakGyroDps);
    /* 记录最近事件快照，便于 OLED 和 Keil Watch 观察。 */
    canopenDiagnostics.lastAccidentEventId = eventId;
    /* 保存最近事件类型。 */
    canopenDiagnostics.lastAccidentEventType = item.data[0];
  }
#else
  /* B/C 节点不创建事故接收队列。 */
#endif
}

/* 根据 CAN-ID 记录 A 节点收到的应用 PDO 帧。 */
static void CanOpenPort_RecordPdoFrame(uint16_t identifier, uint8_t dataLength)
{
#if (CAN_NODE_ROLE == CAN_NODE_ROLE_A)
  /* 保存匹配到的 PDO 健康统计对象。 */
  CanOpenPortPdoHealth_t *health = NULL;
  /* 获取当前接收时间，单位为毫秒。 */
  uint32_t currentTick = HAL_GetTick();
  /* 当前四类应用 PDO 均要求 8 字节，长度异常的帧不计入有效数据。 */
  if (dataLength != 8U)
  {
    return;
  }
  /* 按应用 PDO 的 CAN-ID 选择对应统计对象。 */
  switch (identifier & 0x07FFU)
  {
    /* 记录 B 节点加速度 TPDO。 */
    case 0x182U:
      health = &canopenDiagnostics.imuAccel;
      break;
    /* 记录 B 节点角速度 TPDO。 */
    case 0x282U:
      health = &canopenDiagnostics.imuGyro;
      break;
    /* 记录 C 节点位置 TPDO。 */
    case 0x183U:
      health = &canopenDiagnostics.gnssPosition;
      break;
    /* 记录 C 节点状态 TPDO。 */
    case 0x283U:
      health = &canopenDiagnostics.gnssStatus;
      break;
    /* 其他 CANopen 帧不参与应用数据统计。 */
    default:
      break;
  }
  /* 只更新应用 PDO 对应的统计对象。 */
  if (health != NULL)
  {
    /* 标记该数据源已经收到过至少一帧。 */
    health->frameSeen = 1U;
    /* 保存最近一次应用 PDO 的接收时间。 */
    health->lastRxTick = currentTick;
    /* 累加该应用 PDO 的接收帧数。 */
    health->rxFrameCount++;
    /* 收到新帧后立即标记为当前有效。 */
    health->dataValid = 1U;
  }
#else
  /* B/C 节点不执行 A 节点的数据接收统计。 */
  (void)identifier;
  /* B/C 节点不使用应用 PDO 长度统计参数。 */
  (void)dataLength;
#endif
}

/* 更新单个应用 PDO 数据源的超时状态。 */
static void CanOpenPort_UpdatePdoHealthOne(CanOpenPortPdoHealth_t *health, uint32_t timeoutMs)
{
  /* 保存当前 HAL 毫秒时间戳。 */
  uint32_t currentTick = HAL_GetTick();
  /* 忽略空指针，避免诊断任务异常。 */
  if (health == NULL)
  {
    return;
  }
  /* 从未收到数据时保持无效，但不重复累计超时。 */
  if (health->frameSeen == 0U)
  {
    health->dataValid = 0U;
    return;
  }
  /* 最近一帧超过允许时间且此前仍有效时，记录一次超时事件。 */
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
    /* 最近一帧仍在超时窗口内，保持数据有效。 */
    health->dataValid = 1U;
  }
}

/* 启动时进入 Operational，并在通信错误时退回 Pre-operational。 */
#define CANOPEN_NMT_CONTROL (CO_NMT_STARTUP_TO_OPERATIONAL | CO_NMT_ERR_ON_ERR_REG | CO_ERR_REG_GENERIC_ERR | CO_ERR_REG_COMMUNICATION)

/* 初始化 Heartbeat 诊断状态的默认值。 */
static void CanOpenPort_ResetDiagnostics(void)
{
  /* 清空上一次运行遗留的诊断计数。 */
  (void)memset(&canopenDiagnostics, 0, sizeof(canopenDiagnostics));
  /* 记录 B 的固定 CANopen Node-ID。 */
  canopenDiagnostics.heartbeatB.nodeId = 2U;
  /* 记录 C 的固定 CANopen Node-ID。 */
  canopenDiagnostics.heartbeatC.nodeId = 3U;
  /* 将 B 的初始状态设为未配置，等待协议栈第一次处理。 */
  canopenDiagnostics.heartbeatB.state = 0U;
  /* 将 B 的健康状态设为未配置，等待对象字典配置生效。 */
  canopenDiagnostics.heartbeatB.healthState = CANOPEN_HEALTH_UNCONFIGURED;
  /* 将 C 的初始状态设为未配置，等待协议栈第一次处理。 */
  canopenDiagnostics.heartbeatC.state = 0U;
  /* 将 C 的健康状态设为未配置，等待对象字典配置生效。 */
  canopenDiagnostics.heartbeatC.healthState = CANOPEN_HEALTH_UNCONFIGURED;
  /* 记录设备启动阶段的 NMT 状态。 */
  canopenDiagnostics.nmtState = (uint8_t)CO_NMT_INITIALIZING;
  /* 清除事故事件接收去重状态，避免重启后误丢弃编号较小的新事件。 */
  lastReceivedAccidentEventId = 0U;
  /* 标记当前尚未接收过有效事故事件。 */
  receivedAccidentEventValid = 0U;
}

/* 更新一个被监控节点的状态变化和故障恢复时间。 */
static void CanOpenPort_UpdateHeartbeatOne(uint8_t index, CanOpenPortHeartbeatStatus_t *status)
{
  /* 默认将没有配置的监控项标记为未配置。 */
  uint8_t currentState = 0U;
  /* 保存本次状态变化的本地毫秒节拍。 */
  uint32_t currentTick = HAL_GetTick();
  /* 保存健康状态机进入本周期前的状态。 */
  uint8_t oldHealthState = status->healthState;
  /* 仅在 CANopenNode 对象和对应监控项有效时读取协议栈状态。 */
  if ((canopenObject != NULL) && (canopenObject->HBcons != NULL) &&
      (canopenObject->HBconsMonitoredNodes != NULL) &&
      (index < canopenObject->HBcons->numberOfMonitoredNodes))
  {
    /* 复制 CANopenNode 对该远端节点的 Heartbeat 状态。 */
    currentState = (uint8_t)canopenObject->HBconsMonitoredNodes[index].HBstate;
  }
  /* 只在状态发生变化时更新统计，避免每个 1 ms 周期重复计数。 */
  if (currentState != status->state)
  {
    /* 保存状态变化前的旧状态。 */
    status->previousState = status->state;
    /* 保存状态变化后的新状态。 */
    status->state = currentState;
    /* 保存状态变化发生的时间。 */
    status->lastTransitionTick = currentTick;
    /* 进入超时状态时开始记录故障持续时间。 */
    if (currentState == 3U)
    {
      /* 保存本次故障的开始时间。 */
      status->faultStartTick = currentTick;
      /* 清除上一次恢复阶段的起始时间，允许下一次恢复重新计时。 */
      status->recoveryStartTick = 0U;
      /* 标记该节点存在尚未恢复的超时故障。 */
      status->faultActive = 1U;
      /* 累加该远端节点的超时次数。 */
      status->timeoutCount++;
      /* 累加独立超时事件，作为健康状态升级依据。 */
      status->timeoutEpisodeCount++;
    }
    /* 从超时恢复为活跃时计算本次恢复耗时。 */
    else if ((status->faultActive != 0U) && (currentState == 2U))
    {
      /* 计算从超时到重新收到 Heartbeat 的毫秒数。 */
      status->lastRecoveryTimeMs = currentTick - status->faultStartTick;
      /* 累加该远端节点的恢复次数。 */
      status->recoveryCount++;
      /* 清除已经完成的故障起始时间。 */
      status->faultStartTick = 0U;
      /* 清除该节点尚未恢复的超时标志。 */
      status->faultActive = 0U;
    }
    else
    {
      /* 其他状态变化不需要更新恢复统计。 */
    }
  }
  /* 根据当前原始 Heartbeat 状态更新健康状态机。 */
  if (currentState == 0U)
  {
    /* 未配置的监控项不参与故障判断。 */
    status->healthState = CANOPEN_HEALTH_UNCONFIGURED;
  }
  else if (currentState == 1U)
  {
    /* 已配置但尚未收到有效 Heartbeat。 */
    status->healthState = CANOPEN_HEALTH_UNKNOWN;
  }
  else if (currentState == 3U)
  {
    /* 超时次数达到阈值时锁定为 FAULT，否则保持 TIMEOUT。 */
    if ((status->timeoutEpisodeCount >= CANOPEN_HEALTH_TIMEOUT_EPISODE_LIMIT) ||
        ((status->faultActive != 0U) &&
         ((HAL_GetTick() - status->faultStartTick) >= CANOPEN_HEALTH_RECOVERY_TIMEOUT_MS)))
    {
      /* 连续故障次数或恢复等待时间超过阈值，判定为故障。 */
      status->healthState = CANOPEN_HEALTH_FAULT;
    }
    else
    {
      /* 单次超时先进入 TIMEOUT，等待节点恢复。 */
      status->healthState = CANOPEN_HEALTH_TIMEOUT;
    }
  }
  else
  {
    /* 已收到有效 Heartbeat，处理首次激活或超时后的恢复过程。 */
    if ((status->previousState == 3U) &&
        (status->healthState != CANOPEN_HEALTH_RECOVERING) &&
        (status->recoveryStartTick == 0U))
    {
      /* 记录重新收到 Heartbeat 的时间并进入恢复确认阶段。 */
      status->recoveryStartTick = HAL_GetTick();
      status->healthState = CANOPEN_HEALTH_RECOVERING;
    }
    else if ((status->healthState == CANOPEN_HEALTH_RECOVERING) &&
             ((HAL_GetTick() - status->recoveryStartTick) >= CANOPEN_HEALTH_STABLE_TIME_MS))
    {
      /* Heartbeat 稳定保持一段时间后，确认节点恢复正常。 */
      status->healthState = CANOPEN_HEALTH_ACTIVE;
    }
    else if ((status->healthState != CANOPEN_HEALTH_RECOVERING) &&
             (status->healthState != CANOPEN_HEALTH_ACTIVE))
    {
      /* 首次收到有效 Heartbeat 时直接进入正常状态。 */
      status->healthState = CANOPEN_HEALTH_ACTIVE;
    }
  }
  /* 只在健康状态真正变化时保存旧状态，供上位机判断状态边沿。 */
  if (status->healthState != oldHealthState)
  {
    /* 保存本周期健康状态变化前的值。 */
    status->previousHealthState = oldHealthState;
  }
}

/* 更新 EMCY 观察数据和本机 NMT 状态。 */
static void CanOpenPort_UpdateDiagnostics(void)
{
  /* 保存本周期读取到的 Heartbeat Consumer 错误状态。 */
  bool_t heartbeatError = false;
  /* 保存本周期读取到的 CANopen 错误寄存器。 */
  uint8_t errorRegister = 0U;
  /* 保存本周期的本机 NMT 状态。 */
  uint8_t nmtState = (uint8_t)CO_NMT_INITIALIZING;
  /* 保存 EMCY 信息字段对应的第一个超时监控索引。 */
  uint32_t timeoutInfoCode = 0U;
  /* 读取 CANopenNode 当前的错误和 NMT 状态。 */
  if (canopenObject != NULL)
  {
    /* 读取 0x1001 错误寄存器。 */
    errorRegister = CO_getErrorRegister(canopenObject->em);
    /* 读取 Heartbeat Consumer 错误位。 */
    heartbeatError = CO_isError(canopenObject->em, CO_EM_HEARTBEAT_CONSUMER);
    /* 读取本机 NMT 状态，超时后应为 Pre-operational（127）。 */
    nmtState = (uint8_t)CO_NMT_getInternalState(canopenObject->NMT);
  }
  /* 先更新 B/C 的独立状态，供 EMCY 信息索引和恢复统计使用。 */
  CanOpenPort_UpdateHeartbeatOne(0U, &canopenDiagnostics.heartbeatB);
  /* 更新 C 的独立状态，供 EMCY 信息索引和恢复统计使用。 */
  CanOpenPort_UpdateHeartbeatOne(1U, &canopenDiagnostics.heartbeatC);
  /* 更新 B 节点加速度 TPDO 的新鲜度和超时统计。 */
  CanOpenPort_UpdatePdoHealthOne(&canopenDiagnostics.imuAccel, CANOPEN_IMU_PDO_TIMEOUT_MS);
  /* 更新 B 节点角速度 TPDO 的新鲜度和超时统计。 */
  CanOpenPort_UpdatePdoHealthOne(&canopenDiagnostics.imuGyro, CANOPEN_IMU_PDO_TIMEOUT_MS);
  /* 更新 C 节点位置 TPDO 的新鲜度和超时统计。 */
  CanOpenPort_UpdatePdoHealthOne(&canopenDiagnostics.gnssPosition, CANOPEN_GNSS_PDO_TIMEOUT_MS);
  /* 更新 C 节点状态 TPDO 的新鲜度和超时统计。 */
  CanOpenPort_UpdatePdoHealthOne(&canopenDiagnostics.gnssStatus, CANOPEN_GNSS_PDO_TIMEOUT_MS);
  /* 只有 B 的两类 IMU 帧都新鲜时，才认为 IMU 数据汇总有效。 */
  canopenDiagnostics.imuDataValid =
      (canopenDiagnostics.imuAccel.dataValid != 0U) &&
      (canopenDiagnostics.imuGyro.dataValid != 0U);
  /* 只有 C 的两类 GNSS 帧都新鲜时，才认为 GNSS 数据汇总有效。 */
  canopenDiagnostics.gnssDataValid =
      (canopenDiagnostics.gnssPosition.dataValid != 0U) &&
      (canopenDiagnostics.gnssStatus.dataValid != 0U);
  /* 综合 Heartbeat 和应用数据新鲜度判断 B 节点健康状态。 */
  canopenDiagnostics.nodeBHealthy =
      (canopenDiagnostics.heartbeatB.healthState == CANOPEN_HEALTH_ACTIVE) &&
      (canopenDiagnostics.imuDataValid != 0U);
  /* 综合 Heartbeat 和应用数据新鲜度判断 C 节点健康状态。 */
  canopenDiagnostics.nodeCHealthy =
      (canopenDiagnostics.heartbeatC.healthState == CANOPEN_HEALTH_ACTIVE) &&
      (canopenDiagnostics.gnssDataValid != 0U);
  /* 在 Heartbeat 错误上升沿记录一次 EMCY 故障观察事件。 */
  if (heartbeatError && (canopenDiagnostics.heartbeatErrorActive == 0U))
  {
    /* 累加 Heartbeat EMCY 故障次数。 */
    canopenDiagnostics.emcyReportCount++;
    /* 保存 CANopen 标准 Heartbeat 错误码 0x8130。 */
    canopenDiagnostics.lastEmcyErrorCode = 0x8130U;
    /* 保存 CANopenNode 使用的 Heartbeat Consumer 错误位 0x1B。 */
    canopenDiagnostics.lastEmcyErrorBit = CO_EM_HEARTBEAT_CONSUMER;
    /* B 超时时将 EMCY 信息索引记录为 0。 */
    if (canopenDiagnostics.heartbeatB.state == 3U)
    {
      /* 记录 B 对应的监控数组索引。 */
      timeoutInfoCode = 0U;
    }
    /* C 超时时将 EMCY 信息索引记录为 1。 */
    else if (canopenDiagnostics.heartbeatC.state == 3U)
    {
      /* 记录 C 对应的监控数组索引。 */
      timeoutInfoCode = 1U;
    }
    else
    {
      /* 如果状态已变化但监控项不可读，则保留默认索引 0。 */
      timeoutInfoCode = 0U;
    }
    /* 保存 EMCY 数据字节 4 至 7 对应的信息索引。 */
    canopenDiagnostics.lastEmcyInfoCode = timeoutInfoCode;
    /* 保存 EMCY 事件发生的本地毫秒节拍。 */
    canopenDiagnostics.lastEmcyTick = HAL_GetTick();
  }
  /* 在 Heartbeat 错误下降沿记录一次 EMCY 清除观察事件。 */
  else if ((!heartbeatError) && (canopenDiagnostics.heartbeatErrorActive != 0U))
  {
    /* 累加 Heartbeat EMCY 清除次数。 */
    canopenDiagnostics.emcyClearCount++;
    /* 保存 EMCY 清除事件发生的本地毫秒节拍。 */
    canopenDiagnostics.lastEmcyClearTick = HAL_GetTick();
  }
  else
  {
    /* 错误状态没有变化时不更新 EMCY 边沿统计。 */
  }
  /* 保存本周期最新的 Heartbeat 错误状态。 */
  canopenDiagnostics.heartbeatErrorActive = heartbeatError ? 1U : 0U;
  /* 保存本周期最新的错误寄存器。 */
  canopenDiagnostics.errorRegister = errorRegister;
  /* 保存本周期最新的本机 NMT 状态。 */
  canopenDiagnostics.nmtState = nmtState;
}

/* 根据 B/C 健康状态决定是否暂时保持 A 在 Pre-operational。 */
static void CanOpenPort_ApplyHealthNmtPolicy(void)
{
#if (CAN_NODE_ROLE == CAN_NODE_ROLE_A)
  /* 保存 B 是否仍处于恢复确认或故障状态。 */
  bool_t holdForB = ((canopenDiagnostics.heartbeatB.healthState == CANOPEN_HEALTH_RECOVERING) ||
                     (canopenDiagnostics.heartbeatB.healthState == CANOPEN_HEALTH_FAULT));
  /* 保存 C 是否仍处于恢复确认或故障状态。 */
  bool_t holdForC = ((canopenDiagnostics.heartbeatC.healthState == CANOPEN_HEALTH_RECOVERING) ||
                     (canopenDiagnostics.heartbeatC.healthState == CANOPEN_HEALTH_FAULT));
  /* Heartbeat 正常但 IMU 应用帧超时时，暂缓 A 节点恢复到 Operational。 */
  bool_t holdForBData = ((canopenDiagnostics.heartbeatB.healthState == CANOPEN_HEALTH_ACTIVE) &&
                         (canopenDiagnostics.imuDataValid == 0U));
  /* Heartbeat 正常但 GNSS 应用帧超时时，暂缓 A 节点恢复到 Operational。 */
  bool_t holdForCData = ((canopenDiagnostics.heartbeatC.healthState == CANOPEN_HEALTH_ACTIVE) &&
                         (canopenDiagnostics.gnssDataValid == 0U));
  /* 只在 A 的 CANopenNode 对象有效时下发内部 NMT 策略命令。 */
  if ((canopenObject != NULL) && (canopenObject->NMT != NULL) &&
      (holdForB || holdForC || holdForBData || holdForCData))
  {
    /* 在健康状态稳定前保持 A 为 Pre-operational，禁止过早恢复业务 PDO。 */
    CO_NMT_sendInternalCommand(canopenObject->NMT, CO_NMT_ENTER_PRE_OPERATIONAL);
  }
  else if ((canopenObject != NULL) && (canopenObject->NMT != NULL))
  {
    /* 健康状态稳定且错误已清除时，明确请求 A 回到 Operational。 */
    CO_NMT_sendInternalCommand(canopenObject->NMT, CO_NMT_ENTER_OPERATIONAL);
  }
#else
  /* B/C 不承担远端 Heartbeat 监控，不需要执行 A 的 NMT 健康策略。 */
#endif
}

/* 初始化 CANopenNode 的 Classic CAN 适配层。 */
CO_ReturnError_t CanOpenPort_Init(uint8_t nodeId)
{
  /* 保存当前节点号供后续对象字典和 NMT 初始化使用。 */
  canopenNodeId = nodeId;
  /* 暂存节点号，避免当前最小适配阶段产生未使用变量警告。 */
  (void)canopenNodeId;
  /* 使用 CubeMX 已初始化的 FDCAN1 句柄作为 CANopenNode 硬件对象。 */
  return CO_CANmodule_init(&canopenModule, &hfdcan1, canopenRxArray, 32U,
                           canopenTxArray, 40U, 500U);
}

CO_ReturnError_t CanOpenPort_StackInit(uint8_t nodeId)
{
  uint32_t errorInfo = 0U;
  /* 清空本次启动需要使用的 EMCY、NMT 和 Heartbeat 诊断快照。 */
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
  /* 创建 A 节点事故事件转交队列，接收中断只负责快速复制报文。 */
  accidentRxQueue = xQueueCreate(8U, sizeof(CanOpenAccidentRxItem_t));
  /* 队列创建失败时返回内存错误，避免事故事件在中断中静默丢失。 */
  if (accidentRxQueue == NULL)
  {
    /* 记录事故接收队列初始化失败。 */
    return CO_ERROR_OUT_OF_MEMORY;
  }
  /* A 节点不消费事故 ACK，因此不创建 B 节点专用 ACK 队列。 */
  accidentAckRxQueue = NULL;
#elif (CAN_NODE_ROLE == CAN_NODE_ROLE_B)
  /* 创建 B 节点 ACK 队列，避免在 FDCAN 中断中执行 Flash 擦写。 */
  accidentAckRxQueue = xQueueCreate(8U, sizeof(CanOpenAccidentAckRxItem_t));
  /* ACK 队列创建失败时返回内存错误，避免静默丢失确认。 */
  if (accidentAckRxQueue == NULL)
  {
    /* 记录队列初始化失败并终止协议栈初始化。 */
    return CO_ERROR_OUT_OF_MEMORY;
  }
  /* B 节点不接收事故事件转交队列。 */
  accidentRxQueue = NULL;
#else
  /* C 节点不接收事故事件和 ACK，保持两个队列句柄为空。 */
  accidentRxQueue = NULL;
  accidentAckRxQueue = NULL;
#endif
  CO_CANsetNormalMode(canopenObject->CANmodule);
  return CO_ERROR_NO;
}

void CanOpenPort_RxInterrupt(uint16_t identifier, uint8_t dataLength, const uint8_t *data)
{
  CO_CANrxMsg_t message = {0};
  CO_CANrx_t *matched = NULL;
  if ((canopenObject == NULL) || (data == NULL))
  {
    return;
  }
  message.ident = identifier & 0x07FFU;
  message.DLC = (dataLength > 8U) ? 8U : dataLength;
  memcpy(message.data, data, message.DLC);
  for (uint16_t index = 0U; index < canopenObject->CANmodule->rxSize; index++)
  {
    CO_CANrx_t *entry = &canopenObject->CANmodule->rxArray[index];
    if ((entry->CANrx_callback != NULL) && (((message.ident ^ entry->ident) & entry->mask) == 0U))
    {
      matched = entry;
      break;
    }
  }
  if (matched != NULL)
  {
    matched->CANrx_callback(matched->object, &message);
  }
  /* 在 CANopenNode 协议回调后处理 B 节点事故事件 TPDO。 */
  CanOpenPort_HandleAccidentFrame(message.ident, message.DLC, message.data);
  /* 处理 B 节点收到的 A 事故 ACK。 */
  CanOpenPort_HandleAccidentAck(message.ident, message.DLC, message.data);
  /* 在 CANopenNode 完成 RPDO 回调后记录应用帧接收状态。 */
  CanOpenPort_RecordPdoFrame(message.ident, message.DLC);
}

void CanOpenPort_Process(uint32_t timeDifferenceUs)
{
  /* 保存本次是否收到或产生了 SYNC。 */
  bool_t syncWas = false;
  if (canopenObject != NULL)
  {
    /* 每 100 ms 更新一次 TPDO1 的应用层序号。 */
    tpdoElapsedUs += timeDifferenceUs;
    if (tpdoElapsedUs >= 100000U)
    {
      /* 保留超出的时间，降低周期累计误差。 */
      tpdoElapsedUs -= 100000U;
      /* 修改对象字典中的 0x2000:00 发送序号。 */
#if (CAN_NODE_ROLE == CAN_NODE_ROLE_C)
      /* C 节点每 100 ms 请求发送 GNSS TPDO1。 */
      OD_requestTPDO(OD_ENTRY_H2100, 0U);
      /* C 节点每 100 ms 请求发送 GNSS TPDO2。 */
      OD_requestTPDO(OD_ENTRY_H2102, 1U);
#else
      /* A/B 节点保留原有 0x2000 序号测试。 */
      OD_RAM.x2000_tpdoSequence++;
      /* 请求原有 TPDO1 尽快发送。 */
      OD_requestTPDO(OD_ENTRY_H2000, 0U);
#endif
    }
    (void)CO_process(canopenObject, false, timeDifferenceUs, NULL);
    /* 处理 CANopen SYNC 报文和同步计数器。 */
    syncWas = CO_process_SYNC(canopenObject, timeDifferenceUs, NULL);
    /* 处理接收 PDO。 */
    CO_process_RPDO(canopenObject, syncWas, timeDifferenceUs, NULL);
    /* 处理发送 PDO。 */
    CO_process_TPDO(canopenObject, syncWas, timeDifferenceUs, NULL);
    /* 在任务上下文完成事故事件去重、状态更新和 MQTT 入队。 */
    CanOpenPort_ProcessAccidentEventQueue();
    /* 在任务上下文完成 ACK 匹配、重发停止和 Flash 持久化。 */
    CanOpenPort_ProcessAccidentAckQueue();
    /* B 节点对尚未收到 ACK 的事故事件执行超时重发。 */
    CanOpenPort_ProcessAccidentRetry();
    /* 更新 Heartbeat 超时、EMCY 边沿和本机 NMT 状态统计。 */
    CanOpenPort_UpdateDiagnostics();
    /* 在恢复确认期间保持 A 为 Pre-operational，稳定后由 NMT 自动回到 Operational。 */
    CanOpenPort_ApplyHealthNmtPolicy();
  }
}

/* 返回当前 CANopenNode 的底层模块对象。 */
/* 将 BH-182 快照复制到对象字典，供 C 节点 TPDO 映射发送。 */
void CanOpenPort_UpdateGnss(const BH182_Data_t *data)
{
#if (CAN_NODE_ROLE == CAN_NODE_ROLE_C)
  /* 忽略空指针，避免任务异常破坏协议栈。 */
  if (data == NULL)
  {
    return;
  }
  /* 更新纬度对象。 */
  OD_RAM.x2100_latitudeE7 = data->latitudeE7;
  /* 更新经度对象。 */
  OD_RAM.x2101_longitudeE7 = data->longitudeE7;
  /* 更新海拔对象。 */
  OD_RAM.x2102_altitudeMm = data->altitudeMm;
  /* 将速度限制为 16 位后写入对象。 */
  OD_RAM.x2103_speedMmps = (data->speedMmps > 0xFFFFU) ? 0xFFFFU : (uint16_t)data->speedMmps;
  /* 将卫星数量限制为 8 位后写入对象。 */
  OD_RAM.x2104_satelliteCount = (data->satelliteCount > 0xFFU) ? 0xFFU : (uint8_t)data->satelliteCount;
  /* 更新定位有效标志。 */
  OD_RAM.x2105_fixValid = data->fixValid;
#else
  /* A/B 节点不发送 GNSS TPDO。 */
  (void)data;
#endif
}

/* 将 ICM-42688 采样复制到对象字典，供 B 节点 TPDO 映射发送。 */
void CanOpenPort_UpdateImu(const ICM42688_RawData_t *data, uint8_t valid)
{
#if (CAN_NODE_ROLE == CAN_NODE_ROLE_B)
  /* 忽略空指针，避免无效采样破坏对象字典。 */
  if (data == NULL)
  {
    return;
  }
  /* 更新三轴加速度对象。 */
  OD_RAM.x2110_accelX = data->accelX;
  OD_RAM.x2111_accelY = data->accelY;
  OD_RAM.x2112_accelZ = data->accelZ;
  /* 更新三轴角速度对象。 */
  OD_RAM.x2113_gyroX = data->gyroX;
  OD_RAM.x2114_gyroY = data->gyroY;
  OD_RAM.x2115_gyroZ = data->gyroZ;
  /* 更新 IMU 数据有效标志。 */
  OD_RAM.x2116_valid = valid;
  /* 每次成功同步采样后递增 TPDO1 的 8 位序号。 */
  OD_RAM.x2117_tpdo1Sequence++;
  /* 每次成功同步采样后递增 TPDO2 的 16 位序号。 */
  OD_RAM.x2118_tpdo2Sequence++;
#else
  /* A/C 节点不发送 IMU TPDO。 */
  (void)data;
  (void)valid;
#endif
}

/* 由 B 节点更新事故对象并请求 CANopenNode 发送事件 TPDO3，同时报告 EMCY。 */
CO_ReturnError_t CanOpenPort_SendAccidentEvent(uint8_t eventType, uint32_t eventId,
                                               uint32_t peakAccelMg, uint32_t peakGyroDps)
{
#if (CAN_NODE_ROLE == CAN_NODE_ROLE_B)
  /* 保存 EMCY 信息字段，低字节为事件类型，高 16 位为事件编号。 */
  uint32_t infoCode = (uint32_t)eventType | ((eventId & 0xFFFFU) << 8U);
  /* 只允许发送已经定义的碰撞或翻车事件。 */
  if ((eventType != ACCIDENT_EVENT_COLLISION) &&
      (eventType != ACCIDENT_EVENT_ROLLOVER))
  {
    /* 返回非法参数，避免发送不可解析的事件。 */
    return CO_ERROR_ILLEGAL_ARGUMENT;
  }
  /* 检查 CANopen 对象是否已经完成初始化。 */
  if (canopenObject == NULL)
  {
    /* 协议栈未就绪时返回错误并由上层统计。 */
    return CO_ERROR_ILLEGAL_ARGUMENT;
  }
  /* 将加速度峰值饱和到事件帧的 16 位字段。 */
  uint16_t accel16 = (peakAccelMg > 0xFFFFU) ? 0xFFFFU : (uint16_t)peakAccelMg;
  /* 将角速度峰值饱和到事件帧的 16 位字段。 */
  uint16_t gyro16 = (peakGyroDps > 0xFFFFU) ? 0xFFFFU : (uint16_t)peakGyroDps;
  /* 写入事故 TPDO3 的类型对象，对应数据字节 0。 */
  OD_RAM.x2120_accidentType = eventType;
  /* 写入保留标志对象，对应数据字节 1。 */
  OD_RAM.x2121_accidentFlags = 0U;
  /* 写入事件编号低 16 位，对应数据字节 2 至 3。 */
  OD_RAM.x2122_accidentEventId = (uint16_t)eventId;
  /* 写入加速度峰值对象，对应数据字节 4 至 5。 */
  OD_RAM.x2123_peakAccelMg = accel16;
  /* 写入角速度峰值对象，对应数据字节 6 至 7。 */
  OD_RAM.x2124_peakGyroDps = gyro16;
  /* 请求 CANopenNode 在下一次 TPDO 处理时发送 0x1802/0x1A02 定义的事件 TPDO3。 */
  OD_requestTPDO(OD_ENTRY_H2120, 2U);
  /* 保存最近请求发送的事件编号和类型。 */
  canopenDiagnostics.lastAccidentEventId = (uint16_t)eventId;
  /* 保存最近请求发送的事件类型。 */
  canopenDiagnostics.lastAccidentEventType = eventType;
  /* 累加事故事件 TPDO 请求发送次数。 */
  canopenDiagnostics.accidentEventTxCount++;
  /* 新事件首次发送时建立待确认快照；同编号重发不重置重试计数。 */
  if ((accidentPending.pendingValid == 0U) ||
      ((uint16_t)accidentPending.pendingEventId != (uint16_t)eventId))
  {
    /* 保存待确认事件的编号、类型和峰值。 */
    accidentPending.pendingEventId = (uint16_t)eventId;
    /* 保存完整事件编号，供掉电恢复继续生成单调编号。 */
    accidentPending.lastEventId = eventId;
    accidentPending.pendingEventType = eventType;
    accidentPending.pendingPeakAccelMg = peakAccelMg;
    accidentPending.pendingPeakGyroDps = peakGyroDps;
    accidentPending.pendingRetryCount = 0U;
    accidentPending.pendingValid = 1U;
    /* 从首次发送开始等待 A ACK。 */
    accidentNextRetryTick = HAL_GetTick() + CANOPEN_ACCIDENT_ACK_TIMEOUT_MS;
    /* 将待确认事件立即写入 Flash。 */
    (void)ESP32_Persist_SaveAccident(&accidentPending);
  }
  /* 只在新事件上报一次厂商错误位，避免每个采样点重复产生 EMCY。 */
  CO_errorReport(canopenObject->em, (uint8_t)(CO_EM_MANUFACTURER_START +
                  ((eventType == ACCIDENT_EVENT_ROLLOVER) ? 1U : 0U)),
                 CO_EMC_DEVICE_SPECIFIC, infoCode);
  /* TPDO 请求已成功写入对象字典，实际总线发送由协议栈异步完成。 */
  return CO_ERROR_NO;
#else
  /* A/C 节点不提供 B 节点事故事件发送功能。 */
  (void)eventType;
  (void)eventId;
  (void)peakAccelMg;
  (void)peakGyroDps;
  return CO_ERROR_ILLEGAL_ARGUMENT;
#endif
}

/* A 节点发送 16 位事故事件 ACK，B 收到后停止重发。 */
CO_ReturnError_t CanOpenPort_SendAccidentAck(uint32_t eventId)
{
#if (CAN_NODE_ROLE == CAN_NODE_ROLE_A)
  /* 定义标准 Classic CAN ACK 报文头。 */
  FDCAN_TxHeaderTypeDef header = {0};
  /* 定义 ACK 的两个数据字节。 */
  uint8_t data[2] = {0};
  /* 填写 B 节点 RPDO4 使用的 ACK COB-ID 0x502。 */
  header.Identifier = 0x502U;
  header.IdType = FDCAN_STANDARD_ID;
  header.TxFrameType = FDCAN_DATA_FRAME;
  header.DataLength = FDCAN_DLC_BYTES_2;
  header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  header.BitRateSwitch = FDCAN_BRS_OFF;
  header.FDFormat = FDCAN_CLASSIC_CAN;
  header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  header.MessageMarker = 0U;
  /* 写入 ACK 事件编号低字节。 */
  data[0] = (uint8_t)(eventId & 0xFFU);
  /* 写入 ACK 事件编号高字节。 */
  data[1] = (uint8_t)((eventId >> 8U) & 0xFFU);
  /* 将 ACK 直接放入 FDCAN 发送 FIFO，避免占用 CANopen 内部 TX 索引。 */
  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &header, data) == HAL_OK)
  {
    /* 记录 ACK 已成功写入硬件发送队列。 */
    canopenDiagnostics.accidentAckTxCount++;
    /* 返回 CANopenNode 认可的发送成功状态。 */
    return CO_ERROR_NO;
  }
  /* 记录 ACK 未能写入发送队列，B 节点将通过超时重发保证最终送达。 */
  canopenDiagnostics.accidentAckTxErrorCount++;
  /* 返回发送缓冲区溢出状态。 */
  return CO_ERROR_TX_OVERFLOW;
#else
  /* B/C 节点不发送事故 ACK。 */
  (void)eventId;
  return CO_ERROR_ILLEGAL_ARGUMENT;
#endif
}

/* 恢复 B 节点事故编号和待确认事件，供启动后继续重发。 */
void CanOpenPort_RestoreAccidentState(const ESP32_AccidentPersist_t *snapshot)
{
#if (CAN_NODE_ROLE == CAN_NODE_ROLE_B)
  /* 检查持久化快照指针。 */
  if (snapshot == NULL)
  {
    /* 无效快照直接返回。 */
    return;
  }
  /* 复制完整待确认状态。 */
  accidentPending = *snapshot;
  /* 让协议栈启动后尽快重发尚未确认事件。 */
  if (accidentPending.pendingValid != 0U)
  {
    accidentNextRetryTick = HAL_GetTick() + 100U;
  }
#else
  /* A/C 节点不恢复 B 节点待确认事件。 */
  (void)snapshot;
#endif
}

/* 清除 B 节点已经结束的事故 EMCY 状态位。 */
void CanOpenPort_ClearAccidentError(uint8_t eventType)
{
#if (CAN_NODE_ROLE == CAN_NODE_ROLE_B)
  /* 只有协议栈已经初始化时才能访问 EMCY 对象。 */
  if (canopenObject != NULL)
  {
    /* 计算碰撞或翻车对应的厂商错误位。 */
    uint8_t errorBit = (uint8_t)(CO_EM_MANUFACTURER_START +
                         ((eventType == ACCIDENT_EVENT_ROLLOVER) ? 1U : 0U));
    /* 发送 CANopen 错误清除事件。 */
    CO_errorReset(canopenObject->em, errorBit, (uint32_t)eventType);
  }
#else
  /* A/C 节点不维护 B 节点事故 EMCY。 */
  (void)eventType;
#endif
}

CO_CANmodule_t* CanOpenPort_GetModule(void)
{
  /* 返回静态分配的 CANopenNode 模块对象地址。 */
  return &canopenModule;
}

/* 复制当前诊断快照，避免调用者直接修改适配层内部数据。 */
void CanOpenPort_GetDiagnostics(volatile CanOpenPortDiagnostics_t *diagnostics)
{
  /* 忽略空指针请求，避免破坏协议栈任务。 */
  if (diagnostics == NULL)
  {
    return;
  }
  /* 将当前诊断结构一次性复制给调用者。 */
  /* 使用结构体赋值更新 volatile 快照，确保调试器能读取实时数据。 */
  *diagnostics = canopenDiagnostics;
}
