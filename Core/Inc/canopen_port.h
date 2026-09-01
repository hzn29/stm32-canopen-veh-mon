/* 声明 STM32G474RE 到 CANopenNode 的底层初始化接口。 */
#ifndef CANOPEN_PORT_H
#define CANOPEN_PORT_H

/* 引入 CANopenNode 驱动对象类型。 */
#include "301/CO_driver.h"
#include "bh182.h"
#include "icm42688.h"
#include "esp32_persist.h"

/* 定义节点健康状态，供故障恢复策略和实验统计使用。 */
typedef enum
{
  CANOPEN_HEALTH_UNCONFIGURED = 0U, /* 监控项未配置。 */
  CANOPEN_HEALTH_UNKNOWN = 1U, /* 已配置但尚未收到有效 Heartbeat。 */
  CANOPEN_HEALTH_ACTIVE = 2U, /* Heartbeat 在规定时间内正常到达。 */
  CANOPEN_HEALTH_TIMEOUT = 3U, /* 本次 Heartbeat 已超时。 */
  CANOPEN_HEALTH_RECOVERING = 4U, /* 已重新收到 Heartbeat，正在等待稳定确认。 */
  CANOPEN_HEALTH_FAULT = 5U /* 多次超时或恢复超时，进入故障状态。 */
} CanOpenPortHealthState_t;

/* 初始化 CANopenNode 的 Classic CAN 底层模块。 */
CO_ReturnError_t CanOpenPort_Init(uint8_t nodeId);
CO_ReturnError_t CanOpenPort_StackInit(uint8_t nodeId);
void CanOpenPort_RxInterrupt(uint16_t identifier, uint8_t dataLength, const uint8_t *data);
void CanOpenPort_Process(uint32_t timeDifferenceUs);
/* 将 BH-182 最新快照写入对象字典并触发 C 节点 GNSS TPDO。 */
void CanOpenPort_UpdateGnss(const BH182_Data_t *data);
/* 将 ICM-42688 最新采样写入对象字典并触发 B 节点 TPDO。 */
void CanOpenPort_UpdateImu(const ICM42688_RawData_t *data, uint8_t valid);
/* 由 B 节点发送一次已经确认的碰撞或翻车事件 TPDO，并同步产生 EMCY。 */
CO_ReturnError_t CanOpenPort_SendAccidentEvent(uint8_t eventType, uint32_t eventId,
                                               uint32_t peakAccelMg, uint32_t peakGyroDps);
/* 在事故保持窗口结束后清除 B 节点本地事故 EMCY。 */
void CanOpenPort_ClearAccidentError(uint8_t eventType);
/* A 节点向 B 节点回送事故事件 ACK，确认已经接收指定编号。 */
CO_ReturnError_t CanOpenPort_SendAccidentAck(uint32_t eventId);
/* 从 Flash 恢复 B 节点事故编号和待确认事件状态。 */
void CanOpenPort_RestoreAccidentState(const ESP32_AccidentPersist_t *snapshot);
/* 获取当前 CANopenNode 的底层模块对象。 */
CO_CANmodule_t* CanOpenPort_GetModule(void);

/* 保存一个被监控节点的 Heartbeat 状态和故障恢复统计。 */
typedef struct
{
  uint8_t nodeId; /* 保存被监控节点的 CANopen Node-ID。 */
  uint8_t state; /* 保存当前状态：0 未配置、1 未知、2 活跃、3 超时。 */
  uint8_t previousState; /* 保存上一次采样到的 Heartbeat 状态。 */
  uint8_t faultActive; /* 标记该节点是否存在尚未恢复的超时故障。 */
  uint8_t healthState; /* 保存健康状态：0 未配置、1 未知、2 正常、3 超时、4 恢复中、5 故障。 */
  uint8_t previousHealthState; /* 保存上一次健康状态，用于识别状态边沿。 */
  uint32_t lastTransitionTick; /* 保存最近一次状态变化的 HAL 节拍。 */
  uint32_t faultStartTick; /* 保存最近一次超时故障开始的 HAL 节拍。 */
  uint32_t recoveryStartTick; /* 保存最近一次收到恢复 Heartbeat 的 HAL 节拍。 */
  uint32_t lastRecoveryTimeMs; /* 保存最近一次从超时到恢复所用的毫秒数。 */
  uint32_t timeoutCount; /* 保存该节点累计进入超时状态的次数。 */
  uint32_t timeoutEpisodeCount; /* 保存该节点累计的独立超时事件次数。 */
  uint32_t recoveryCount; /* 保存该节点累计恢复为活跃状态的次数。 */
} CanOpenPortHeartbeatStatus_t;

/* 保存 A 节点接收到的应用 PDO 数据新鲜度和故障统计。 */
typedef struct
{
  /* 保存该数据源是否至少收到过一帧。 */
  uint8_t frameSeen;
  /* 保存该数据源最近一次接收的 HAL 毫秒时间戳。 */
  uint32_t lastRxTick;
  /* 保存该数据源接收到的应用帧数量。 */
  uint32_t rxFrameCount;
  /* 保存该数据源累计发生数据超时的次数。 */
  uint32_t timeoutCount;
  /* 保存该数据源当前是否在超时窗口内。 */
  uint8_t dataValid;
} CanOpenPortPdoHealth_t;

/* 保存 A 节点的 EMCY、NMT 和 Heartbeat 诊断快照。 */
typedef struct
{
  uint8_t nmtState; /* 保存本机 NMT 状态值：5 为 Operational、127 为 Pre-operational。 */
  uint8_t errorRegister; /* 保存 CANopen 对象 0x1001 的错误寄存器值。 */
  uint8_t heartbeatErrorActive; /* 标记 Heartbeat Consumer 错误是否处于活动状态。 */
  uint32_t emcyReportCount; /* 保存检测到的 Heartbeat EMCY 故障边沿次数。 */
  uint32_t emcyClearCount; /* 保存检测到的 Heartbeat EMCY 清除边沿次数。 */
  uint16_t lastEmcyErrorCode; /* 保存最近一次 Heartbeat EMCY 错误码，正常为 0x8130。 */
  uint8_t lastEmcyErrorBit; /* 保存最近一次 EMCY 的错误位，Heartbeat Consumer 为 0x1B。 */
  uint32_t lastEmcyInfoCode; /* 保存最近一次 EMCY 的信息索引，0 表示 B，1 表示 C。 */
  uint32_t lastEmcyTick; /* 保存最近一次 Heartbeat EMCY 产生的 HAL 节拍。 */
  uint32_t lastEmcyClearTick; /* 保存最近一次 Heartbeat EMCY 清除的 HAL 节拍。 */
  CanOpenPortHeartbeatStatus_t heartbeatB; /* 保存 Node-ID 2（B）的状态统计。 */
  CanOpenPortHeartbeatStatus_t heartbeatC; /* 保存 Node-ID 3（C）的状态统计。 */
  /* 保存 B 节点 IMU 加速度 TPDO 的数据健康状态。 */
  CanOpenPortPdoHealth_t imuAccel;
  /* 保存 B 节点 IMU 角速度 TPDO 的数据健康状态。 */
  CanOpenPortPdoHealth_t imuGyro;
  /* 保存 C 节点 GNSS 位置 TPDO 的数据健康状态。 */
  CanOpenPortPdoHealth_t gnssPosition;
  /* 保存 C 节点 GNSS 状态 TPDO 的数据健康状态。 */
  CanOpenPortPdoHealth_t gnssStatus;
  /* 保存 B 节点 IMU 数据汇总是否可用。 */
  uint8_t imuDataValid;
  /* 保存 C 节点 GNSS 数据汇总是否可用。 */
  uint8_t gnssDataValid;
  /* 保存 B 节点综合健康状态。 */
  uint8_t nodeBHealthy;
  /* 保存 C 节点综合健康状态。 */
  uint8_t nodeCHealthy;
  /* 保存 A 节点收到的事故事件帧总数。 */
  uint32_t accidentEventRxCount;
  /* 保存 A 节点按事件编号过滤的重复帧数。 */
  uint32_t accidentEventDuplicateCount;
  /* 保存 A 节点丢弃的格式或类型错误事件帧数。 */
  uint32_t accidentEventInvalidCount;
  /* 保存 B 节点成功发送的事故事件帧数。 */
  uint32_t accidentEventTxCount;
  /* 保存 B 节点发送事故事件失败的次数。 */
  uint32_t accidentEventTxErrorCount;
  /* 保存最近接收或发送的事故事件编号。 */
  uint32_t lastAccidentEventId;
  /* 保存最近接收或发送的事故事件类型。 */
  uint8_t lastAccidentEventType;
  uint32_t accidentAckRxCount; /* 保存 B 节点收到 A ACK 的次数。 */
  uint32_t accidentAckTxCount; /* 保存 A 节点成功发送 ACK 的次数。 */
  uint32_t accidentAckTxErrorCount; /* 保存 A 节点发送 ACK 失败的次数。 */
  uint32_t accidentRetryCount; /* 保存 B 节点事故事件重发次数。 */
  uint32_t accidentAckTimeoutCount; /* 保存 B 节点等待 ACK 超时次数。 */
} CanOpenPortDiagnostics_t;

/* 读取当前 CANopen 诊断快照，供调试界面或实验记录使用。 */
void CanOpenPort_GetDiagnostics(volatile CanOpenPortDiagnostics_t *diagnostics);

#endif /* CANOPEN_PORT_H */
