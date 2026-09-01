/* ESP32-12F ESP-AT 串口与 MQTT 任务接口。 */
#ifndef ESP32_AT_H
#define ESP32_AT_H

/* 引入 STM32 HAL 串口类型。 */
#include "stm32g4xx_hal.h"

/* 定义 ESP32 网络状态，供 Keil Watch 和上层状态机观察。 */
typedef enum
{
  ESP32_STATE_OFFLINE = 0U, /* 尚未开始串口通信。 */
  ESP32_STATE_WAIT_CONFIG = 1U, /* 等待填写网络配置。 */
  ESP32_STATE_AT_READY = 2U, /* ESP-AT 基础命令可用。 */
  ESP32_STATE_WIFI_CONNECTING = 3U, /* 正在连接 Wi-Fi。 */
  ESP32_STATE_MQTT_CONNECTING = 4U, /* 正在连接 MQTT 服务端。 */
  ESP32_STATE_MQTT_CONNECTED = 5U, /* MQTT 已建立连接。 */
  ESP32_STATE_ERROR = 6U /* 最近一次操作失败。 */
} ESP32_State_t;

/* 定义允许进入应用层的 MQTT 下行命令类型。 */
typedef enum
{
  ESP32_DOWNLINK_PUBLISH = 1U, /* 请求立即发布一次遥测。 */
  ESP32_DOWNLINK_STATUS = 2U, /* 请求立即发布状态遥测。 */
  ESP32_DOWNLINK_SET_PERIOD = 3U /* 请求修改遥测发布周期。 */
} ESP32_DownlinkCommandType_t;

/* 定义 ESP32 运行统计，所有字段可在调试器中直接观察。 */
typedef struct
{
  uint8_t state; /* 保存当前网络状态枚举值。 */
  uint32_t commandCount; /* 统计已发送的 AT 命令数量。 */
  uint32_t commandErrorCount; /* 统计 AT 命令失败数量。 */
  uint32_t publishCount; /* 统计成功发布的 MQTT 消息数量。 */
  uint32_t publishErrorCount; /* 统计发布失败数量。 */
  uint32_t reconnectCount; /* 统计 MQTT 重连次数。 */
  uint32_t downlinkCount; /* 统计收到的 MQTT 下行命令数量。 */
  uint32_t downlinkAcceptedCount; /* 统计通过白名单校验的命令数量。 */
  uint32_t downlinkRejectedCount; /* 统计被拒绝的命令数量。 */
  uint32_t downlinkExecutedCount; /* 统计执行成功的下行命令数量。 */
  uint32_t downlinkExecutionErrorCount; /* 统计执行失败的下行命令数量。 */
  uint32_t downlinkAckCount; /* 统计成功发送的命令应答数量。 */
  uint32_t downlinkAckErrorCount; /* 统计命令应答发送失败数量。 */
  uint32_t downlinkQueueFullCount; /* 统计因命令队列已满而丢弃的数量。 */
  uint32_t lastDownlinkCmdId; /* 保存最近处理的下行命令编号。 */
  uint32_t lastDownlinkLatencyMs; /* 保存最近命令从接收到执行的延迟。 */
  uint32_t maxDownlinkLatencyMs; /* 保存命令接收到执行的最大延迟。 */
  uint32_t downlinkTestPassCount; /* 统计本地可靠性测试通过次数。 */
  uint32_t downlinkTestFailCount; /* 统计本地可靠性测试失败次数。 */
  uint32_t parameterLoadErrorCount; /* 统计 Flash 参数无效或读取失败次数。 */
  uint32_t parameterSaveCount; /* 统计 Flash 参数保存成功次数。 */
  uint32_t parameterSaveErrorCount; /* 统计 Flash 参数保存失败次数。 */
  uint32_t currentPublishPeriodMs; /* 保存当前遥测发布周期。 */
  uint32_t uartRxOverflowCount; /* 统计串口接收环形缓冲溢出次数。 */
  uint32_t lastCommandTick; /* 保存最近一次 AT 命令时间。 */
  uint32_t lastPublishTick; /* 保存最近一次发布尝试时间。 */
  uint32_t lastErrorCode; /* 保存最近一次软件错误代码。 */
  uint32_t accidentEventQueuedCount; /* 统计成功进入 MQTT 可靠队列的事故事件数。 */
  uint32_t accidentEventSentCount; /* 统计成功发布到 MQTT 的事故事件数。 */
  uint32_t accidentEventRetryCount; /* 统计 MQTT 离线或发送失败后的重试次数。 */
  uint32_t accidentEventDropCount; /* 统计 MQTT 事件队列已满导致的丢弃数。 */
  uint32_t accidentEventLastId; /* 保存最近一次入队或发布的事故事件编号。 */
} ESP32_Status_t;

/* 绑定 USART2 句柄并清零 ESP32 驱动状态。 */
HAL_StatusTypeDef ESP32_Init(UART_HandleTypeDef *huart);
/* 启动 USART2 单字节中断接收。 */
HAL_StatusTypeDef ESP32_StartReceive(void);
/* 将 USART2 接收到的一个字节放入环形缓冲。 */
void ESP32_OnRxByte(uint8_t byte);
/* 处理 HAL 单字节接收完成回调并自动继续接收。 */
void ESP32_OnRxComplete(void);
/* 处理 USART2 错误并重新启动接收。 */
void ESP32_OnUartError(void);
/* 执行 ESP32 网络和 MQTT 状态机的 FreeRTOS 任务。 */
void ESP32_MqttTask(void *argument);
/* 请求 MQTT 任务立即发布一帧当前遥测数据。 */
void ESP32_RequestImmediatePublish(void);
/* 将 A 节点收到的 B 事故事件放入 MQTT 可靠上报队列。 */
void ESP32_QueueAccidentEvent(uint32_t eventId, uint8_t eventType,
                              uint16_t peakAccelMg, uint16_t peakGyroDps);
/* 复制 ESP32 运行状态快照。 */
void ESP32_GetStatus(volatile ESP32_Status_t *status);
/* 注入固定测试报文并检查下行解析和队列计数。 */
HAL_StatusTypeDef ESP32_RunDownlinkReliabilityTest(void);

#endif /* ESP32_AT_H */
