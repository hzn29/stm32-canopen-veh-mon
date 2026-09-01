/* ESP32-12F ESP-AT 串口与 MQTT 实现。 */
#include "esp32_at.h"
/* 引入 ESP-AT 和 OneNET 参数。 */
#include "esp32_mqtt_config.h"
/* 引入运行参数 Flash 持久化接口。 */
#include "esp32_persist.h"
/* 引入 CANopen 节点角色和 FreeRTOS 配置。 */
#include "canopen_node_config.h"
/* 引入 CANopenNode 总头文件以定义 OD_t 等对象字典类型。 */
#include "CANopen.h"
/* 引入 CANopen 对象字典数据。 */
#include "OD.h"
/* 引入 A 节点诊断数据接口。 */
#include "canopen_port.h"
/* 引入车辆统一告警状态机接口。 */
#include "vehicle_alarm.h"
/* 引入车辆碰撞和翻车检测状态接口。 */
#include "accident_detector.h"
/* 引入字符串处理函数。 */
#include <string.h>
/* 引入格式化输出函数。 */
#include <stdio.h>
/* 引入 FreeRTOS 延时接口。 */
#include "FreeRTOS.h"
/* 引入 FreeRTOS 任务接口。 */
#include "task.h"
/* 引入 FreeRTOS 消息队列接口。 */
#include "queue.h"

/* 定义 ESP-AT 响应环形缓冲大小。 */
#define ESP32_RX_RING_SIZE 512U
/* 定义 AT 响应临时缓存大小。 */
#define ESP32_RESPONSE_SIZE 192U
/* 定义 MQTT JSON 遥测缓存大小。 */
#define ESP32_PAYLOAD_SIZE 512U
/* 定义 MQTT 下行异步行缓存大小。 */
#define ESP32_DOWNLINK_LINE_SIZE 256U
/* 定义允许的最小遥测发布周期。 */
#define ESP32_MIN_PUBLISH_PERIOD_MS 100U
/* 定义允许的最大遥测发布周期。 */
#define ESP32_MAX_PUBLISH_PERIOD_MS 60000U
/* 定义事故事件可靠队列容量，允许 MQTT 短时离线后补发。 */
#define ESP32_ACCIDENT_EVENT_QUEUE_SIZE 8U

/* 保存 ESP32 绑定的 USART2 句柄。 */
static UART_HandleTypeDef *esp32Uart;
/* 保存单字节中断接收目标。 */
static uint8_t esp32RxByte;
/* 保存串口接收环形缓冲。 */
static uint8_t esp32RxRing[ESP32_RX_RING_SIZE];
/* 保存环形缓冲写指针。 */
static volatile uint16_t esp32RxHead;
/* 保存环形缓冲读指针。 */
static volatile uint16_t esp32RxTail;
/* 保存驱动状态，供任务和调试器读取。 */
static volatile ESP32_Status_t esp32Status;
/* 定义下行命令队列元素。 */
typedef struct
{
  uint8_t type; /* 保存白名单命令类型。 */
  uint32_t value; /* 保存命令参数，例如发布周期。 */
  uint32_t cmdId; /* 保存云端下行命令编号。 */
  uint32_t receivedTick; /* 保存命令进入队列的本地时刻。 */
} ESP32_DownlinkCommand_t;
/* 定义待发送的 MQTT 应答队列元素。 */
typedef struct
{
  uint32_t cmdId; /* 保存需要应答的命令编号。 */
  uint32_t errorCode; /* 保存执行结果错误码，零表示成功。 */
  char result[16]; /* 保存 accepted、executed 或 rejected。 */
} ESP32_DownlinkAck_t;
/* 定义需要可靠上报的事故事件队列元素。 */
typedef struct
{
  uint32_t eventId; /* 保存 B 节点事件编号。 */
  uint8_t eventType; /* 保存碰撞或翻车类型。 */
  uint16_t peakAccelMg; /* 保存事件加速度峰值。 */
  uint16_t peakGyroDps; /* 保存事件角速度峰值。 */
  uint32_t eventTick; /* 保存 A 节点收到事件的本地时刻。 */
  uint8_t retryCount; /* 保存当前事件已经重试的次数。 */
} ESP32_AccidentEvent_t;
/* 保存下行命令 FreeRTOS 队列句柄。 */
static QueueHandle_t esp32DownlinkQueue;
/* 保存 MQTT 应答 FreeRTOS 队列句柄。 */
static QueueHandle_t esp32AckQueue;
/* 保存事故事件 MQTT 可靠上报队列句柄。 */
static QueueHandle_t esp32AccidentQueue;
/* 保存当前正在组装的 ESP-AT 异步通知行。 */
static char esp32DownlinkLine[ESP32_DOWNLINK_LINE_SIZE];
/* 保存异步通知行的当前长度。 */
static uint16_t esp32DownlinkLineLength;
/* 标记是否收到需要立即发布的下行命令。 */
static volatile uint8_t esp32PublishRequested;
/* 保存最近一次已经写入 Flash 的发布周期，避免重复擦写。 */
static uint32_t esp32PersistedPeriodMs;
/* 保存事故事件发送失败后的下一次重试时刻，避免快速空转。 */
static uint32_t esp32AccidentRetryTick;

/* 解析并执行一条 ESP-AT MQTT 下行通知。 */
static void ESP32_HandleDownlinkLine(const char *line);
/* 从串口环形缓冲中提取并处理所有完整异步通知行。 */
static void ESP32_DrainAsync(void);
/* 将一个串口字节送入异步通知行组装器。 */
static void ESP32_FeedAsyncByte(uint8_t byte);
/* 在 MQTT 任务上下文执行已入队的下行命令。 */
static void ESP32_ProcessDownlinkQueue(void);
/* 发布一条指定主题的文本 MQTT 消息。 */
static HAL_StatusTypeDef ESP32_PublishTopic(const char *topic, const char *payload);
/* 向云端发送一条下行命令处理结果。 */
static void ESP32_QueueDownlinkAck(uint32_t cmdId, const char *result, uint32_t errorCode);
/* 在 MQTT 任务上下文发送应答队列中的消息。 */
static void ESP32_ProcessAckQueue(void);
/* 在 MQTT 任务上下文发送事故事件队列中的首个事件。 */
static void ESP32_ProcessAccidentEventQueue(void);

/* 计算环形缓冲下一个指针位置。 */
static uint16_t ESP32_NextIndex(uint16_t index)
{
  /* 环形缓冲到末尾后回到零。 */
  return (uint16_t)((index + 1U) % ESP32_RX_RING_SIZE);
}

/* 从环形缓冲取出一个字节。 */
static uint8_t ESP32_ReadByte(uint8_t *byte)
{
  /* 检查调用者和缓冲中是否存在数据。 */
  if ((byte == NULL) || (esp32RxTail == esp32RxHead))
  {
    /* 没有可读数据时返回失败。 */
    return 0U;
  }
  /* 复制一个字节给调用者。 */
  *byte = esp32RxRing[esp32RxTail];
  /* 推进环形缓冲读指针。 */
  esp32RxTail = ESP32_NextIndex(esp32RxTail);
  /* 返回读取成功。 */
  return 1U;
}

/* 清空尚未处理的 ESP-AT 响应。 */
static void ESP32_ClearRx(void)
{
  /* 将读指针追到当前写指针。 */
  esp32RxTail = esp32RxHead;
}

/* 解析 ESP-AT 的 MQTT 下行通知并执行受限命令。 */
static void ESP32_HandleDownlinkLine(const char *line)
{
  /* 保存 MQTT 下行有效载荷起始位置。 */
  const char *payload = NULL;
  /* 保存下行通知中的临时数值。 */
  unsigned long requestedPeriod = 0UL;
  /* 保存 JSON value 字段的位置。 */
  const char *valueField = NULL;
  /* 保存 JSON cmd_id 字段的位置。 */
  const char *cmdIdField = NULL;
  /* 保存解析出的命令编号。 */
  unsigned long requestedCmdId = 0UL;
  /* 保存下行命令入队结果。 */
  BaseType_t queueResult = pdFAIL;
  /* 仅处理 ESP-AT MQTT 订阅异步通知。 */
  if ((line == NULL) || (strstr(line, "+MQTTSUBRECV:") == NULL))
  {
    /* 其他 AT 响应交给同步命令等待函数处理。 */
    return;
  }
  /* 定位通知格式中的第三个逗号，即长度字段后的有效载荷。 */
  payload = strchr(line, ',');
  if (payload != NULL)
  {
    /* 跳过连接编号字段。 */
    payload = strchr(payload + 1, ',');
  }
  if (payload != NULL)
  {
    /* 跳过主题字段，定位长度字段结束位置。 */
    payload = strchr(payload + 1, ',');
  }
  if (payload == NULL)
  {
    /* 通知格式不完整时计为拒绝命令。 */
    esp32Status.downlinkRejectedCount++;
    return;
  }
  /* 跳过长度字段末尾的逗号，指向实际命令文本。 */
  payload++;
  /* 跳过 ESP-AT 数据字段的可选引号。 */
  if (*payload == '"')
  {
    payload++;
  }
  /* 定位可选的命令编号字段。 */
  cmdIdField = strstr(payload, "\"cmd_id\"");
  if (cmdIdField != NULL)
  {
    /* 从 JSON 字段中解析十进制命令编号。 */
    (void)sscanf(cmdIdField, "%*[^0-9]%lu", &requestedCmdId);
  }
  /* 统计收到一条 MQTT 下行消息。 */
  esp32Status.downlinkCount++;
  /* 接受立即上报命令。 */
  if ((strstr(payload, "\"cmd\":\"publish\"") != NULL) ||
      (strstr(payload, "\"cmd\": \"publish\"") != NULL))
  {
    /* 保存立即发布命令，交给 MQTT 任务安全执行。 */
    ESP32_DownlinkCommand_t command = {ESP32_DOWNLINK_PUBLISH, 0U,
                                       (uint32_t)requestedCmdId, HAL_GetTick()};
    /* 将命令放入应用队列。 */
    if ((esp32DownlinkQueue != NULL) &&
        (xQueueSendToBack(esp32DownlinkQueue, &command, 0U) == pdPASS))
    {
      /* 统计一条通过白名单的命令。 */
      esp32Status.downlinkAcceptedCount++;
      /* 将已入队结果交给 MQTT 任务发送。 */
      ESP32_QueueDownlinkAck(command.cmdId, "accepted", 0U);
    }
    else
    {
      /* 队列满时拒绝该命令。 */
      esp32Status.downlinkRejectedCount++;
      /* 记录队列满导致的命令丢弃。 */
      esp32Status.downlinkQueueFullCount++;
      /* 将拒绝结果交给 MQTT 任务发送。 */
      ESP32_QueueDownlinkAck((uint32_t)requestedCmdId, "rejected", 7U);
    }
    return;
  }
  /* 接受状态查询命令，当前实现以立即发布遥测作为响应。 */
  if ((strstr(payload, "\"cmd\":\"status\"") != NULL) ||
      (strstr(payload, "\"cmd\": \"status\"") != NULL))
  {
    /* 保存状态查询命令，交给 MQTT 任务安全执行。 */
    ESP32_DownlinkCommand_t command = {ESP32_DOWNLINK_STATUS, 0U,
                                       (uint32_t)requestedCmdId, HAL_GetTick()};
    /* 将命令放入应用队列。 */
    if ((esp32DownlinkQueue != NULL) &&
        (xQueueSendToBack(esp32DownlinkQueue, &command, 0U) == pdPASS))
    {
      /* 统计一条通过白名单的命令。 */
      esp32Status.downlinkAcceptedCount++;
      /* 将已入队结果交给 MQTT 任务发送。 */
      ESP32_QueueDownlinkAck(command.cmdId, "accepted", 0U);
    }
    else
    {
      /* 队列满时拒绝该命令。 */
      esp32Status.downlinkRejectedCount++;
      /* 记录队列满导致的命令丢弃。 */
      esp32Status.downlinkQueueFullCount++;
      /* 将拒绝结果交给 MQTT 任务发送。 */
      ESP32_QueueDownlinkAck((uint32_t)requestedCmdId, "rejected", 7U);
    }
    return;
  }
  /* 尝试解析设置发布周期命令。 */
  /* 定位设置周期命令中的 value 字段。 */
  valueField = strstr(payload, "\"value\"");
  if ((strstr(payload, "set_period") != NULL) &&
      (valueField != NULL) &&
      (sscanf(valueField, "%*[^0-9]%lu", &requestedPeriod) == 1) &&
      (requestedPeriod >= ESP32_MIN_PUBLISH_PERIOD_MS) &&
      (requestedPeriod <= ESP32_MAX_PUBLISH_PERIOD_MS))
  {
    /* 保存新的发布周期命令，交给 MQTT 任务安全执行。 */
    ESP32_DownlinkCommand_t command = {ESP32_DOWNLINK_SET_PERIOD,
                                       (uint32_t)requestedPeriod,
                                       (uint32_t)requestedCmdId, HAL_GetTick()};
    /* 将命令放入应用队列。 */
    if ((esp32DownlinkQueue != NULL) &&
        (xQueueSendToBack(esp32DownlinkQueue, &command, 0U) == pdPASS))
    {
      /* 统计一条通过白名单的命令。 */
      esp32Status.downlinkAcceptedCount++;
      /* 将已入队结果交给 MQTT 任务发送。 */
      ESP32_QueueDownlinkAck(command.cmdId, "accepted", 0U);
    }
    else
    {
      /* 队列满时拒绝该命令。 */
      esp32Status.downlinkRejectedCount++;
      /* 记录队列满导致的命令丢弃。 */
      esp32Status.downlinkQueueFullCount++;
      /* 将拒绝结果交给 MQTT 任务发送。 */
      ESP32_QueueDownlinkAck((uint32_t)requestedCmdId, "rejected", 7U);
    }
    return;
  }
  /* 其他命令不执行，避免云端输入直接变成 AT 命令。 */
  esp32Status.downlinkRejectedCount++;
  /* 保存下行白名单拒绝错误代码。 */
  esp32Status.lastErrorCode = 5U;
  /* 将非法命令拒绝结果交给 MQTT 任务发送。 */
  ESP32_QueueDownlinkAck((uint32_t)requestedCmdId, "rejected", 5U);
}

/* 从环形缓冲提取完整行，识别 MQTT 异步下行消息。 */
static void ESP32_FeedAsyncByte(uint8_t byte)
{
  /* 忽略回车，使用换行作为一行结束标记。 */
  if (byte == '\r')
  {
    /* 回车不参与行缓存。 */
    return;
  }
  /* 收到换行时处理已组装的异步通知。 */
  if (byte == '\n')
  {
    /* 确保行缓存以字符串结束符结尾。 */
    esp32DownlinkLine[esp32DownlinkLineLength] = '\0';
    /* 解析当前异步通知行。 */
    ESP32_HandleDownlinkLine(esp32DownlinkLine);
    /* 清空下一行的组装长度。 */
    esp32DownlinkLineLength = 0U;
    return;
  }
  /* 缓存未满时追加普通响应字符。 */
  if (esp32DownlinkLineLength < (ESP32_DOWNLINK_LINE_SIZE - 1U))
  {
    /* 将字符写入异步通知缓存。 */
    esp32DownlinkLine[esp32DownlinkLineLength++] = (char)byte;
  }
  else
  {
    /* 超长行丢弃并从下一行重新同步。 */
    esp32DownlinkLineLength = 0U;
    /* 记录一条下行格式错误。 */
    esp32Status.downlinkRejectedCount++;
  }
}

/* 从环形缓冲中提取并处理所有完整异步通知行。 */
static void ESP32_DrainAsync(void)
{
  /* 保存本次从环形缓冲取出的字节。 */
  uint8_t byte = 0U;
  /* 持续处理当前已经到达的所有字节。 */
  while (ESP32_ReadByte(&byte) != 0U)
  {
    /* 将字节送入异步通知组装器。 */
    ESP32_FeedAsyncByte(byte);
  }
}

/* 在任务上下文取出下行命令并更新应用状态。 */
static void ESP32_ProcessDownlinkQueue(void)
{
  /* 保存从队列取出的命令。 */
  ESP32_DownlinkCommand_t command = {0};
  /* 队列非空时逐条处理命令。 */
  while ((esp32DownlinkQueue != NULL) &&
         (xQueueReceive(esp32DownlinkQueue, &command, 0U) == pdPASS))
  {
    /* 根据命令类型执行受限动作。 */
    if ((command.type == ESP32_DOWNLINK_PUBLISH) ||
        (command.type == ESP32_DOWNLINK_STATUS))
    {
      /* 请求 MQTT 任务尽快发布一次遥测。 */
      esp32PublishRequested = 1U;
      /* 记录最近命令编号和排队延迟。 */
      esp32Status.lastDownlinkCmdId = command.cmdId;
      esp32Status.lastDownlinkLatencyMs = HAL_GetTick() - command.receivedTick;
      /* 更新最大命令排队延迟。 */
      if (esp32Status.lastDownlinkLatencyMs > esp32Status.maxDownlinkLatencyMs)
      {
        /* 保存新的最大延迟。 */
        esp32Status.maxDownlinkLatencyMs = esp32Status.lastDownlinkLatencyMs;
      }
      /* 统计命令已执行，并发送执行成功应答。 */
      esp32Status.downlinkExecutedCount++;
      ESP32_QueueDownlinkAck(command.cmdId, "executed", 0U);
    }
    else if (command.type == ESP32_DOWNLINK_SET_PERIOD)
    {
      /* 更新已经通过范围校验的运行时发布周期。 */
      esp32Status.currentPublishPeriodMs = command.value;
      /* 仅当周期真正变化时才擦写 Flash，降低参数页磨损。 */
      if (command.value != esp32PersistedPeriodMs)
      {
        /* 将新的发布周期写入保留 Flash 页。 */
        if (ESP32_Persist_SavePeriod(command.value) == HAL_OK)
        {
          /* 记录最近一次已经持久化的周期。 */
          esp32PersistedPeriodMs = command.value;
          /* 统计一次参数保存成功。 */
          esp32Status.parameterSaveCount++;
        }
        else
        {
          /* 统计一次参数保存失败，但保留当前 RAM 配置继续运行。 */
          esp32Status.parameterSaveErrorCount++;
        }
      }
      /* 修改周期后立即发布一条确认状态。 */
      esp32PublishRequested = 1U;
      /* 记录最近命令编号和排队延迟。 */
      esp32Status.lastDownlinkCmdId = command.cmdId;
      esp32Status.lastDownlinkLatencyMs = HAL_GetTick() - command.receivedTick;
      /* 更新最大命令排队延迟。 */
      if (esp32Status.lastDownlinkLatencyMs > esp32Status.maxDownlinkLatencyMs)
      {
        /* 保存新的最大延迟。 */
        esp32Status.maxDownlinkLatencyMs = esp32Status.lastDownlinkLatencyMs;
      }
      /* 统计命令已执行，并发送执行成功应答。 */
      esp32Status.downlinkExecutedCount++;
      ESP32_QueueDownlinkAck(command.cmdId, "executed", 0U);
    }
    else
    {
      /* 理论上不会收到未定义类型，发现时仅记录拒绝。 */
      esp32Status.downlinkRejectedCount++;
      /* 统计一条执行失败的下行命令。 */
      esp32Status.downlinkExecutionErrorCount++;
      /* 发送未知命令类型的失败应答。 */
      ESP32_QueueDownlinkAck(command.cmdId, "error", 8U);
    }
  }
}

/* 等待 ESP-AT 响应中出现指定字符串。 */
static HAL_StatusTypeDef ESP32_WaitFor(const char *token, uint32_t timeoutMs)
{
  /* 定义用于保存最近响应文本的缓存。 */
  char response[ESP32_RESPONSE_SIZE] = {0};
  /* 保存响应缓存当前长度。 */
  uint16_t length = 0U;
  /* 保存等待开始时刻。 */
  uint32_t startTick = HAL_GetTick();
  /* 持续读取环形缓冲直到超时或匹配结果。 */
  while ((HAL_GetTick() - startTick) < timeoutMs)
  {
    /* 保存本次取出的字符。 */
    uint8_t byte = 0U;
    /* 读取一个响应字符。 */
    if (ESP32_ReadByte(&byte) != 0U)
    {
      /* 缓存未满时追加响应字符。 */
      if (length < (ESP32_RESPONSE_SIZE - 1U))
      {
        /* 将接收字节写入字符串缓存。 */
        response[length++] = (char)byte;
        /* 保证缓存始终是 C 字符串。 */
        response[length] = '\0';
      }
      /* 同步等待期间也解析可能插入的 MQTT 异步通知。 */
      ESP32_FeedAsyncByte(byte);
      /* 检查是否收到明确错误响应。 */
      if (strstr(response, "ERROR") != NULL)
      {
        /* 记录 AT 响应错误。 */
        esp32Status.lastErrorCode = 2U;
        /* 返回 HAL 错误。 */
        return HAL_ERROR;
      }
      /* 检查是否收到调用者期望的响应。 */
      if ((token != NULL) && (strstr(response, token) != NULL))
      {
        /* 返回响应匹配成功。 */
        return HAL_OK;
      }
    }
    else
    {
      /* 没有新字节时短暂让出 CPU。 */
      vTaskDelay(pdMS_TO_TICKS(1U));
    }
  }
  /* 记录等待超时错误。 */
  esp32Status.lastErrorCode = 3U;
  /* 返回超时状态。 */
  return HAL_TIMEOUT;
}

/* 发送一条 ESP-AT 命令并等待 OK。 */
static HAL_StatusTypeDef ESP32_Command(const char *command, uint32_t timeoutMs)
{
  /* 检查串口句柄和命令指针。 */
  if ((esp32Uart == NULL) || (command == NULL))
  {
    /* 记录参数错误。 */
    esp32Status.lastErrorCode = 1U;
    /* 返回参数错误。 */
    return HAL_ERROR;
  }
  /* 丢弃上一条命令的残留响应。 */
  ESP32_ClearRx();
  /* 统计 AT 命令发送次数。 */
  esp32Status.commandCount++;
  /* 保存命令发送时间。 */
  esp32Status.lastCommandTick = HAL_GetTick();
  /* 通过 USART2 发送完整命令。 */
  if (HAL_UART_Transmit(esp32Uart, (uint8_t *)command,
                        (uint16_t)strlen(command), 1000U) != HAL_OK)
  {
    /* 统计串口发送失败。 */
    esp32Status.commandErrorCount++;
    /* 返回发送错误。 */
    return HAL_ERROR;
  }
  /* 等待 ESP-AT 返回 OK。 */
  if (ESP32_WaitFor("OK", timeoutMs) != HAL_OK)
  {
    /* 统计响应失败。 */
    esp32Status.commandErrorCount++;
    /* 返回命令失败。 */
    return HAL_ERROR;
  }
  /* 返回命令成功。 */
  return HAL_OK;
}

/* 初始化 ESP32 驱动状态并绑定 USART2。 */
HAL_StatusTypeDef ESP32_Init(UART_HandleTypeDef *huart)
{
  /* 保存 USART2 句柄。 */
  esp32Uart = huart;
  /* 保存从 Flash 读取的发布周期临时值。 */
  uint32_t persistedPeriodMs = ESP32_MQTT_PUBLISH_PERIOD_MS;
  /* 清零环形缓冲写指针。 */
  esp32RxHead = 0U;
  /* 清零环形缓冲读指针。 */
  esp32RxTail = 0U;
  /* 清零统计状态。 */
  memset((void *)&esp32Status, 0, sizeof(esp32Status));
  /* 初始状态设为离线。 */
  esp32Status.state = ESP32_STATE_OFFLINE;
  /* 使用配置头中的默认值初始化发布周期。 */
  esp32Status.currentPublishPeriodMs = ESP32_MQTT_PUBLISH_PERIOD_MS;
  /* 初始化事故事件重试时刻，允许连接后立即发送队首事件。 */
  esp32AccidentRetryTick = HAL_GetTick();
  /* 初始化最近持久化周期为编译期默认值。 */
  esp32PersistedPeriodMs = ESP32_MQTT_PUBLISH_PERIOD_MS;
  /* 尝试恢复上次保存的发布周期。 */
  if (ESP32_Persist_LoadPeriod(ESP32_MQTT_PUBLISH_PERIOD_MS,
                               &persistedPeriodMs) == HAL_OK)
  {
    /* 将有效的 Flash 参数复制到运行时状态。 */
    esp32Status.currentPublishPeriodMs = persistedPeriodMs;
    /* 记录 Flash 中已持久化的周期，后续避免重复擦写。 */
    esp32PersistedPeriodMs = persistedPeriodMs;
  }
  else
  {
    /* 无有效保存记录时保留默认值并统计一次加载失败。 */
    esp32Status.parameterLoadErrorCount++;
  }
  /* 清除待发布命令标志。 */
  esp32PublishRequested = 0U;
  /* 清除异步通知行缓存长度。 */
  esp32DownlinkLineLength = 0U;
  /* 创建八项下行命令队列。 */
  esp32DownlinkQueue = xQueueCreate(8U, sizeof(ESP32_DownlinkCommand_t));
  /* 创建八项 MQTT 应答队列，避免在串口接收上下文发送 AT 命令。 */
  esp32AckQueue = xQueueCreate(8U, sizeof(ESP32_DownlinkAck_t));
  /* 创建事故事件可靠队列，离线期间不清除队列元素。 */
  esp32AccidentQueue = xQueueCreate(ESP32_ACCIDENT_EVENT_QUEUE_SIZE,
                                     sizeof(ESP32_AccidentEvent_t));
  /* 队列创建失败时记录内存错误。 */
  if (esp32DownlinkQueue == NULL)
  {
    /* 保存 FreeRTOS 队列创建失败代码。 */
    esp32Status.lastErrorCode = 6U;
  }
  /* 应答队列创建失败时记录内存错误。 */
  if (esp32AckQueue == NULL)
  {
    /* 保存应答队列创建失败代码。 */
    esp32Status.lastErrorCode = 6U;
  }
  /* 事故事件队列创建失败时记录内存错误。 */
  if (esp32AccidentQueue == NULL)
  {
    /* 保存事故队列创建失败代码。 */
    esp32Status.lastErrorCode = 6U;
  }
  /* 检查 USART2 句柄是否有效。 */
  if (esp32Uart == NULL)
  {
    /* 记录无效句柄错误。 */
    esp32Status.lastErrorCode = 1U;
    /* 返回初始化失败。 */
    return HAL_ERROR;
  }
  /* 返回初始化成功。 */
  return HAL_OK;
}

/* 启动 ESP32 USART2 单字节中断接收。 */
HAL_StatusTypeDef ESP32_StartReceive(void)
{
  /* 检查 ESP32 串口句柄是否已经绑定。 */
  if (esp32Uart == NULL)
  {
    /* 返回初始化错误。 */
    return HAL_ERROR;
  }
  /* 启动一个字节的中断接收。 */
  return HAL_UART_Receive_IT(esp32Uart, &esp32RxByte, 1U);
}

/* 接收中断回调中追加一个字节并重新装载中断。 */
void ESP32_OnRxByte(uint8_t byte)
{
  /* 计算写入字节后的下一个位置。 */
  uint16_t nextHead = ESP32_NextIndex(esp32RxHead);
  /* 检查环形缓冲是否已满。 */
  if (nextHead == esp32RxTail)
  {
    /* 丢弃最旧数据以保留最新 ESP-AT 响应。 */
    esp32RxTail = ESP32_NextIndex(esp32RxTail);
    /* 统计一次环形缓冲溢出。 */
    esp32Status.uartRxOverflowCount++;
  }
  /* 将新字节写入环形缓冲。 */
  esp32RxRing[esp32RxHead] = byte;
  /* 发布新的写指针。 */
  esp32RxHead = nextHead;
  /* 继续接收下一个字节。 */
  if (esp32Uart != NULL)
  {
    /* 重新启动单字节中断接收。 */
    (void)HAL_UART_Receive_IT(esp32Uart, &esp32RxByte, 1U);
  }
}

/* 读取 HAL 保存的单字节并转交环形缓冲。 */
void ESP32_OnRxComplete(void)
{
  /* 将 HAL 接收缓存中的字节交给统一处理函数。 */
  ESP32_OnRxByte(esp32RxByte);
}

/* 处理 USART2 错误并恢复单字节接收。 */
void ESP32_OnUartError(void)
{
  /* 记录 UART 错误。 */
  esp32Status.lastErrorCode = 4U;
  /* 重新装载单字节接收。 */
  if (esp32Uart != NULL)
  {
    /* 重新启动 USART2 中断接收。 */
    (void)HAL_UART_Receive_IT(esp32Uart, &esp32RxByte, 1U);
  }
}

/* 在 A 节点构造 OneNET 遥测 JSON。 */
static int ESP32_BuildPayload(char *payload, uint16_t size)
{
  /* 保存 A 节点的 CANopen 诊断快照。 */
  CanOpenPortDiagnostics_t diagnostics = {0};
  /* 保存车辆统一告警状态快照。 */
  VehicleAlarmStatus_t alarmStatus = {0};
  /* 保存车辆意外检测状态快照。 */
  AccidentDetectorStatus_t accident = {0};
  /* 检查输出缓存指针和容量。 */
  if ((payload == NULL) || (size == 0U))
  {
    /* 返回格式化失败。 */
    return -1;
  }
  /* 从 CANopen 适配层复制一致的诊断快照。 */
  CanOpenPort_GetDiagnostics(&diagnostics);
  /* 从统一状态机复制告警状态。 */
  VehicleAlarm_GetStatus(&alarmStatus);
  /* 复制碰撞和翻车检测状态。 */
  AccidentDetector_GetStatus(&accident);
  /* 输出时间、IMU、GNSS、节点健康状态和统一告警状态。 */
  return snprintf(payload, size,
                  "{\"ts\":%lu,\"imu\":{\"ax\":%d,\"ay\":%d,\"az\":%d,\"gx\":%d,\"gy\":%d,\"gz\":%d,\"valid\":%u},\"gnss\":{\"lat_e7\":%ld,\"lon_e7\":%ld,\"alt_mm\":%ld,\"speed_mmps\":%u,\"sat\":%u,\"fix\":%u},\"health\":{\"b\":%u,\"c\":%u},\"alarm\":{\"state\":%u,\"active\":%lu,\"raw\":%lu},\"accident\":{\"active\":%u,\"type\":%u,\"name\":\"%s\",\"event_id\":%lu,\"collision_count\":%lu,\"rollover_count\":%lu,\"peak_accel_mg\":%lu,\"peak_gyro_dps\":%lu,\"last_event_ms\":%lu}}",
                  (unsigned long)HAL_GetTick(),
                  (int)OD_RAM.x2110_accelX, (int)OD_RAM.x2111_accelY,
                  (int)OD_RAM.x2112_accelZ, (int)OD_RAM.x2113_gyroX,
                  (int)OD_RAM.x2114_gyroY, (int)OD_RAM.x2115_gyroZ,
                  (unsigned int)diagnostics.imuDataValid,
                  (long)OD_RAM.x2100_latitudeE7, (long)OD_RAM.x2101_longitudeE7,
                  (long)OD_RAM.x2102_altitudeMm, (unsigned int)OD_RAM.x2103_speedMmps,
                  (unsigned int)OD_RAM.x2104_satelliteCount,
                  (unsigned int)OD_RAM.x2105_fixValid,
                  (unsigned int)diagnostics.nodeBHealthy,
                  (unsigned int)diagnostics.nodeCHealthy,
                  (unsigned int)alarmStatus.state,
                  (unsigned long)alarmStatus.activeAlarmBits,
                  (unsigned long)alarmStatus.rawAlarmBits,
                  (unsigned int)accident.active,
                  (unsigned int)accident.eventType,
                  AccidentDetector_GetEventText(accident.eventType),
                  (unsigned long)accident.lastEventId,
                  (unsigned long)accident.collisionCount,
                  (unsigned long)accident.rolloverCount,
                  (unsigned long)accident.peakAccelMg,
                  (unsigned long)accident.peakGyroDps,
                  (unsigned long)accident.lastEventTick);
}

/* 建立 ESP-AT Wi-Fi 和 MQTT 连接。 */
static HAL_StatusTypeDef ESP32_Connect(void)
{
  /* 保存格式化后的 AT 命令。 */
  char command[ESP32_RESPONSE_SIZE] = {0};
  /* 配置未完成时暂停连接尝试。 */
#if (ESP32_MQTT_CONFIGURED == 0U)
  /* 将状态设置为等待配置。 */
  esp32Status.state = ESP32_STATE_WAIT_CONFIG;
  /* 返回配置未完成。 */
  return HAL_ERROR;
#else
  /* 测试 ESP-AT 基础响应。 */
  if (ESP32_Command("AT\r\n", 1000U) != HAL_OK)
  {
    /* 标记连接错误。 */
    esp32Status.state = ESP32_STATE_ERROR;
    /* 返回连接失败。 */
    return HAL_ERROR;
  }
  /* 关闭 ESP-AT 回显，避免响应被重复解析。 */
  if (ESP32_Command("ATE0\r\n", 1000U) != HAL_OK)
  {
    /* 标记连接错误。 */
    esp32Status.state = ESP32_STATE_ERROR;
    /* 返回连接失败。 */
    return HAL_ERROR;
  }
  /* 设置 ESP32 为 Station 模式。 */
  if (ESP32_Command("AT+CWMODE=1\r\n", 1000U) != HAL_OK)
  {
    /* 标记连接错误。 */
    esp32Status.state = ESP32_STATE_ERROR;
    /* 返回连接失败。 */
    return HAL_ERROR;
  }
  /* 标记 ESP-AT 基础命令已经准备完成。 */
  esp32Status.state = ESP32_STATE_AT_READY;
  /* 进入 Wi-Fi 连接状态。 */
  esp32Status.state = ESP32_STATE_WIFI_CONNECTING;
  /* 生成 Wi-Fi 连接命令。 */
  (void)snprintf(command, sizeof(command), "AT+CWJAP=\"%s\",\"%s\"\r\n",
                 ESP32_WIFI_SSID, ESP32_WIFI_PASSWORD);
  /* 执行 Wi-Fi 连接，允许较长的 DHCP 和认证时间。 */
  if (ESP32_Command(command, 20000U) != HAL_OK)
  {
    /* 标记连接错误。 */
    esp32Status.state = ESP32_STATE_ERROR;
    /* 返回连接失败。 */
    return HAL_ERROR;
  }
  /* 配置 MQTT 用户信息。 */
  (void)snprintf(command, sizeof(command),
                 "AT+MQTTUSERCFG=0,%u,\"%s\",\"%s\",\"%s\",%u,0,\"\"\r\n",
                 (unsigned int)ESP32_MQTT_SCHEME,
                 ESP32_MQTT_CLIENT_ID, ESP32_MQTT_USERNAME,
                 ESP32_MQTT_PASSWORD, (unsigned int)ESP32_MQTT_KEEPALIVE_SEC);
  /* 执行 MQTT 用户参数配置。 */
  if (ESP32_Command(command, 3000U) != HAL_OK)
  {
    /* 标记连接错误。 */
    esp32Status.state = ESP32_STATE_ERROR;
    /* 返回连接失败。 */
    return HAL_ERROR;
  }
  /* 进入 MQTT 连接状态。 */
  esp32Status.state = ESP32_STATE_MQTT_CONNECTING;
  /* 生成 MQTT 服务端连接命令。 */
  (void)snprintf(command, sizeof(command), "AT+MQTTCONN=0,\"%s\",%u,0\r\n",
                 ESP32_MQTT_HOST, (unsigned int)ESP32_MQTT_PORT);
  /* 执行 MQTT 服务端连接。 */
  if (ESP32_Command(command, 10000U) != HAL_OK)
  {
    /* 标记连接错误。 */
    esp32Status.state = ESP32_STATE_ERROR;
    /* 返回连接失败。 */
    return HAL_ERROR;
  }
  /* 生成 MQTT 下行主题订阅命令。 */
  (void)snprintf(command, sizeof(command), "AT+MQTTSUB=0,\"%s\",1\r\n",
                 ESP32_MQTT_SUB_TOPIC);
  /* 订阅 OneNET 下行控制主题。 */
  if (ESP32_Command(command, 5000U) != HAL_OK)
  {
    /* 标记订阅失败，避免误报为可用的下行链路。 */
    esp32Status.state = ESP32_STATE_ERROR;
    /* 返回连接失败。 */
    return HAL_ERROR;
  }
  /* 标记 MQTT 已连接。 */
  esp32Status.state = ESP32_STATE_MQTT_CONNECTED;
  /* 统计一次成功连接。 */
  esp32Status.reconnectCount++;
  /* 返回连接成功。 */
  return HAL_OK;
#endif
}

/* 发布一条指定主题的文本 MQTT 消息。 */
static HAL_StatusTypeDef ESP32_PublishTopic(const char *topic, const char *payload)
{
  /* 保存 MQTT 发布命令。 */
  char command[ESP32_PAYLOAD_SIZE + ESP32_RESPONSE_SIZE] = {0};
  /* 检查主题和消息指针。 */
  if ((topic == NULL) || (payload == NULL))
  {
    /* 记录发布参数错误。 */
    esp32Status.lastErrorCode = 1U;
    /* 返回发布失败。 */
    return HAL_ERROR;
  }
  /* 生成 ESP-AT MQTT 发布命令。 */
  (void)snprintf(command, sizeof(command), "AT+MQTTPUB=0,\"%s\",\"%s\",1,0\r\n",
                 topic, payload);
  /* 执行 MQTT 发布。 */
  if (ESP32_Command(command, 5000U) != HAL_OK)
  {
    /* 返回发布失败。 */
    return HAL_ERROR;
  }
  /* 返回发布成功。 */
  return HAL_OK;
}

/* 发布一条 JSON 遥测消息到 OneNET。 */
static HAL_StatusTypeDef ESP32_Publish(void)
{
  /* 保存遥测 JSON。 */
  char payload[ESP32_PAYLOAD_SIZE] = {0};
  /* 生成 A 节点数据汇总。 */
  if (ESP32_BuildPayload(payload, sizeof(payload)) < 0)
  {
    /* 统计一次格式化失败。 */
    esp32Status.publishErrorCount++;
    /* 返回发布失败。 */
    return HAL_ERROR;
  }
  /* 发布遥测数据到配置的上报主题。 */
  if (ESP32_PublishTopic(ESP32_MQTT_PUB_TOPIC, payload) != HAL_OK)
  {
    /* 统计一次发布失败。 */
    esp32Status.publishErrorCount++;
    /* 返回发布失败。 */
    return HAL_ERROR;
  }
  /* 统计一次发布成功。 */
  esp32Status.publishCount++;
  /* 保存最近发布时刻。 */
  esp32Status.lastPublishTick = HAL_GetTick();
  /* 返回发布成功。 */
  return HAL_OK;
}

/* 请求 MQTT 任务立即发布一帧当前遥测数据。 */
void ESP32_RequestImmediatePublish(void)
{
  /* 设置发布请求标志，由 MQTT 任务上下文执行实际 AT 命令。 */
  esp32PublishRequested = 1U;
}

/* 将事故事件加入可靠队列，函数本身不执行阻塞式 UART/MQTT 操作。 */
void ESP32_QueueAccidentEvent(uint32_t eventId, uint8_t eventType,
                              uint16_t peakAccelMg, uint16_t peakGyroDps)
{
  /* 创建待入队的事件快照。 */
  ESP32_AccidentEvent_t event = {0};
  /* 检查事件类型和队列句柄，避免无效数据进入云端链路。 */
  if ((eventType != ACCIDENT_EVENT_COLLISION) &&
      (eventType != ACCIDENT_EVENT_ROLLOVER))
  {
    /* 记录无效事件并直接返回。 */
    esp32Status.accidentEventDropCount++;
    return;
  }
  /* 检查队列是否已经由 ESP32_Init 创建。 */
  if (esp32AccidentQueue == NULL)
  {
    /* 初始化尚未完成时不能缓存事件，记录一次丢弃。 */
    esp32Status.accidentEventDropCount++;
    return;
  }
  /* 填充事件编号、类型、峰值和接收时间。 */
  event.eventId = eventId;
  event.eventType = eventType;
  event.peakAccelMg = peakAccelMg;
  event.peakGyroDps = peakGyroDps;
  event.eventTick = HAL_GetTick();
  event.retryCount = 0U;
  /* 非阻塞写入队列，避免 CAN 接收路径被 MQTT 阻塞。 */
  if (xQueueSendToBack(esp32AccidentQueue, &event, 0U) == pdPASS)
  {
    /* 只有实际入队后才增加成功计数。 */
    esp32Status.accidentEventQueuedCount++;
    /* 保存最近入队的事件编号。 */
    esp32Status.accidentEventLastId = eventId;
    /* 请求 MQTT 任务尽快调度事件发布。 */
    esp32PublishRequested = 1U;
  }
  else
  {
    /* 队列满时保留统计，提示需要扩大队列或提高发布能力。 */
    esp32Status.accidentEventDropCount++;
  }
}

/* 将下行处理结果放入应答队列。 */
static void ESP32_QueueDownlinkAck(uint32_t cmdId, const char *result, uint32_t errorCode)
{
  /* 创建一条空的应答队列元素。 */
  ESP32_DownlinkAck_t ack = {0};
  /* 检查应答结果字符串。 */
  if ((result == NULL) || (esp32AckQueue == NULL))
  {
    /* 队列不可用时记录应答错误。 */
    esp32Status.downlinkAckErrorCount++;
    return;
  }
  /* 保存命令编号和错误码。 */
  ack.cmdId = cmdId;
  ack.errorCode = errorCode;
  /* 复制结果字符串并保证结尾。 */
  (void)snprintf(ack.result, sizeof(ack.result), "%s", result);
  /* 非阻塞放入应答队列。 */
  if (xQueueSendToBack(esp32AckQueue, &ack, 0U) != pdPASS)
  {
    /* 统计应答队列满导致的发送失败。 */
    esp32Status.downlinkAckErrorCount++;
  }
}

/* 在 MQTT 任务上下文发送一条应答队列消息。 */
static void ESP32_ProcessAckQueue(void)
{
  /* 保存从应答队列取出的元素。 */
  ESP32_DownlinkAck_t ack = {0};
  /* 循环处理已经排队的全部应答。 */
  while ((esp32AckQueue != NULL) &&
         (xQueueReceive(esp32AckQueue, &ack, 0U) == pdPASS))
  {
    /* 保存应答 JSON 文本。 */
    char payload[128] = {0};
    /* 生成包含命令编号、结果和时间戳的应答。 */
    (void)snprintf(payload, sizeof(payload),
                   "{\"cmd_id\":%lu,\"result\":\"%s\",\"error\":%lu,\"ts\":%lu}",
                   (unsigned long)ack.cmdId, ack.result,
                   (unsigned long)ack.errorCode,
                   (unsigned long)HAL_GetTick());
    /* MQTT 未连接时保留失败统计，等待下一次命令测试。 */
    if ((esp32Status.state == ESP32_STATE_MQTT_CONNECTED) &&
        (ESP32_PublishTopic(ESP32_MQTT_ACK_TOPIC, payload) == HAL_OK))
    {
      /* 统计一条成功发送的应答。 */
      esp32Status.downlinkAckCount++;
    }
    else
    {
      /* 统计一条应答发送失败。 */
      esp32Status.downlinkAckErrorCount++;
    }
  }
}

/* 在 MQTT 任务上下文发送首个事故事件，成功后才从队列删除。 */
static void ESP32_ProcessAccidentEventQueue(void)
{
  /* 保存队列首个事件的只读快照。 */
  ESP32_AccidentEvent_t event = {0};
  /* 保存待发送的事故 JSON。 */
  char payload[192] = {0};
  /* MQTT 未连接时不取出队列元素，保证事件可以在重连后补发。 */
  if ((esp32AccidentQueue == NULL) ||
      (esp32Status.state != ESP32_STATE_MQTT_CONNECTED))
  {
    return;
  }
  /* 失败重试之间至少间隔 500 ms，避免串口错误时占满任务。 */
  if ((int32_t)(HAL_GetTick() - esp32AccidentRetryTick) < 0)
  {
    return;
  }
  /* 只查看队首元素，发送成功后再显式出队。 */
  if (xQueuePeek(esp32AccidentQueue, &event, 0U) != pdPASS)
  {
    return;
  }
  /* 生成包含事件编号和接收时延的专用事故 JSON。 */
  (void)snprintf(payload, sizeof(payload),
                 "{\"event_id\":%lu,\"type\":%u,\"name\":\"%s\",\"peak_accel_mg\":%u,\"peak_gyro_dps\":%u,\"received_ms\":%lu,\"retry\":%u}",
                 (unsigned long)event.eventId, (unsigned int)event.eventType,
                 AccidentDetector_GetEventText(event.eventType),
                 (unsigned int)event.peakAccelMg, (unsigned int)event.peakGyroDps,
                 (unsigned long)event.eventTick, (unsigned int)event.retryCount);
  /* 只有 OneNET/ESP-AT 返回成功，才删除队首事件。 */
  if (ESP32_PublishTopic(ESP32_MQTT_ACCIDENT_TOPIC, payload) == HAL_OK)
  {
    /* 删除刚才成功发送的队首元素。 */
    (void)xQueueReceive(esp32AccidentQueue, &event, 0U);
    /* 累加事故事件成功上报数量。 */
    esp32Status.accidentEventSentCount++;
    /* 保存最近成功上报的事件编号。 */
    esp32Status.accidentEventLastId = event.eventId;
    /* 允许下一条事件立即发送。 */
    esp32AccidentRetryTick = HAL_GetTick();
  }
  else
  {
    /* 取出后增加重试计数，再放回队首，保持事件不丢失。 */
    (void)xQueueReceive(esp32AccidentQueue, &event, 0U);
    if (event.retryCount < 0xFFU)
    {
      /* 防止重试计数溢出。 */
      event.retryCount++;
    }
    /* 重新放回队列；若队列异常满则记录一次不可恢复丢弃。 */
    if (xQueueSendToFront(esp32AccidentQueue, &event, 0U) != pdPASS)
    {
      /* 队列恢复失败时只能统计丢弃，便于现场发现可靠性缺口。 */
      esp32Status.accidentEventDropCount++;
    }
    /* 累加重试次数并延后下一次尝试。 */
    esp32Status.accidentEventRetryCount++;
    /* 将连接状态置为错误，促使任务执行完整的 Wi-Fi/MQTT 重连流程。 */
    esp32Status.state = ESP32_STATE_ERROR;
    esp32AccidentRetryTick = HAL_GetTick() + 500U;
  }
}

/* 执行 ESP32 网络连接和 MQTT 周期发布任务。 */
void ESP32_MqttTask(void *argument)
{
  /* 标记任务参数当前未使用。 */
  (void)argument;
  /* 记录下一次遥测发布时间。 */
  uint32_t nextPublishTick = HAL_GetTick();
  /* 持续执行网络状态机。 */
  for (;;)
  {
#if (CAN_NODE_ROLE == CAN_NODE_ROLE_A)
    /* 优先处理 ESP-AT 异步通知，避免下行消息滞留在环形缓冲。 */
    ESP32_DrainAsync();
    /* 在任务上下文执行已经通过白名单的下行命令。 */
    ESP32_ProcessDownlinkQueue();
    /* 未连接时执行联网流程。 */
    if (esp32Status.state != ESP32_STATE_MQTT_CONNECTED)
    {
      /* 尝试建立 Wi-Fi 和 MQTT 连接。 */
      if (ESP32_Connect() != HAL_OK)
      {
        /* 连接失败后等待下一轮重试。 */
        vTaskDelay(pdMS_TO_TICKS(ESP32_MQTT_RETRY_PERIOD_MS));
      }
      else
      {
        /* 连接成功后立即安排一次数据发布。 */
        nextPublishTick = HAL_GetTick();
      }
    }
    /* MQTT 已连接时按固定周期发布汇总数据。 */
    else if ((esp32PublishRequested != 0U) ||
             ((int32_t)(HAL_GetTick() - nextPublishTick) >= 0))
    {
      /* 在 MQTT 已连接时发送命令接收和执行结果应答。 */
      ESP32_ProcessAckQueue();
      /* 优先发送可靠事故事件，成功后才从队列删除。 */
      ESP32_ProcessAccidentEventQueue();
      /* 清除立即发布请求，避免同一条命令重复触发。 */
      esp32PublishRequested = 0U;
      /* 推进下一次发布时间，避免任务执行时间造成周期漂移。 */
      nextPublishTick = HAL_GetTick() + esp32Status.currentPublishPeriodMs;
      /* 发布当前 CANopen 数据汇总。 */
      if (ESP32_Publish() != HAL_OK)
      {
        /* 发布失败后回到离线状态并触发重连。 */
        esp32Status.state = ESP32_STATE_ERROR;
      }
    }
    else
    {
      /* 在 MQTT 已连接时发送命令接收和执行结果应答。 */
      ESP32_ProcessAckQueue();
      /* 在空闲周期继续尝试发送离线期间保留的事故事件。 */
      ESP32_ProcessAccidentEventQueue();
      /* 未到发布时间时短暂挂起任务。 */
      vTaskDelay(pdMS_TO_TICKS(10U));
    }
#else
    /* B/C 节点不连接云端，仅保留驱动接口。 */
    esp32Status.state = ESP32_STATE_OFFLINE;
    /* B/C 节点低频休眠，避免占用 CPU。 */
    vTaskDelay(pdMS_TO_TICKS(1000U));
#endif
  }
}

/* 复制 ESP32 状态快照。 */
void ESP32_GetStatus(volatile ESP32_Status_t *status)
{
  /* 检查调用者指针。 */
  if (status == NULL)
  {
    /* 忽略无效指针。 */
    return;
  }
  /* 一次性复制状态结构。 */
  *status = esp32Status;
}

/* 注入固定测试报文并检查下行解析和队列计数。 */
HAL_StatusTypeDef ESP32_RunDownlinkReliabilityTest(void)
{
  /* 保存测试开始前的接收、接受和拒绝计数。 */
  uint32_t receivedBefore = esp32Status.downlinkCount;
  uint32_t acceptedBefore = esp32Status.downlinkAcceptedCount;
  uint32_t rejectedBefore = esp32Status.downlinkRejectedCount;
  /* 构造带命令编号的合法测试通知。 */
  const char *publishLine = "+MQTTSUBRECV:0,\"test\",25,{\"cmd_id\":1001,\"cmd\":\"publish\"}";
  /* 构造带命令编号和周期值的合法测试通知。 */
  const char *periodLine = "+MQTTSUBRECV:0,\"test\",37,{\"cmd_id\":1002,\"cmd\":\"set_period\",\"value\":1000}";
  /* 构造白名单之外的非法测试通知。 */
  const char *invalidLine = "+MQTTSUBRECV:0,\"test\",22,{\"cmd_id\":1003,\"cmd\":\"reboot\"}";
  /* 检查测试运行所需的队列。 */
  if (esp32DownlinkQueue == NULL)
  {
    /* 队列不存在时测试失败。 */
    esp32Status.downlinkTestFailCount++;
    return HAL_ERROR;
  }
  /* 依次注入三条测试通知。 */
  ESP32_HandleDownlinkLine(publishLine);
  ESP32_HandleDownlinkLine(periodLine);
  ESP32_HandleDownlinkLine(invalidLine);
  /* 执行入队的合法命令并生成执行应答。 */
  ESP32_ProcessDownlinkQueue();
  /* 判断接收、接受和拒绝计数是否符合预期。 */
  if ((esp32Status.downlinkCount == (receivedBefore + 3U)) &&
      (esp32Status.downlinkAcceptedCount >= (acceptedBefore + 2U)) &&
      (esp32Status.downlinkRejectedCount >= (rejectedBefore + 1U)))
  {
    /* 统计一次通过的本地可靠性测试。 */
    esp32Status.downlinkTestPassCount++;
    return HAL_OK;
  }
  /* 统计一次失败的本地可靠性测试。 */
  esp32Status.downlinkTestFailCount++;
  return HAL_ERROR;
}
