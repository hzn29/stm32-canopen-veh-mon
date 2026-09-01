#include "esp32_at.h"
#include "esp32_mqtt_config.h"
#include "esp32_persist.h"
#include "canopen_node_config.h"
#include "CANopen.h"
#include "OD.h"
#include "canopen_port.h"
#include "vehicle_alarm.h"
#include "accident_detector.h"
#include <string.h>
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#define ESP32_RX_RING_SIZE 512U
#define ESP32_RESPONSE_SIZE 192U
#define ESP32_PAYLOAD_SIZE 512U
#define ESP32_DOWNLINK_LINE_SIZE 256U
#define ESP32_MIN_PUBLISH_PERIOD_MS 100U
#define ESP32_MAX_PUBLISH_PERIOD_MS 60000U
#define ESP32_ACCIDENT_EVENT_QUEUE_SIZE 8U

static UART_HandleTypeDef *esp32Uart;
static uint8_t esp32RxByte;
static uint8_t esp32RxRing[ESP32_RX_RING_SIZE];
static volatile uint16_t esp32RxHead;
static volatile uint16_t esp32RxTail;
static volatile ESP32_Status_t esp32Status;
typedef struct
{
  uint8_t type; /* 淇濆瓨鐧藉悕鍗曞懡浠ょ被鍨嬨€?*/
  uint32_t value; /* 淇濆瓨鍛戒护鍙傛暟锛屼緥濡傚彂甯冨懆鏈熴€?*/
  uint32_t cmdId; /* 淇濆瓨浜戠涓嬭鍛戒护缂栧彿銆?*/
  uint32_t receivedTick; /* 淇濆瓨鍛戒护杩涘叆闃熷垪鐨勬湰鍦版椂鍒汇€?*/
} ESP32_DownlinkCommand_t;
typedef struct
{
  uint32_t cmdId; /* 淇濆瓨闇€瑕佸簲绛旂殑鍛戒护缂栧彿銆?*/
  uint32_t errorCode; /* 淇濆瓨鎵ц缁撴灉閿欒鐮侊紝闆惰〃绀烘垚鍔熴€?*/
  char result[16]; /* 淇濆瓨 accepted銆乪xecuted 鎴?rejected銆?*/
} ESP32_DownlinkAck_t;
typedef struct
{
  uint32_t eventId; /* 淇濆瓨 B 鑺傜偣浜嬩欢缂栧彿銆?*/
  uint8_t eventType; /* 淇濆瓨纰版挒鎴栫炕杞︾被鍨嬨€?*/
  uint16_t peakAccelMg; /* 淇濆瓨浜嬩欢鍔犻€熷害宄板€笺€?*/
  uint16_t peakGyroDps; /* 淇濆瓨浜嬩欢瑙掗€熷害宄板€笺€?*/
  uint32_t eventTick; /* 淇濆瓨 A 鑺傜偣鏀跺埌浜嬩欢鐨勬湰鍦版椂鍒汇€?*/
  uint8_t retryCount; /* 淇濆瓨褰撳墠浜嬩欢宸茬粡閲嶈瘯鐨勬鏁般€?*/
} ESP32_AccidentEvent_t;
static QueueHandle_t esp32DownlinkQueue;
static QueueHandle_t esp32AckQueue;
static QueueHandle_t esp32AccidentQueue;
static char esp32DownlinkLine[ESP32_DOWNLINK_LINE_SIZE];
static uint16_t esp32DownlinkLineLength;
static volatile uint8_t esp32PublishRequested;
static uint32_t esp32PersistedPeriodMs;
static uint32_t esp32AccidentRetryTick;

static void ESP32_HandleDownlinkLine(const char *line);
static void ESP32_DrainAsync(void);
static void ESP32_FeedAsyncByte(uint8_t byte);
static void ESP32_ProcessDownlinkQueue(void);
static HAL_StatusTypeDef ESP32_PublishTopic(const char *topic, const char *payload);
static void ESP32_QueueDownlinkAck(uint32_t cmdId, const char *result, uint32_t errorCode);
static void ESP32_ProcessAckQueue(void);
static void ESP32_ProcessAccidentEventQueue(void);

static uint16_t ESP32_NextIndex(uint16_t index)
{
  return (uint16_t)((index + 1U) % ESP32_RX_RING_SIZE);
}

static uint8_t ESP32_ReadByte(uint8_t *byte)
{
  if ((byte == NULL) || (esp32RxTail == esp32RxHead))
  {
    return 0U;
  }
  *byte = esp32RxRing[esp32RxTail];
  esp32RxTail = ESP32_NextIndex(esp32RxTail);
  return 1U;
}

static void ESP32_ClearRx(void)
{
  esp32RxTail = esp32RxHead;
}

static void ESP32_HandleDownlinkLine(const char *line)
{
  const char *payload = NULL;
  unsigned long requestedPeriod = 0UL;
  const char *valueField = NULL;
  const char *cmdIdField = NULL;
  unsigned long requestedCmdId = 0UL;
  BaseType_t queueResult = pdFAIL;
  if ((line == NULL) || (strstr(line, "+MQTTSUBRECV:") == NULL))
  {
    return;
  }
  payload = strchr(line, ',');
  if (payload != NULL)
  {
    payload = strchr(payload + 1, ',');
  }
  if (payload != NULL)
  {
    payload = strchr(payload + 1, ',');
  }
  if (payload == NULL)
  {
    esp32Status.downlinkRejectedCount++;
    return;
  }
  payload++;
  if (*payload == '"')
  {
    payload++;
  }
  cmdIdField = strstr(payload, "\"cmd_id\"");
  if (cmdIdField != NULL)
  {
    (void)sscanf(cmdIdField, "%*[^0-9]%lu", &requestedCmdId);
  }
  esp32Status.downlinkCount++;
  if ((strstr(payload, "\"cmd\":\"publish\"") != NULL) ||
      (strstr(payload, "\"cmd\": \"publish\"") != NULL))
  {
    ESP32_DownlinkCommand_t command = {ESP32_DOWNLINK_PUBLISH, 0U,
                                       (uint32_t)requestedCmdId, HAL_GetTick()};
    if ((esp32DownlinkQueue != NULL) &&
        (xQueueSendToBack(esp32DownlinkQueue, &command, 0U) == pdPASS))
    {
      esp32Status.downlinkAcceptedCount++;
      ESP32_QueueDownlinkAck(command.cmdId, "accepted", 0U);
    }
    else
    {
      esp32Status.downlinkRejectedCount++;
      esp32Status.downlinkQueueFullCount++;
      ESP32_QueueDownlinkAck((uint32_t)requestedCmdId, "rejected", 7U);
    }
    return;
  }
  if ((strstr(payload, "\"cmd\":\"status\"") != NULL) ||
      (strstr(payload, "\"cmd\": \"status\"") != NULL))
  {
    ESP32_DownlinkCommand_t command = {ESP32_DOWNLINK_STATUS, 0U,
                                       (uint32_t)requestedCmdId, HAL_GetTick()};
    if ((esp32DownlinkQueue != NULL) &&
        (xQueueSendToBack(esp32DownlinkQueue, &command, 0U) == pdPASS))
    {
      esp32Status.downlinkAcceptedCount++;
      ESP32_QueueDownlinkAck(command.cmdId, "accepted", 0U);
    }
    else
    {
      esp32Status.downlinkRejectedCount++;
      esp32Status.downlinkQueueFullCount++;
      ESP32_QueueDownlinkAck((uint32_t)requestedCmdId, "rejected", 7U);
    }
    return;
  }
  valueField = strstr(payload, "\"value\"");
  if ((strstr(payload, "set_period") != NULL) &&
      (valueField != NULL) &&
      (sscanf(valueField, "%*[^0-9]%lu", &requestedPeriod) == 1) &&
      (requestedPeriod >= ESP32_MIN_PUBLISH_PERIOD_MS) &&
      (requestedPeriod <= ESP32_MAX_PUBLISH_PERIOD_MS))
  {
    ESP32_DownlinkCommand_t command = {ESP32_DOWNLINK_SET_PERIOD,
                                       (uint32_t)requestedPeriod,
                                       (uint32_t)requestedCmdId, HAL_GetTick()};
    if ((esp32DownlinkQueue != NULL) &&
        (xQueueSendToBack(esp32DownlinkQueue, &command, 0U) == pdPASS))
    {
      esp32Status.downlinkAcceptedCount++;
      ESP32_QueueDownlinkAck(command.cmdId, "accepted", 0U);
    }
    else
    {
      esp32Status.downlinkRejectedCount++;
      esp32Status.downlinkQueueFullCount++;
      ESP32_QueueDownlinkAck((uint32_t)requestedCmdId, "rejected", 7U);
    }
    return;
  }
  esp32Status.downlinkRejectedCount++;
  esp32Status.lastErrorCode = 5U;
  ESP32_QueueDownlinkAck((uint32_t)requestedCmdId, "rejected", 5U);
}

static void ESP32_FeedAsyncByte(uint8_t byte)
{
  if (byte == '\r')
  {
    return;
  }
  if (byte == '\n')
  {
    esp32DownlinkLine[esp32DownlinkLineLength] = '\0';
    ESP32_HandleDownlinkLine(esp32DownlinkLine);
    esp32DownlinkLineLength = 0U;
    return;
  }
  if (esp32DownlinkLineLength < (ESP32_DOWNLINK_LINE_SIZE - 1U))
  {
    esp32DownlinkLine[esp32DownlinkLineLength++] = (char)byte;
  }
  else
  {
    esp32DownlinkLineLength = 0U;
    esp32Status.downlinkRejectedCount++;
  }
}

static void ESP32_DrainAsync(void)
{
  uint8_t byte = 0U;
  while (ESP32_ReadByte(&byte) != 0U)
  {
    ESP32_FeedAsyncByte(byte);
  }
}

static void ESP32_ProcessDownlinkQueue(void)
{
  ESP32_DownlinkCommand_t command = {0};
  while ((esp32DownlinkQueue != NULL) &&
         (xQueueReceive(esp32DownlinkQueue, &command, 0U) == pdPASS))
  {
    if ((command.type == ESP32_DOWNLINK_PUBLISH) ||
        (command.type == ESP32_DOWNLINK_STATUS))
    {
      esp32PublishRequested = 1U;
      esp32Status.lastDownlinkCmdId = command.cmdId;
      esp32Status.lastDownlinkLatencyMs = HAL_GetTick() - command.receivedTick;
      if (esp32Status.lastDownlinkLatencyMs > esp32Status.maxDownlinkLatencyMs)
      {
        esp32Status.maxDownlinkLatencyMs = esp32Status.lastDownlinkLatencyMs;
      }
      esp32Status.downlinkExecutedCount++;
      ESP32_QueueDownlinkAck(command.cmdId, "executed", 0U);
    }
    else if (command.type == ESP32_DOWNLINK_SET_PERIOD)
    {
      esp32Status.currentPublishPeriodMs = command.value;
      if (command.value != esp32PersistedPeriodMs)
      {
        if (ESP32_Persist_SavePeriod(command.value) == HAL_OK)
        {
          esp32PersistedPeriodMs = command.value;
          esp32Status.parameterSaveCount++;
        }
        else
        {
          esp32Status.parameterSaveErrorCount++;
        }
      }
      esp32PublishRequested = 1U;
      esp32Status.lastDownlinkCmdId = command.cmdId;
      esp32Status.lastDownlinkLatencyMs = HAL_GetTick() - command.receivedTick;
      if (esp32Status.lastDownlinkLatencyMs > esp32Status.maxDownlinkLatencyMs)
      {
        esp32Status.maxDownlinkLatencyMs = esp32Status.lastDownlinkLatencyMs;
      }
      esp32Status.downlinkExecutedCount++;
      ESP32_QueueDownlinkAck(command.cmdId, "executed", 0U);
    }
    else
    {
      esp32Status.downlinkRejectedCount++;
      esp32Status.downlinkExecutionErrorCount++;
      ESP32_QueueDownlinkAck(command.cmdId, "error", 8U);
    }
  }
}

static HAL_StatusTypeDef ESP32_WaitFor(const char *token, uint32_t timeoutMs)
{
  char response[ESP32_RESPONSE_SIZE] = {0};
  uint16_t length = 0U;
  uint32_t startTick = HAL_GetTick();
  while ((HAL_GetTick() - startTick) < timeoutMs)
  {
    uint8_t byte = 0U;
    if (ESP32_ReadByte(&byte) != 0U)
    {
      if (length < (ESP32_RESPONSE_SIZE - 1U))
      {
        response[length++] = (char)byte;
        response[length] = '\0';
      }
      ESP32_FeedAsyncByte(byte);
      if (strstr(response, "ERROR") != NULL)
      {
        esp32Status.lastErrorCode = 2U;
        return HAL_ERROR;
      }
      if ((token != NULL) && (strstr(response, token) != NULL))
      {
        return HAL_OK;
      }
    }
    else
    {
      vTaskDelay(pdMS_TO_TICKS(1U));
    }
  }
  esp32Status.lastErrorCode = 3U;
  return HAL_TIMEOUT;
}

static HAL_StatusTypeDef ESP32_Command(const char *command, uint32_t timeoutMs)
{
  if ((esp32Uart == NULL) || (command == NULL))
  {
    esp32Status.lastErrorCode = 1U;
    return HAL_ERROR;
  }
  ESP32_ClearRx();
  esp32Status.commandCount++;
  esp32Status.lastCommandTick = HAL_GetTick();
  if (HAL_UART_Transmit(esp32Uart, (uint8_t *)command,
                        (uint16_t)strlen(command), 1000U) != HAL_OK)
  {
    esp32Status.commandErrorCount++;
    return HAL_ERROR;
  }
  if (ESP32_WaitFor("OK", timeoutMs) != HAL_OK)
  {
    esp32Status.commandErrorCount++;
    return HAL_ERROR;
  }
  return HAL_OK;
}

HAL_StatusTypeDef ESP32_Init(UART_HandleTypeDef *huart)
{
  esp32Uart = huart;
  uint32_t persistedPeriodMs = ESP32_MQTT_PUBLISH_PERIOD_MS;
  esp32RxHead = 0U;
  esp32RxTail = 0U;
  memset((void *)&esp32Status, 0, sizeof(esp32Status));
  esp32Status.state = ESP32_STATE_OFFLINE;
  esp32Status.currentPublishPeriodMs = ESP32_MQTT_PUBLISH_PERIOD_MS;
  esp32AccidentRetryTick = HAL_GetTick();
  esp32PersistedPeriodMs = ESP32_MQTT_PUBLISH_PERIOD_MS;
  if (ESP32_Persist_LoadPeriod(ESP32_MQTT_PUBLISH_PERIOD_MS,
                               &persistedPeriodMs) == HAL_OK)
  {
    esp32Status.currentPublishPeriodMs = persistedPeriodMs;
    esp32PersistedPeriodMs = persistedPeriodMs;
  }
  else
  {
    esp32Status.parameterLoadErrorCount++;
  }
  esp32PublishRequested = 0U;
  esp32DownlinkLineLength = 0U;
  esp32DownlinkQueue = xQueueCreate(8U, sizeof(ESP32_DownlinkCommand_t));
  esp32AckQueue = xQueueCreate(8U, sizeof(ESP32_DownlinkAck_t));
  esp32AccidentQueue = xQueueCreate(ESP32_ACCIDENT_EVENT_QUEUE_SIZE,
                                     sizeof(ESP32_AccidentEvent_t));
  if (esp32DownlinkQueue == NULL)
  {
    esp32Status.lastErrorCode = 6U;
  }
  if (esp32AckQueue == NULL)
  {
    esp32Status.lastErrorCode = 6U;
  }
  if (esp32AccidentQueue == NULL)
  {
    esp32Status.lastErrorCode = 6U;
  }
  if (esp32Uart == NULL)
  {
    esp32Status.lastErrorCode = 1U;
    return HAL_ERROR;
  }
  return HAL_OK;
}

HAL_StatusTypeDef ESP32_StartReceive(void)
{
  if (esp32Uart == NULL)
  {
    return HAL_ERROR;
  }
  return HAL_UART_Receive_IT(esp32Uart, &esp32RxByte, 1U);
}

void ESP32_OnRxByte(uint8_t byte)
{
  uint16_t nextHead = ESP32_NextIndex(esp32RxHead);
  if (nextHead == esp32RxTail)
  {
    esp32RxTail = ESP32_NextIndex(esp32RxTail);
    esp32Status.uartRxOverflowCount++;
  }
  esp32RxRing[esp32RxHead] = byte;
  esp32RxHead = nextHead;
  if (esp32Uart != NULL)
  {
    (void)HAL_UART_Receive_IT(esp32Uart, &esp32RxByte, 1U);
  }
}

void ESP32_OnRxComplete(void)
{
  ESP32_OnRxByte(esp32RxByte);
}

void ESP32_OnUartError(void)
{
  esp32Status.lastErrorCode = 4U;
  if (esp32Uart != NULL)
  {
    (void)HAL_UART_Receive_IT(esp32Uart, &esp32RxByte, 1U);
  }
}

static int ESP32_BuildPayload(char *payload, uint16_t size)
{
  CanOpenPortDiagnostics_t diagnostics = {0};
  VehicleAlarmStatus_t alarmStatus = {0};
  AccidentDetectorStatus_t accident = {0};
  if ((payload == NULL) || (size == 0U))
  {
    return -1;
  }
  CanOpenPort_GetDiagnostics(&diagnostics);
  VehicleAlarm_GetStatus(&alarmStatus);
  AccidentDetector_GetStatus(&accident);
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

static HAL_StatusTypeDef ESP32_Connect(void)
{
  char command[ESP32_RESPONSE_SIZE] = {0};
#if (ESP32_MQTT_CONFIGURED == 0U)
  esp32Status.state = ESP32_STATE_WAIT_CONFIG;
  return HAL_ERROR;
#else
  if (ESP32_Command("AT\r\n", 1000U) != HAL_OK)
  {
    esp32Status.state = ESP32_STATE_ERROR;
    return HAL_ERROR;
  }
  if (ESP32_Command("ATE0\r\n", 1000U) != HAL_OK)
  {
    esp32Status.state = ESP32_STATE_ERROR;
    return HAL_ERROR;
  }
  if (ESP32_Command("AT+CWMODE=1\r\n", 1000U) != HAL_OK)
  {
    esp32Status.state = ESP32_STATE_ERROR;
    return HAL_ERROR;
  }
  esp32Status.state = ESP32_STATE_AT_READY;
  esp32Status.state = ESP32_STATE_WIFI_CONNECTING;
  (void)snprintf(command, sizeof(command), "AT+CWJAP=\"%s\",\"%s\"\r\n",
                 ESP32_WIFI_SSID, ESP32_WIFI_PASSWORD);
  if (ESP32_Command(command, 20000U) != HAL_OK)
  {
    esp32Status.state = ESP32_STATE_ERROR;
    return HAL_ERROR;
  }
  (void)snprintf(command, sizeof(command),
                 "AT+MQTTUSERCFG=0,%u,\"%s\",\"%s\",\"%s\",%u,0,\"\"\r\n",
                 (unsigned int)ESP32_MQTT_SCHEME,
                 ESP32_MQTT_CLIENT_ID, ESP32_MQTT_USERNAME,
                 ESP32_MQTT_PASSWORD, (unsigned int)ESP32_MQTT_KEEPALIVE_SEC);
  if (ESP32_Command(command, 3000U) != HAL_OK)
  {
    esp32Status.state = ESP32_STATE_ERROR;
    return HAL_ERROR;
  }
  esp32Status.state = ESP32_STATE_MQTT_CONNECTING;
  (void)snprintf(command, sizeof(command), "AT+MQTTCONN=0,\"%s\",%u,0\r\n",
                 ESP32_MQTT_HOST, (unsigned int)ESP32_MQTT_PORT);
  if (ESP32_Command(command, 10000U) != HAL_OK)
  {
    esp32Status.state = ESP32_STATE_ERROR;
    return HAL_ERROR;
  }
  (void)snprintf(command, sizeof(command), "AT+MQTTSUB=0,\"%s\",1\r\n",
                 ESP32_MQTT_SUB_TOPIC);
  if (ESP32_Command(command, 5000U) != HAL_OK)
  {
    esp32Status.state = ESP32_STATE_ERROR;
    return HAL_ERROR;
  }
  esp32Status.state = ESP32_STATE_MQTT_CONNECTED;
  esp32Status.reconnectCount++;
  return HAL_OK;
#endif
}

static HAL_StatusTypeDef ESP32_PublishTopic(const char *topic, const char *payload)
{
  char command[ESP32_PAYLOAD_SIZE + ESP32_RESPONSE_SIZE] = {0};
  if ((topic == NULL) || (payload == NULL))
  {
    esp32Status.lastErrorCode = 1U;
    return HAL_ERROR;
  }
  (void)snprintf(command, sizeof(command), "AT+MQTTPUB=0,\"%s\",\"%s\",1,0\r\n",
                 topic, payload);
  if (ESP32_Command(command, 5000U) != HAL_OK)
  {
    return HAL_ERROR;
  }
  return HAL_OK;
}

static HAL_StatusTypeDef ESP32_Publish(void)
{
  char payload[ESP32_PAYLOAD_SIZE] = {0};
  if (ESP32_BuildPayload(payload, sizeof(payload)) < 0)
  {
    esp32Status.publishErrorCount++;
    return HAL_ERROR;
  }
  if (ESP32_PublishTopic(ESP32_MQTT_PUB_TOPIC, payload) != HAL_OK)
  {
    esp32Status.publishErrorCount++;
    return HAL_ERROR;
  }
  esp32Status.publishCount++;
  esp32Status.lastPublishTick = HAL_GetTick();
  return HAL_OK;
}

void ESP32_RequestImmediatePublish(void)
{
  esp32PublishRequested = 1U;
}

void ESP32_QueueAccidentEvent(uint32_t eventId, uint8_t eventType,
                              uint16_t peakAccelMg, uint16_t peakGyroDps)
{
  ESP32_AccidentEvent_t event = {0};
  if ((eventType != ACCIDENT_EVENT_COLLISION) &&
      (eventType != ACCIDENT_EVENT_ROLLOVER))
  {
    esp32Status.accidentEventDropCount++;
    return;
  }
  if (esp32AccidentQueue == NULL)
  {
    esp32Status.accidentEventDropCount++;
    return;
  }
  event.eventId = eventId;
  event.eventType = eventType;
  event.peakAccelMg = peakAccelMg;
  event.peakGyroDps = peakGyroDps;
  event.eventTick = HAL_GetTick();
  event.retryCount = 0U;
  if (xQueueSendToBack(esp32AccidentQueue, &event, 0U) == pdPASS)
  {
    esp32Status.accidentEventQueuedCount++;
    esp32Status.accidentEventLastId = eventId;
    esp32PublishRequested = 1U;
  }
  else
  {
    esp32Status.accidentEventDropCount++;
  }
}

static void ESP32_QueueDownlinkAck(uint32_t cmdId, const char *result, uint32_t errorCode)
{
  ESP32_DownlinkAck_t ack = {0};
  if ((result == NULL) || (esp32AckQueue == NULL))
  {
    esp32Status.downlinkAckErrorCount++;
    return;
  }
  ack.cmdId = cmdId;
  ack.errorCode = errorCode;
  (void)snprintf(ack.result, sizeof(ack.result), "%s", result);
  if (xQueueSendToBack(esp32AckQueue, &ack, 0U) != pdPASS)
  {
    esp32Status.downlinkAckErrorCount++;
  }
}

static void ESP32_ProcessAckQueue(void)
{
  ESP32_DownlinkAck_t ack = {0};
  while ((esp32AckQueue != NULL) &&
         (xQueueReceive(esp32AckQueue, &ack, 0U) == pdPASS))
  {
    char payload[128] = {0};
    (void)snprintf(payload, sizeof(payload),
                   "{\"cmd_id\":%lu,\"result\":\"%s\",\"error\":%lu,\"ts\":%lu}",
                   (unsigned long)ack.cmdId, ack.result,
                   (unsigned long)ack.errorCode,
                   (unsigned long)HAL_GetTick());
    if ((esp32Status.state == ESP32_STATE_MQTT_CONNECTED) &&
        (ESP32_PublishTopic(ESP32_MQTT_ACK_TOPIC, payload) == HAL_OK))
    {
      esp32Status.downlinkAckCount++;
    }
    else
    {
      esp32Status.downlinkAckErrorCount++;
    }
  }
}

static void ESP32_ProcessAccidentEventQueue(void)
{
  ESP32_AccidentEvent_t event = {0};
  char payload[192] = {0};
  if ((esp32AccidentQueue == NULL) ||
      (esp32Status.state != ESP32_STATE_MQTT_CONNECTED))
  {
    return;
  }
  if ((int32_t)(HAL_GetTick() - esp32AccidentRetryTick) < 0)
  {
    return;
  }
  if (xQueuePeek(esp32AccidentQueue, &event, 0U) != pdPASS)
  {
    return;
  }
  (void)snprintf(payload, sizeof(payload),
                 "{\"event_id\":%lu,\"type\":%u,\"name\":\"%s\",\"peak_accel_mg\":%u,\"peak_gyro_dps\":%u,\"received_ms\":%lu,\"retry\":%u}",
                 (unsigned long)event.eventId, (unsigned int)event.eventType,
                 AccidentDetector_GetEventText(event.eventType),
                 (unsigned int)event.peakAccelMg, (unsigned int)event.peakGyroDps,
                 (unsigned long)event.eventTick, (unsigned int)event.retryCount);
  if (ESP32_PublishTopic(ESP32_MQTT_ACCIDENT_TOPIC, payload) == HAL_OK)
  {
    (void)xQueueReceive(esp32AccidentQueue, &event, 0U);
    esp32Status.accidentEventSentCount++;
    esp32Status.accidentEventLastId = event.eventId;
    esp32AccidentRetryTick = HAL_GetTick();
  }
  else
  {
    (void)xQueueReceive(esp32AccidentQueue, &event, 0U);
    if (event.retryCount < 0xFFU)
    {
      event.retryCount++;
    }
    if (xQueueSendToFront(esp32AccidentQueue, &event, 0U) != pdPASS)
    {
      esp32Status.accidentEventDropCount++;
    }
    esp32Status.accidentEventRetryCount++;
    esp32Status.state = ESP32_STATE_ERROR;
    esp32AccidentRetryTick = HAL_GetTick() + 500U;
  }
}

void ESP32_MqttTask(void *argument)
{
  (void)argument;
  uint32_t nextPublishTick = HAL_GetTick();
  for (;;)
  {
#if (CAN_NODE_ROLE == CAN_NODE_ROLE_A)
    ESP32_DrainAsync();
    ESP32_ProcessDownlinkQueue();
    if (esp32Status.state != ESP32_STATE_MQTT_CONNECTED)
    {
      if (ESP32_Connect() != HAL_OK)
      {
        vTaskDelay(pdMS_TO_TICKS(ESP32_MQTT_RETRY_PERIOD_MS));
      }
      else
      {
        nextPublishTick = HAL_GetTick();
      }
    }
    else if ((esp32PublishRequested != 0U) ||
             ((int32_t)(HAL_GetTick() - nextPublishTick) >= 0))
    {
      ESP32_ProcessAckQueue();
      ESP32_ProcessAccidentEventQueue();
      esp32PublishRequested = 0U;
      nextPublishTick = HAL_GetTick() + esp32Status.currentPublishPeriodMs;
      if (ESP32_Publish() != HAL_OK)
      {
        esp32Status.state = ESP32_STATE_ERROR;
      }
    }
    else
    {
      ESP32_ProcessAckQueue();
      ESP32_ProcessAccidentEventQueue();
      vTaskDelay(pdMS_TO_TICKS(10U));
    }
#else
    esp32Status.state = ESP32_STATE_OFFLINE;
    vTaskDelay(pdMS_TO_TICKS(1000U));
#endif
  }
}

void ESP32_GetStatus(volatile ESP32_Status_t *status)
{
  if (status == NULL)
  {
    return;
  }
  *status = esp32Status;
}

HAL_StatusTypeDef ESP32_RunDownlinkReliabilityTest(void)
{
  uint32_t receivedBefore = esp32Status.downlinkCount;
  uint32_t acceptedBefore = esp32Status.downlinkAcceptedCount;
  uint32_t rejectedBefore = esp32Status.downlinkRejectedCount;
  const char *publishLine = "+MQTTSUBRECV:0,\"test\",25,{\"cmd_id\":1001,\"cmd\":\"publish\"}";
  const char *periodLine = "+MQTTSUBRECV:0,\"test\",37,{\"cmd_id\":1002,\"cmd\":\"set_period\",\"value\":1000}";
  const char *invalidLine = "+MQTTSUBRECV:0,\"test\",22,{\"cmd_id\":1003,\"cmd\":\"reboot\"}";
  if (esp32DownlinkQueue == NULL)
  {
    esp32Status.downlinkTestFailCount++;
    return HAL_ERROR;
  }
  ESP32_HandleDownlinkLine(publishLine);
  ESP32_HandleDownlinkLine(periodLine);
  ESP32_HandleDownlinkLine(invalidLine);
  ESP32_ProcessDownlinkQueue();
  if ((esp32Status.downlinkCount == (receivedBefore + 3U)) &&
      (esp32Status.downlinkAcceptedCount >= (acceptedBefore + 2U)) &&
      (esp32Status.downlinkRejectedCount >= (rejectedBefore + 1U)))
  {
    esp32Status.downlinkTestPassCount++;
    return HAL_OK;
  }
  esp32Status.downlinkTestFailCount++;
  return HAL_ERROR;
}
