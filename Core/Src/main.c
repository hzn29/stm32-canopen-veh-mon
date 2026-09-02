/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
#include "main.h"
#include "cmsis_os.h"
#include "canopen_node_config.h"
#include "canopen_port.h"
#include "esp32_at.h"
#include "esp32_mqtt_config.h"
#include "ssd1306.h"
#include "vehicle_alarm.h"
#include "accident_detector.h"
#include <stdio.h>

/* USER CODE BEGIN Includes */


/* USER CODE END Includes */

/* USER CODE BEGIN PTD */

typedef struct
{
  FDCAN_RxHeaderTypeDef header;
  uint8_t data[64];
} CanRxMessage_t;

typedef struct
{
  volatile uint32_t receivedCount;
  volatile uint32_t lastSequence;
  volatile uint32_t lostCount;
  volatile uint32_t lastReceiveTick;
  volatile uint32_t interArrivalMs;
  volatile uint32_t jitterMs;
  volatile uint32_t maxInterArrivalMs;
  volatile uint32_t maxJitterMs;
  volatile uint32_t senderTick;
  volatile uint32_t receiveTick;
  volatile uint8_t initialized;
} CanNodeStats_t; /* 瀹氫箟鑺傜偣缁熻缁撴瀯浣撶被鍨嬪悕绉般€?*/

/* USER CODE END PTD */

static uint8_t FdcanDataLengthBytes(uint32_t dataLengthCode)
{
  switch (dataLengthCode)
  {
    case FDCAN_DLC_BYTES_0: return 0U;
    case FDCAN_DLC_BYTES_1: return 1U;
    case FDCAN_DLC_BYTES_2: return 2U;
    case FDCAN_DLC_BYTES_3: return 3U;
    case FDCAN_DLC_BYTES_4: return 4U;
    case FDCAN_DLC_BYTES_5: return 5U;
    case FDCAN_DLC_BYTES_6: return 6U;
    case FDCAN_DLC_BYTES_7: return 7U;
    case FDCAN_DLC_BYTES_8: return 8U;
    case FDCAN_DLC_BYTES_12: return 12U;
    case FDCAN_DLC_BYTES_16: return 16U;
    case FDCAN_DLC_BYTES_20: return 20U;
    case FDCAN_DLC_BYTES_24: return 24U;
    case FDCAN_DLC_BYTES_32: return 32U;
    case FDCAN_DLC_BYTES_48: return 48U;
    case FDCAN_DLC_BYTES_64: return 64U;
    default: return 0U;
  }
}

/* USER CODE BEGIN PD */

#define CAN_ID_NODE_A 0x100U /* 鑺傜偣 A 鐨?ID 鏈€灏忥紝鍥犳鍏锋湁鏈€楂樹徊瑁佷紭鍏堢骇銆?*/
#define CAN_ID_NODE_B 0x300U /* 鑺傜偣 B 鐨勪徊瑁佷紭鍏堢骇浣庝簬鑺傜偣 A銆?*/
#define CAN_ID_NODE_C 0x500U /* 鑺傜偣 C 鐨勪徊瑁佷紭鍏堢骇鏈€浣庛€?*/
#define CAN_ID_TIME_SYNC 0x080U /* 鍚屾鎶ユ枃 ID 浣庝簬涓氬姟鎶ユ枃锛屽叿鏈夋洿楂樹徊瑁佷紭鍏堢骇銆?*/
#define CAN_TAG_NODE_A 0xA1U /* 鑺傜偣 A 鍦ㄦ暟鎹瓧鑺?4 涓啓鍏ユ鏍囪瘑銆?*/
#define CAN_TAG_NODE_B 0xB1U /* 鑺傜偣 B 鍦ㄦ暟鎹瓧鑺?4 涓啓鍏ユ鏍囪瘑銆?*/
#define CAN_TAG_NODE_C 0xC1U /* 鑺傜偣 C 鍦ㄦ暟鎹瓧鑺?4 涓啓鍏ユ鏍囪瘑銆?*/
#define CAN_TAG_TIME_SYNC 0x5AU /* 鎺ユ敹绔娇鐢ㄨ鏍囪瘑璇嗗埆鏃堕棿鍚屾鎶ユ枃銆?*/

#define CAN_NODE_A_PERIOD_MS 10U /* 鑺傜偣 A 鐨勭悊璁哄彂閫佸懆鏈熶负 10 ms銆?*/
#define CAN_NODE_B_PERIOD_MS 20U /* 鑺傜偣 B 鐨勭悊璁哄彂閫佸懆鏈熶负 20 ms銆?*/
#define CAN_NODE_C_PERIOD_MS 100U /* 鑺傜偣 C 鐨勭悊璁哄彂閫佸懆鏈熶负 100 ms銆?*/
#define CAN_TIME_SYNC_PERIOD_MS 100U /* 鑺傜偣 A 姣?100 ms 鍙戝竷涓€娆′富鏃堕挓銆?*/
#define CAN_LEGACY_TIME_SYNC_ENABLE 0U /* CANopen 宸ョ▼涓嶅啀鍙戦€佽嚜瀹氫箟鏃堕棿鎴冲悓姝ュ抚銆?*/
#define CAN_TIMESTAMP_OFFSET 8U /* 鍙戦€佹椂闂存埑浠庢暟鎹瓧鑺?8 寮€濮嬪瓨鏀俱€?*/
#define CAN_SCHED_MODE_FIXED_PRIORITY 0U /* 浣跨敤鍥哄畾 CAN ID 鍜屽浐瀹氬彂閫佸懆鏈熴€?*/
#define CAN_SCHED_MODE CAN_SCHED_MODE_FIXED_PRIORITY /* 鍚庣画绛栫暐瀵规瘮鏃跺彲鎵╁睍姝ゅ畯銆?*/

#if (CAN_NODE_ROLE == CAN_NODE_ROLE_A) /* 鍒ゆ柇鏄惁缂栬瘧鑺傜偣 A 鍥轰欢銆?*/
#define CAN_LOCAL_TX_ID CAN_ID_NODE_A /* 鑺傜偣 A 浣跨敤鏈€楂樹紭鍏堢骇鐨?0x100銆?*/
#define CAN_LOCAL_NODE_TAG CAN_TAG_NODE_A /* 鑺傜偣 A 鍐欏叆鑷繁鐨勫簲鐢ㄥ眰鏍囪瘑銆?*/
#define CAN_LOCAL_TX_PERIOD_MS CAN_NODE_A_PERIOD_MS /* 鑺傜偣 A 姣?10 ms 鍙戦€佷竴甯ф帶鍒舵姤鏂囥€?*/
#elif (CAN_NODE_ROLE == CAN_NODE_ROLE_B) /* 鍒ゆ柇鏄惁缂栬瘧鑺傜偣 B 鍥轰欢銆?*/
#define CAN_LOCAL_TX_ID CAN_ID_NODE_B /* 鑺傜偣 B 浣跨敤涓紭鍏堢骇鐨?0x300銆?*/
#define CAN_LOCAL_NODE_TAG CAN_TAG_NODE_B /* 鑺傜偣 B 鍐欏叆鑷繁鐨勫簲鐢ㄥ眰鏍囪瘑銆?*/
#define CAN_LOCAL_TX_PERIOD_MS CAN_NODE_B_PERIOD_MS /* 鑺傜偣 B 姣?20 ms 鍙戦€佷竴甯х姸鎬佹姤鏂囥€?*/
#elif (CAN_NODE_ROLE == CAN_NODE_ROLE_C) /* 鍒ゆ柇鏄惁缂栬瘧鑺傜偣 C 鍥轰欢銆?*/
#define CAN_LOCAL_TX_ID CAN_ID_NODE_C /* 鑺傜偣 C 浣跨敤浣庝紭鍏堢骇鐨?0x500銆?*/
#define CAN_LOCAL_NODE_TAG CAN_TAG_NODE_C /* 鑺傜偣 C 鍐欏叆鑷繁鐨勫簲鐢ㄥ眰鏍囪瘑銆?*/
#define CAN_LOCAL_TX_PERIOD_MS CAN_NODE_C_PERIOD_MS /* 鑺傜偣 C 姣?100 ms 鍙戦€佷竴甯х洃娴嬫姤鏂囥€?*/
#else /* 褰撳墠瑙掕壊缂栧彿涓嶅湪鍏佽鑼冨洿鍐呫€?*/
#error "CAN_NODE_ROLE 蹇呴』璁剧疆涓?CAN_NODE_ROLE_A銆丆AN_NODE_ROLE_B 鎴?CAN_NODE_ROLE_C" /* 闃绘鐢熸垚瑙掕壊閰嶇疆閿欒鐨勫浐浠躲€?*/
#endif /* 缁撴潫鏈満鍙戦€佸弬鏁伴€夋嫨銆?*/

/* USER CODE END PD */

/* USER CODE BEGIN PM */

#define APP_LED_GPIO_Port GPIOA /* 瀹氫箟鐢ㄦ埛 LED 鎵€杩炴帴鐨?GPIO 绔彛銆?*/
#define APP_LED_Pin GPIO_PIN_5 /* 瀹氫箟鐢ㄦ埛 LED 鎵€杩炴帴鐨?GPIO 寮曡剼銆?*/

/* USER CODE END PM */

FDCAN_HandleTypeDef hfdcan1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
IWDG_HandleTypeDef hiwdg;
volatile ESP32_Status_t esp32Status;
volatile VehicleAlarmStatus_t vehicleAlarmStatus;
  volatile AccidentDetectorStatus_t accidentStatus;
  static ESP32_AccidentPersist_t accidentPersistSnapshot;

osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* USER CODE BEGIN PV */

static QueueHandle_t canRxQueue;
static volatile uint32_t rxCount;
volatile CanOpenPortDiagnostics_t diagnostics;
volatile HAL_StatusTypeDef icm42688Status;
volatile uint8_t icm42688WhoAmI;
  volatile ICM42688_RawData_t icm42688RawData;
  volatile BH182_Data_t bh182Data;
  volatile HAL_StatusTypeDef bh182Status;
static uint32_t icm42688LastSampleTick;
#if (CAN_NODE_ROLE == CAN_NODE_ROLE_B)
#define APP_IMU_SAMPLE_PERIOD_MS 1U
#else
#define APP_IMU_SAMPLE_PERIOD_MS 10U
#endif
static uint32_t accidentLastUpdateTick;
static uint8_t lastReportedAccidentEvent;
static volatile uint32_t rxSequence;
static volatile uint32_t rxCountA;
static volatile uint32_t rxSequenceA;
static volatile uint32_t rxCountB;
static volatile uint32_t rxSequenceB;
static volatile uint32_t rxCountC;
static volatile uint32_t rxSequenceC;
static volatile uint32_t rxQueueDropCount;
static volatile CanNodeStats_t rxStatsA;
static volatile CanNodeStats_t rxStatsB;
static volatile CanNodeStats_t rxStatsC;
static volatile int32_t canTimeOffsetMs;
static volatile uint32_t canSyncRxCount;
static volatile uint32_t canMasterTick;
static volatile uint32_t canSyncLocalTick;
static volatile uint32_t canSyncOffsetAbsMs;
static TaskHandle_t canTxTaskHandle;
static TaskHandle_t canRxTaskHandle;

static TaskHandle_t canRecoveryTaskHandle;
static TaskHandle_t esp32MqttTaskHandle;
static volatile uint32_t appHeartbeatTick;
static uint32_t vehicleAlarmLastUpdateTick;
volatile HAL_StatusTypeDef oledStatus;
static TaskHandle_t oledTaskHandle;
static volatile uint32_t canErrorCallbackCount;
static volatile uint32_t canErrorWarningCount;
static volatile uint32_t canErrorPassiveCount;
static volatile uint32_t canBusOffCount;
static volatile uint8_t canBusOffActive;
static volatile uint32_t canRecoveryCount;
static volatile uint32_t canRecoveryFailCount;

/* USER CODE END PV */

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_IWDG_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */

static void CanTxTask(void *argument);
static void CanRxTask(void *argument);
static void CanRecoveryTask(void *argument);
static void IwdgMonitorTask(void *argument);
static void OledDashboardTask(void *argument);
static uint32_t CanDecodeSequence(const uint8_t *data);
static uint32_t CanDecodeTimestamp(const uint8_t *data);
static void CanUpdateNodeStats(volatile CanNodeStats_t *stats, /* 浼犲叆寰呮洿鏂扮殑鑺傜偣缁熻瀵硅薄銆?*/
                               uint32_t receivedSequence, /* 浼犲叆褰撳墠鎶ユ枃鐨勫簲鐢ㄥ眰搴忓彿銆?*/
                               uint32_t senderTick, /* 浼犲叆褰撳墠鎶ユ枃鐨勫彂閫佺鑺傛媿銆?*/
                               uint32_t expectedPeriodMs); /* 浼犲叆璇ヨ妭鐐圭殑鐞嗚鍛ㄦ湡銆?*/
static uint32_t CanGetGlobalTick(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */


  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  SystemClock_Config();

  MX_IWDG_Init();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_FDCAN1_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  icm42688Status = ICM42688_Init(&hspi2, GPIOB, GPIO_PIN_12);
  if (icm42688Status == HAL_OK)
  {
    icm42688Status = ICM42688_ReadWhoAmI((uint8_t *)&icm42688WhoAmI);
  }
  if (BH182_Init(&huart1) != HAL_OK)
  {
    bh182Status = HAL_ERROR;
    memset((void *)&bh182Data, 0, sizeof(bh182Data));
  }
  else
  {
    bh182Status = BH182_Start();
  }
  if (ESP32_Init(&huart2) != HAL_OK)
  {
    esp32Status.state = ESP32_STATE_OFFLINE;
  }
  else
  {
    if (ESP32_StartReceive() != HAL_OK)
    {
      esp32Status.state = ESP32_STATE_OFFLINE;
    }
  }
  oledStatus = SSD1306_Init(&hi2c1, (uint16_t)(0x3CU << 1U));
  VehicleAlarm_Init();
  AccidentDetector_Init();
  if (ESP32_Persist_LoadAccident(&accidentPersistSnapshot) == HAL_OK)
  {
    AccidentDetector_RestoreLastEventId(accidentPersistSnapshot.lastEventId);
  }
////////////////////////////////////////////閰嶇疆杩囨护鍣ㄥ苟鍚姩 FDCAN1/////////////////////////////////////////
  FDCAN_FilterTypeDef canFilter = {0};
  canFilter.IdType = FDCAN_STANDARD_ID;
  canFilter.FilterIndex = 0U;
  canFilter.FilterType = FDCAN_FILTER_MASK;
  canFilter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  canFilter.FilterID1 = 0x000U;
  canFilter.FilterID2 = 0x000U;
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &canFilter) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, /* 閰嶇疆 FDCAN1 鐨勫叏灞€杩囨护鍣ㄣ€?*/
                                   FDCAN_ACCEPT_IN_RX_FIFO0, /* 灏嗘湭鍖归厤鐨勬爣鍑嗗抚璺敱鍒?RX FIFO0銆?*/
                                   FDCAN_ACCEPT_IN_RX_FIFO0, /* 灏嗘湭鍖归厤鐨勬墿灞曞抚璺敱鍒?RX FIFO0銆?*/
                                   FDCAN_REJECT_REMOTE, /* 鎷掔粷鏈尮閰嶇殑鏍囧噯杩滅▼甯с€?*/
                                   FDCAN_REJECT_REMOTE) != HAL_OK) /* 鎷掔粷鏈尮閰嶇殑鎵╁睍杩滅▼甯с€?*/
  {
    Error_Handler();
  }
  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  if (CanOpenPort_StackInit(CANOPEN_NODE_ID) != CO_ERROR_NO)
  {
    Error_Handler();
  }
  if (HAL_FDCAN_ActivateNotification(&hfdcan1, /* 涓?FDCAN1 寮€鍚€氱煡鍔熻兘銆?*/
                                     FDCAN_IT_RX_FIFO0_NEW_MESSAGE | /* 寮€鍚?RX FIFO0 鏂版姤鏂囦腑鏂€?*/
                                     FDCAN_IT_ERROR_WARNING | /* 寮€鍚?Error Warning 鐘舵€佸彉鍖栦腑鏂€?*/
                                     FDCAN_IT_ERROR_PASSIVE | /* 寮€鍚?Error Passive 鐘舵€佸彉鍖栦腑鏂€?*/
                                     FDCAN_IT_BUS_OFF, /* 寮€鍚?Bus-Off 鐘舵€佸彉鍖栦腑鏂€?*/
                                     0U) != HAL_OK) /* 涓嶅紑鍚?TX 缂撳啿鍖轰腑鏂帺鐮併€?*/
  {
    Error_Handler();
  }

  /* USER CODE END 2 */

  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  canRxQueue = xQueueCreate(8U, sizeof(CanRxMessage_t));
  if (canRxQueue == NULL)
  {
    Error_Handler();
  }
  /* USER CODE END RTOS_QUEUES */

  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  if (xTaskCreate(CanTxTask, /* 鎸囧畾鍙戦€佷换鍔″叆鍙ｅ嚱鏁般€?*/
                  "canTxTask", /* 鎸囧畾鍙戦€佷换鍔″悕绉般€?*/
                  256U, /* 涓哄彂閫佷换鍔″垎閰?256 涓?FreeRTOS 鏍堝瓧銆?*/
                  NULL, /* 鍙戦€佷换鍔′笉鎺ユ敹浠诲姟鍙傛暟銆?*/
                  tskIDLE_PRIORITY + 2U, /* 灏嗗彂閫佷换鍔¤缃负鏅€氫紭鍏堢骇銆?*/
                  &canTxTaskHandle) != pdPASS) /* 妫€鏌ュ彂閫佷换鍔″垱寤虹粨鏋溿€?*/
  {
    Error_Handler();
  }
  if (xTaskCreate(CanRxTask, /* 鎸囧畾鎺ユ敹浠诲姟鍏ュ彛鍑芥暟銆?*/
                  "canRxTask", /* 鎸囧畾鎺ユ敹浠诲姟鍚嶇О銆?*/
                  256U, /* 涓烘帴鏀朵换鍔″垎閰?256 涓?FreeRTOS 鏍堝瓧銆?*/
                  NULL, /* 鎺ユ敹浠诲姟涓嶆帴鏀朵换鍔″弬鏁般€?*/
                  tskIDLE_PRIORITY + 3U, /* 灏嗘帴鏀朵换鍔¤缃负楂樹簬鍙戦€佷换鍔°€?*/
                  &canRxTaskHandle) != pdPASS) /* 妫€鏌ユ帴鏀朵换鍔″垱寤虹粨鏋溿€?*/
  {
    Error_Handler();
  }
  if (xTaskCreate(CanRecoveryTask, /* 鎸囧畾 CAN 鏁呴殰鎭㈠浠诲姟鍏ュ彛鍑芥暟銆?*/
                  "canRecoveryTask", /* 鎸囧畾 CAN 鏁呴殰鎭㈠浠诲姟鍚嶇О銆?*/
                  256U, /* 涓烘仮澶嶄换鍔″垎閰?256 涓?FreeRTOS 鏍堝瓧銆?*/
                  NULL, /* 鎭㈠浠诲姟涓嶆帴鏀朵换鍔″弬鏁般€?*/
                  tskIDLE_PRIORITY + 4U, /* 璁剧疆鎭㈠浠诲姟浼樺厛绾ч珮浜庢敹鍙戜换鍔°€?*/
                  &canRecoveryTaskHandle) != pdPASS) /* 妫€鏌ユ仮澶嶄换鍔″垱寤虹粨鏋溿€?*/
  {
    Error_Handler();
  }
  CanOpenPort_RestoreAccidentState(&accidentPersistSnapshot);
  if (xTaskCreate(IwdgMonitorTask, /* 鎸囧畾鐪嬮棬鐙楃洃鎺т换鍔″叆鍙ｅ嚱鏁般€?*/
                  "iwdgMonitor", /* 鎸囧畾鐪嬮棬鐙椾换鍔″悕绉般€?*/
                  128U, /* 涓虹湅闂ㄧ嫍浠诲姟鍒嗛厤鏈€灏忚繍琛屾爤銆?*/
                  NULL, /* 鐪嬮棬鐙椾换鍔′笉鎺ユ敹浠诲姟鍙傛暟銆?*/
                  tskIDLE_PRIORITY + 5U, /* 璁剧疆楂樹簬搴旂敤浠诲姟鐨勪紭鍏堢骇銆?*/
                  NULL) != pdPASS) /* 妫€鏌ョ湅闂ㄧ嫍浠诲姟鍒涘缓缁撴灉銆?*/
  {
    Error_Handler();
  }
#if (CAN_NODE_ROLE == CAN_NODE_ROLE_A)
  if (xTaskCreate(OledDashboardTask, /* 鎸囧畾 OLED 鏄剧ず浠诲姟鍏ュ彛鍑芥暟銆?*/
                  "oledTask", /* 鎸囧畾 OLED 浠诲姟鍚嶇О銆?*/
                  384U, /* 涓烘牸寮忓寲瀛楃涓插拰鏄剧ず浠诲姟鍒嗛厤鏍堢┖闂淬€?*/
                  NULL, /* OLED 浠诲姟涓嶆帴鏀朵换鍔″弬鏁般€?*/
                  tskIDLE_PRIORITY + 1U, /* 璁剧疆浣庝簬 CAN 鎺ユ敹浠诲姟鐨勪紭鍏堢骇銆?*/
                  &oledTaskHandle) != pdPASS) /* 妫€鏌?OLED 浠诲姟鍒涘缓缁撴灉銆?*/
  {
    oledStatus = HAL_ERROR;
  }
  if (xTaskCreate(ESP32_MqttTask, /* 鎸囧畾 ESP32 MQTT 浠诲姟鍏ュ彛鍑芥暟銆?*/
                  "esp32Mqtt", /* 鎸囧畾 ESP32 MQTT 浠诲姟鍚嶇О銆?*/
                  512U, /* 涓?JSON 鍜?AT 鍛戒护缂撳瓨鍒嗛厤 512 涓?FreeRTOS 鏍堝瓧銆?*/
                  NULL, /* MQTT 浠诲姟涓嶆帴鏀朵换鍔″弬鏁般€?*/
                  tskIDLE_PRIORITY + 1U, /* 璁剧疆浣庝簬 CAN 鎺ユ敹浠诲姟鐨勬櫘閫氫紭鍏堢骇銆?*/
                  &esp32MqttTaskHandle) != pdPASS) /* 妫€鏌?MQTT 浠诲姟鍒涘缓缁撴灉銆?*/
  {
    esp32Status.state = ESP32_STATE_ERROR;
  }
#endif
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* USER CODE END RTOS_EVENTS */

  osKernelStart();


  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 4;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 6;
  hfdcan1.Init.NominalTimeSeg2 = 1;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

static void MX_IWDG_Init(void)
{
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
  hiwdg.Init.Reload = 3999U;
  hiwdg.Init.Window = IWDG_WINDOW_DISABLE;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00303D5BU;
  hi2c1.Init.OwnAddress1 = 0U;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0U;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  GPIO_InitTypeDef gpioInit = {0};
  GPIO_InitTypeDef icmCsInit = {0};
  HAL_GPIO_WritePin(APP_LED_GPIO_Port, APP_LED_Pin, GPIO_PIN_RESET);
  gpioInit.Pin = APP_LED_Pin;
  gpioInit.Mode = GPIO_MODE_OUTPUT_PP;
  gpioInit.Pull = GPIO_NOPULL;
  gpioInit.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(APP_LED_GPIO_Port, &gpioInit);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
  icmCsInit.Pin = GPIO_PIN_12;
  icmCsInit.Mode = GPIO_MODE_OUTPUT_PP;
  icmCsInit.Pull = GPIO_NOPULL;
  icmCsInit.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &icmCsInit);

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t rxFifo0Interrupts)
{
  CanRxMessage_t message = {0};
  uint32_t syncReceiveTick = 0U;
  if ((hfdcan == &hfdcan1) && /* 鍙帴鍙楁潵鑷?FDCAN1 瀹炰緥鐨勫洖璋冦€?*/
      ((rxFifo0Interrupts & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0U)) /* 鍙帴鍙?RX FIFO0 鏂版姤鏂囦簨浠躲€?*/
  {
    if (HAL_FDCAN_GetRxMessage(hfdcan, /* 浠庤Е鍙戝洖璋冪殑 FDCAN 瀹炰緥璇诲彇鎶ユ枃銆?*/
                               FDCAN_RX_FIFO0, /* 浠?RX FIFO0 璇诲彇寰呭鐞嗘姤鏂囥€?*/
                               &message.header, /* 灏嗘帴鏀舵姤鏂囧ご淇濆瓨鍒版湰鍦版姤鏂囧璞°€?*/
                               message.data) == HAL_OK) /* 灏嗘帴鏀舵暟鎹繚瀛樺埌鏈湴鎶ユ枃瀵硅薄銆?*/
    {
      CanOpenPort_RxInterrupt(message.header.Identifier,
                              FdcanDataLengthBytes(message.header.DataLength),
                              message.data);
      syncReceiveTick = HAL_GetTick();
#if (CAN_LEGACY_TIME_SYNC_ENABLE == 1U)
      if ((message.header.Identifier == CAN_ID_TIME_SYNC) && /* 妫€鏌ュ悓姝ユ姤鏂?ID銆?*/
          (message.header.IdType == FDCAN_STANDARD_ID) && /* 妫€鏌ュ悓姝ユ姤鏂囦娇鐢ㄦ爣鍑?ID銆?*/
          (message.header.DataLength == FDCAN_DLC_BYTES_8) && /* 妫€鏌ュ悓姝ユ姤鏂囦负 8 瀛楄妭 Classic CAN 甯с€?*/
          (message.header.FDFormat == FDCAN_CLASSIC_CAN) && /* 妫€鏌ュ悓姝ユ姤鏂囦负 Classic CAN 鏍煎紡銆?*/
          (message.header.BitRateSwitch == FDCAN_BRS_OFF) && /* 妫€鏌ュ悓姝ユ姤鏂囧叧闂?BRS銆?*/
          (message.data[4] == CAN_TAG_TIME_SYNC)) /* 妫€鏌ュ悓姝ユ姤鏂囨暟鎹爣璇嗐€?*/
      {
#if (CAN_NODE_ROLE != CAN_NODE_ROLE_A) /* 浠?B銆丆 鑺傜偣闇€瑕佽绠楃浉瀵?A 鐨勬椂閽熷亸绉汇€?*/
        uint32_t receivedMasterTick = CanDecodeTimestamp(message.data);
        canMasterTick = receivedMasterTick;
        canSyncLocalTick = syncReceiveTick;
        canTimeOffsetMs = (int32_t)(syncReceiveTick - receivedMasterTick);
        canSyncOffsetAbsMs = (uint32_t)(canTimeOffsetMs >= 0 ? canTimeOffsetMs : -canTimeOffsetMs);
        canSyncRxCount++;
#endif /* 缁撴潫 B銆丆 鑺傜偣鏃堕棿鍋忕Щ璁＄畻銆?*/
        return;
      }
#endif /* 缁撴潫鏃х増鑷畾涔夋椂闂村悓姝ュ抚瑙ｆ瀽銆?*/
      BaseType_t xHigherPriorityTaskWoken = pdFALSE;
      if (xQueueSendToBackFromISR(canRxQueue, /* 鎸囧畾鎺ユ敹娑堟伅闃熷垪銆?*/
                                  &message, /* 灏嗘姤鏂囧ご鍜屾暟鎹鍒跺埌闃熷垪銆?*/
                                  &xHigherPriorityTaskWoken) != pdPASS) /* 鍦ㄤ腑鏂腑涓嶇瓑寰呴槦鍒楃┖闂淬€傦紙xHigherPriorityTaskWoken杈撳嚭鍙傛暟锛?*/
      {
        rxQueueDropCount++;
      }
      portYIELD_FROM_ISR(xHigherPriorityTaskWoken); /* 锛堢敤xHigherPriorityTaskWoken璇锋眰涓婁笅鏂囧垏鎹級 */
    }
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart2)
  {
    ESP32_OnRxComplete();
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart2)
  {
    ESP32_OnUartError();
  }
}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
  if ((hfdcan == &hfdcan1) && (ErrorStatusITs != 0U))
  {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if ((ErrorStatusITs & FDCAN_IT_ERROR_WARNING) != 0U)
    {
      canErrorWarningCount++;
    }
    if ((ErrorStatusITs & FDCAN_IT_ERROR_PASSIVE) != 0U)
    {
      canErrorPassiveCount++;
    }
    if ((ErrorStatusITs & FDCAN_IT_BUS_OFF) != 0U)
    {
      canBusOffCount++;
      canBusOffActive = 1U;
      if (canRecoveryTaskHandle != NULL)
      {
        vTaskNotifyGiveFromISR(canRecoveryTaskHandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
      }
    }
  }
}

void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan)
{
  if (hfdcan == &hfdcan1)
  {
    canErrorCallbackCount++;
  }
}

static void CanRecoveryTask(void *argument)
{
  (void)argument;
  for (;;)
  {
    (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(100U));
    if (HAL_FDCAN_Stop(&hfdcan1) == HAL_OK)
    {
      vTaskDelay(pdMS_TO_TICKS(10U));
      if (HAL_FDCAN_Start(&hfdcan1) == HAL_OK)
      {
        if (HAL_FDCAN_ActivateNotification(&hfdcan1,
                                           FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
                                           FDCAN_IT_ERROR_WARNING |
                                           FDCAN_IT_ERROR_PASSIVE |
                                           FDCAN_IT_BUS_OFF,
                                           0U) == HAL_OK)
        {
          canRecoveryCount++;
          canBusOffActive = 0U;
        }
        else
        {
          canRecoveryFailCount++;
        }
      }
      else
      {
        canRecoveryFailCount++;
      }
    }
    else
    {
      canRecoveryFailCount++;
    }
  }
}

static void IwdgMonitorTask(void *argument)
{
  (void)argument;
  uint32_t nowTick = 0U;
  for (;;)
  {
    nowTick = HAL_GetTick();
    if ((nowTick - appHeartbeatTick) < ESP32_IWDG_TIMEOUT_MS)
    {
      (void)HAL_IWDG_Refresh(&hiwdg);
    }
    vTaskDelay(pdMS_TO_TICKS(ESP32_IWDG_REFRESH_PERIOD_MS));
  }
}

static void CanTxTask(void *argument)
{
  (void)argument;
  FDCAN_TxHeaderTypeDef txHeader = {0};
  uint8_t txData[8] = {0};
  uint32_t sequence = 0U;
  uint32_t transmitTick = 0U;
  uint32_t syncSequence = 0U;
  uint32_t lastSyncTick = 0U;
  FDCAN_TxHeaderTypeDef syncHeader = {0};
  uint8_t syncData[8] = {0};
  txHeader.Identifier = CAN_LOCAL_TX_ID;
  txHeader.IdType = FDCAN_STANDARD_ID;
  txHeader.TxFrameType = FDCAN_DATA_FRAME;
  txHeader.DataLength = FDCAN_DLC_BYTES_8;
  txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  txHeader.BitRateSwitch = FDCAN_BRS_OFF;
  txHeader.FDFormat = FDCAN_CLASSIC_CAN;
  txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  txHeader.MessageMarker = 0U;
  syncHeader.Identifier = CAN_ID_TIME_SYNC;
  syncHeader.IdType = FDCAN_STANDARD_ID;
  syncHeader.TxFrameType = FDCAN_DATA_FRAME;
  syncHeader.DataLength = FDCAN_DLC_BYTES_8;
  syncHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  syncHeader.BitRateSwitch = FDCAN_BRS_OFF;
  syncHeader.FDFormat = FDCAN_CLASSIC_CAN;
  syncHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  syncHeader.MessageMarker = 0U;
  lastSyncTick = HAL_GetTick() - CAN_TIME_SYNC_PERIOD_MS;
  for (;;)
  {
    txData[0] = (uint8_t)(sequence >> 0U);
    txData[1] = (uint8_t)(sequence >> 8U);
    txData[2] = (uint8_t)(sequence >> 16U);
    txData[3] = (uint8_t)(sequence >> 24U);
    txData[4] = CAN_LOCAL_NODE_TAG;
    txData[5] = 0x03U;
    txData[6] = 0x00U;
    txData[7] = 0x00U;
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, txData) != HAL_OK)
    {
      Error_Handler();
    }
    sequence++;
#if (CAN_LEGACY_TIME_SYNC_ENABLE == 1U)
#if (CAN_NODE_ROLE == CAN_NODE_ROLE_A) /* 浠呰妭鐐?A 璐熻矗鍙戝竷涓绘椂閽熷悓姝ュ抚銆?*/
    if ((HAL_GetTick() - lastSyncTick) >= CAN_TIME_SYNC_PERIOD_MS)
    {
      lastSyncTick = HAL_GetTick();
      syncData[0] = (uint8_t)(syncSequence >> 0U);
      syncData[1] = (uint8_t)(syncSequence >> 8U);
      syncData[2] = (uint8_t)(syncSequence >> 16U);
      syncData[3] = (uint8_t)(syncSequence >> 24U);
      syncData[4] = CAN_TAG_TIME_SYNC;
      syncData[5] = 0x03U;
      syncData[6] = 0x00U;
      syncData[7] = 0x00U;
      transmitTick = HAL_GetTick();
      if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &syncHeader, syncData) != HAL_OK)
      {
        Error_Handler();
      }
      syncSequence++;
    }
#endif /* 缁撴潫鑺傜偣 A 涓绘椂閽熷悓姝ュ抚鍙戦€侀€昏緫銆?*/
#endif /* 缁撴潫鏃х増鑷畾涔夋椂闂村悓姝ュ抚鍙戦€併€?*/
    vTaskDelay(pdMS_TO_TICKS(CAN_LOCAL_TX_PERIOD_MS));
  }
}

static void CanRxTask(void *argument)
{
  (void)argument;
  CanRxMessage_t message = {0};
  for (;;)
  {
    if (xQueueReceive(canRxQueue, /* 鎸囧畾鎺ユ敹娑堟伅闃熷垪銆?*/
                      &message, /* 灏嗛槦鍒楁姤鏂囧鍒跺埌鏈湴瀵硅薄銆?*/
                      portMAX_DELAY) == pdPASS) /* 鏃犻檺绛夊緟鐩村埌鏀跺埌鎶ユ枃銆?*/
    {
      if ((message.header.IdType == FDCAN_STANDARD_ID) && /* 妫€鏌ュ抚鏄惁浣跨敤 11 浣嶆爣璇嗙銆?*/
          (message.header.DataLength == FDCAN_DLC_BYTES_8) && /* 妫€鏌ヨ浇鑽锋槸鍚﹀寘鍚?8 涓瓧鑺傘€?*/
          (message.header.FDFormat == FDCAN_CLASSIC_CAN) && /* 妫€鏌ユ帴鏀跺抚鏄惁涓?Classic CAN 鏍煎紡銆?*/
          (message.header.BitRateSwitch == FDCAN_BRS_OFF) && /* 妫€鏌ユ帴鏀跺抚鏄惁鍏抽棴 BRS銆?*/
          (message.data[5] == 0x03U)) /* 妫€鏌ユ姤鏂囨槸鍚﹀睘浜庝笁鑺傜偣 CAN FD 瀹為獙銆?*/
      {
        uint32_t receivedSequence = CanDecodeSequence(message.data);
        uint32_t senderTick = CanDecodeTimestamp(message.data);
        if ((message.header.Identifier == CAN_ID_NODE_A) && /* 妫€鏌ユ槸鍚︿负鑺傜偣 A 鐨?CAN ID銆?*/
            (message.data[4] == CAN_TAG_NODE_A)) /* 妫€鏌ヨ妭鐐?A 鐨勫簲鐢ㄥ眰鏍囪瘑銆?*/
        {
          rxSequenceA = receivedSequence;
          rxCountA++;
          CanUpdateNodeStats(&rxStatsA, receivedSequence, senderTick, CAN_NODE_A_PERIOD_MS);
        }
        else if ((message.header.Identifier == CAN_ID_NODE_B) && /* 妫€鏌ユ槸鍚︿负鑺傜偣 B 鐨?CAN ID銆?*/
                 (message.data[4] == CAN_TAG_NODE_B)) /* 妫€鏌ヨ妭鐐?B 鐨勫簲鐢ㄥ眰鏍囪瘑銆?*/
        {
          rxSequenceB = receivedSequence;
          rxCountB++;
          CanUpdateNodeStats(&rxStatsB, receivedSequence, senderTick, CAN_NODE_B_PERIOD_MS);
        }
        else if ((message.header.Identifier == CAN_ID_NODE_C) && /* 妫€鏌ユ槸鍚︿负鑺傜偣 C 鐨?CAN ID銆?*/
                 (message.data[4] == CAN_TAG_NODE_C)) /* 妫€鏌ヨ妭鐐?C 鐨勫簲鐢ㄥ眰鏍囪瘑銆?*/
        {
          rxSequenceC = receivedSequence;
          rxCountC++;
          CanUpdateNodeStats(&rxStatsC, receivedSequence, senderTick, CAN_NODE_C_PERIOD_MS);
        }
        else /* 褰撳墠鎶ユ枃鐨?ID 鍜岃妭鐐规爣璇嗕笉鏋勬垚鏈夋晥缁勫悎銆?*/
        {
          continue;
        }
        rxSequence = receivedSequence;
        rxCount++;
        HAL_GPIO_TogglePin(APP_LED_GPIO_Port, APP_LED_Pin);
      }
    }
  }
}

static uint32_t CanDecodeTimestamp(const uint8_t *data)
{
  return ((uint32_t)data[CAN_TIMESTAMP_OFFSET + 0U] << 0U) |
         ((uint32_t)data[CAN_TIMESTAMP_OFFSET + 1U] << 8U) |
         ((uint32_t)data[CAN_TIMESTAMP_OFFSET + 2U] << 16U) |
         ((uint32_t)data[CAN_TIMESTAMP_OFFSET + 3U] << 24U);
}

static uint32_t CanGetGlobalTick(void)
{
  return HAL_GetTick() - (uint32_t)canTimeOffsetMs;
}

static void CanUpdateNodeStats(volatile CanNodeStats_t *stats, /* 浼犲叆寰呮洿鏂扮殑鑺傜偣缁熻瀵硅薄銆?*/
                               uint32_t receivedSequence, /* 浼犲叆褰撳墠鎶ユ枃鐨勫簲鐢ㄥ眰搴忓彿銆?*/
                               uint32_t senderTick, /* 浼犲叆褰撳墠鎶ユ枃鐨勫彂閫佺鑺傛媿銆?*/
                               uint32_t expectedPeriodMs) /* 浼犲叆璇ヨ妭鐐圭殑鐞嗚鍛ㄦ湡銆?*/
{
  uint32_t receiveTick = CanGetGlobalTick();
  uint32_t sequenceDelta = 0U;
  uint32_t interArrivalMs = 0U;
  uint32_t jitterMs = 0U;
  if (stats->initialized != 0U)
  {
    sequenceDelta = receivedSequence - stats->lastSequence;
    if ((sequenceDelta > 1U) && (sequenceDelta < 0x80000000U))
    {
      stats->lostCount += sequenceDelta - 1U;
    }
    interArrivalMs = receiveTick - stats->lastReceiveTick;
    stats->interArrivalMs = interArrivalMs;
    if (interArrivalMs >= expectedPeriodMs)
    {
      jitterMs = interArrivalMs - expectedPeriodMs;
    }
    else
    {
      jitterMs = expectedPeriodMs - interArrivalMs;
    }
    stats->jitterMs = jitterMs;
    if (interArrivalMs > stats->maxInterArrivalMs)
    {
      stats->maxInterArrivalMs = interArrivalMs;
    }
    if (jitterMs > stats->maxJitterMs)
    {
      stats->maxJitterMs = jitterMs;
    }
  }
  stats->lastSequence = receivedSequence;
  stats->senderTick = senderTick;
  stats->receiveTick = receiveTick;
  stats->lastReceiveTick = receiveTick;
  stats->receivedCount++;
  stats->initialized = 1U;
}

static uint32_t CanDecodeSequence(const uint8_t *data)
{
  return ((uint32_t)data[0] << 0U) |
         ((uint32_t)data[1] << 8U) |
         ((uint32_t)data[2] << 16U) |
         ((uint32_t)data[3] << 24U);
}

static void OledDashboardTask(void *argument)
{
  (void)argument;
  CanOpenPortDiagnostics_t snapshot = {0};
  char line[24] = {0};
  uint32_t alarmBits = 0U;
  VehicleAlarmStatus_t alarmSnapshot = {0};
  AccidentDetectorStatus_t accidentSnapshot = {0};
  uint8_t displayPage = 0U;
  uint32_t nextPageTick = 0U;
  for (;;)
  {
    if (oledStatus != HAL_OK)
    {
      oledStatus = SSD1306_Init(&hi2c1, (uint16_t)(0x3CU << 1U));
      vTaskDelay(pdMS_TO_TICKS(1000U));
      continue;
    }
    CanOpenPort_GetDiagnostics(&snapshot);
    VehicleAlarm_GetStatus(&alarmSnapshot);
    AccidentDetector_GetStatus(&accidentSnapshot);
    SSD1306_Clear();
    alarmBits = alarmSnapshot.activeAlarmBits;
    {
      uint32_t nowTick = HAL_GetTick();
      if (alarmSnapshot.state != VEHICLE_ALARM_NORMAL)
      {
        displayPage = 1U;
        nextPageTick = nowTick + 3000U;
      }
      else if ((int32_t)(nowTick - nextPageTick) >= 0)
      {
        displayPage = (displayPage == 0U) ? 1U : 0U;
        nextPageTick = nowTick + 3000U;
      }
    }
    if (displayPage == 0U)
    {
    (void)snprintf(line, sizeof(line), "SPD:%lu mm/s", (unsigned long)OD_RAM.x2103_speedMmps);
    SSD1306_SetCursor(0U, 0U);
    SSD1306_WriteString(line);
    (void)snprintf(line, sizeof(line), "GPS:%c SAT:%02u", (OD_RAM.x2105_fixValid != 0U) ? 'Y' : 'N', (unsigned int)OD_RAM.x2104_satelliteCount);
    SSD1306_SetCursor(0U, 1U);
    SSD1306_WriteString(line);
    (void)snprintf(line, sizeof(line), "LAT:%ld", (long)OD_RAM.x2100_latitudeE7);
    SSD1306_SetCursor(0U, 2U);
    SSD1306_WriteString(line);
    (void)snprintf(line, sizeof(line), "LON:%ld", (long)OD_RAM.x2101_longitudeE7);
    SSD1306_SetCursor(0U, 3U);
    SSD1306_WriteString(line);
    (void)snprintf(line, sizeof(line), "AX:%d AY:%d", (int)OD_RAM.x2110_accelX, (int)OD_RAM.x2111_accelY);
    SSD1306_SetCursor(0U, 4U);
    SSD1306_WriteString(line);
    (void)snprintf(line, sizeof(line), "AZ:%d GX:%d", (int)OD_RAM.x2112_accelZ, (int)OD_RAM.x2113_gyroX);
    SSD1306_SetCursor(0U, 5U);
    SSD1306_WriteString(line);
    (void)snprintf(line, sizeof(line), "GY:%d GZ:%d", (int)OD_RAM.x2114_gyroY, (int)OD_RAM.x2115_gyroZ);
    SSD1306_SetCursor(0U, 6U);
    SSD1306_WriteString(line);
    (void)snprintf(line, sizeof(line), "B:%c C:%c A:%s", (snapshot.nodeBHealthy != 0U) ? 'Y' : 'N', (snapshot.nodeCHealthy != 0U) ? 'Y' : 'N', AccidentDetector_GetEventText(accidentSnapshot.eventType));
    SSD1306_SetCursor(0U, 7U);
    SSD1306_WriteString(line);
    }
    else
    {
      (void)snprintf(line, sizeof(line), "STATE:%s", VehicleAlarm_GetStateText(alarmSnapshot.state));
      SSD1306_SetCursor(0U, 0U);
      SSD1306_WriteString(line);
      (void)snprintf(line, sizeof(line), "IMU:%s", (snapshot.imuDataValid != 0U) ? "OK" : "FAIL");
      SSD1306_SetCursor(0U, 1U);
      SSD1306_WriteString(line);
      (void)snprintf(line, sizeof(line), "GNSS:%s", (snapshot.gnssDataValid != 0U) ? "OK" : "FAIL");
      SSD1306_SetCursor(0U, 2U);
      SSD1306_WriteString(line);
      (void)snprintf(line, sizeof(line), "NODE B:%s", (snapshot.nodeBHealthy != 0U) ? "OK" : "FAIL");
      SSD1306_SetCursor(0U, 3U);
      SSD1306_WriteString(line);
      (void)snprintf(line, sizeof(line), "NODE C:%s", (snapshot.nodeCHealthy != 0U) ? "OK" : "FAIL");
      SSD1306_SetCursor(0U, 4U);
      SSD1306_WriteString(line);
      (void)snprintf(line, sizeof(line), "MQTT:%s", (esp32Status.state == ESP32_STATE_MQTT_CONNECTED) ? "OK" : "OFF");
      SSD1306_SetCursor(0U, 5U);
      SSD1306_WriteString(line);
      (void)snprintf(line, sizeof(line), "AL:0x%02lX", (unsigned long)alarmBits);
      SSD1306_SetCursor(0U, 6U);
      SSD1306_WriteString(line);
      (void)snprintf(line, sizeof(line), "BTO:%03lu CTO:%03lu", (unsigned long)snapshot.heartbeatB.timeoutCount, (unsigned long)snapshot.heartbeatC.timeoutCount);
      SSD1306_SetCursor(0U, 7U);
      SSD1306_WriteString(line);
    }
    if (SSD1306_UpdateScreen() != HAL_OK)
    {
      oledStatus = HAL_ERROR;
    }
    vTaskDelay(pdMS_TO_TICKS(200U));
  }
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  ICM42688_RawData_t sample = {0};
  BH182_Data_t bh182Snapshot = {0};
  for(;;)
  {
    appHeartbeatTick = HAL_GetTick();
    ESP32_GetStatus(&esp32Status);
    CanOpenPort_Process(1000U);
    CanOpenPort_GetDiagnostics(&diagnostics);
#if (CAN_NODE_ROLE == CAN_NODE_ROLE_A)
    if ((HAL_GetTick() - accidentLastUpdateTick) >= 10U)
    {
      accidentLastUpdateTick = HAL_GetTick();
      AccidentDetector_Update(NULL, 0U);
      AccidentDetector_GetStatus(&accidentStatus);
    }
#endif
    if ((HAL_GetTick() - vehicleAlarmLastUpdateTick) >= 100U)
    {
      vehicleAlarmLastUpdateTick = HAL_GetTick();
      VehicleAlarm_Update((const CanOpenPortDiagnostics_t *)&diagnostics,
                          canBusOffActive,
                          (esp32Status.state == ESP32_STATE_MQTT_CONNECTED) ? 1U : 0U);
      VehicleAlarm_GetStatus(&vehicleAlarmStatus);
    }
    BH182_GetData(&bh182Snapshot);
    bh182Data = bh182Snapshot;
    CanOpenPort_UpdateGnss(&bh182Snapshot);
    if ((icm42688Status == HAL_OK) &&
        ((HAL_GetTick() - icm42688LastSampleTick) >= APP_IMU_SAMPLE_PERIOD_MS))
    {
      icm42688LastSampleTick = HAL_GetTick();
      if (ICM42688_ReadRaw(&sample) == HAL_OK)
      {
        icm42688RawData = sample;
        CanOpenPort_UpdateImu(&sample, 1U);
#if (CAN_NODE_ROLE == CAN_NODE_ROLE_B)
        AccidentDetector_Update(&sample, 1U);
        AccidentDetector_GetStatus(&accidentStatus);
        if ((accidentStatus.active != 0U) &&
            (accidentStatus.eventType != lastReportedAccidentEvent))
        {
          if (CanOpenPort_SendAccidentEvent(accidentStatus.eventType,
                                            accidentStatus.lastEventId,
                                            accidentStatus.peakAccelMg,
                                            accidentStatus.peakGyroDps) == CO_ERROR_NO)
          {
            lastReportedAccidentEvent = accidentStatus.eventType;
          }
        }
        if ((accidentStatus.active == 0U) &&
            (lastReportedAccidentEvent != ACCIDENT_EVENT_NONE))
        {
          CanOpenPort_ClearAccidentError(lastReportedAccidentEvent);
          lastReportedAccidentEvent = ACCIDENT_EVENT_NONE;
        }
#endif
      }
      else
      {
        icm42688Status = HAL_ERROR;
        CanOpenPort_UpdateImu(&sample, 0U);
#if (CAN_NODE_ROLE == CAN_NODE_ROLE_B)
        AccidentDetector_Update(NULL, 0U);
        AccidentDetector_GetStatus(&accidentStatus);
        if ((accidentStatus.active == 0U) &&
            (lastReportedAccidentEvent != ACCIDENT_EVENT_NONE))
        {
          CanOpenPort_ClearAccidentError(lastReportedAccidentEvent);
          lastReportedAccidentEvent = ACCIDENT_EVENT_NONE;
        }
#endif
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1U));
  }
  /* USER CODE END 5 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
