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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
/* 引入节点角色配置，保证 A/B/C 条件编译宏在本文件可用。 */
#include "canopen_node_config.h"
/* 引入 CANopen 适配层接口和 A 节点诊断数据。 */
#include "canopen_port.h"
/* 引入 ESP32-12F USART2 和 MQTT 任务接口。 */
#include "esp32_at.h"
/* 引入 MQTT 参数和看门狗周期配置。 */
#include "esp32_mqtt_config.h"
/* 引入 SSD1306 OLED 驱动接口。 */
#include "ssd1306.h"
/* 引入车辆统一告警状态机接口。 */
#include "vehicle_alarm.h"
/* 引入车辆碰撞和翻车检测接口。 */
#include "accident_detector.h"
/* 引入格式化字符串接口，用于生成 OLED 文本。 */
#include <stdio.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* 原生 FreeRTOS 头文件已在公共包含区域引入。 */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* 定义从 FDCAN 中断传递给接收任务的应用报文结构。 */
typedef struct
{
  /* 保存从硬件接收 FIFO 复制出的报文头信息。 */
  FDCAN_RxHeaderTypeDef header;
  /* 保存 Classic CAN 报文的最大 8 字节数据载荷。 */
  uint8_t data[8];
} CanRxMessage_t;

/* 保存单个节点的调度基线统计量。 */
typedef struct
{
  /* 保存该节点已经接收的有效报文数量。 */
  volatile uint32_t receivedCount;
  /* 保存该节点最近一次接收报文的序号。 */
  volatile uint32_t lastSequence;
  /* 保存根据序号跳变估算的丢帧数量。 */
  volatile uint32_t lostCount;
  /* 保存该节点最近一次接收时的本地节拍。 */
  volatile uint32_t lastReceiveTick;
  /* 保存该节点最近两帧之间的实际接收间隔。 */
  volatile uint32_t interArrivalMs;
  /* 保存该节点最近一次接收间隔的绝对抖动。 */
  volatile uint32_t jitterMs;
  /* 保存该节点观测到的最大接收间隔。 */
  volatile uint32_t maxInterArrivalMs;
  /* 保存该节点观测到的最大周期抖动。 */
  volatile uint32_t maxJitterMs;
  /* 保存报文中的发送端节拍字段，供后续时钟校准使用。 */
  volatile uint32_t senderTick;
  /* 保存接收端记录报文时的本地节拍。 */
  volatile uint32_t receiveTick;
  /* 标记是否已经收到该节点的第一帧报文。 */
  volatile uint8_t initialized;
} CanNodeStats_t; /* 定义节点统计结构体类型名称。 */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* 定义节点 A 的标准 CAN 报文 ID。 */
#define CAN_ID_NODE_A 0x100U /* 节点 A 的 ID 最小，因此具有最高仲裁优先级。 */
/* 定义节点 B 的标准 CAN 报文 ID。 */
#define CAN_ID_NODE_B 0x300U /* 节点 B 的仲裁优先级低于节点 A。 */
/* 定义节点 C 的标准 CAN 报文 ID。 */
#define CAN_ID_NODE_C 0x500U /* 节点 C 的仲裁优先级最低。 */
/* 定义时间同步报文的标准 CAN ID。 */
#define CAN_ID_TIME_SYNC 0x080U /* 同步报文 ID 低于业务报文，具有更高仲裁优先级。 */
/* 定义节点 A 的应用层数据标识。 */
#define CAN_TAG_NODE_A 0xA1U /* 节点 A 在数据字节 4 中写入此标识。 */
/* 定义节点 B 的应用层数据标识。 */
#define CAN_TAG_NODE_B 0xB1U /* 节点 B 在数据字节 4 中写入此标识。 */
/* 定义节点 C 的应用层数据标识。 */
#define CAN_TAG_NODE_C 0xC1U /* 节点 C 在数据字节 4 中写入此标识。 */
/* 定义时间同步报文的数据标识。 */
#define CAN_TAG_TIME_SYNC 0x5AU /* 接收端使用该标识识别时间同步报文。 */

/* 定义节点 A 的固定周期，供调度基线统计使用。 */
#define CAN_NODE_A_PERIOD_MS 10U /* 节点 A 的理论发送周期为 10 ms。 */
/* 定义节点 B 的固定周期，供调度基线统计使用。 */
#define CAN_NODE_B_PERIOD_MS 20U /* 节点 B 的理论发送周期为 20 ms。 */
/* 定义节点 C 的固定周期，供调度基线统计使用。 */
#define CAN_NODE_C_PERIOD_MS 100U /* 节点 C 的理论发送周期为 100 ms。 */
/* 定义时间同步报文的发送周期。 */
#define CAN_TIME_SYNC_PERIOD_MS 100U /* 节点 A 每 100 ms 发布一次主时钟。 */
/* 禁用旧版自定义时间同步帧，为 CANopen SYNC 保留 COB-ID 0x080。 */
#define CAN_LEGACY_TIME_SYNC_ENABLE 0U /* CANopen 工程不再发送自定义时间戳同步帧。 */
/* 定义发送时间戳在 CAN FD 数据区中的起始字节位置。 */
#define CAN_TIMESTAMP_OFFSET 8U /* 发送时间戳从数据字节 8 开始存放。 */
/* 定义固定优先级调度基线模式编号。 */
#define CAN_SCHED_MODE_FIXED_PRIORITY 0U /* 使用固定 CAN ID 和固定发送周期。 */
/* 选择当前编译固件使用固定优先级调度基线。 */
#define CAN_SCHED_MODE CAN_SCHED_MODE_FIXED_PRIORITY /* 后续策略对比时可扩展此宏。 */

/* 根据当前角色选择本机的发送参数。 */
#if (CAN_NODE_ROLE == CAN_NODE_ROLE_A) /* 判断是否编译节点 A 固件。 */
#define CAN_LOCAL_TX_ID CAN_ID_NODE_A /* 节点 A 使用最高优先级的 0x100。 */
#define CAN_LOCAL_NODE_TAG CAN_TAG_NODE_A /* 节点 A 写入自己的应用层标识。 */
#define CAN_LOCAL_TX_PERIOD_MS CAN_NODE_A_PERIOD_MS /* 节点 A 每 10 ms 发送一帧控制报文。 */
#elif (CAN_NODE_ROLE == CAN_NODE_ROLE_B) /* 判断是否编译节点 B 固件。 */
#define CAN_LOCAL_TX_ID CAN_ID_NODE_B /* 节点 B 使用中优先级的 0x300。 */
#define CAN_LOCAL_NODE_TAG CAN_TAG_NODE_B /* 节点 B 写入自己的应用层标识。 */
#define CAN_LOCAL_TX_PERIOD_MS CAN_NODE_B_PERIOD_MS /* 节点 B 每 20 ms 发送一帧状态报文。 */
#elif (CAN_NODE_ROLE == CAN_NODE_ROLE_C) /* 判断是否编译节点 C 固件。 */
#define CAN_LOCAL_TX_ID CAN_ID_NODE_C /* 节点 C 使用低优先级的 0x500。 */
#define CAN_LOCAL_NODE_TAG CAN_TAG_NODE_C /* 节点 C 写入自己的应用层标识。 */
#define CAN_LOCAL_TX_PERIOD_MS CAN_NODE_C_PERIOD_MS /* 节点 C 每 100 ms 发送一帧监测报文。 */
#else /* 当前角色编号不在允许范围内。 */
#error "CAN_NODE_ROLE 必须设置为 CAN_NODE_ROLE_A、CAN_NODE_ROLE_B 或 CAN_NODE_ROLE_C" /* 阻止生成角色配置错误的固件。 */
#endif /* 结束本机发送参数选择。 */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* 将 NUCLEO-G474RE 用户 LED 指定为 PA5，用于指示远端报文接收结果。 */
#define APP_LED_GPIO_Port GPIOA /* 定义用户 LED 所连接的 GPIO 端口。 */
/* 指定 NUCLEO-G474RE 用户 LED 的引脚号。 */
#define APP_LED_Pin GPIO_PIN_5 /* 定义用户 LED 所连接的 GPIO 引脚。 */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
FDCAN_HandleTypeDef hfdcan1;

/* 保存 SSD1306 使用的 I2C1 外设句柄。 */
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
/* 保存独立看门狗外设句柄。 */
IWDG_HandleTypeDef hiwdg;
/* 保存 ESP32-12F 的运行状态快照，供 Keil Watch 观察。 */
volatile ESP32_Status_t esp32Status;
/* 保存车辆统一告警状态快照，供 Keil Watch 观察。 */
volatile VehicleAlarmStatus_t vehicleAlarmStatus;
  /* 保存车辆意外检测状态，供 Keil Watch 和 MQTT 读取。 */
  volatile AccidentDetectorStatus_t accidentStatus;
  /* 保存事故事件掉电恢复快照，供 B 节点启动后继续重发。 */
  static ESP32_AccidentPersist_t accidentPersistSnapshot;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* USER CODE BEGIN PV */

/* 保存 RX 回调和接收任务使用的 FreeRTOS 消息队列句柄。 */
static QueueHandle_t canRxQueue;
/* 统计成功接收的远端报文数量，供调试器观察。 */
static volatile uint32_t rxCount;
/* 保存 CANopen 诊断快照，供 Keil Watch/Live Expressions 实时观察。 */
volatile CanOpenPortDiagnostics_t diagnostics;
/* 保存 ICM-42688 初始化结果，供调试器观察。 */
volatile HAL_StatusTypeDef icm42688Status;
/* 保存 ICM-42688 WHO_AM_I 返回值，预期为 0x47。 */
volatile uint8_t icm42688WhoAmI;
/* 保存最近一次 ICM-42688 六轴原始采样值。 */
  volatile ICM42688_RawData_t icm42688RawData;
  /* 保存 BH-182 最近一次定位数据，供 Keil Watch 观察。 */
  volatile BH182_Data_t bh182Data;
  /* 保存 BH-182 驱动初始化和 DMA 接收启动结果。 */
  volatile HAL_StatusTypeDef bh182Status;
/* 保存上一次 ICM-42688 采样时刻。 */
static uint32_t icm42688LastSampleTick;
/* B 节点使用 1 ms 周期进行本地高速事故检测，A/C 保持 10 ms 兼容周期。 */
#if (CAN_NODE_ROLE == CAN_NODE_ROLE_B)
#define APP_IMU_SAMPLE_PERIOD_MS 1U
#else
#define APP_IMU_SAMPLE_PERIOD_MS 10U
#endif
/* 保存 A 节点意外检测的最近更新时间。 */
static uint32_t accidentLastUpdateTick;
/* 保存上一次已经触发即时上报的意外类型。 */
static uint8_t lastReportedAccidentEvent;
/* 保存最近一次接收到的 4 字节序号。 */
static volatile uint32_t rxSequence;
/* 统计节点 A 报文的有效接收数量。 */
static volatile uint32_t rxCountA;
/* 保存节点 A 最近一次有效报文的序号。 */
static volatile uint32_t rxSequenceA;
/* 统计节点 B 报文的有效接收数量。 */
static volatile uint32_t rxCountB;
/* 保存节点 B 最近一次有效报文的序号。 */
static volatile uint32_t rxSequenceB;
/* 统计节点 C 报文的有效接收数量。 */
static volatile uint32_t rxCountC;
/* 保存节点 C 最近一次有效报文的序号。 */
static volatile uint32_t rxSequenceC;
/* 统计应用队列已满导致丢弃的报文数量。 */
static volatile uint32_t rxQueueDropCount;
/* 保存节点 A 的固定优先级调度基线统计量。 */
static volatile CanNodeStats_t rxStatsA;
/* 保存节点 B 的固定优先级调度基线统计量。 */
static volatile CanNodeStats_t rxStatsB;
/* 保存节点 C 的固定优先级调度基线统计量。 */
static volatile CanNodeStats_t rxStatsC;
/* 保存本节点相对于 A 主时钟的本地节拍偏移量。 */
static volatile int32_t canTimeOffsetMs;
/* 统计已经接收到的时间同步报文数量。 */
static volatile uint32_t canSyncRxCount;
/* 保存最近一次同步报文中的 A 主时钟节拍。 */
static volatile uint32_t canMasterTick;
/* 保存最近一次同步报文到达本节点中断时的本地节拍。 */
static volatile uint32_t canSyncLocalTick;
/* 保存最近一次同步报文计算出的偏移绝对值，不能单独视为传输延迟。 */
static volatile uint32_t canSyncOffsetAbsMs;
/* 保存周期性 CAN 发送任务的 FreeRTOS 任务句柄。 */
static TaskHandle_t canTxTaskHandle;
/* 保存 CAN 接收任务的 FreeRTOS 任务句柄。 */
static TaskHandle_t canRxTaskHandle;

/* 保存 CAN 故障恢复任务的 FreeRTOS 任务句柄。 */
static TaskHandle_t canRecoveryTaskHandle;
/* 保存 A 节点 ESP32 MQTT 任务句柄。 */
static TaskHandle_t esp32MqttTaskHandle;
/* 保存应用任务最近一次运行节拍，供看门狗监控任务检查。 */
static volatile uint32_t appHeartbeatTick;
/* 保存上一次运行统一告警状态机的系统节拍。 */
static uint32_t vehicleAlarmLastUpdateTick;
/* 保存 OLED 初始化结果，供调试器观察。 */
volatile HAL_StatusTypeDef oledStatus;
/* 保存 OLED 显示任务句柄。 */
static TaskHandle_t oledTaskHandle;
/* 统计 FDCAN 错误回调被触发的次数。 */
static volatile uint32_t canErrorCallbackCount;
/* 统计 Error Warning 状态变化次数。 */
static volatile uint32_t canErrorWarningCount;
/* 统计 Error Passive 状态变化次数。 */
static volatile uint32_t canErrorPassiveCount;
/* 统计 Bus-Off 状态变化次数。 */
static volatile uint32_t canBusOffCount;
/* 标记 FDCAN 是否仍处于未恢复的 Bus-Off 状态。 */
static volatile uint8_t canBusOffActive;
/* 统计 FDCAN 自动恢复成功次数。 */
static volatile uint32_t canRecoveryCount;
/* 统计 FDCAN 自动恢复失败次数。 */
static volatile uint32_t canRecoveryFailCount;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_FDCAN1_Init(void);
/* 声明 I2C1 初始化函数。 */
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* 声明独立看门狗初始化函数。 */
static void MX_IWDG_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */

/* 声明周期性 CAN 发送任务。 */
static void CanTxTask(void *argument);
/* 声明阻塞式 CAN 接收任务。 */
static void CanRxTask(void *argument);
/* 声明负责 Bus-Off 自动恢复的 FreeRTOS 任务。 */
static void CanRecoveryTask(void *argument);
/* 声明独立看门狗监控任务。 */
static void IwdgMonitorTask(void *argument);
/* 声明 A 节点 OLED 仪表盘任务。 */
static void OledDashboardTask(void *argument);
/* 声明用于解码 4 字节小端序号的辅助函数。 */
static uint32_t CanDecodeSequence(const uint8_t *data);
/* 声明用于解码数据区发送时间戳的辅助函数。 */
static uint32_t CanDecodeTimestamp(const uint8_t *data);
/* 声明用于更新单个节点调度统计量的辅助函数。 */
static void CanUpdateNodeStats(volatile CanNodeStats_t *stats, /* 传入待更新的节点统计对象。 */
                               uint32_t receivedSequence, /* 传入当前报文的应用层序号。 */
                               uint32_t senderTick, /* 传入当前报文的发送端节拍。 */
                               uint32_t expectedPeriodMs); /* 传入该节点的理论周期。 */
/* 声明返回校准到 A 主时钟的逻辑时间函数。 */
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

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* 初始化独立看门狗，初始化成功后硬件开始倒计时。 */
  MX_IWDG_Init();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_FDCAN1_Init();
  /* 初始化 OLED 使用的硬件 I2C1。 */
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  /* 初始化 ICM-42688，并检查 SPI 通信是否返回预期设备标识。 */
  icm42688Status = ICM42688_Init(&hspi2, GPIOB, GPIO_PIN_12);
  /* 仅在初始化成功时读取 WHO_AM_I，避免传感器未接好时反复访问 SPI。 */
  if (icm42688Status == HAL_OK)
  {
    /* 保存 ICM-42688 的设备标识，预期值为 0x47。 */
    icm42688Status = ICM42688_ReadWhoAmI((uint8_t *)&icm42688WhoAmI);
  }
  /* 初始化 BH-182 UART DMA 接收驱动。 */
  if (BH182_Init(&huart1) != HAL_OK)
  {
    /* 驱动初始化失败时保留 CANopen 运行并记录为空闲状态。 */
    bh182Status = HAL_ERROR;
    memset((void *)&bh182Data, 0, sizeof(bh182Data));
  }
  else
  {
    /* 启动 BH-182 的 DMA 空闲中断接收。 */
    bh182Status = BH182_Start();
  }
  /* 初始化 ESP32-12F 的 USART2 AT 驱动。 */
  if (ESP32_Init(&huart2) != HAL_OK)
  {
    /* 驱动句柄异常时保留 CANopen 运行，并标记 ESP32 离线。 */
    esp32Status.state = ESP32_STATE_OFFLINE;
  }
  else
  {
    /* 启动 ESP32 USART2 的单字节中断接收。 */
    if (ESP32_StartReceive() != HAL_OK)
    {
      /* 接收启动失败时标记 ESP32 离线，不影响 CANopen 启动。 */
      esp32Status.state = ESP32_STATE_OFFLINE;
    }
  }
  /* 初始化 0.96 寸 128x64 SSD1306 OLED，常见模块地址为 0x3C。 */
  oledStatus = SSD1306_Init(&hi2c1, (uint16_t)(0x3CU << 1U));
  /* 初始化车辆统一告警状态机。 */
  VehicleAlarm_Init();
  /* 初始化车辆碰撞和翻车检测状态。 */
  AccidentDetector_Init();
  /* 读取事故事件编号和待确认状态，Flash 无有效记录时使用全新状态。 */
  if (ESP32_Persist_LoadAccident(&accidentPersistSnapshot) == HAL_OK)
  {
    /* 恢复 B 节点事件编号生成器，避免重启后产生重复编号。 */
    AccidentDetector_RestoreLastEventId(accidentPersistSnapshot.lastEventId);
  }
////////////////////////////////////////////配置过滤器并启动 FDCAN1/////////////////////////////////////////
  /* 声明一个用于双节点实验的标准 ID 掩码过滤器。（初始化为默认值） */
  FDCAN_FilterTypeDef canFilter = {0};
  /* 为过滤器选择 11 位标准标识符。 */
  canFilter.IdType = FDCAN_STANDARD_ID;
  /* 使用已配置的第一个标准过滤器槽位。 */
  canFilter.FilterIndex = 0U;
  /* 使用掩码比较方式匹配标识符。 */
  canFilter.FilterType = FDCAN_FILTER_MASK;
  /* 将通过过滤的报文路由到 RX FIFO0。 */
  canFilter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  /* 将过滤器标识符设为 0，配合全 0 掩码接收所有标识符。 */
  canFilter.FilterID1 = 0x000U;
  /* 将掩码设为 0，使所有标准标识符都能通过。 */
  canFilter.FilterID2 = 0x000U;
  /* 在启动 FDCAN 外设前安装标准 ID 过滤器。（通知硬件） */
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &canFilter) != HAL_OK)
  {
    /* 如果过滤器配置失败，则进入生成的错误处理函数。 */
    Error_Handler();
  }
  /* 调试阶段将未匹配的标准帧和扩展帧路由到 RX FIFO0。 */
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, /* 配置 FDCAN1 的全局过滤器。 */
                                   FDCAN_ACCEPT_IN_RX_FIFO0, /* 将未匹配的标准帧路由到 RX FIFO0。 */
                                   FDCAN_ACCEPT_IN_RX_FIFO0, /* 将未匹配的扩展帧路由到 RX FIFO0。 */
                                   FDCAN_REJECT_REMOTE, /* 拒绝未匹配的标准远程帧。 */
                                   FDCAN_REJECT_REMOTE) != HAL_OK) /* 拒绝未匹配的扩展远程帧。 */
  {
    /* 如果全局过滤器配置失败，则进入生成的错误处理函数。 */
    Error_Handler();
  }
  /* 启动已配置的 FDCAN1 外部总线控制器。 */
  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  {
    /* 如果 FDCAN 启动失败，则进入生成的错误处理函数。 */
    Error_Handler();
  }
  /* 使用当前 Node-ID 初始化 CANopenNode Classic CAN 底层模块。 */
  if (CanOpenPort_StackInit(CANOPEN_NODE_ID) != CO_ERROR_NO)
  {
    /* CANopenNode 底层初始化失败时进入统一错误处理。 */
    Error_Handler();
  }
  /* 使能 RX FIFO0 收到新报文时产生的中断。 */
  if (HAL_FDCAN_ActivateNotification(&hfdcan1, /* 为 FDCAN1 开启通知功能。 */
                                     FDCAN_IT_RX_FIFO0_NEW_MESSAGE | /* 开启 RX FIFO0 新报文中断。 */
                                     FDCAN_IT_ERROR_WARNING | /* 开启 Error Warning 状态变化中断。 */
                                     FDCAN_IT_ERROR_PASSIVE | /* 开启 Error Passive 状态变化中断。 */
                                     FDCAN_IT_BUS_OFF, /* 开启 Bus-Off 状态变化中断。 */
                                     0U) != HAL_OK) /* 不开启 TX 缓冲区中断掩码。 */
  {
    /* 如果通知配置失败，则进入生成的错误处理函数。 */
    Error_Handler();
  }

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* 使用 FreeRTOS 动态内存创建 8 个报文槽位的消息队列。 */
  canRxQueue = xQueueCreate(8U, sizeof(CanRxMessage_t));
  /* 在任务使用队列前确认队列分配成功。 */
  if (canRxQueue == NULL)
  {
    /* 如果队列分配失败，则进入生成的错误处理函数。 */
    Error_Handler();
  }
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* 使用原生 FreeRTOS 创建周期性 CAN 发送任务。 */
  if (xTaskCreate(CanTxTask, /* 指定发送任务入口函数。 */
                  "canTxTask", /* 指定发送任务名称。 */
                  256U, /* 为发送任务分配 256 个 FreeRTOS 栈字。 */
                  NULL, /* 发送任务不接收任务参数。 */
                  tskIDLE_PRIORITY + 2U, /* 将发送任务设置为普通优先级。 */
                  &canTxTaskHandle) != pdPASS) /* 检查发送任务创建结果。 */
  {
    /* 如果发送任务创建失败，则进入生成的错误处理函数。 */
    Error_Handler();
  }
  /* 使用原生 FreeRTOS 创建阻塞式 CAN 接收任务。 */
  if (xTaskCreate(CanRxTask, /* 指定接收任务入口函数。 */
                  "canRxTask", /* 指定接收任务名称。 */
                  256U, /* 为接收任务分配 256 个 FreeRTOS 栈字。 */
                  NULL, /* 接收任务不接收任务参数。 */
                  tskIDLE_PRIORITY + 3U, /* 将接收任务设置为高于发送任务。 */
                  &canRxTaskHandle) != pdPASS) /* 检查接收任务创建结果。 */
  {
    /* 如果接收任务创建失败，则进入生成的错误处理函数。 */
    Error_Handler();
  }
  /* 创建 CAN 故障恢复任务，由任务上下文执行 FDCAN 停止和重启。 */
  if (xTaskCreate(CanRecoveryTask, /* 指定 CAN 故障恢复任务入口函数。 */
                  "canRecoveryTask", /* 指定 CAN 故障恢复任务名称。 */
                  256U, /* 为恢复任务分配 256 个 FreeRTOS 栈字。 */
                  NULL, /* 恢复任务不接收任务参数。 */
                  tskIDLE_PRIORITY + 4U, /* 设置恢复任务优先级高于收发任务。 */
                  &canRecoveryTaskHandle) != pdPASS) /* 检查恢复任务创建结果。 */
  {
    /* 如果恢复任务创建失败，则进入错误处理函数。 */
    Error_Handler();
  }
  /* 将掉电恢复的待确认事故事件交给 B 节点 CANopen 重发状态机。 */
  CanOpenPort_RestoreAccidentState(&accidentPersistSnapshot);
  /* 创建高优先级看门狗监控任务。 */
  if (xTaskCreate(IwdgMonitorTask, /* 指定看门狗监控任务入口函数。 */
                  "iwdgMonitor", /* 指定看门狗任务名称。 */
                  128U, /* 为看门狗任务分配最小运行栈。 */
                  NULL, /* 看门狗任务不接收任务参数。 */
                  tskIDLE_PRIORITY + 5U, /* 设置高于应用任务的优先级。 */
                  NULL) != pdPASS) /* 检查看门狗任务创建结果。 */
  {
    /* 看门狗任务创建失败时进入错误处理。 */
    Error_Handler();
  }
  /* 仅在 A 节点创建负责云端上报的 ESP32 MQTT 任务。 */
#if (CAN_NODE_ROLE == CAN_NODE_ROLE_A)
  /* 创建 A 节点 OLED 仪表盘任务。 */
  if (xTaskCreate(OledDashboardTask, /* 指定 OLED 显示任务入口函数。 */
                  "oledTask", /* 指定 OLED 任务名称。 */
                  384U, /* 为格式化字符串和显示任务分配栈空间。 */
                  NULL, /* OLED 任务不接收任务参数。 */
                  tskIDLE_PRIORITY + 1U, /* 设置低于 CAN 接收任务的优先级。 */
                  &oledTaskHandle) != pdPASS) /* 检查 OLED 任务创建结果。 */
  {
    /* OLED 任务创建失败时保留 CANopen 和 MQTT 运行。 */
    oledStatus = HAL_ERROR;
  }
  /* 创建 ESP32 网络状态机任务。 */
  if (xTaskCreate(ESP32_MqttTask, /* 指定 ESP32 MQTT 任务入口函数。 */
                  "esp32Mqtt", /* 指定 ESP32 MQTT 任务名称。 */
                  512U, /* 为 JSON 和 AT 命令缓存分配 512 个 FreeRTOS 栈字。 */
                  NULL, /* MQTT 任务不接收任务参数。 */
                  tskIDLE_PRIORITY + 1U, /* 设置低于 CAN 接收任务的普通优先级。 */
                  &esp32MqttTaskHandle) != pdPASS) /* 检查 MQTT 任务创建结果。 */
  {
    /* MQTT 任务创建失败时记录错误，但不停止本地 CANopen。 */
    esp32Status.state = ESP32_STATE_ERROR;
  }
#endif
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
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
  /* SPI2 parameter configuration*/
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

/* 初始化独立看门狗，使用约 8 秒的超时时间。 */
static void MX_IWDG_Init(void)
{
  /* 选择独立看门狗外设实例。 */
  hiwdg.Instance = IWDG;
  /* 设置 LSI 分频系数为 64。 */
  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
  /* 按 32 kHz LSI 计算约 8 秒重载值。 */
  hiwdg.Init.Reload = 3999U;
  /* 禁止窗口模式，允许在整个周期内刷新。 */
  hiwdg.Init.Window = IWDG_WINDOW_DISABLE;
  /* 启动独立看门狗。 */
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    /* 看门狗初始化失败时进入统一错误处理。 */
    Error_Handler();
  }
}

/* 初始化 I2C1，使用 100 kHz 标准模式驱动 SSD1306。 */
static void MX_I2C1_Init(void)
{
  /* 选择 I2C1 外设实例。 */
  hi2c1.Instance = I2C1;
  /* 设置 16 MHz PCLK1 下的 100 kHz I2C 时序。 */
  hi2c1.Init.Timing = 0x00303D5BU;
  /* 使用 7 位从机地址模式。 */
  hi2c1.Init.OwnAddress1 = 0U;
  /* 关闭双地址模式。 */
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  /* 不使用第二个从机地址。 */
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  /* 将第二地址清零。 */
  hi2c1.Init.OwnAddress2 = 0U;
  /* 禁止地址掩码。 */
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  /* 关闭通用呼叫响应。 */
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  /* 关闭时钟拉伸。 */
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  /* 初始化 I2C1。 */
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    /* I2C 初始化失败时进入统一错误处理。 */
    Error_Handler();
  }
  /* 配置模拟滤波器。 */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    /* 模拟滤波器配置失败时进入统一错误处理。 */
    Error_Handler();
  }
}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* 声明用户 LED 的 GPIO 初始化结构体。 */
  GPIO_InitTypeDef gpioInit = {0};
  /* 声明 ICM-42688 片选引脚的 GPIO 配置结构体。 */
  GPIO_InitTypeDef icmCsInit = {0};
  /* 在启用输出模式前先将用户 LED 拉低。 */
  HAL_GPIO_WritePin(APP_LED_GPIO_Port, APP_LED_Pin, GPIO_PIN_RESET);
  /* 选择用户 LED 引脚为推挽输出。 */
  gpioInit.Pin = APP_LED_Pin;
  /* 使用推挽输出驱动用户 LED。 */
  gpioInit.Mode = GPIO_MODE_OUTPUT_PP;
  /* 不为 LED 引脚启用内部上拉或下拉。 */
  gpioInit.Pull = GPIO_NOPULL;
  /* LED 仅用于状态指示，因此使用低速输出。 */
  gpioInit.Speed = GPIO_SPEED_FREQ_LOW;
  /* 将用户 LED 配置应用到 GPIOA。 */
  HAL_GPIO_Init(APP_LED_GPIO_Port, &gpioInit);

  /* 声明 ICM-42688 片选引脚的 GPIO 配置结构体。 */
  /* 初始化时将 ICM-42688 片选保持为高电平。 */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
  /* 选择 PB12 作为 ICM-42688 片选引脚。 */
  icmCsInit.Pin = GPIO_PIN_12;
  /* 使用推挽输出模式驱动片选。 */
  icmCsInit.Mode = GPIO_MODE_OUTPUT_PP;
  /* 不启用内部上下拉。 */
  icmCsInit.Pull = GPIO_NOPULL;
  /* 使用高速 GPIO，保证片选边沿稳定。 */
  icmCsInit.Speed = GPIO_SPEED_FREQ_HIGH;
  /* 初始化 ICM-42688 片选 GPIO。 */
  HAL_GPIO_Init(GPIOB, &icmCsInit);

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* 接收一帧 FDCAN FIFO0 报文，并转发到 RTOS 接收队列。 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t rxFifo0Interrupts)
{
  /* 分配本地报文对象，供中断复制硬件报文。 */
  CanRxMessage_t message = {0};
  /* 保存同步报文到达中断时的本地节拍。 */
  uint32_t syncReceiveTick = 0U;
  /* 只处理 FDCAN1 的 RX FIFO0 新报文通知。 */
  if ((hfdcan == &hfdcan1) && /* 只接受来自 FDCAN1 实例的回调。 */
      ((rxFifo0Interrupts & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0U)) /* 只接受 RX FIFO0 新报文事件。 */
  {
    /* 将报文头和数据载荷从硬件 RX FIFO 复制出来。（读取硬件，不阻塞） */
    if (HAL_FDCAN_GetRxMessage(hfdcan, /* 从触发回调的 FDCAN 实例读取报文。 */
                               FDCAN_RX_FIFO0, /* 从 RX FIFO0 读取待处理报文。 */
                               &message.header, /* 将接收报文头保存到本地报文对象。 */
                               message.data) == HAL_OK) /* 将接收数据保存到本地报文对象。 */
    {
      /* 在中断中立即记录同步报文的接收时刻，避免队列延迟影响偏移估计。 */
      /* 将硬件帧交给 CANopenNode，触发协议对象的接收回调。 */
      CanOpenPort_RxInterrupt(message.header.Identifier,
                              (uint8_t)(message.header.DataLength >> 16U),
                              message.data);
      syncReceiveTick = HAL_GetTick();
      /* 仅在显式启用旧版同步实验时解析自定义时间同步帧。 */
#if (CAN_LEGACY_TIME_SYNC_ENABLE == 1U)
      /* 判断当前报文是否为标准格式的时间同步报文。 */
      if ((message.header.Identifier == CAN_ID_TIME_SYNC) && /* 检查同步报文 ID。 */
          (message.header.IdType == FDCAN_STANDARD_ID) && /* 检查同步报文使用标准 ID。 */
          (message.header.DataLength == FDCAN_DLC_BYTES_8) && /* 检查同步报文为 8 字节 Classic CAN 帧。 */
          (message.header.FDFormat == FDCAN_CLASSIC_CAN) && /* 检查同步报文为 Classic CAN 格式。 */
          (message.header.BitRateSwitch == FDCAN_BRS_OFF) && /* 检查同步报文关闭 BRS。 */
          (message.data[4] == CAN_TAG_TIME_SYNC)) /* 检查同步报文数据标识。 */
      {
#if (CAN_NODE_ROLE != CAN_NODE_ROLE_A) /* 仅 B、C 节点需要计算相对 A 的时钟偏移。 */
        /* B、C 节点解码 A 节点发布的主时钟节拍。 */
        /* 解码 A 节点发布的主时钟节拍。 */
        uint32_t receivedMasterTick = CanDecodeTimestamp(message.data);
        /* 保存最近一次主时钟节拍。 */
        canMasterTick = receivedMasterTick;
        /* 保存同步报文到达中断时的本地节拍。 */
        canSyncLocalTick = syncReceiveTick;
        /* 计算本地时钟相对于主时钟的带符号偏移量。 */
        canTimeOffsetMs = (int32_t)(syncReceiveTick - receivedMasterTick);
        /* 记录本地与主时钟偏移绝对值的毫秒估计值。 */
        canSyncOffsetAbsMs = (uint32_t)(canTimeOffsetMs >= 0 ? canTimeOffsetMs : -canTimeOffsetMs);
        /* 累加同步报文接收次数。 */
        canSyncRxCount++;
#endif /* 结束 B、C 节点时间偏移计算。 */
        /* 同步报文不进入业务报文队列，直接结束本次中断处理。 */
        return;
      }
#endif /* 结束旧版自定义时间同步帧解析。 */
      /* 保存可能被唤醒的高优先级任务状态。 */
      BaseType_t xHigherPriorityTaskWoken = pdFALSE;
      /* 不阻塞中断回调，将报文复制到原生 FreeRTOS 队列。（写队列，不阻塞） */
      if (xQueueSendToBackFromISR(canRxQueue, /* 指定接收消息队列。 */
                                  &message, /* 将报文头和数据复制到队列。 */
                                  &xHigherPriorityTaskWoken) != pdPASS) /* 在中断中不等待队列空间。（xHigherPriorityTaskWoken输出参数） */
      {
        /* 统计队列已满事件，供后续溢出分析。 */
        rxQueueDropCount++;
      }
      /* 如果接收任务被唤醒，则在退出中断前请求任务切换。（在 ISR 退出时立即切换到该高优先级任务，而不是等到下一个 SysTick） */
      portYIELD_FROM_ISR(xHigherPriorityTaskWoken); /* （用xHigherPriorityTaskWoken请求上下文切换） */
    }
  }
}

/* 周期性发送当前节点的三节点实线通信报文。 */
/* 处理 FDCAN Error Warning、Error Passive 和 Bus-Off 状态变化中断。 */
/* 处理 USART2 单字节接收完成回调，将 ESP32 字节交给环形缓冲。 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* 仅处理 ESP32 使用的 USART2 接收回调。 */
  if (huart == &huart2)
  {
    /* 将驱动内部接收字节交给 ESP32 环形缓冲。 */
    /* 通过驱动导出的回调入口继续接收。 */
    ESP32_OnRxComplete();
  }
}

/* 处理 USART 错误并让 ESP32 驱动重新装载接收。 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  /* 仅处理 ESP32 使用的 USART2 错误。 */
  if (huart == &huart2)
  {
    /* 通知 ESP32 驱动重新启动接收。 */
    ESP32_OnUartError();
  }
}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
  /* 只处理当前工程使用的 FDCAN1 实例。 */
  if ((hfdcan == &hfdcan1) && (ErrorStatusITs != 0U))
  {
    /* 保存可能被恢复任务唤醒的高优先级任务状态。 */
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    /* 统计 Error Warning 状态变化事件。 */
    if ((ErrorStatusITs & FDCAN_IT_ERROR_WARNING) != 0U)
    {
      /* 在中断上下文中递增 Error Warning 计数器。 */
      canErrorWarningCount++;
    }
    /* 统计 Error Passive 状态变化事件。 */
    if ((ErrorStatusITs & FDCAN_IT_ERROR_PASSIVE) != 0U)
    {
      /* 在中断上下文中递增 Error Passive 计数器。 */
      canErrorPassiveCount++;
    }
    /* 检测 Bus-Off 状态变化并请求恢复任务处理。 */
    if ((ErrorStatusITs & FDCAN_IT_BUS_OFF) != 0U)
    {
      /* 在中断上下文中递增 Bus-Off 计数器。 */
      canBusOffCount++;
      /* 标记 FDCAN 已进入尚未恢复的 Bus-Off 状态。 */
      canBusOffActive = 1U;
      /* 确认恢复任务已经创建后再发送任务通知。 */
      if (canRecoveryTaskHandle != NULL)
      {
        /* 使用 ISR 安全接口唤醒恢复任务。 */
        vTaskNotifyGiveFromISR(canRecoveryTaskHandle, &xHigherPriorityTaskWoken);
        /* 如果恢复任务优先级更高，则在退出中断时立即切换任务。 */
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
      }
    }
  }
}

/* 处理未归类 FDCAN 协议错误和硬件错误中断。 */
void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan)
{
  /* 只统计当前工程使用的 FDCAN1 实例错误。 */
  if (hfdcan == &hfdcan1)
  {
    /* 在中断上下文中递增通用错误回调计数器。 */
    canErrorCallbackCount++;
  }
}

/* 在 FreeRTOS 任务上下文中执行 FDCAN Bus-Off 自动恢复。 */
static void CanRecoveryTask(void *argument)
{
  /* 标记恢复任务参数当前未使用。 */
  (void)argument;
  /* 任务持续等待 Bus-Off 中断发送的通知。 */
  for (;;)
  {
    /* 阻塞等待恢复请求并在取出后清零通知计数。 */
    (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    /* 等待总线错误状态稳定，避免立即重启造成重复错误。 */
    vTaskDelay(pdMS_TO_TICKS(100U));
    /* 先停止处于 Bus-Off 状态的 FDCAN 控制器。 */
    if (HAL_FDCAN_Stop(&hfdcan1) == HAL_OK)
    {
      /* 为收发器和控制器留出短暂恢复时间。 */
      vTaskDelay(pdMS_TO_TICKS(10U));
      /* 重新启动 FDCAN 控制器。 */
      if (HAL_FDCAN_Start(&hfdcan1) == HAL_OK)
      {
        /* 重新开启接收和错误状态通知。 */
        if (HAL_FDCAN_ActivateNotification(&hfdcan1,
                                           FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
                                           FDCAN_IT_ERROR_WARNING |
                                           FDCAN_IT_ERROR_PASSIVE |
                                           FDCAN_IT_BUS_OFF,
                                           0U) == HAL_OK)
        {
          /* 记录一次 FDCAN 自动恢复成功。 */
          canRecoveryCount++;
          /* 清除 FDCAN Bus-Off 活动标记。 */
          canBusOffActive = 0U;
        }
        else
        {
          /* 记录重新开启通知失败。 */
          canRecoveryFailCount++;
        }
      }
      else
      {
        /* 记录 FDCAN 重新启动失败。 */
        canRecoveryFailCount++;
      }
    }
    else
    {
      /* 记录 FDCAN 停止失败。 */
      canRecoveryFailCount++;
    }
  }
}

/* 在最高优先级任务中检查应用运行并刷新独立看门狗。 */
static void IwdgMonitorTask(void *argument)
{
  /* 标记任务参数当前未使用。 */
  (void)argument;
  /* 保存当前系统节拍。 */
  uint32_t nowTick = 0U;
  /* 持续执行看门狗监控。 */
  for (;;)
  {
    /* 读取应用任务最近运行节拍。 */
    nowTick = HAL_GetTick();
    /* 仅当默认应用任务仍在运行时刷新看门狗。 */
    if ((nowTick - appHeartbeatTick) < ESP32_IWDG_TIMEOUT_MS)
    {
      /* 刷新独立看门狗计数器，防止正常运行时复位。 */
      (void)HAL_IWDG_Refresh(&hiwdg);
    }
    /* 等待下一次监控周期。 */
    vTaskDelay(pdMS_TO_TICKS(ESP32_IWDG_REFRESH_PERIOD_MS));
  }
}

static void CanTxTask(void *argument)
{
  /* 标记 FreeRTOS 任务参数为有意未使用。 */
  (void)argument;
  /* 分配并初始化 FDCAN 发送报文头。(发送头，全部零初始化) */
  FDCAN_TxHeaderTypeDef txHeader = {0};
  /* 分配 8 字节 Classic CAN 数据载荷并全部清零。 */
  uint8_t txData[8] = {0};
  /* 保存单调递增的应用层序号。 （递增序列号） */
  uint32_t sequence = 0U;
  /* 保存当前报文生成时的本地 FreeRTOS 节拍。 */
  uint32_t transmitTick = 0U;
  /* 保存 A 主节点发布同步帧时使用的同步序号。 */
  uint32_t syncSequence = 0U;
  /* 保存 A 主节点上一次发布同步帧的本地节拍。 */
  uint32_t lastSyncTick = 0U;
  /* 保存时间同步报文头。 */
  FDCAN_TxHeaderTypeDef syncHeader = {0};
  /* 保存时间同步报文的 8 字节 Classic CAN 数据载荷。 */
  uint8_t syncData[8] = {0};
  /* 将测试报文配置为标准数据帧，并使用当前节点的发送 ID。 */
  txHeader.Identifier = CAN_LOCAL_TX_ID;
  /* 选择 11 位标准标识符。 */
  txHeader.IdType = FDCAN_STANDARD_ID;
  /* 选择数据帧，而不是远程帧。 */
  txHeader.TxFrameType = FDCAN_DATA_FRAME;
  /* 在 Classic CAN 模式下发送 8 个数据字节。 */
  txHeader.DataLength = FDCAN_DLC_BYTES_8;
  /* 将测试帧发送节点标记为错误主动状态。 */
  txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  /* 关闭 CAN FD 数据段位速率切换。 */
  txHeader.BitRateSwitch = FDCAN_BRS_OFF;
  /* 选择 Classic CAN 帧格式。 */
  txHeader.FDFormat = FDCAN_CLASSIC_CAN;
  /* 不将发送事件保存到发送事件 FIFO。 */
  txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  /* 将未使用的报文标记设为 0。 */
  txHeader.MessageMarker = 0U;
  /* 选择时间同步报文的标准 ID。 */
  syncHeader.Identifier = CAN_ID_TIME_SYNC;
  /* 选择时间同步报文使用标准 11 位 ID。 */
  syncHeader.IdType = FDCAN_STANDARD_ID;
  /* 选择同步报文为数据帧。 */
  syncHeader.TxFrameType = FDCAN_DATA_FRAME;
  /* 使用 8 字节 Classic CAN 载荷承载同步信息。 */
  syncHeader.DataLength = FDCAN_DLC_BYTES_8;
  /* 将同步报文发送节点标记为错误主动状态。 */
  syncHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  /* 关闭同步报文的数据段位速率切换。 */
  syncHeader.BitRateSwitch = FDCAN_BRS_OFF;
  /* 选择同步报文为 Classic CAN 帧。 */
  syncHeader.FDFormat = FDCAN_CLASSIC_CAN;
  /* 不保存同步报文的发送事件。 */
  syncHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  /* 将同步报文标记字段清零。 */
  syncHeader.MessageMarker = 0U;
  /* 将同步周期起点初始化为当前节拍减去一个完整周期。 */
  lastSyncTick = HAL_GetTick() - CAN_TIME_SYNC_PERIOD_MS;
  /* 持续运行发送任务，直到 RTOS 停止该线程。 */
  for (;;)
  {
    /* 将序号最低字节编码到数据字节 0。 */
    txData[0] = (uint8_t)(sequence >> 0U);
    /* 将序号次低字节编码到数据字节 1。 */
    txData[1] = (uint8_t)(sequence >> 8U);
    /* 将序号次高字节编码到数据字节 2。 */
    txData[2] = (uint8_t)(sequence >> 16U);
    /* 将序号最高字节编码到数据字节 3。 */
    txData[3] = (uint8_t)(sequence >> 24U);
    /* 在数据字节 4 中写入当前节点的应用层标识。 */
    txData[4] = CAN_LOCAL_NODE_TAG;
    /* 在数据字节 5 中标识三节点 CAN FD 实线实验。 */
    txData[5] = 0x03U;
    /* 清零第一个保留数据字节。 */
    txData[6] = 0x00U;
    /* 清零第二个保留数据字节。 */
    txData[7] = 0x00U;
    /* Classic CAN 业务报文仅使用字节 0 至 7，不写入 CAN FD 扩展字段。 */
    /* 将测试报文放入 FDCAN 发送 FIFO。(fdcan,发送头结构体，数据缓冲区) */
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, txData) != HAL_OK)
    {
      /* 如果发送 FIFO 拒绝报文，则进入生成的错误处理函数。 */
      Error_Handler();
    }
    /* 当前报文入队后递增序号。 */
    sequence++;
    /* 仅在显式启用旧版同步实验时发布自定义时间同步帧。 */
#if (CAN_LEGACY_TIME_SYNC_ENABLE == 1U)
#if (CAN_NODE_ROLE == CAN_NODE_ROLE_A) /* 仅节点 A 负责发布主时钟同步帧。 */
    /* 判断是否到达下一次 100 ms 同步帧发送时刻。 */
    if ((HAL_GetTick() - lastSyncTick) >= CAN_TIME_SYNC_PERIOD_MS)
    {
      /* 更新上一次同步帧发送节拍。 */
      lastSyncTick = HAL_GetTick();
      /* 将同步序号最低字节写入同步数据字节 0。 */
      syncData[0] = (uint8_t)(syncSequence >> 0U);
      /* 将同步序号次低字节写入同步数据字节 1。 */
      syncData[1] = (uint8_t)(syncSequence >> 8U);
      /* 将同步序号次高字节写入同步数据字节 2。 */
      syncData[2] = (uint8_t)(syncSequence >> 16U);
      /* 将同步序号最高字节写入同步数据字节 3。 */
      syncData[3] = (uint8_t)(syncSequence >> 24U);
      /* 写入时间同步报文的数据标识。 */
      syncData[4] = CAN_TAG_TIME_SYNC;
      /* 写入当前实验编号，便于 PCAN-View 区分测试报文。 */
      syncData[5] = 0x03U;
      /* 清零同步报文的第一个保留数据字节。 */
      syncData[6] = 0x00U;
      /* 清零同步报文的第二个保留数据字节。 */
      syncData[7] = 0x00U;
      /* 读取 A 主节点当前节拍作为同步时间基准。 */
      transmitTick = HAL_GetTick();
      /* Classic CAN 同步实验只保留 8 字节数据，不再写入 CAN FD 扩展字段。 */
      /* 将时间同步报文放入 FDCAN 发送 FIFO。 */
      if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &syncHeader, syncData) != HAL_OK)
      {
        /* 同步报文入队失败时进入统一错误处理。 */
        Error_Handler();
      }
      /* 当前同步报文入队后递增同步序号。 */
      syncSequence++;
    }
#endif /* 结束节点 A 主时钟同步帧发送逻辑。 */
#endif /* 结束旧版自定义时间同步帧发送。 */
    /* 使用当前节点角色对应的周期发送下一帧测试报文。 */
    vTaskDelay(pdMS_TO_TICKS(CAN_LOCAL_TX_PERIOD_MS));
  }
}

  /* 从 FreeRTOS 队列取出 A、B、C 节点报文并更新分类计数器。 */
static void CanRxTask(void *argument)
{
  /* 标记 FreeRTOS 任务参数为有意未使用。 */
  (void)argument;
  /* 为每一帧接收报文分配一个本地报文对象。 */
  CanRxMessage_t message = {0};
  /* 持续运行接收任务，直到 RTOS 停止该线程。 */
  for (;;)
  {
    /* 阻塞等待 RX 回调将报文放入 FreeRTOS 队列。（直到下一次xQueueSendToBackFromISR写队列，不阻塞） */
    if (xQueueReceive(canRxQueue, /* 指定接收消息队列。 */
                      &message, /* 将队列报文复制到本地对象。 */
                      portMAX_DELAY) == pdPASS) /* 无限等待直到收到报文。 */
    {
      /* 只处理具有标准 ID、64 字节载荷且属于实验 3 的报文。 */
      if ((message.header.IdType == FDCAN_STANDARD_ID) && /* 检查帧是否使用 11 位标识符。 */
          (message.header.DataLength == FDCAN_DLC_BYTES_8) && /* 检查载荷是否包含 8 个字节。 */
          (message.header.FDFormat == FDCAN_CLASSIC_CAN) && /* 检查接收帧是否为 Classic CAN 格式。 */
          (message.header.BitRateSwitch == FDCAN_BRS_OFF) && /* 检查接收帧是否关闭 BRS。 */
          (message.data[5] == 0x03U)) /* 检查报文是否属于三节点 CAN FD 实验。 */
      {
        /* 解码当前报文中的 32 位小端发送序号。 */
        uint32_t receivedSequence = CanDecodeSequence(message.data);
        /* 解码当前报文中的发送端节拍字段。 */
        uint32_t senderTick = CanDecodeTimestamp(message.data);
        /* 将节点 A 的 CAN ID 与数据区标识同时作为有效性条件。 */
        if ((message.header.Identifier == CAN_ID_NODE_A) && /* 检查是否为节点 A 的 CAN ID。 */
            (message.data[4] == CAN_TAG_NODE_A)) /* 检查节点 A 的应用层标识。 */
        {
          /* 保存节点 A 最近一次的发送序号。 */
          rxSequenceA = receivedSequence;
          /* 增加节点 A 的有效报文接收计数。 */
          rxCountA++;
          /* 更新节点 A 的序号、丢帧、接收间隔和抖动统计。 */
          CanUpdateNodeStats(&rxStatsA, receivedSequence, senderTick, CAN_NODE_A_PERIOD_MS);
        }
        /* 将节点 B 的 CAN ID 与数据区标识同时作为有效性条件。 */
        else if ((message.header.Identifier == CAN_ID_NODE_B) && /* 检查是否为节点 B 的 CAN ID。 */
                 (message.data[4] == CAN_TAG_NODE_B)) /* 检查节点 B 的应用层标识。 */
        {
          /* 保存节点 B 最近一次的发送序号。 */
          rxSequenceB = receivedSequence;
          /* 增加节点 B 的有效报文接收计数。 */
          rxCountB++;
          /* 更新节点 B 的序号、丢帧、接收间隔和抖动统计。 */
          CanUpdateNodeStats(&rxStatsB, receivedSequence, senderTick, CAN_NODE_B_PERIOD_MS);
        }
        /* 将节点 C 的 CAN ID 与数据区标识同时作为有效性条件。 */
        else if ((message.header.Identifier == CAN_ID_NODE_C) && /* 检查是否为节点 C 的 CAN ID。 */
                 (message.data[4] == CAN_TAG_NODE_C)) /* 检查节点 C 的应用层标识。 */
        {
          /* 保存节点 C 最近一次的发送序号。 */
          rxSequenceC = receivedSequence;
          /* 增加节点 C 的有效报文接收计数。 */
          rxCountC++;
          /* 更新节点 C 的序号、丢帧、接收间隔和抖动统计。 */
          CanUpdateNodeStats(&rxStatsC, receivedSequence, senderTick, CAN_NODE_C_PERIOD_MS);
        }
        else /* 当前报文的 ID 和节点标识不构成有效组合。 */
        {
          /* 跳过不属于 A、B、C 三类实验报文的帧。 */
          continue;
        }
        /* 保存最近一次有效报文的序号，供快速观察。 */
        rxSequence = receivedSequence;
        /* 增加所有有效 A、B、C 报文的总接收计数。 */
        rxCount++;
        /* 翻转用户 LED，提供可见的有效报文接收指示。 */
        HAL_GPIO_TogglePin(APP_LED_GPIO_Port, APP_LED_Pin);
      }
    }
  }
}

/* 将报文数据字节 8 至 11 解码为小端 32 位发送时间戳。 */
static uint32_t CanDecodeTimestamp(const uint8_t *data)
{
  /* 合并发送时间戳的最低有效字节。 */
  return ((uint32_t)data[CAN_TIMESTAMP_OFFSET + 0U] << 0U) |
         /* 合并发送时间戳的次低有效字节。 */
         ((uint32_t)data[CAN_TIMESTAMP_OFFSET + 1U] << 8U) |
         /* 合并发送时间戳的次高有效字节。 */
         ((uint32_t)data[CAN_TIMESTAMP_OFFSET + 2U] << 16U) |
         /* 合并发送时间戳的最高有效字节。 */
         ((uint32_t)data[CAN_TIMESTAMP_OFFSET + 3U] << 24U);
}

/* 将本节点本地 HAL 节拍转换为 A 主节点的逻辑全局节拍。 */
static uint32_t CanGetGlobalTick(void)
{
  /* 使用无符号减法处理 HAL_GetTick() 的 32 位自然回绕。 */
  return HAL_GetTick() - (uint32_t)canTimeOffsetMs;
}

/* 更新一个节点的固定优先级调度基线统计量。 */
static void CanUpdateNodeStats(volatile CanNodeStats_t *stats, /* 传入待更新的节点统计对象。 */
                               uint32_t receivedSequence, /* 传入当前报文的应用层序号。 */
                               uint32_t senderTick, /* 传入当前报文的发送端节拍。 */
                               uint32_t expectedPeriodMs) /* 传入该节点的理论周期。 */
{
  /* 记录接收当前报文时换算到主节点的逻辑节拍。 */
  uint32_t receiveTick = CanGetGlobalTick();
  /* 保存当前报文与上一帧之间的序号差。 */
  uint32_t sequenceDelta = 0U;
  /* 保存当前报文与上一帧之间的本地接收间隔。 */
  uint32_t interArrivalMs = 0U;
  /* 保存当前接收间隔相对于理论周期的绝对偏差。 */
  uint32_t jitterMs = 0U;
  /* 判断当前报文是否不是该节点的第一帧。 */
  if (stats->initialized != 0U)
  {
    /* 计算 32 位序号的无符号差值，支持自然回绕。 */
    sequenceDelta = receivedSequence - stats->lastSequence;
    /* 仅把正向跳号且未跨越半个序号空间的情况计为丢帧。 */
    if ((sequenceDelta > 1U) && (sequenceDelta < 0x80000000U))
    {
      /* 累加本次序号跳变中缺失的应用报文数量。 */
      stats->lostCount += sequenceDelta - 1U;
    }
    /* 计算本地接收时间戳之间的毫秒间隔。 */
    interArrivalMs = receiveTick - stats->lastReceiveTick;
    /* 保存最近一次本地接收间隔。 */
    stats->interArrivalMs = interArrivalMs;
    /* 计算接收间隔相对于期望周期的绝对抖动。 */
    if (interArrivalMs >= expectedPeriodMs)
    {
      /* 记录接收间隔大于理论周期时的偏差。 */
      jitterMs = interArrivalMs - expectedPeriodMs;
    }
    else
    {
      /* 记录接收间隔小于理论周期时的偏差。 */
      jitterMs = expectedPeriodMs - interArrivalMs;
    }
    /* 保存最近一次周期抖动。 */
    stats->jitterMs = jitterMs;
    /* 更新该节点观测到的最大接收间隔。 */
    if (interArrivalMs > stats->maxInterArrivalMs)
    {
      /* 保存新的最大接收间隔。 */
      stats->maxInterArrivalMs = interArrivalMs;
    }
    /* 更新该节点观测到的最大周期抖动。 */
    if (jitterMs > stats->maxJitterMs)
    {
      /* 保存新的最大周期抖动。 */
      stats->maxJitterMs = jitterMs;
    }
  }
  /* 保存当前报文的最新序号。 */
  stats->lastSequence = receivedSequence;
  /* 保存当前报文中的发送端节拍字段。 */
  stats->senderTick = senderTick;
  /* 保存接收端记录当前报文的本地节拍。 */
  stats->receiveTick = receiveTick;
  /* 保存下一帧计算接收间隔时使用的本地节拍。 */
  stats->lastReceiveTick = receiveTick;
  /* 累加该节点的有效接收帧数量。 */
  stats->receivedCount++;
  /* 标记该节点已经完成第一帧接收。 */
  stats->initialized = 1U;
}

/* 将报文数据字节 0 至 3 解码为小端 32 位序号。 */
static uint32_t CanDecodeSequence(const uint8_t *data)
{
  /* 合并最低有效字节。 */
  return ((uint32_t)data[0] << 0U) |
         /* 合并次低有效字节。 */
         ((uint32_t)data[1] << 8U) |
         /* 合并次高有效字节。 */
         ((uint32_t)data[2] << 16U) |
         /* 合并最高有效字节。 */
         ((uint32_t)data[3] << 24U);
}

/* 周期刷新 A 节点的 SSD1306 仪表盘。 */
static void OledDashboardTask(void *argument)
{
  /* 标记任务参数当前未使用。 */
  (void)argument;
  /* 保存一份诊断快照，避免显示过程中读取多次 volatile 数据。 */
  CanOpenPortDiagnostics_t snapshot = {0};
  /* 保存每一行的 ASCII 文本。 */
  char line[24] = {0};
  /* 保存当前告警位图。 */
  uint32_t alarmBits = 0U;
  /* 保存车辆统一告警状态快照。 */
  VehicleAlarmStatus_t alarmSnapshot = {0};
  /* 保存车辆意外检测状态快照。 */
  AccidentDetectorStatus_t accidentSnapshot = {0};
  /* 保存当前 OLED 页面，0 为数据页，1 为故障页。 */
  uint8_t displayPage = 0U;
  /* 保存无告警时的下一次页面切换时刻。 */
  uint32_t nextPageTick = 0U;
  /* 周期运行 OLED 显示任务。 */
  for (;;)
  {
    /* OLED 未成功初始化时仅等待，不访问无效 I2C 设备。 */
    if (oledStatus != HAL_OK)
    {
      /* 周期重试 SSD1306 初始化，支持 OLED 重新上电后恢复显示。 */
      oledStatus = SSD1306_Init(&hi2c1, (uint16_t)(0x3CU << 1U));
      /* 等待下一次初始化重试。 */
      vTaskDelay(pdMS_TO_TICKS(1000U));
      continue;
    }
    /* 复制 A 节点当前 CANopen 诊断数据。 */
    CanOpenPort_GetDiagnostics(&snapshot);
    /* 复制统一告警状态，确保 OLED 与 MQTT 使用同一判定结果。 */
    VehicleAlarm_GetStatus(&alarmSnapshot);
    /* 复制意外检测状态，确保 OLED 与 MQTT 使用同一事件结果。 */
    AccidentDetector_GetStatus(&accidentSnapshot);
    /* 清除上一帧 OLED 显示缓存。 */
    SSD1306_Clear();
    /* 使用状态机确认后的活动告警位。 */
    alarmBits = alarmSnapshot.activeAlarmBits;
    /* 获取当前系统节拍用于页面轮换。 */
    {
      /* 保存当前页面判断节拍。 */
      uint32_t nowTick = HAL_GetTick();
      /* 有告警时立即进入故障页面并延后自动轮换。 */
      if (alarmSnapshot.state != VEHICLE_ALARM_NORMAL)
      {
        /* 选择故障页面。 */
        displayPage = 1U;
        /* 告警清除后至少保持故障页面 3 秒。 */
        nextPageTick = nowTick + 3000U;
      }
      else if ((int32_t)(nowTick - nextPageTick) >= 0)
      {
        /* 无告警时在数据页和故障页之间轮换。 */
        displayPage = (displayPage == 0U) ? 1U : 0U;
        /* 安排下一次页面切换。 */
        nextPageTick = nowTick + 3000U;
      }
    }
    /* 根据页面选择显示实时数据或故障详情。 */
    if (displayPage == 0U)
    {
    /* 格式化速度显示行。 */
    (void)snprintf(line, sizeof(line), "SPD:%lu mm/s", (unsigned long)OD_RAM.x2103_speedMmps);
    /* 写入第一行速度。 */
    /* 设置第一行光标位置。 */
    SSD1306_SetCursor(0U, 0U);
    /* 写入第一行速度文本。 */
    SSD1306_WriteString(line);
    /* 格式化 GNSS 状态显示行。 */
    (void)snprintf(line, sizeof(line), "GPS:%c SAT:%02u", (OD_RAM.x2105_fixValid != 0U) ? 'Y' : 'N', (unsigned int)OD_RAM.x2104_satelliteCount);
    /* 写入第二行 GNSS 状态。 */
    /* 设置第二行光标位置。 */
    SSD1306_SetCursor(0U, 1U);
    /* 写入第二行 GNSS 文本。 */
    SSD1306_WriteString(line);
    /* 格式化纬度显示行。 */
    (void)snprintf(line, sizeof(line), "LAT:%ld", (long)OD_RAM.x2100_latitudeE7);
    /* 写入第三行纬度。 */
    /* 设置第三行光标位置。 */
    SSD1306_SetCursor(0U, 2U);
    /* 写入第三行纬度文本。 */
    SSD1306_WriteString(line);
    /* 格式化经度显示行。 */
    (void)snprintf(line, sizeof(line), "LON:%ld", (long)OD_RAM.x2101_longitudeE7);
    /* 写入第四行经度。 */
    /* 设置第四行光标位置。 */
    SSD1306_SetCursor(0U, 3U);
    /* 写入第四行经度文本。 */
    SSD1306_WriteString(line);
    /* 格式化加速度 X/Y 显示行。 */
    (void)snprintf(line, sizeof(line), "AX:%d AY:%d", (int)OD_RAM.x2110_accelX, (int)OD_RAM.x2111_accelY);
    /* 写入第五行加速度。 */
    /* 设置第五行光标位置。 */
    SSD1306_SetCursor(0U, 4U);
    /* 写入第五行加速度文本。 */
    SSD1306_WriteString(line);
    /* 格式化加速度 Z 和角速度 X 显示行。 */
    (void)snprintf(line, sizeof(line), "AZ:%d GX:%d", (int)OD_RAM.x2112_accelZ, (int)OD_RAM.x2113_gyroX);
    /* 写入第六行传感器数据。 */
    /* 设置第六行光标位置。 */
    SSD1306_SetCursor(0U, 5U);
    /* 写入第六行传感器文本。 */
    SSD1306_WriteString(line);
    /* 格式化角速度 Y/Z 显示行。 */
    (void)snprintf(line, sizeof(line), "GY:%d GZ:%d", (int)OD_RAM.x2114_gyroY, (int)OD_RAM.x2115_gyroZ);
    /* 写入第七行角速度。 */
    /* 设置第七行光标位置。 */
    SSD1306_SetCursor(0U, 6U);
    /* 写入第七行角速度文本。 */
    SSD1306_WriteString(line);
    /* 格式化节点健康和告警位显示行。 */
    (void)snprintf(line, sizeof(line), "B:%c C:%c A:%s", (snapshot.nodeBHealthy != 0U) ? 'Y' : 'N', (snapshot.nodeCHealthy != 0U) ? 'Y' : 'N', AccidentDetector_GetEventText(accidentSnapshot.eventType));
    /* 写入第八行健康状态。 */
    /* 设置第八行光标位置。 */
    SSD1306_SetCursor(0U, 7U);
    /* 写入第八行健康文本。 */
    SSD1306_WriteString(line);
    }
    else
    {
      /* 格式化当前告警状态标题。 */
      (void)snprintf(line, sizeof(line), "STATE:%s", VehicleAlarm_GetStateText(alarmSnapshot.state));
      /* 写入故障页面标题。 */
      SSD1306_SetCursor(0U, 0U);
      SSD1306_WriteString(line);
      /* 显示 IMU 状态。 */
      (void)snprintf(line, sizeof(line), "IMU:%s", (snapshot.imuDataValid != 0U) ? "OK" : "FAIL");
      /* 写入 IMU 故障状态。 */
      SSD1306_SetCursor(0U, 1U);
      SSD1306_WriteString(line);
      /* 显示 GNSS 状态。 */
      (void)snprintf(line, sizeof(line), "GNSS:%s", (snapshot.gnssDataValid != 0U) ? "OK" : "FAIL");
      /* 写入 GNSS 故障状态。 */
      SSD1306_SetCursor(0U, 2U);
      SSD1306_WriteString(line);
      /* 显示 B 节点健康状态。 */
      (void)snprintf(line, sizeof(line), "NODE B:%s", (snapshot.nodeBHealthy != 0U) ? "OK" : "FAIL");
      /* 写入 B 节点故障状态。 */
      SSD1306_SetCursor(0U, 3U);
      SSD1306_WriteString(line);
      /* 显示 C 节点健康状态。 */
      (void)snprintf(line, sizeof(line), "NODE C:%s", (snapshot.nodeCHealthy != 0U) ? "OK" : "FAIL");
      /* 写入 C 节点故障状态。 */
      SSD1306_SetCursor(0U, 4U);
      SSD1306_WriteString(line);
      /* 显示 MQTT 状态。 */
      (void)snprintf(line, sizeof(line), "MQTT:%s", (esp32Status.state == ESP32_STATE_MQTT_CONNECTED) ? "OK" : "OFF");
      /* 写入 MQTT 故障状态。 */
      SSD1306_SetCursor(0U, 5U);
      SSD1306_WriteString(line);
      /* 显示告警位图。 */
      (void)snprintf(line, sizeof(line), "AL:0x%02lX", (unsigned long)alarmBits);
      /* 写入告警位图。 */
      SSD1306_SetCursor(0U, 6U);
      SSD1306_WriteString(line);
      /* 显示 B/C 累计超时次数。 */
      (void)snprintf(line, sizeof(line), "BTO:%03lu CTO:%03lu", (unsigned long)snapshot.heartbeatB.timeoutCount, (unsigned long)snapshot.heartbeatC.timeoutCount);
      /* 写入超时计数。 */
      SSD1306_SetCursor(0U, 7U);
      SSD1306_WriteString(line);
    }
    /* 将完整仪表盘缓存刷新到 OLED。 */
    if (SSD1306_UpdateScreen() != HAL_OK)
    {
      /* 记录 I2C 显示通信失败，后续周期继续尝试。 */
      oledStatus = HAL_ERROR;
    }
    /* 每 200 ms 刷新一次仪表盘。 */
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
  /* 保存一次 SPI 读取结果，避免直接把 volatile 变量地址传给驱动。 */
  ICM42688_RawData_t sample = {0};
  /* 保存一次 BH-182 读取结果，避免把 volatile 全局变量地址传给驱动。 */
  BH182_Data_t bh182Snapshot = {0};
  /* Infinite loop */
  for(;;)
  {
    /* 记录默认应用任务仍在运行，供看门狗监控任务判断。 */
    appHeartbeatTick = HAL_GetTick();
    /* 将 ESP32 驱动状态复制到全局变量，供调试器实时观察。 */
    ESP32_GetStatus(&esp32Status);
    /* 每 1 ms 运行一次 CANopenNode 协议状态机。 */
    CanOpenPort_Process(1000U);
    /* 将 B/C Heartbeat 和 A EMCY 诊断数据复制到全局调试快照。 */
    CanOpenPort_GetDiagnostics(&diagnostics);
    /* A 节点仅维护 B 节点事件的保持窗口，事故判断在 B 节点本地完成。 */
#if (CAN_NODE_ROLE == CAN_NODE_ROLE_A)
    /* 每 10 ms 调用一次无输入更新，使远端事件保持窗口能够自动过期。 */
    if ((HAL_GetTick() - accidentLastUpdateTick) >= 10U)
    {
      /* 保存本次事故状态维护时刻。 */
      accidentLastUpdateTick = HAL_GetTick();
      /* 传入空数据只执行事件保持窗口维护，不使用 A 的 TPDO 做检测。 */
      AccidentDetector_Update(NULL, 0U);
      /* 复制远端事故事件快照，供 OLED、MQTT 和调试器读取。 */
      AccidentDetector_GetStatus(&accidentStatus);
    }
#endif
    /* 每 100 ms 执行一次车辆告警状态机判定。 */
    if ((HAL_GetTick() - vehicleAlarmLastUpdateTick) >= 100U)
    {
      /* 保存本次告警判定的运行时刻。 */
      vehicleAlarmLastUpdateTick = HAL_GetTick();
      /* 使用 CANopen、Bus-Off 和 MQTT 的统一输入更新告警状态。 */
      VehicleAlarm_Update((const CanOpenPortDiagnostics_t *)&diagnostics,
                          canBusOffActive,
                          (esp32Status.state == ESP32_STATE_MQTT_CONNECTED) ? 1U : 0U);
      /* 复制状态机快照，供 OLED、云端和调试器读取。 */
      VehicleAlarm_GetStatus(&vehicleAlarmStatus);
    }
    /* 更新 BH-182 定位数据快照，供 Keil Watch 和后续 CANopen TPDO 使用。 */
    /* 从 BH-182 驱动复制一份一致的定位数据快照。 */
    BH182_GetData(&bh182Snapshot);
    /* 将定位快照保存到供调试器观察的 volatile 全局变量。 */
    bh182Data = bh182Snapshot;
    /* 将 BH-182 快照同步到 CANopen 对象字典，供 C 节点 TPDO 发送。 */
    CanOpenPort_UpdateGnss(&bh182Snapshot);
    /* 按节点角色读取 ICM-42688 六轴原始数据，B 节点周期为 1 ms。 */
    if ((icm42688Status == HAL_OK) &&
        ((HAL_GetTick() - icm42688LastSampleTick) >= APP_IMU_SAMPLE_PERIOD_MS))
    {
      /* 保存本次采样时刻，用于形成固定的读取周期。 */
      icm42688LastSampleTick = HAL_GetTick();
      /* 从 ICM-42688 读取加速度计和陀螺仪原始数据。 */
      if (ICM42688_ReadRaw(&sample) == HAL_OK)
      {
        /* 将本次成功采样复制到供调试器观察的全局变量。 */
        icm42688RawData = sample;
        /* 将有效 IMU 采样同步到 CANopen 对象字典并触发 TPDO。 */
        CanOpenPort_UpdateImu(&sample, 1U);
#if (CAN_NODE_ROLE == CAN_NODE_ROLE_B)
        /* B 节点使用本地 ICM-42688 原始采样执行高速事故检测。 */
        AccidentDetector_Update(&sample, 1U);
        /* 复制本地检测状态，供事件发送判断和调试器观察。 */
        AccidentDetector_GetStatus(&accidentStatus);
        /* 仅在新事件确认后发送一次 CANopen 事件 TPDO/EMCY。 */
        if ((accidentStatus.active != 0U) &&
            (accidentStatus.eventType != lastReportedAccidentEvent))
        {
          /* 发送失败时保留旧标记，下一次采样会继续尝试可靠发送。 */
          if (CanOpenPort_SendAccidentEvent(accidentStatus.eventType,
                                            accidentStatus.lastEventId,
                                            accidentStatus.peakAccelMg,
                                            accidentStatus.peakGyroDps) == CO_ERROR_NO)
          {
            /* 记录已经成功发送的事件类型，避免每个采样点重复发 EMCY。 */
            lastReportedAccidentEvent = accidentStatus.eventType;
          }
        }
        /* 事件保持窗口结束后清除对应的本地 CANopen EMCY。 */
        if ((accidentStatus.active == 0U) &&
            (lastReportedAccidentEvent != ACCIDENT_EVENT_NONE))
        {
          /* 发送错误清除并允许下一次同类型事件再次上报。 */
          CanOpenPort_ClearAccidentError(lastReportedAccidentEvent);
          /* 清除已上报事件标记。 */
          lastReportedAccidentEvent = ACCIDENT_EVENT_NONE;
        }
#endif
      }
      else
      {
        /* 保存最近一次 SPI 读取失败状态，便于定位传感器或连线故障。 */
        icm42688Status = HAL_ERROR;
        /* 将 IMU 无效状态同步到 CANopen 对象字典。 */
        CanOpenPort_UpdateImu(&sample, 0U);
#if (CAN_NODE_ROLE == CAN_NODE_ROLE_B)
        /* 即使 SPI 临时失败也维护事故保持窗口，避免 EMCY 长时间不清除。 */
        AccidentDetector_Update(NULL, 0U);
        /* 复制维护后的事故状态。 */
        AccidentDetector_GetStatus(&accidentStatus);
        /* 保持窗口到期后清除相应 EMCY 并允许下一次事件重新上报。 */
        if ((accidentStatus.active == 0U) &&
            (lastReportedAccidentEvent != ACCIDENT_EVENT_NONE))
        {
          /* 发送 CANopen 事故错误清除帧。 */
          CanOpenPort_ClearAccidentError(lastReportedAccidentEvent);
          /* 清除本地已上报事件标记。 */
          lastReportedAccidentEvent = ACCIDENT_EVENT_NONE;
        }
#endif
      }
    }
    /* 使用 FreeRTOS 节拍延时让出默认任务的 CPU 时间。 */
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
  /* User can add his own implementation to report the HAL error return state */
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
