/* 基于 ICM-42688 数据的车辆碰撞与翻车检测实现。 */
#include "accident_detector.h"
/* 引入 HAL_GetTick 时间戳接口。 */
#include "stm32g4xx_hal.h"
/* 引入清零和复制函数。 */
#include <string.h>

/* ICM-42688 当前配置为 +/-16 g，对应约 2048 LSB/g。 */
#define ACCIDENT_ACCEL_LSB_PER_G 2048L
/* ICM-42688 当前配置为 +/-2000 dps，对应约 16.384 LSB/dps。 */
#define ACCIDENT_GYRO_LSB_PER_DPS 16L
/* 碰撞判定的加速度阈值，超过 6 g 才进入确认流程。 */
#define ACCIDENT_COLLISION_ACCEL_MG 6000UL
/* 翻车判定要求 Z 轴重力分量低于 0.50 g。 */
#define ACCIDENT_ROLLOVER_Z_MG 500UL
/* 翻车判定要求水平加速度分量高于 0.75 g。 */
#define ACCIDENT_ROLLOVER_HORIZONTAL_MG 750UL
/* 翻车判定也覆盖 Z 轴反向重力超过 0.70 g 的倒置状态。 */
#define ACCIDENT_ROLLOVER_INVERTED_Z_RAW (-1434L)
/* 一个 10 ms 采样点超过碰撞阈值即可确认碰撞，避免漏掉短脉冲冲击。 */
#define ACCIDENT_COLLISION_CONFIRM_SAMPLES 1U
/* 连续 500 个 1 ms 采样点满足姿态阈值才确认翻车，保持约 500 ms 姿态确认时间。 */
#define ACCIDENT_ROLLOVER_CONFIRM_SAMPLES 500U
/* 事件确认后保持 10 秒，确保 MQTT 至少有机会上传事件。 */
#define ACCIDENT_EVENT_HOLD_MS 10000UL

/* 保存模块内部状态。 */
static volatile AccidentDetectorStatus_t accidentStatus;
/* 保存连续碰撞阈值命中的采样数。 */
static uint16_t collisionConfirmSamples;
/* 保存连续翻车姿态阈值命中的采样数。 */
static uint16_t rolloverConfirmSamples;
/* 保存本节点生成的下一个事件编号，零编号保留给无效事件。 */
static uint32_t nextEventId;

/* 返回带符号整数的绝对值。 */
static uint32_t Accident_Abs(int16_t value)
{
  /* 将负数转换为正数，避免直接使用可能溢出的标准 abs。 */
  int32_t signedValue = (int32_t)value;
  /* 返回带符号值的绝对值。 */
  return (uint32_t)((signedValue < 0) ? -signedValue : signedValue);
}

/* 将三轴原始数据合成为近似加速度值，单位 mg。 */
static uint32_t Accident_AccelMagnitudeMg(const ICM42688_RawData_t *data)
{
  /* 保存三个轴的毫克值。 */
  uint32_t xMg = (Accident_Abs(data->accelX) * 1000UL) / ACCIDENT_ACCEL_LSB_PER_G;
  /* 保存 Y 轴毫克值。 */
  uint32_t yMg = (Accident_Abs(data->accelY) * 1000UL) / ACCIDENT_ACCEL_LSB_PER_G;
  /* 保存 Z 轴毫克值。 */
  uint32_t zMg = (Accident_Abs(data->accelZ) * 1000UL) / ACCIDENT_ACCEL_LSB_PER_G;
  /* 保存三轴中最大的毫克值。 */
  uint32_t maximum = xMg;
  /* 保存三轴中间大小的毫克值。 */
  uint32_t middle = yMg;
  /* 保存三轴中最小的毫克值。 */
  uint32_t minimum = zMg;
  /* 将 X/Y/Z 排序，便于使用无平方根的向量长度近似。 */
  if (maximum < middle)
  {
    /* 交换 X 轴和 Y 轴的排序位置。 */
    uint32_t temporary = maximum;
    /* 保存交换后的最大值。 */
    maximum = middle;
    /* 保存交换后的中间值。 */
    middle = temporary;
  }
  /* 将 Z 轴纳入排序结果。 */
  if (maximum < minimum)
  {
    /* 交换最大值和最小值。 */
    uint32_t temporary = maximum;
    /* 保存交换后的最大值。 */
    maximum = minimum;
    /* 保存交换后的最小值。 */
    minimum = temporary;
  }
  /* 修正 X/Y 排序后中间值和最小值的相对顺序。 */
  if (middle < minimum)
  {
    /* 交换中间值和最小值。 */
    uint32_t temporary = middle;
    /* 保存交换后的中间值。 */
    middle = minimum;
    /* 保存交换后的最小值。 */
    minimum = temporary;
  }
  /* 使用 max + 0.5*middle + 0.25*minimum 近似三维向量长度。 */
  uint32_t magnitudeMg = maximum + (middle / 2U) + (minimum / 4U);
  /* 达到碰撞阈值后使用阈值作为饱和值。 */
  return (magnitudeMg >= ACCIDENT_COLLISION_ACCEL_MG) ? ACCIDENT_COLLISION_ACCEL_MG : magnitudeMg;
}

/* 将三轴原始陀螺仪数据合成为近似角速度值，单位 dps。 */
static uint32_t Accident_GyroMagnitudeDps(const ICM42688_RawData_t *data)
{
  /* 保存三个轴的绝对角速度并取最大值，避免复杂浮点运算。 */
  uint32_t gyroX = Accident_Abs(data->gyroX) / ACCIDENT_GYRO_LSB_PER_DPS;
  /* 保存 Y 轴角速度。 */
  uint32_t gyroY = Accident_Abs(data->gyroY) / ACCIDENT_GYRO_LSB_PER_DPS;
  /* 保存 Z 轴角速度。 */
  uint32_t gyroZ = Accident_Abs(data->gyroZ) / ACCIDENT_GYRO_LSB_PER_DPS;
  /* 返回三个轴中的最大角速度。 */
  uint32_t maximum = (gyroX > gyroY) ? gyroX : gyroY;
  /* 将 Z 轴较大值纳入最大角速度。 */
  return (gyroZ > maximum) ? gyroZ : maximum;
}

/* 记录一次新的车辆意外事件。 */
static void Accident_Raise(uint8_t eventType, uint32_t accelMg, uint32_t gyroDps)
{
  /* 获取当前系统时间。 */
  uint32_t nowTick = HAL_GetTick();
  /* 事件已经处于保持窗口时不重复增加事件计数。 */
  if ((accidentStatus.active != 0U) && (accidentStatus.eventType == eventType))
  {
    /* 延长保持时间，避免事件在持续冲击期间提前清除。 */
    accidentStatus.eventHoldUntilTick = nowTick + ACCIDENT_EVENT_HOLD_MS;
    /* 更新保持窗口内的加速度峰值。 */
    if (accelMg > accidentStatus.peakAccelMg)
    {
      /* 保存更大的加速度峰值。 */
      accidentStatus.peakAccelMg = accelMg;
    }
    /* 更新保持窗口内的角速度峰值。 */
    if (gyroDps > accidentStatus.peakGyroDps)
    {
      /* 保存更大的角速度峰值。 */
      accidentStatus.peakGyroDps = gyroDps;
    }
    /* 当前事件已经记录，直接返回。 */
    return;
  }
  /* 标记新的事件处于活动状态。 */
  accidentStatus.active = 1U;
  /* 保存事件类型。 */
  accidentStatus.eventType = eventType;
  /* 保存事件发生时间。 */
  accidentStatus.lastEventTick = nowTick;
  /* 为新事件分配递增编号，供 CANopen 和 MQTT 端到端去重。 */
  nextEventId++;
  /* 保存本次新事件编号。 */
  accidentStatus.lastEventId = nextEventId;
  /* 设置事件保持截止时间。 */
  accidentStatus.eventHoldUntilTick = nowTick + ACCIDENT_EVENT_HOLD_MS;
  /* 保存本次事件的初始峰值。 */
  accidentStatus.peakAccelMg = accelMg;
  /* 保存本次事件的初始角速度峰值。 */
  accidentStatus.peakGyroDps = gyroDps;
  /* 按事件类型增加对应计数。 */
  if (eventType == ACCIDENT_EVENT_COLLISION)
  {
    /* 增加碰撞事件计数。 */
    accidentStatus.collisionCount++;
  }
  else if (eventType == ACCIDENT_EVENT_ROLLOVER)
  {
    /* 增加翻车事件计数。 */
    accidentStatus.rolloverCount++;
  }
  else
  {
    /* 保留未知事件类型，不增加专用计数。 */
  }
}

/* 初始化车辆意外检测状态。 */
void AccidentDetector_Init(void)
{
  /* 清零事件状态和历史计数。 */
  (void)memset((void *)&accidentStatus, 0, sizeof(accidentStatus));
  /* 清零碰撞连续确认计数。 */
  collisionConfirmSamples = 0U;
  /* 清零翻车连续确认计数。 */
  rolloverConfirmSamples = 0U;
  /* 从编号 1 开始生成事件，避免零值被误判为无效。 */
  nextEventId = 0U;
}

/* 使用一帧 IMU 原始数据更新碰撞和翻车判断。 */
void AccidentDetector_Update(const ICM42688_RawData_t *data, uint8_t valid)
{
  /* 保存本次采样的系统时间。 */
  uint32_t nowTick = HAL_GetTick();
  /* 保存水平加速度平方值。 */
  uint32_t horizontalMgSquare = 0U;
  /* 保存翻车水平加速度阈值平方。 */
  uint32_t horizontalThresholdSquare = ACCIDENT_ROLLOVER_HORIZONTAL_MG * ACCIDENT_ROLLOVER_HORIZONTAL_MG;
  /* 保存当前 Z 轴绝对加速度。 */
  uint32_t zMg = 0U;
  /* 事件保持窗口到期后清除当前活动状态，即使本次 IMU 数据无效。 */
  if ((accidentStatus.active != 0U) && ((int32_t)(nowTick - accidentStatus.eventHoldUntilTick) >= 0))
  {
    /* 清除当前活动事件，但保留累计次数和最近峰值。 */
    accidentStatus.active = 0U;
    /* 清除当前事件类型。 */
    accidentStatus.eventType = ACCIDENT_EVENT_NONE;
  }
  /* 检查输入指针和有效标志。 */
  if ((data == NULL) || (valid == 0U))
  {
    /* 无效数据不能触发意外，只更新输入有效标志。 */
    accidentStatus.imuValid = 0U;
    return;
  }
  /* 标记本次输入数据有效。 */
  accidentStatus.imuValid = 1U;
  /* 计算当前三轴合成加速度。 */
  accidentStatus.currentAccelMg = Accident_AccelMagnitudeMg(data);
  /* 计算当前最大角速度。 */
  accidentStatus.currentGyroDps = Accident_GyroMagnitudeDps(data);
  /* 检查碰撞高加速度阈值。 */
  if (accidentStatus.currentAccelMg >= ACCIDENT_COLLISION_ACCEL_MG)
  {
    /* 累加碰撞确认采样数。 */
    if (collisionConfirmSamples < ACCIDENT_COLLISION_CONFIRM_SAMPLES)
    {
      /* 防止确认计数溢出。 */
      collisionConfirmSamples++;
    }
    /* 达到确认次数后上报碰撞事件。 */
    if (collisionConfirmSamples >= ACCIDENT_COLLISION_CONFIRM_SAMPLES)
    {
      /* 记录碰撞事件及峰值数据。 */
      Accident_Raise(ACCIDENT_EVENT_COLLISION, accidentStatus.currentAccelMg, accidentStatus.currentGyroDps);
      /* 清除本次碰撞确认计数，等待下一次独立冲击。 */
      collisionConfirmSamples = 0U;
    }
  }
  else
  {
    /* 当前采样未达到碰撞阈值，清除连续确认计数。 */
    collisionConfirmSamples = 0U;
  }
  /* 计算 X/Y 水平加速度平方值。 */
  horizontalMgSquare = (uint32_t)(((uint64_t)Accident_Abs(data->accelX) * 1000UL / ACCIDENT_ACCEL_LSB_PER_G) *
                                  ((uint64_t)Accident_Abs(data->accelX) * 1000UL / ACCIDENT_ACCEL_LSB_PER_G) +
                                  ((uint64_t)Accident_Abs(data->accelY) * 1000UL / ACCIDENT_ACCEL_LSB_PER_G) *
                                  ((uint64_t)Accident_Abs(data->accelY) * 1000UL / ACCIDENT_ACCEL_LSB_PER_G));
  /* 计算 Z 轴绝对加速度，单位为 mg。 */
  zMg = (Accident_Abs(data->accelZ) * 1000UL) / ACCIDENT_ACCEL_LSB_PER_G;
  /* 判断 Z 轴重力分量和水平分量是否同时符合翻车特征。 */
  if (((zMg <= ACCIDENT_ROLLOVER_Z_MG) && (horizontalMgSquare >= horizontalThresholdSquare)) ||
      ((int32_t)data->accelZ <= ACCIDENT_ROLLOVER_INVERTED_Z_RAW))
  {
    /* 累加翻车姿态确认采样数。 */
    if (rolloverConfirmSamples < ACCIDENT_ROLLOVER_CONFIRM_SAMPLES)
    {
      /* 防止确认计数溢出。 */
      rolloverConfirmSamples++;
    }
    /* 姿态持续满足阈值后确认翻车。 */
    if (rolloverConfirmSamples >= ACCIDENT_ROLLOVER_CONFIRM_SAMPLES)
    {
      /* 记录翻车事件及当前峰值数据。 */
      Accident_Raise(ACCIDENT_EVENT_ROLLOVER, accidentStatus.currentAccelMg, accidentStatus.currentGyroDps);
      /* 清除翻车确认计数，等待下一次独立翻车。 */
      rolloverConfirmSamples = 0U;
      /* 标记已经完成一次翻车确认。 */
      accidentStatus.rolloverCandidate = 0U;
    }
    else
    {
      /* 标记正在确认翻车姿态。 */
      accidentStatus.rolloverCandidate = 1U;
    }
  }
  else
  {
    /* 姿态恢复正常时清除连续翻车确认计数。 */
    rolloverConfirmSamples = 0U;
    /* 清除翻车候选标志。 */
    accidentStatus.rolloverCandidate = 0U;
  }
}

/* 将远端 B 节点已经确认的事件复制到本地状态并启动保持窗口。 */
void AccidentDetector_ApplyRemoteEvent(uint8_t eventType, uint32_t eventId,
                                        uint32_t peakAccelMg, uint32_t peakGyroDps)
{
  /* 只接受定义范围内的碰撞或翻车事件。 */
  if ((eventType != ACCIDENT_EVENT_COLLISION) &&
      (eventType != ACCIDENT_EVENT_ROLLOVER))
  {
    /* 忽略未知事件类型，避免污染 A 节点上报数据。 */
    return;
  }
  /* 使用统一事件记录逻辑更新计数和保持截止时间。 */
  Accident_Raise(eventType, peakAccelMg, peakGyroDps);
  /* 用 B 节点的事件编号覆盖本地临时编号，确保云端能够去重。 */
  accidentStatus.lastEventId = eventId;
  /* 远端事件不是本地 IMU 采样，明确标记输入有效性为未知。 */
  accidentStatus.imuValid = 0U;
}

/* 从 Flash 记录恢复事件编号生成器，避免重启后从一重新编号。 */
void AccidentDetector_RestoreLastEventId(uint32_t lastEventId)
{
  /* 仅在 Flash 中存在更大的有效编号时推进编号生成器。 */
  if (lastEventId > nextEventId)
  {
    /* 保存已经分配过的最大编号，下一次新事件会在此基础上递增。 */
    nextEventId = lastEventId;
  }
}

/* 复制当前车辆意外检测状态。 */
void AccidentDetector_GetStatus(volatile AccidentDetectorStatus_t *status)
{
  /* 检查输出指针。 */
  if (status == NULL)
  {
    /* 无效指针直接返回。 */
    return;
  }
  /* 复制完整状态快照。 */
  *status = accidentStatus;
}

/* 返回适合显示或上报的意外类型文本。 */
const char *AccidentDetector_GetEventText(uint8_t eventType)
{
  /* 返回碰撞文本。 */
  if (eventType == ACCIDENT_EVENT_COLLISION)
  {
    /* 返回碰撞英文标识。 */
    return "COLLISION";
  }
  /* 返回翻车文本。 */
  if (eventType == ACCIDENT_EVENT_ROLLOVER)
  {
    /* 返回翻车英文标识。 */
    return "ROLLOVER";
  }
  /* 返回无事件文本。 */
  return "NONE";
}
