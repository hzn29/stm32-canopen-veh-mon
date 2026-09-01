#include "accident_detector.h"
#include "stm32g4xx_hal.h"
#include <string.h>

#define ACCIDENT_ACCEL_LSB_PER_G 2048L
#define ACCIDENT_GYRO_LSB_PER_DPS 16L
#define ACCIDENT_COLLISION_ACCEL_MG 6000UL
#define ACCIDENT_ROLLOVER_Z_MG 500UL
#define ACCIDENT_ROLLOVER_HORIZONTAL_MG 750UL
#define ACCIDENT_ROLLOVER_INVERTED_Z_RAW (-1434L)
#define ACCIDENT_COLLISION_CONFIRM_SAMPLES 1U
#define ACCIDENT_ROLLOVER_CONFIRM_SAMPLES 500U
#define ACCIDENT_EVENT_HOLD_MS 10000UL

static volatile AccidentDetectorStatus_t accidentStatus;
static uint16_t collisionConfirmSamples;
static uint16_t rolloverConfirmSamples;
static uint32_t nextEventId;

static uint32_t Accident_Abs(int16_t value)
{
  int32_t signedValue = (int32_t)value;
  return (uint32_t)((signedValue < 0) ? -signedValue : signedValue);
}

static uint32_t Accident_AccelMagnitudeMg(const ICM42688_RawData_t *data)
{
  uint32_t xMg = (Accident_Abs(data->accelX) * 1000UL) / ACCIDENT_ACCEL_LSB_PER_G;
  uint32_t yMg = (Accident_Abs(data->accelY) * 1000UL) / ACCIDENT_ACCEL_LSB_PER_G;
  uint32_t zMg = (Accident_Abs(data->accelZ) * 1000UL) / ACCIDENT_ACCEL_LSB_PER_G;
  uint32_t maximum = xMg;
  uint32_t middle = yMg;
  uint32_t minimum = zMg;
  if (maximum < middle)
  {
    uint32_t temporary = maximum;
    maximum = middle;
    middle = temporary;
  }
  if (maximum < minimum)
  {
    uint32_t temporary = maximum;
    maximum = minimum;
    minimum = temporary;
  }
  if (middle < minimum)
  {
    uint32_t temporary = middle;
    middle = minimum;
    minimum = temporary;
  }
  uint32_t magnitudeMg = maximum + (middle / 2U) + (minimum / 4U);
  return (magnitudeMg >= ACCIDENT_COLLISION_ACCEL_MG) ? ACCIDENT_COLLISION_ACCEL_MG : magnitudeMg;
}

static uint32_t Accident_GyroMagnitudeDps(const ICM42688_RawData_t *data)
{
  uint32_t gyroX = Accident_Abs(data->gyroX) / ACCIDENT_GYRO_LSB_PER_DPS;
  uint32_t gyroY = Accident_Abs(data->gyroY) / ACCIDENT_GYRO_LSB_PER_DPS;
  uint32_t gyroZ = Accident_Abs(data->gyroZ) / ACCIDENT_GYRO_LSB_PER_DPS;
  uint32_t maximum = (gyroX > gyroY) ? gyroX : gyroY;
  return (gyroZ > maximum) ? gyroZ : maximum;
}

static void Accident_Raise(uint8_t eventType, uint32_t accelMg, uint32_t gyroDps)
{
  uint32_t nowTick = HAL_GetTick();
  if ((accidentStatus.active != 0U) && (accidentStatus.eventType == eventType))
  {
    accidentStatus.eventHoldUntilTick = nowTick + ACCIDENT_EVENT_HOLD_MS;
    if (accelMg > accidentStatus.peakAccelMg)
    {
      accidentStatus.peakAccelMg = accelMg;
    }
    if (gyroDps > accidentStatus.peakGyroDps)
    {
      accidentStatus.peakGyroDps = gyroDps;
    }
    return;
  }
  accidentStatus.active = 1U;
  accidentStatus.eventType = eventType;
  accidentStatus.lastEventTick = nowTick;
  nextEventId++;
  accidentStatus.lastEventId = nextEventId;
  accidentStatus.eventHoldUntilTick = nowTick + ACCIDENT_EVENT_HOLD_MS;
  accidentStatus.peakAccelMg = accelMg;
  accidentStatus.peakGyroDps = gyroDps;
  if (eventType == ACCIDENT_EVENT_COLLISION)
  {
    accidentStatus.collisionCount++;
  }
  else if (eventType == ACCIDENT_EVENT_ROLLOVER)
  {
    accidentStatus.rolloverCount++;
  }
  else
  {
  }
}

void AccidentDetector_Init(void)
{
  (void)memset((void *)&accidentStatus, 0, sizeof(accidentStatus));
  collisionConfirmSamples = 0U;
  rolloverConfirmSamples = 0U;
  nextEventId = 0U;
}

void AccidentDetector_Update(const ICM42688_RawData_t *data, uint8_t valid)
{
  uint32_t nowTick = HAL_GetTick();
  uint32_t horizontalMgSquare = 0U;
  uint32_t horizontalThresholdSquare = ACCIDENT_ROLLOVER_HORIZONTAL_MG * ACCIDENT_ROLLOVER_HORIZONTAL_MG;
  uint32_t zMg = 0U;
  if ((accidentStatus.active != 0U) && ((int32_t)(nowTick - accidentStatus.eventHoldUntilTick) >= 0))
  {
    accidentStatus.active = 0U;
    accidentStatus.eventType = ACCIDENT_EVENT_NONE;
  }
  if ((data == NULL) || (valid == 0U))
  {
    accidentStatus.imuValid = 0U;
    return;
  }
  accidentStatus.imuValid = 1U;
  accidentStatus.currentAccelMg = Accident_AccelMagnitudeMg(data);
  accidentStatus.currentGyroDps = Accident_GyroMagnitudeDps(data);
  if (accidentStatus.currentAccelMg >= ACCIDENT_COLLISION_ACCEL_MG)
  {
    if (collisionConfirmSamples < ACCIDENT_COLLISION_CONFIRM_SAMPLES)
    {
      collisionConfirmSamples++;
    }
    if (collisionConfirmSamples >= ACCIDENT_COLLISION_CONFIRM_SAMPLES)
    {
      Accident_Raise(ACCIDENT_EVENT_COLLISION, accidentStatus.currentAccelMg, accidentStatus.currentGyroDps);
      collisionConfirmSamples = 0U;
    }
  }
  else
  {
    collisionConfirmSamples = 0U;
  }
  horizontalMgSquare = (uint32_t)(((uint64_t)Accident_Abs(data->accelX) * 1000UL / ACCIDENT_ACCEL_LSB_PER_G) *
                                  ((uint64_t)Accident_Abs(data->accelX) * 1000UL / ACCIDENT_ACCEL_LSB_PER_G) +
                                  ((uint64_t)Accident_Abs(data->accelY) * 1000UL / ACCIDENT_ACCEL_LSB_PER_G) *
                                  ((uint64_t)Accident_Abs(data->accelY) * 1000UL / ACCIDENT_ACCEL_LSB_PER_G));
  zMg = (Accident_Abs(data->accelZ) * 1000UL) / ACCIDENT_ACCEL_LSB_PER_G;
  if (((zMg <= ACCIDENT_ROLLOVER_Z_MG) && (horizontalMgSquare >= horizontalThresholdSquare)) ||
      ((int32_t)data->accelZ <= ACCIDENT_ROLLOVER_INVERTED_Z_RAW))
  {
    if (rolloverConfirmSamples < ACCIDENT_ROLLOVER_CONFIRM_SAMPLES)
    {
      rolloverConfirmSamples++;
    }
    if (rolloverConfirmSamples >= ACCIDENT_ROLLOVER_CONFIRM_SAMPLES)
    {
      Accident_Raise(ACCIDENT_EVENT_ROLLOVER, accidentStatus.currentAccelMg, accidentStatus.currentGyroDps);
      rolloverConfirmSamples = 0U;
      accidentStatus.rolloverCandidate = 0U;
    }
    else
    {
      accidentStatus.rolloverCandidate = 1U;
    }
  }
  else
  {
    rolloverConfirmSamples = 0U;
    accidentStatus.rolloverCandidate = 0U;
  }
}

void AccidentDetector_ApplyRemoteEvent(uint8_t eventType, uint32_t eventId,
                                        uint32_t peakAccelMg, uint32_t peakGyroDps)
{
  if ((eventType != ACCIDENT_EVENT_COLLISION) &&
      (eventType != ACCIDENT_EVENT_ROLLOVER))
  {
    return;
  }
  Accident_Raise(eventType, peakAccelMg, peakGyroDps);
  accidentStatus.lastEventId = eventId;
  accidentStatus.imuValid = 0U;
}

void AccidentDetector_RestoreLastEventId(uint32_t lastEventId)
{
  if (lastEventId > nextEventId)
  {
    nextEventId = lastEventId;
  }
}

void AccidentDetector_GetStatus(volatile AccidentDetectorStatus_t *status)
{
  if (status == NULL)
  {
    return;
  }
  *status = accidentStatus;
}

const char *AccidentDetector_GetEventText(uint8_t eventType)
{
  if (eventType == ACCIDENT_EVENT_COLLISION)
  {
    return "COLLISION";
  }
  if (eventType == ACCIDENT_EVENT_ROLLOVER)
  {
    return "ROLLOVER";
  }
  return "NONE";
}
