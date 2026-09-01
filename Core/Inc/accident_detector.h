/* 基于 ICM-42688 数据的车辆碰撞与翻车检测接口。 */
#ifndef ACCIDENT_DETECTOR_H
#define ACCIDENT_DETECTOR_H

/* 引入 ICM-42688 原始数据类型。 */
#include "icm42688.h"
/* 引入固定宽度整数类型。 */
#include <stdint.h>

/* 定义车辆意外事件类型。 */
typedef enum
{
  ACCIDENT_EVENT_NONE = 0U, /* 当前没有检测到意外。 */
  ACCIDENT_EVENT_COLLISION = 1U, /* 检测到剧烈碰撞或冲击。 */
  ACCIDENT_EVENT_ROLLOVER = 2U /* 检测到疑似翻车姿态。 */
} AccidentEventType_t;

/* 保存碰撞与翻车检测结果，供 OLED、MQTT 和调试器读取。 */
typedef struct
{
  uint8_t active; /* 标记当前是否处于意外保持窗口。 */
  uint8_t eventType; /* 保存当前意外类型。 */
  uint8_t imuValid; /* 标记本次输入 IMU 数据是否有效。 */
  uint8_t rolloverCandidate; /* 标记当前是否正在确认翻车姿态。 */
  uint32_t collisionCount; /* 保存累计碰撞事件次数。 */
  uint32_t rolloverCount; /* 保存累计翻车事件次数。 */
  uint32_t lastEventTick; /* 保存最近一次事件发生的 HAL 时间戳。 */
  uint32_t eventHoldUntilTick; /* 保存当前事件保持窗口结束时间。 */
  uint32_t peakAccelMg; /* 保存最近一次事件期间的最大加速度，单位 mg。 */
  uint32_t peakGyroDps; /* 保存最近一次事件期间的最大角速度，单位 dps。 */
  uint32_t currentAccelMg; /* 保存当前加速度合成值，单位 mg。 */
  uint32_t currentGyroDps; /* 保存当前角速度合成值，单位 dps。 */
  uint32_t lastEventId; /* 保存最近一次事件的递增编号，用于 CAN/MQTT 去重。 */
} AccidentDetectorStatus_t;

/* 初始化车辆意外检测状态。 */
void AccidentDetector_Init(void);
/* 使用一帧 IMU 原始数据更新碰撞和翻车判断。 */
void AccidentDetector_Update(const ICM42688_RawData_t *data, uint8_t valid);
/* 将 B 节点通过 CANopen 发送的事件快照应用到 A 节点本地状态。 */
void AccidentDetector_ApplyRemoteEvent(uint8_t eventType, uint32_t eventId,
                                        uint32_t peakAccelMg, uint32_t peakGyroDps);
/* 从掉电保存记录恢复本节点已经分配过的最大事件编号。 */
void AccidentDetector_RestoreLastEventId(uint32_t lastEventId);
/* 复制当前车辆意外检测状态。 */
void AccidentDetector_GetStatus(volatile AccidentDetectorStatus_t *status);
/* 返回适合显示或上报的意外类型文本。 */
const char *AccidentDetector_GetEventText(uint8_t eventType);

#endif /* ACCIDENT_DETECTOR_H */
