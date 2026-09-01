#ifndef ACCIDENT_DETECTOR_H
#define ACCIDENT_DETECTOR_H

#include "icm42688.h"
#include <stdint.h>

typedef enum
{
  ACCIDENT_EVENT_NONE = 0U, /* 褰撳墠娌℃湁妫€娴嬪埌鎰忓銆?*/
  ACCIDENT_EVENT_COLLISION = 1U, /* 妫€娴嬪埌鍓х儓纰版挒鎴栧啿鍑汇€?*/
  ACCIDENT_EVENT_ROLLOVER = 2U /* 妫€娴嬪埌鐤戜技缈昏溅濮挎€併€?*/
} AccidentEventType_t;

typedef struct
{
  uint8_t active; /* 鏍囪褰撳墠鏄惁澶勪簬鎰忓淇濇寔绐楀彛銆?*/
  uint8_t eventType; /* 淇濆瓨褰撳墠鎰忓绫诲瀷銆?*/
  uint8_t imuValid; /* 鏍囪鏈杈撳叆 IMU 鏁版嵁鏄惁鏈夋晥銆?*/
  uint8_t rolloverCandidate; /* 鏍囪褰撳墠鏄惁姝ｅ湪纭缈昏溅濮挎€併€?*/
  uint32_t collisionCount; /* 淇濆瓨绱纰版挒浜嬩欢娆℃暟銆?*/
  uint32_t rolloverCount; /* 淇濆瓨绱缈昏溅浜嬩欢娆℃暟銆?*/
  uint32_t lastEventTick; /* 淇濆瓨鏈€杩戜竴娆′簨浠跺彂鐢熺殑 HAL 鏃堕棿鎴炽€?*/
  uint32_t eventHoldUntilTick; /* 淇濆瓨褰撳墠浜嬩欢淇濇寔绐楀彛缁撴潫鏃堕棿銆?*/
  uint32_t peakAccelMg; /* 淇濆瓨鏈€杩戜竴娆′簨浠舵湡闂寸殑鏈€澶у姞閫熷害锛屽崟浣?mg銆?*/
  uint32_t peakGyroDps; /* 淇濆瓨鏈€杩戜竴娆′簨浠舵湡闂寸殑鏈€澶ц閫熷害锛屽崟浣?dps銆?*/
  uint32_t currentAccelMg; /* 淇濆瓨褰撳墠鍔犻€熷害鍚堟垚鍊硷紝鍗曚綅 mg銆?*/
  uint32_t currentGyroDps; /* 淇濆瓨褰撳墠瑙掗€熷害鍚堟垚鍊硷紝鍗曚綅 dps銆?*/
  uint32_t lastEventId; /* 淇濆瓨鏈€杩戜竴娆′簨浠剁殑閫掑缂栧彿锛岀敤浜?CAN/MQTT 鍘婚噸銆?*/
} AccidentDetectorStatus_t;

void AccidentDetector_Init(void);
void AccidentDetector_Update(const ICM42688_RawData_t *data, uint8_t valid);
void AccidentDetector_ApplyRemoteEvent(uint8_t eventType, uint32_t eventId,
                                        uint32_t peakAccelMg, uint32_t peakGyroDps);
void AccidentDetector_RestoreLastEventId(uint32_t lastEventId);
void AccidentDetector_GetStatus(volatile AccidentDetectorStatus_t *status);
const char *AccidentDetector_GetEventText(uint8_t eventType);

#endif /* ACCIDENT_DETECTOR_H */
