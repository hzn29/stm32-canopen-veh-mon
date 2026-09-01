#include "esp32_persist.h"
#include "esp32_mqtt_config.h"
#include <string.h>

#define ESP32_PERSIST_FLASH_ADDRESS 0x0807F800U
#define ESP32_PERSIST_MAGIC 0x45535032UL
#define ESP32_PERSIST_VERSION 2U

typedef struct
{
  uint32_t magic; /* 淇濆瓨璁板綍鏈夋晥鏍囪瘑銆?*/
  uint32_t version; /* 淇濆瓨璁板綍鐗堟湰鍙枫€?*/
  uint32_t periodMs; /* 淇濆瓨 MQTT 鍙戝竷鍛ㄦ湡銆?*/
  uint32_t lastEventId; /* 淇濆瓨宸茬粡鍒嗛厤杩囩殑鏈€澶т簨鏁呬簨浠剁紪鍙枫€?*/
  uint32_t pendingEventId; /* 淇濆瓨灏氭湭鏀跺埌 ACK 鐨勪簨鏁呬簨浠剁紪鍙枫€?*/
  uint32_t pendingPeakAccelMg; /* 淇濆瓨寰呯‘璁や簨浠剁殑鍔犻€熷害宄板€笺€?*/
  uint32_t pendingPeakGyroDps; /* 淇濆瓨寰呯‘璁や簨浠剁殑瑙掗€熷害宄板€笺€?*/
  uint32_t pendingMeta; /* 淇濆瓨浜嬩欢绫诲瀷銆佹湁鏁堟爣蹇楀拰閲嶈瘯娆℃暟銆?*/
  uint32_t reserved; /* 淇濈暀瀛楁锛屼繚璇佸弻瀛楀啓鍏ュ拰鐗堟湰鎵╁睍绌洪棿銆?*/
  uint32_t crc; /* 淇濆瓨鍓嶅叓涓瓧娈电殑鏍￠獙鍊笺€?*/
} ESP32_PersistRecord_t;

static uint32_t ESP32_Persist_Crc(const uint32_t *data, uint32_t words)
{
  uint32_t crc = 0xFFFFFFFFUL;
  uint32_t index = 0U;
  for (index = 0U; index < (words * 4U); index++)
  {
    crc ^= ((const uint8_t *)data)[index];
    for (uint32_t bit = 0U; bit < 8U; bit++)
    {
      crc = ((crc & 1U) != 0U) ? ((crc >> 1U) ^ 0xEDB88320UL) : (crc >> 1U);
    }
  }
  return crc ^ 0xFFFFFFFFUL;
}

HAL_StatusTypeDef ESP32_Persist_LoadPeriod(uint32_t defaultPeriodMs, uint32_t *periodMs)
{
  const ESP32_PersistRecord_t *record = (const ESP32_PersistRecord_t *)ESP32_PERSIST_FLASH_ADDRESS;
  if (periodMs == NULL)
  {
    return HAL_ERROR;
  }
  *periodMs = defaultPeriodMs;
  if ((record->magic != ESP32_PERSIST_MAGIC) ||
      (record->version != ESP32_PERSIST_VERSION) ||
      (record->crc != ESP32_Persist_Crc(&record->magic, 8U)))
  {
    return HAL_ERROR;
  }
  *periodMs = record->periodMs;
  return HAL_OK;
}

HAL_StatusTypeDef ESP32_Persist_SavePeriod(uint32_t periodMs)
{
  ESP32_PersistRecord_t record = {0};
  FLASH_EraseInitTypeDef erase = {0};
  uint32_t pageError = 0U;
  uint32_t index = 0U;
  uint64_t flashWords[5] = {0U, 0U, 0U, 0U, 0U};
  uint32_t address = 0U;
  uint64_t value = 0U;
  const ESP32_PersistRecord_t *oldRecord = (const ESP32_PersistRecord_t *)ESP32_PERSIST_FLASH_ADDRESS;
  record.magic = ESP32_PERSIST_MAGIC;
  record.version = ESP32_PERSIST_VERSION;
  record.periodMs = periodMs;
  if ((oldRecord->magic == ESP32_PERSIST_MAGIC) &&
      (oldRecord->version == ESP32_PERSIST_VERSION) &&
      (oldRecord->crc == ESP32_Persist_Crc(&oldRecord->magic, 8U)))
  {
    record.lastEventId = oldRecord->lastEventId;
    record.pendingEventId = oldRecord->pendingEventId;
    record.pendingPeakAccelMg = oldRecord->pendingPeakAccelMg;
    record.pendingPeakGyroDps = oldRecord->pendingPeakGyroDps;
    record.pendingMeta = oldRecord->pendingMeta;
  }
  record.crc = ESP32_Persist_Crc(&record.magic, 8U);
  (void)memcpy(flashWords, &record, sizeof(record));
  if (HAL_FLASH_Unlock() != HAL_OK)
  {
    return HAL_ERROR;
  }
  erase.TypeErase = FLASH_TYPEERASE_PAGES;
#if defined(FLASH_OPTR_DBANK)
  erase.Banks = (ESP32_PERSIST_FLASH_ADDRESS < (FLASH_BASE + FLASH_BANK_SIZE)) ?
                FLASH_BANK_1 : FLASH_BANK_2;
  erase.Page = ((ESP32_PERSIST_FLASH_ADDRESS - FLASH_BASE) % FLASH_BANK_SIZE) /
               FLASH_PAGE_SIZE;
#else
  erase.Banks = FLASH_BANK_1;
  erase.Page = (ESP32_PERSIST_FLASH_ADDRESS - FLASH_BASE) / FLASH_PAGE_SIZE;
#endif
  erase.NbPages = 1U;
  if (HAL_FLASHEx_Erase(&erase, &pageError) != HAL_OK)
  {
    (void)HAL_FLASH_Lock();
    return HAL_ERROR;
  }
  for (index = 0U; index < ((sizeof(record) + sizeof(uint64_t) - 1U) / sizeof(uint64_t)); index++)
  {
    address = ESP32_PERSIST_FLASH_ADDRESS + (index * sizeof(uint64_t));
    value = flashWords[index];
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, value) != HAL_OK)
    {
      (void)HAL_FLASH_Lock();
      return HAL_ERROR;
    }
  }
  (void)HAL_FLASH_Lock();
  return HAL_OK;
}

HAL_StatusTypeDef ESP32_Persist_LoadAccident(ESP32_AccidentPersist_t *snapshot)
{
  const ESP32_PersistRecord_t *record = (const ESP32_PersistRecord_t *)ESP32_PERSIST_FLASH_ADDRESS;
  if (snapshot == NULL)
  {
    return HAL_ERROR;
  }
  (void)memset(snapshot, 0, sizeof(*snapshot));
  if ((record->magic != ESP32_PERSIST_MAGIC) ||
      (record->version != ESP32_PERSIST_VERSION) ||
      (record->crc != ESP32_Persist_Crc(&record->magic, 8U)))
  {
    return HAL_ERROR;
  }
  snapshot->lastEventId = record->lastEventId;
  snapshot->pendingEventId = record->pendingEventId;
  snapshot->pendingPeakAccelMg = record->pendingPeakAccelMg;
  snapshot->pendingPeakGyroDps = record->pendingPeakGyroDps;
  snapshot->pendingEventType = (uint8_t)(record->pendingMeta & 0xFFU);
  snapshot->pendingValid = (uint8_t)((record->pendingMeta >> 8U) & 0x01U);
  snapshot->pendingRetryCount = (uint8_t)((record->pendingMeta >> 16U) & 0xFFU);
  return HAL_OK;
}

HAL_StatusTypeDef ESP32_Persist_SaveAccident(const ESP32_AccidentPersist_t *snapshot)
{
  ESP32_PersistRecord_t record = {0};
  FLASH_EraseInitTypeDef erase = {0};
  uint32_t pageError = 0U;
  uint32_t index = 0U;
  uint64_t flashWords[5] = {0U, 0U, 0U, 0U, 0U};
  uint32_t address = 0U;
  uint64_t value = 0U;
  const ESP32_PersistRecord_t *oldRecord = (const ESP32_PersistRecord_t *)ESP32_PERSIST_FLASH_ADDRESS;
  if (snapshot == NULL)
  {
    return HAL_ERROR;
  }
  record.magic = ESP32_PERSIST_MAGIC;
  record.version = ESP32_PERSIST_VERSION;
  record.periodMs = ESP32_MQTT_PUBLISH_PERIOD_MS;
  if ((oldRecord->magic == ESP32_PERSIST_MAGIC) &&
      (oldRecord->version == ESP32_PERSIST_VERSION) &&
      (oldRecord->crc == ESP32_Persist_Crc(&oldRecord->magic, 8U)))
  {
    record.periodMs = oldRecord->periodMs;
  }
  record.lastEventId = snapshot->lastEventId;
  record.pendingEventId = snapshot->pendingEventId;
  record.pendingPeakAccelMg = snapshot->pendingPeakAccelMg;
  record.pendingPeakGyroDps = snapshot->pendingPeakGyroDps;
  record.pendingMeta = (uint32_t)snapshot->pendingEventType |
                       ((uint32_t)(snapshot->pendingValid & 0x01U) << 8U) |
                       ((uint32_t)snapshot->pendingRetryCount << 16U);
  record.crc = ESP32_Persist_Crc(&record.magic, 8U);
  (void)memcpy(flashWords, &record, sizeof(record));
  if (HAL_FLASH_Unlock() != HAL_OK)
  {
    return HAL_ERROR;
  }
  erase.TypeErase = FLASH_TYPEERASE_PAGES;
#if defined(FLASH_OPTR_DBANK)
  erase.Banks = (ESP32_PERSIST_FLASH_ADDRESS < (FLASH_BASE + FLASH_BANK_SIZE)) ? FLASH_BANK_1 : FLASH_BANK_2;
  erase.Page = ((ESP32_PERSIST_FLASH_ADDRESS - FLASH_BASE) % FLASH_BANK_SIZE) / FLASH_PAGE_SIZE;
#else
  erase.Banks = FLASH_BANK_1;
  erase.Page = (ESP32_PERSIST_FLASH_ADDRESS - FLASH_BASE) / FLASH_PAGE_SIZE;
#endif
  erase.NbPages = 1U;
  if (HAL_FLASHEx_Erase(&erase, &pageError) != HAL_OK)
  {
    (void)HAL_FLASH_Lock();
    return HAL_ERROR;
  }
  for (index = 0U; index < ((sizeof(record) + sizeof(uint64_t) - 1U) / sizeof(uint64_t)); index++)
  {
    address = ESP32_PERSIST_FLASH_ADDRESS + (index * sizeof(uint64_t));
    value = flashWords[index];
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, value) != HAL_OK)
    {
      (void)HAL_FLASH_Lock();
      return HAL_ERROR;
    }
  }
  (void)HAL_FLASH_Lock();
  return HAL_OK;
}
