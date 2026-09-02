#include "esp32_persist.h"
#include "esp32_mqtt_config.h"
#include <string.h>

#define ESP32_PERSIST_PAGE0 0x0807F000U
#define ESP32_PERSIST_PAGE1 0x0807F800U
#define ESP32_PERSIST_MAGIC 0x45535032UL
#define ESP32_PERSIST_VERSION 3U
#define ESP32_PERSIST_SLOT_SIZE 48U
#define ESP32_PERSIST_SLOTS_PER_PAGE (FLASH_PAGE_SIZE / ESP32_PERSIST_SLOT_SIZE)

typedef struct
{
  uint32_t magic;
  uint32_t version;
  uint32_t sequence;
  uint32_t periodMs;
  uint32_t lastEventId;
  uint32_t pendingEventId;
  uint32_t pendingPeakAccelMg;
  uint32_t pendingPeakGyroDps;
  uint32_t pendingMeta;
  uint32_t reserved;
  uint32_t crc;
} ESP32_PersistRecord_t;

static uint32_t ESP32_Persist_Crc(const uint32_t *data, uint32_t words)
{
  uint32_t crc = 0xFFFFFFFFUL;
  uint32_t index;
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

static uint8_t ESP32_Persist_RecordValid(const ESP32_PersistRecord_t *record)
{
  return (record != NULL) &&
         (record->magic == ESP32_PERSIST_MAGIC) &&
         (record->version == ESP32_PERSIST_VERSION) &&
         (record->crc == ESP32_Persist_Crc(&record->magic, 10U));
}

static const ESP32_PersistRecord_t *ESP32_Persist_FindLatest(uint32_t *address)
{
  const ESP32_PersistRecord_t *latest = NULL;
  uint32_t latestAddress = 0U;
  const uint32_t pages[2] = {ESP32_PERSIST_PAGE0, ESP32_PERSIST_PAGE1};
  for (uint32_t page = 0U; page < 2U; page++)
  {
    for (uint32_t slot = 0U; slot < ESP32_PERSIST_SLOTS_PER_PAGE; slot++)
    {
      uint32_t candidateAddress = pages[page] + (slot * ESP32_PERSIST_SLOT_SIZE);
      const ESP32_PersistRecord_t *candidate =
          (const ESP32_PersistRecord_t *)candidateAddress;
      if (ESP32_Persist_RecordValid(candidate) &&
          ((latest == NULL) || ((int32_t)(candidate->sequence - latest->sequence) > 0)))
      {
        latest = candidate;
        latestAddress = candidateAddress;
      }
    }
  }
  if (address != NULL)
  {
    *address = latestAddress;
  }
  return latest;
}

static uint32_t ESP32_Persist_FindFreeSlot(uint32_t pageAddress)
{
  for (uint32_t slot = 0U; slot < ESP32_PERSIST_SLOTS_PER_PAGE; slot++)
  {
    const uint32_t address = pageAddress + (slot * ESP32_PERSIST_SLOT_SIZE);
    const ESP32_PersistRecord_t *record = (const ESP32_PersistRecord_t *)address;
    if (record->magic == 0xFFFFFFFFUL)
    {
      return address;
    }
  }
  return 0U;
}

static HAL_StatusTypeDef ESP32_Persist_ErasePage(uint32_t pageAddress)
{
  FLASH_EraseInitTypeDef erase = {0};
  uint32_t pageError = 0U;
  erase.TypeErase = FLASH_TYPEERASE_PAGES;
#if defined(FLASH_OPTR_DBANK)
  erase.Banks = (pageAddress < (FLASH_BASE + FLASH_BANK_SIZE)) ? FLASH_BANK_1 : FLASH_BANK_2;
  erase.Page = ((pageAddress - FLASH_BASE) % FLASH_BANK_SIZE) / FLASH_PAGE_SIZE;
#else
  erase.Banks = FLASH_BANK_1;
  erase.Page = (pageAddress - FLASH_BASE) / FLASH_PAGE_SIZE;
#endif
  erase.NbPages = 1U;
  return (HAL_FLASHEx_Erase(&erase, &pageError) == HAL_OK) ? HAL_OK : HAL_ERROR;
}

static HAL_StatusTypeDef ESP32_Persist_WriteRecord(uint32_t address,
                                                   const ESP32_PersistRecord_t *record)
{
  uint64_t words[ESP32_PERSIST_SLOT_SIZE / sizeof(uint64_t)] = {0U};
  (void)memcpy(words, record, sizeof(*record));
  for (uint32_t index = 0U; index < (ESP32_PERSIST_SLOT_SIZE / sizeof(uint64_t)); index++)
  {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                          address + (index * sizeof(uint64_t)), words[index]) != HAL_OK)
    {
      return HAL_ERROR;
    }
  }
  return HAL_OK;
}

static HAL_StatusTypeDef ESP32_Persist_Append(const ESP32_PersistRecord_t *record)
{
  uint32_t latestAddress = 0U;
  const ESP32_PersistRecord_t *latest = ESP32_Persist_FindLatest(&latestAddress);
  uint32_t targetPage = ESP32_PERSIST_PAGE0;
  uint32_t targetAddress;
  if (latest != NULL)
  {
    targetPage = (latestAddress >= ESP32_PERSIST_PAGE1) ? ESP32_PERSIST_PAGE1 : ESP32_PERSIST_PAGE0;
  }
  targetAddress = ESP32_Persist_FindFreeSlot(targetPage);
  if (targetAddress == 0U)
  {
    targetPage = (targetPage == ESP32_PERSIST_PAGE0) ? ESP32_PERSIST_PAGE1 : ESP32_PERSIST_PAGE0;
    if (ESP32_Persist_ErasePage(targetPage) != HAL_OK)
    {
      return HAL_ERROR;
    }
    targetAddress = targetPage;
  }
  return ESP32_Persist_WriteRecord(targetAddress, record);
}

static HAL_StatusTypeDef ESP32_Persist_SaveRecord(ESP32_PersistRecord_t *record)
{
  const ESP32_PersistRecord_t *latest = ESP32_Persist_FindLatest(NULL);
  record->sequence = (latest == NULL) ? 1U : (latest->sequence + 1U);
  record->crc = ESP32_Persist_Crc(&record->magic, 10U);
  if (HAL_FLASH_Unlock() != HAL_OK)
  {
    return HAL_ERROR;
  }
  HAL_StatusTypeDef status = ESP32_Persist_Append(record);
  (void)HAL_FLASH_Lock();
  return status;
}

HAL_StatusTypeDef ESP32_Persist_LoadPeriod(uint32_t defaultPeriodMs, uint32_t *periodMs)
{
  const ESP32_PersistRecord_t *record = ESP32_Persist_FindLatest(NULL);
  if (periodMs == NULL)
  {
    return HAL_ERROR;
  }
  *periodMs = (record == NULL) ? defaultPeriodMs : record->periodMs;
  return (record == NULL) ? HAL_ERROR : HAL_OK;
}

HAL_StatusTypeDef ESP32_Persist_SavePeriod(uint32_t periodMs)
{
  ESP32_PersistRecord_t record = {0};
  const ESP32_PersistRecord_t *oldRecord = ESP32_Persist_FindLatest(NULL);
  record.magic = ESP32_PERSIST_MAGIC;
  record.version = ESP32_PERSIST_VERSION;
  record.periodMs = periodMs;
  if (oldRecord != NULL)
  {
    record.lastEventId = oldRecord->lastEventId;
    record.pendingEventId = oldRecord->pendingEventId;
    record.pendingPeakAccelMg = oldRecord->pendingPeakAccelMg;
    record.pendingPeakGyroDps = oldRecord->pendingPeakGyroDps;
    record.pendingMeta = oldRecord->pendingMeta;
  }
  return ESP32_Persist_SaveRecord(&record);
}

HAL_StatusTypeDef ESP32_Persist_LoadAccident(ESP32_AccidentPersist_t *snapshot)
{
  const ESP32_PersistRecord_t *record = ESP32_Persist_FindLatest(NULL);
  if (snapshot == NULL)
  {
    return HAL_ERROR;
  }
  (void)memset(snapshot, 0, sizeof(*snapshot));
  if (record == NULL)
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
  const ESP32_PersistRecord_t *oldRecord = ESP32_Persist_FindLatest(NULL);
  if (snapshot == NULL)
  {
    return HAL_ERROR;
  }
  record.magic = ESP32_PERSIST_MAGIC;
  record.version = ESP32_PERSIST_VERSION;
  record.periodMs = (oldRecord != NULL) ? oldRecord->periodMs : ESP32_MQTT_PUBLISH_PERIOD_MS;
  record.lastEventId = snapshot->lastEventId;
  record.pendingEventId = snapshot->pendingEventId;
  record.pendingPeakAccelMg = snapshot->pendingPeakAccelMg;
  record.pendingPeakGyroDps = snapshot->pendingPeakGyroDps;
  record.pendingMeta = (uint32_t)snapshot->pendingEventType |
                       ((uint32_t)(snapshot->pendingValid & 0x01U) << 8U) |
                       ((uint32_t)snapshot->pendingRetryCount << 16U);
  return ESP32_Persist_SaveRecord(&record);
}
