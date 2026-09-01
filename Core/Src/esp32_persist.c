/* ESP32 MQTT 运行参数的 STM32G4 Flash 持久化实现。 */
#include "esp32_persist.h"
/* 引入 MQTT 默认周期，用于事故记录保存时保留完整参数记录。 */
#include "esp32_mqtt_config.h"
/* 引入内存复制函数，保证双字写入缓冲区按 64 位对齐。 */
#include <string.h>

/* STM32G474RE 512 KB Flash 的最后一个 2 KB 页作为参数区。 */
#define ESP32_PERSIST_FLASH_ADDRESS 0x0807F800U
/* 参数记录的有效标识。 */
#define ESP32_PERSIST_MAGIC 0x45535032UL
/* 参数记录的版本号。 */
#define ESP32_PERSIST_VERSION 2U

/* 定义写入 Flash 的参数记录格式。 */
typedef struct
{
  uint32_t magic; /* 保存记录有效标识。 */
  uint32_t version; /* 保存记录版本号。 */
  uint32_t periodMs; /* 保存 MQTT 发布周期。 */
  uint32_t lastEventId; /* 保存已经分配过的最大事故事件编号。 */
  uint32_t pendingEventId; /* 保存尚未收到 ACK 的事故事件编号。 */
  uint32_t pendingPeakAccelMg; /* 保存待确认事件的加速度峰值。 */
  uint32_t pendingPeakGyroDps; /* 保存待确认事件的角速度峰值。 */
  uint32_t pendingMeta; /* 保存事件类型、有效标志和重试次数。 */
  uint32_t reserved; /* 保留字段，保证双字写入和版本扩展空间。 */
  uint32_t crc; /* 保存前八个字段的校验值。 */
} ESP32_PersistRecord_t;

/* 计算参数记录的简单 CRC32 校验值。 */
static uint32_t ESP32_Persist_Crc(const uint32_t *data, uint32_t words)
{
  /* 使用固定初值避免全零记录被误判为有效。 */
  uint32_t crc = 0xFFFFFFFFUL;
  /* 保存当前处理的字。 */
  uint32_t index = 0U;
  /* 逐字节计算 CRC32。 */
  for (index = 0U; index < (words * 4U); index++)
  {
    /* 将当前字节异或到 CRC 低八位。 */
    crc ^= ((const uint8_t *)data)[index];
    /* 处理当前字节的八个比特。 */
    for (uint32_t bit = 0U; bit < 8U; bit++)
    {
      /* 按 CRC32 多项式推进寄存器。 */
      crc = ((crc & 1U) != 0U) ? ((crc >> 1U) ^ 0xEDB88320UL) : (crc >> 1U);
    }
  }
  /* 返回最终 CRC32 值。 */
  return crc ^ 0xFFFFFFFFUL;
}

/* 从保留 Flash 页读取发布周期，失败时返回默认值。 */
HAL_StatusTypeDef ESP32_Persist_LoadPeriod(uint32_t defaultPeriodMs, uint32_t *periodMs)
{
  /* 将 Flash 地址解释为只读参数记录。 */
  const ESP32_PersistRecord_t *record = (const ESP32_PersistRecord_t *)ESP32_PERSIST_FLASH_ADDRESS;
  /* 检查输出指针。 */
  if (periodMs == NULL)
  {
    /* 输出指针无效时返回参数错误。 */
    return HAL_ERROR;
  }
  /* 默认使用编译期发布周期。 */
  *periodMs = defaultPeriodMs;
  /* 检查记录标识、版本和校验值。 */
  if ((record->magic != ESP32_PERSIST_MAGIC) ||
      (record->version != ESP32_PERSIST_VERSION) ||
      (record->crc != ESP32_Persist_Crc(&record->magic, 8U)))
  {
    /* 无有效记录时返回未初始化状态。 */
    return HAL_ERROR;
  }
  /* 返回 Flash 中保存的发布周期。 */
  *periodMs = record->periodMs;
  /* 返回读取成功。 */
  return HAL_OK;
}

/* 将发布周期写入保留 Flash 页。 */
HAL_StatusTypeDef ESP32_Persist_SavePeriod(uint32_t periodMs)
{
  /* 定义待写入的参数记录并清零。 */
  ESP32_PersistRecord_t record = {0};
  /* 定义 Flash 擦除参数。 */
  FLASH_EraseInitTypeDef erase = {0};
  /* 保存擦除失败页编号。 */
  uint32_t pageError = 0U;
  /* 保存当前写入的双字索引。 */
  uint32_t index = 0U;
  /* 定义按 64 位对齐的 Flash 写入缓冲区。 */
  uint64_t flashWords[5] = {0U, 0U, 0U, 0U, 0U};
  /* 保存当前写入地址。 */
  uint32_t address = 0U;
  /* 保存当前双字数据。 */
  uint64_t value = 0U;
  /* 读取旧记录，保存事故记录更新时当前 MQTT 周期。 */
  const ESP32_PersistRecord_t *oldRecord = (const ESP32_PersistRecord_t *)ESP32_PERSIST_FLASH_ADDRESS;
  /* 填充参数记录字段。 */
  record.magic = ESP32_PERSIST_MAGIC;
  /* 填充记录版本号。 */
  record.version = ESP32_PERSIST_VERSION;
  /* 填充发布周期。 */
  record.periodMs = periodMs;
  /* 读取并保留当前有效事故字段，避免修改周期时覆盖待确认事件。 */
  /* 仅在旧记录头、版本和 CRC 均有效时复制事故字段。 */
  if ((oldRecord->magic == ESP32_PERSIST_MAGIC) &&
      (oldRecord->version == ESP32_PERSIST_VERSION) &&
      (oldRecord->crc == ESP32_Persist_Crc(&oldRecord->magic, 8U)))
  {
    /* 保留事件编号和待确认事件，但保持本次请求传入的新发布周期。 */
    record.lastEventId = oldRecord->lastEventId;
    record.pendingEventId = oldRecord->pendingEventId;
    record.pendingPeakAccelMg = oldRecord->pendingPeakAccelMg;
    record.pendingPeakGyroDps = oldRecord->pendingPeakGyroDps;
    record.pendingMeta = oldRecord->pendingMeta;
  }
  /* 计算记录校验值。 */
  record.crc = ESP32_Persist_Crc(&record.magic, 8U);
  /* 将记录复制到 64 位对齐的写入缓冲区。 */
  (void)memcpy(flashWords, &record, sizeof(record));
  /* 解锁 Flash 编程控制器。 */
  if (HAL_FLASH_Unlock() != HAL_OK)
  {
    /* 解锁失败时返回错误。 */
    return HAL_ERROR;
  }
  /* 配置按页擦除最后一页参数区。 */
  erase.TypeErase = FLASH_TYPEERASE_PAGES;
  /* 根据芯片是否启用双 Bank 选择参数页所在 Bank。 */
#if defined(FLASH_OPTR_DBANK)
  /* 选择最后地址所属的 Flash Bank。 */
  erase.Banks = (ESP32_PERSIST_FLASH_ADDRESS < (FLASH_BASE + FLASH_BANK_SIZE)) ?
                FLASH_BANK_1 : FLASH_BANK_2;
  /* 计算 Bank 内的页号。 */
  erase.Page = ((ESP32_PERSIST_FLASH_ADDRESS - FLASH_BASE) % FLASH_BANK_SIZE) /
               FLASH_PAGE_SIZE;
#else
  /* 单 Bank 器件选择 Bank 1。 */
  erase.Banks = FLASH_BANK_1;
  /* 计算单 Bank 页号。 */
  erase.Page = (ESP32_PERSIST_FLASH_ADDRESS - FLASH_BASE) / FLASH_PAGE_SIZE;
#endif
  /* 只擦除一个参数页。 */
  erase.NbPages = 1U;
  /* 擦除参数页。 */
  if (HAL_FLASHEx_Erase(&erase, &pageError) != HAL_OK)
  {
    /* 锁定 Flash 并返回擦除失败。 */
    (void)HAL_FLASH_Lock();
    return HAL_ERROR;
  }
  /* 按 STM32G4 要求以 64 位双字写入参数记录。 */
  for (index = 0U; index < ((sizeof(record) + sizeof(uint64_t) - 1U) / sizeof(uint64_t)); index++)
  {
    /* 计算当前双字的 Flash 地址。 */
    address = ESP32_PERSIST_FLASH_ADDRESS + (index * sizeof(uint64_t));
    /* 取出当前双字数据。 */
    value = flashWords[index];
    /* 写入当前双字。 */
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, value) != HAL_OK)
    {
      /* 锁定 Flash 并返回写入失败。 */
      (void)HAL_FLASH_Lock();
      return HAL_ERROR;
    }
  }
  /* 重新锁定 Flash 编程控制器。 */
  (void)HAL_FLASH_Lock();
  /* 返回保存成功。 */
  return HAL_OK;
}

/* 从参数页读取事故编号和待确认事件快照。 */
HAL_StatusTypeDef ESP32_Persist_LoadAccident(ESP32_AccidentPersist_t *snapshot)
{
  /* 将 Flash 参数区解释为只读记录。 */
  const ESP32_PersistRecord_t *record = (const ESP32_PersistRecord_t *)ESP32_PERSIST_FLASH_ADDRESS;
  /* 检查输出指针。 */
  if (snapshot == NULL)
  {
    /* 输出指针无效时返回错误。 */
    return HAL_ERROR;
  }
  /* 先清零输出，保证无效记录不会遗留随机状态。 */
  (void)memset(snapshot, 0, sizeof(*snapshot));
  /* 校验版本和 CRC。 */
  if ((record->magic != ESP32_PERSIST_MAGIC) ||
      (record->version != ESP32_PERSIST_VERSION) ||
      (record->crc != ESP32_Persist_Crc(&record->magic, 8U)))
  {
    /* 无效记录不能恢复事故状态。 */
    return HAL_ERROR;
  }
  /* 复制已经分配过的最大事件编号。 */
  snapshot->lastEventId = record->lastEventId;
  /* 复制待确认事件编号和峰值。 */
  snapshot->pendingEventId = record->pendingEventId;
  snapshot->pendingPeakAccelMg = record->pendingPeakAccelMg;
  snapshot->pendingPeakGyroDps = record->pendingPeakGyroDps;
  /* 从元字段解码事件类型、有效标志和重试次数。 */
  snapshot->pendingEventType = (uint8_t)(record->pendingMeta & 0xFFU);
  snapshot->pendingValid = (uint8_t)((record->pendingMeta >> 8U) & 0x01U);
  snapshot->pendingRetryCount = (uint8_t)((record->pendingMeta >> 16U) & 0xFFU);
  /* 返回恢复成功。 */
  return HAL_OK;
}

/* 将事故编号和待确认事件快照写入参数页。 */
HAL_StatusTypeDef ESP32_Persist_SaveAccident(const ESP32_AccidentPersist_t *snapshot)
{
  /* 保存合并后的 Flash 记录。 */
  ESP32_PersistRecord_t record = {0};
  /* 保存 Flash 擦除参数。 */
  FLASH_EraseInitTypeDef erase = {0};
  /* 保存擦除失败页编号。 */
  uint32_t pageError = 0U;
  /* 保存双字写入索引。 */
  uint32_t index = 0U;
  /* 保存 64 位写入缓冲。 */
  uint64_t flashWords[5] = {0U, 0U, 0U, 0U, 0U};
  /* 保存当前写入地址和值。 */
  uint32_t address = 0U;
  uint64_t value = 0U;
  /* 读取旧记录，保留当前 MQTT 周期参数。 */
  const ESP32_PersistRecord_t *oldRecord = (const ESP32_PersistRecord_t *)ESP32_PERSIST_FLASH_ADDRESS;
  /* 检查输入指针。 */
  if (snapshot == NULL)
  {
    /* 输入无效时返回错误。 */
    return HAL_ERROR;
  }
  /* 填充记录头和事故状态字段。 */
  record.magic = ESP32_PERSIST_MAGIC;
  record.version = ESP32_PERSIST_VERSION;
  record.periodMs = ESP32_MQTT_PUBLISH_PERIOD_MS;
  /* 旧记录有效时恢复原有 MQTT 周期。 */
  if ((oldRecord->magic == ESP32_PERSIST_MAGIC) &&
      (oldRecord->version == ESP32_PERSIST_VERSION) &&
      (oldRecord->crc == ESP32_Persist_Crc(&oldRecord->magic, 8U)))
  {
    /* 保留原有周期，避免事故状态写入覆盖下行设置。 */
    record.periodMs = oldRecord->periodMs;
  }
  record.lastEventId = snapshot->lastEventId;
  record.pendingEventId = snapshot->pendingEventId;
  record.pendingPeakAccelMg = snapshot->pendingPeakAccelMg;
  record.pendingPeakGyroDps = snapshot->pendingPeakGyroDps;
  record.pendingMeta = (uint32_t)snapshot->pendingEventType |
                       ((uint32_t)(snapshot->pendingValid & 0x01U) << 8U) |
                       ((uint32_t)snapshot->pendingRetryCount << 16U);
  /* 计算前八个字段的 CRC。 */
  record.crc = ESP32_Persist_Crc(&record.magic, 8U);
  /* 将记录复制到双字对齐缓冲区。 */
  (void)memcpy(flashWords, &record, sizeof(record));
  /* 解锁 Flash。 */
  if (HAL_FLASH_Unlock() != HAL_OK)
  {
    /* 解锁失败时返回错误。 */
    return HAL_ERROR;
  }
  /* 配置擦除最后一页参数区。 */
  erase.TypeErase = FLASH_TYPEERASE_PAGES;
#if defined(FLASH_OPTR_DBANK)
  /* 选择记录地址所在 Bank 和页号。 */
  erase.Banks = (ESP32_PERSIST_FLASH_ADDRESS < (FLASH_BASE + FLASH_BANK_SIZE)) ? FLASH_BANK_1 : FLASH_BANK_2;
  erase.Page = ((ESP32_PERSIST_FLASH_ADDRESS - FLASH_BASE) % FLASH_BANK_SIZE) / FLASH_PAGE_SIZE;
#else
  /* 单 Bank 器件固定使用 Bank 1。 */
  erase.Banks = FLASH_BANK_1;
  erase.Page = (ESP32_PERSIST_FLASH_ADDRESS - FLASH_BASE) / FLASH_PAGE_SIZE;
#endif
  erase.NbPages = 1U;
  /* 擦除旧参数记录。 */
  if (HAL_FLASHEx_Erase(&erase, &pageError) != HAL_OK)
  {
    /* 擦除失败时锁定 Flash 并返回错误。 */
    (void)HAL_FLASH_Lock();
    return HAL_ERROR;
  }
  /* 逐个双字写入新的事故记录。 */
  for (index = 0U; index < ((sizeof(record) + sizeof(uint64_t) - 1U) / sizeof(uint64_t)); index++)
  {
    /* 计算当前双字地址和值。 */
    address = ESP32_PERSIST_FLASH_ADDRESS + (index * sizeof(uint64_t));
    value = flashWords[index];
    /* 编程当前双字。 */
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, value) != HAL_OK)
    {
      /* 写入失败时锁定 Flash 并返回错误。 */
      (void)HAL_FLASH_Lock();
      return HAL_ERROR;
    }
  }
  /* 完成后重新锁定 Flash。 */
  (void)HAL_FLASH_Lock();
  /* 返回保存成功。 */
  return HAL_OK;
}
