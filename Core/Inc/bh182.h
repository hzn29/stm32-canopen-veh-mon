#ifndef BH182_H
#define BH182_H

#include "main.h"

#define BH182_RX_BUFFER_SIZE 256U

typedef struct
{
  uint8_t fixValid;
  int32_t latitudeE7;
  int32_t longitudeE7;
  int32_t altitudeMm;
  uint32_t speedMmps;
  uint32_t satelliteCount;
  uint32_t sentenceCount;
  uint32_t checksumErrorCount;
  uint32_t parseErrorCount;
} BH182_Data_t;

HAL_StatusTypeDef BH182_Init(UART_HandleTypeDef *huart);

HAL_StatusTypeDef BH182_Start(void);

void BH182_OnRxEvent(uint16_t size);

void BH182_GetData(BH182_Data_t *data);

#endif /* BH182_H */
