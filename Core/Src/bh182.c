#include "bh182.h"

#include <string.h>
#include <stdlib.h>

#define BH182_LINE_BUFFER_SIZE 128U

static UART_HandleTypeDef *bh182Uart;
static uint8_t bh182RxBuffer[BH182_RX_BUFFER_SIZE];
static volatile uint16_t bh182LastRxPosition;
static char bh182LineBuffer[BH182_LINE_BUFFER_SIZE];
static uint16_t bh182LineLength;
static volatile BH182_Data_t bh182Data;

static uint8_t BH182_HexValue(char value)
{
  if ((value >= '0') && (value <= '9'))
  {
    return (uint8_t)(value - '0');
  }
  if ((value >= 'A') && (value <= 'F'))
  {
    return (uint8_t)(value - 'A' + 10U);
  }
  if ((value >= 'a') && (value <= 'f'))
  {
    return (uint8_t)(value - 'a' + 10U);
  }
  return 0xFFU;
}

static uint8_t BH182_ChecksumValid(char *line)
{
  char *star;
  uint8_t checksum = 0U;
  uint8_t receivedChecksum;
  char *cursor;
  if (line[0] != '$')
  {
    return 0U;
  }
  star = strchr(line, '*');
  if ((star == NULL) || (star[1] == '\0') || (star[2] == '\0'))
  {
    return 0U;
  }
  for (cursor = line + 1; cursor < star; cursor++)
  {
    checksum ^= (uint8_t)(*cursor);
  }
  receivedChecksum = (uint8_t)((BH182_HexValue(star[1]) << 4U) |
                               BH182_HexValue(star[2]));
  return ((BH182_HexValue(star[1]) != 0xFFU) &&
          (BH182_HexValue(star[2]) != 0xFFU) &&
          (checksum == receivedChecksum));
}

static uint8_t BH182_ParseCoordinate(const char *text, char direction, int32_t *value)
{
  uint32_t whole = 0U;
  uint32_t fraction = 0U;
  uint32_t scale = 1U;
  uint8_t afterDecimal = 0U;
  const char *cursor = text;
  uint32_t degrees;
  uint32_t minutes;
  uint64_t minutesE7;
  uint64_t coordinateE7;
  if ((text == NULL) || (value == NULL) ||
      ((direction != 'N') && (direction != 'S') &&
       (direction != 'E') && (direction != 'W')))
  {
    return 0U;
  }
  while (*cursor != '\0')
  {
    if (*cursor == '.')
    {
      afterDecimal = 1U;
    }
    else if ((*cursor >= '0') && (*cursor <= '9'))
    {
      if (afterDecimal == 0U)
      {
        whole = whole * 10U + (uint32_t)(*cursor - '0');
      }
      else if (scale < 1000000U)
      {
        fraction = fraction * 10U + (uint32_t)(*cursor - '0');
        scale *= 10U;
      }
    }
    else
    {
      return 0U;
    }
    cursor++;
  }
  degrees = whole / 100U;
  minutes = whole % 100U;
  minutesE7 = (uint64_t)minutes * 10000000ULL;
  minutesE7 += ((uint64_t)fraction * 10000000ULL) / scale;
  minutesE7 /= 60ULL;
  coordinateE7 = (uint64_t)degrees * 10000000ULL + minutesE7;
  if ((direction == 'S') || (direction == 'W'))
  {
    *value = -(int32_t)coordinateE7;
  }
  else
  {
    *value = (int32_t)coordinateE7;
  }
  return 1U;
}

static int32_t BH182_ParseMilli(const char *text)
{
  int32_t sign = 1;
  int32_t whole = 0;
  int32_t fraction = 0;
  uint8_t digits = 0U;
  const char *cursor = text;
  if (*cursor == '-')
  {
    sign = -1;
    cursor++;
  }
  while (*cursor != '\0')
  {
    if (*cursor == '.')
    {
      digits = 0x80U;
    }
    else if ((*cursor >= '0') && (*cursor <= '9'))
    {
      if ((digits & 0x80U) == 0U)
      {
        whole = whole * 10 + (int32_t)(*cursor - '0');
      }
      else if ((digits & 0x7FU) < 3U)
      {
        fraction = fraction * 10 + (int32_t)(*cursor - '0');
        digits++;
      }
    }
    else
    {
      break;
    }
    cursor++;
  }
  if ((digits & 0x7FU) == 0U)
  {
    fraction = 0;
  }
  else if ((digits & 0x7FU) == 1U)
  {
    fraction *= 100;
  }
  else if ((digits & 0x7FU) == 2U)
  {
    fraction *= 10;
  }
  return sign * (whole * 1000 + fraction);
}

static void BH182_ParseLine(char *line)
{
  char *fields[16] = {0};
  char *cursor;
  char *star;
  uint8_t fieldCount = 0U;
  int32_t coordinate;
  if (BH182_ChecksumValid(line) == 0U)
  {
    bh182Data.checksumErrorCount++;
    return;
  }
  star = strchr(line, '*');
  *star = '\0';
  fields[fieldCount++] = line + 1;
  for (cursor = line + 1; (*cursor != '\0') && (fieldCount < 16U); cursor++)
  {
    if (*cursor == ',')
    {
      *cursor = '\0';
      fields[fieldCount++] = cursor + 1;
    }
  }
  if (fieldCount == 0U)
  {
    bh182Data.parseErrorCount++;
    return;
  }
  if ((strncmp(fields[0], "GNRMC", 5U) == 0) ||
      (strncmp(fields[0], "GPRMC", 5U) == 0))
  {
    if (fieldCount < 8U)
    {
      bh182Data.parseErrorCount++;
      return;
    }
    bh182Data.fixValid = (fields[2][0] == 'A') ? 1U : 0U;
    if ((bh182Data.fixValid != 0U) &&
        (BH182_ParseCoordinate(fields[3], fields[4][0], &coordinate) != 0U))
    {
      bh182Data.latitudeE7 = coordinate;
    }
    if ((bh182Data.fixValid != 0U) &&
        (BH182_ParseCoordinate(fields[5], fields[6][0], &coordinate) != 0U))
    {
      bh182Data.longitudeE7 = coordinate;
    }
    if (bh182Data.fixValid != 0U)
    {
      bh182Data.speedMmps = (uint32_t)(((int64_t)BH182_ParseMilli(fields[7]) * 514444LL) / 1000000LL);
    }
    bh182Data.sentenceCount++;
  }
  else if ((strncmp(fields[0], "GNGGA", 5U) == 0) ||
           (strncmp(fields[0], "GPGGA", 5U) == 0))
  {
    if (fieldCount < 10U)
    {
      bh182Data.parseErrorCount++;
      return;
    }
    bh182Data.fixValid = (strtoul(fields[6], NULL, 10) != 0U) ? 1U : 0U;
    bh182Data.satelliteCount = strtoul(fields[7], NULL, 10);
    bh182Data.altitudeMm = BH182_ParseMilli(fields[9]) * 1000;
    bh182Data.sentenceCount++;
  }
}

static void BH182_PushByte(uint8_t byte)
{
  if (byte == '$')
  {
    bh182LineLength = 0U;
    bh182LineBuffer[bh182LineLength++] = (char)byte;
    return;
  }
  if (bh182LineLength == 0U)
  {
    return;
  }
  if (byte == '\r')
  {
    return;
  }
  if (byte == '\n')
  {
    bh182LineBuffer[bh182LineLength] = '\0';
    BH182_ParseLine(bh182LineBuffer);
    bh182LineLength = 0U;
    return;
  }
  if (bh182LineLength < (BH182_LINE_BUFFER_SIZE - 1U))
  {
    bh182LineBuffer[bh182LineLength++] = (char)byte;
  }
  else
  {
    bh182LineLength = 0U;
    bh182Data.parseErrorCount++;
  }
}

HAL_StatusTypeDef BH182_Init(UART_HandleTypeDef *huart)
{
  if (huart == NULL)
  {
    return HAL_ERROR;
  }
  bh182Uart = huart;
  bh182LastRxPosition = 0U;
  bh182LineLength = 0U;
  memset((void *)bh182RxBuffer, 0, sizeof(bh182RxBuffer));
  memset((void *)bh182LineBuffer, 0, sizeof(bh182LineBuffer));
  memset((void *)&bh182Data, 0, sizeof(bh182Data));
  return HAL_OK;
}

HAL_StatusTypeDef BH182_Start(void)
{
  if (bh182Uart == NULL)
  {
    return HAL_ERROR;
  }
  if (HAL_UARTEx_ReceiveToIdle_DMA(bh182Uart,
                                   bh182RxBuffer,
                                   BH182_RX_BUFFER_SIZE) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (bh182Uart->hdmarx != NULL)
  {
    __HAL_DMA_DISABLE_IT(bh182Uart->hdmarx, DMA_IT_HT);
  }
  return HAL_OK;
}

void BH182_OnRxEvent(uint16_t size)
{
  uint16_t writePosition = size;
  uint16_t readPosition = bh182LastRxPosition;
  uint16_t availableBytes = 0U;
  HAL_UART_RxEventTypeTypeDef eventType;
  if (bh182Uart == NULL)
  {
    return;
  }
  eventType = HAL_UARTEx_GetRxEventType(bh182Uart);
  if (writePosition >= BH182_RX_BUFFER_SIZE)
  {
    writePosition = 0U;
  }
  if (writePosition > readPosition)
  {
    availableBytes = writePosition - readPosition;
  }
  else if (writePosition < readPosition)
  {
    availableBytes = (BH182_RX_BUFFER_SIZE - readPosition) + writePosition;
  }
  else if (eventType == HAL_UART_RXEVENT_TC)
  {
    availableBytes = BH182_RX_BUFFER_SIZE;
  }
  while (availableBytes > 0U)
  {
    BH182_PushByte(bh182RxBuffer[readPosition]);
    availableBytes--;
    readPosition++;
    if (readPosition >= BH182_RX_BUFFER_SIZE)
    {
      readPosition = 0U;
    }
  }
  bh182LastRxPosition = writePosition;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart == bh182Uart)
  {
    BH182_OnRxEvent(Size);
  }
}

void BH182_GetData(BH182_Data_t *data)
{
  if (data == NULL)
  {
    return;
  }
  __disable_irq();
  *data = bh182Data;
  __enable_irq();
}
