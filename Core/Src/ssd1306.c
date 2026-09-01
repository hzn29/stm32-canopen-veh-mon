#include "ssd1306.h"
#include <string.h>

static I2C_HandleTypeDef *ssd1306I2c;
static uint16_t ssd1306Address;
static uint8_t ssd1306Buffer[128U * 64U / 8U];
static uint8_t ssd1306CursorColumn;
static uint8_t ssd1306CursorPage;

static HAL_StatusTypeDef SSD1306_WriteCommand(const uint8_t *commands, uint8_t length)
{
  uint8_t packet[17U] = {0U};
  if ((ssd1306I2c == NULL) || (commands == NULL) || (length > 16U))
  {
    return HAL_ERROR;
  }
  packet[0] = 0x00U;
  (void)memcpy(&packet[1], commands, length);
  return HAL_I2C_Master_Transmit(ssd1306I2c, ssd1306Address,
                                 packet, (uint16_t)length + 1U, 100U);
}

static void SSD1306_GetGlyph(char character, uint8_t glyph[5U])
{
  static const uint8_t question[5U] = {0x02U, 0x01U, 0x51U, 0x09U, 0x06U};
  (void)memset(glyph, 0, 5U);
  if (character == ' ')
  {
    return;
  }
  if ((character >= 'a') && (character <= 'z'))
  {
    character = (char)(character - 'a' + 'A');
  }
  if ((character >= '0') && (character <= '9'))
  {
    static const uint8_t digits[10U][5U] = {
      {0x3EU,0x51U,0x49U,0x45U,0x3EU}, {0x00U,0x42U,0x7FU,0x40U,0x00U},
      {0x62U,0x51U,0x49U,0x49U,0x46U}, {0x22U,0x49U,0x49U,0x49U,0x36U},
      {0x18U,0x14U,0x12U,0x7FU,0x10U}, {0x2FU,0x49U,0x49U,0x49U,0x31U},
      {0x3EU,0x49U,0x49U,0x49U,0x32U}, {0x01U,0x71U,0x09U,0x05U,0x03U},
      {0x36U,0x49U,0x49U,0x49U,0x36U}, {0x26U,0x49U,0x49U,0x49U,0x3EU}
    };
    (void)memcpy(glyph, digits[(uint8_t)character - (uint8_t)'0'], 5U);
  }
  else if ((character >= 'A') && (character <= 'Z'))
  {
    static const uint8_t letters[26U][5U] = {
      {0x7EU,0x11U,0x11U,0x11U,0x7EU}, {0x7FU,0x49U,0x49U,0x49U,0x36U},
      {0x3EU,0x41U,0x41U,0x41U,0x22U}, {0x7FU,0x41U,0x41U,0x22U,0x1CU},
      {0x7FU,0x49U,0x49U,0x49U,0x41U}, {0x7FU,0x09U,0x09U,0x09U,0x01U},
      {0x3EU,0x41U,0x49U,0x49U,0x7AU}, {0x7FU,0x08U,0x08U,0x08U,0x7FU},
      {0x00U,0x41U,0x7FU,0x41U,0x00U}, {0x20U,0x40U,0x41U,0x3FU,0x01U},
      {0x7FU,0x08U,0x14U,0x22U,0x41U}, {0x7FU,0x40U,0x40U,0x40U,0x40U},
      {0x7FU,0x02U,0x04U,0x02U,0x7FU}, {0x7FU,0x04U,0x08U,0x10U,0x7FU},
      {0x3EU,0x41U,0x41U,0x41U,0x3EU}, {0x7FU,0x09U,0x09U,0x09U,0x06U},
      {0x3EU,0x41U,0x51U,0x21U,0x5EU}, {0x7FU,0x09U,0x19U,0x29U,0x46U},
      {0x46U,0x49U,0x49U,0x49U,0x31U}, {0x01U,0x01U,0x7FU,0x01U,0x01U},
      {0x3FU,0x40U,0x40U,0x40U,0x3FU}, {0x1FU,0x20U,0x40U,0x20U,0x1FU},
      {0x3FU,0x40U,0x38U,0x40U,0x3FU}, {0x63U,0x14U,0x08U,0x14U,0x63U},
      {0x07U,0x08U,0x70U,0x08U,0x07U}, {0x61U,0x51U,0x49U,0x45U,0x43U}
    };
    (void)memcpy(glyph, letters[(uint8_t)character - (uint8_t)'A'], 5U);
  }
  else if (character == ':')
  {
    glyph[1] = 0x36U;
  }
  else if (character == '-')
  {
    glyph[2] = 0x08U;
  }
  else if (character == '.')
  {
    glyph[2] = 0x40U;
  }
  else if (character == '%')
  {
    glyph[0] = 0x63U;
    glyph[1] = 0x13U;
    glyph[2] = 0x08U;
    glyph[3] = 0x64U;
  }
  else if (character == '/')
  {
    glyph[0] = 0x20U;
    glyph[1] = 0x10U;
    glyph[2] = 0x08U;
    glyph[3] = 0x04U;
    glyph[4] = 0x02U;
  }
  else
  {
    (void)memcpy(glyph, question, 5U);
  }
}

HAL_StatusTypeDef SSD1306_Init(I2C_HandleTypeDef *hi2c, uint16_t address)
{
  static const uint8_t initCommands[] = {
    0xAEU, 0xD5U, 0x80U, 0xA8U, 0x3FU, 0xD3U, 0x00U, 0x40U,
    0x8DU, 0x14U, 0x20U, 0x00U, 0xA1U, 0xC8U, 0xDAU, 0x12U,
    0x81U, 0xCFU, 0xD9U, 0xF1U, 0xDBU, 0x40U, 0xA4U, 0xA6U, 0xAFU
  };
  uint8_t initPacket[sizeof(initCommands) + 1U] = {0U};
  ssd1306I2c = hi2c;
  ssd1306Address = address;
  initPacket[0] = 0x00U;
  (void)memcpy(&initPacket[1], initCommands, sizeof(initCommands));
  SSD1306_Clear();
  if (HAL_I2C_IsDeviceReady(ssd1306I2c, ssd1306Address, 2U, 100U) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (HAL_I2C_Master_Transmit(ssd1306I2c, ssd1306Address,
                              initPacket, (uint16_t)sizeof(initPacket), 100U) != HAL_OK)
  {
    return HAL_ERROR;
  }
  return SSD1306_UpdateScreen();
}

void SSD1306_Clear(void)
{
  (void)memset(ssd1306Buffer, 0, sizeof(ssd1306Buffer));
  ssd1306CursorColumn = 0U;
  ssd1306CursorPage = 0U;
}

void SSD1306_SetCursor(uint8_t column, uint8_t page)
{
  ssd1306CursorColumn = (column < 128U) ? column : 0U;
  ssd1306CursorPage = (page < 8U) ? page : 0U;
}

void SSD1306_WriteString(const char *text)
{
  uint8_t glyph[5U] = {0U};
  if (text == NULL)
  {
    return;
  }
  while (*text != '\0')
  {
    if (*text == '\n')
    {
      ssd1306CursorColumn = 0U;
      ssd1306CursorPage = (ssd1306CursorPage < 7U) ? (ssd1306CursorPage + 1U) : 0U;
      text++;
      continue;
    }
    if (ssd1306CursorColumn > 122U)
    {
      ssd1306CursorColumn = 0U;
      ssd1306CursorPage = (ssd1306CursorPage < 7U) ? (ssd1306CursorPage + 1U) : 0U;
    }
    SSD1306_GetGlyph(*text, glyph);
    for (uint8_t column = 0U; column < 5U; column++)
    {
      ssd1306Buffer[(ssd1306CursorPage * 128U) + ssd1306CursorColumn + column] = glyph[column];
    }
    ssd1306CursorColumn = (uint8_t)(ssd1306CursorColumn + 6U);
    text++;
  }
}

HAL_StatusTypeDef SSD1306_UpdateScreen(void)
{
  uint8_t commands[4U] = {0U, 0U, 0U, 0U};
  uint8_t packet[129U] = {0U};
  uint8_t page = 0U;
  if (ssd1306I2c == NULL)
  {
    return HAL_ERROR;
  }
  packet[0] = 0x40U;
  for (page = 0U; page < 8U; page++)
  {
    commands[0] = 0xB0U + page;
    commands[1] = 0x00U;
    commands[2] = 0x10U;
    if (SSD1306_WriteCommand(commands, 3U) != HAL_OK)
    {
      return HAL_ERROR;
    }
    (void)memcpy(&packet[1], &ssd1306Buffer[page * 128U], 128U);
    if (HAL_I2C_Master_Transmit(ssd1306I2c, ssd1306Address,
                                packet, sizeof(packet), 100U) != HAL_OK)
    {
      return HAL_ERROR;
    }
  }
  return HAL_OK;
}
