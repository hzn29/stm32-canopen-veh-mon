#ifndef SSD1306_H
#define SSD1306_H

#include "stm32g4xx_hal.h"

HAL_StatusTypeDef SSD1306_Init(I2C_HandleTypeDef *hi2c, uint16_t address);
void SSD1306_Clear(void);
void SSD1306_SetCursor(uint8_t column, uint8_t page);
void SSD1306_WriteString(const char *text);
HAL_StatusTypeDef SSD1306_UpdateScreen(void);

#endif /* SSD1306_H */
