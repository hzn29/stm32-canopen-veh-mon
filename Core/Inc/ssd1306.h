/* SSD1306 128x64 I2C OLED 显示驱动接口。 */
#ifndef SSD1306_H
#define SSD1306_H

/* 引入 STM32 HAL I2C 类型。 */
#include "stm32g4xx_hal.h"

/* 初始化 SSD1306 控制器和显示缓存。 */
HAL_StatusTypeDef SSD1306_Init(I2C_HandleTypeDef *hi2c, uint16_t address);
/* 清空显示缓存。 */
void SSD1306_Clear(void);
/* 设置文本光标，column 为像素列，page 为 8 像素页。 */
void SSD1306_SetCursor(uint8_t column, uint8_t page);
/* 向当前光标位置写入 ASCII 字符串。 */
void SSD1306_WriteString(const char *text);
/* 将显示缓存刷新到 OLED 面板。 */
HAL_StatusTypeDef SSD1306_UpdateScreen(void);

#endif /* SSD1306_H */
