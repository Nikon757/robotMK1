#ifndef _OLED_H_
#define _OLED_H_

#include "stm32f10x.h"

/*
 * 0.96" SSD1306 128x64 OLED on the board's OLED header (J9), on the shared
 * I2C bus (I2C_Bus.c, call I2C_Bus_Init first). Text grid: 4 lines x 16
 * columns of 8x16 characters.
 *
 * Writes block (a full clear takes tens of milliseconds), so use these at
 * boot only, not from the main loop. If no display answers, every call
 * returns immediately.
 */
void OLED_Init(void);			// power-up wait (~100 ms), init, blank screen
void OLED_Clear(void);
void OLED_ShowString(uint8_t Line, uint8_t Column, const char *String);	// Line 1..4, Column 1..16

#endif
