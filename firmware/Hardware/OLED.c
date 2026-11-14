#include "stm32f10x.h"                  // Device header
#include "OLED.h"
#include "OLED_Font.h"
#include "I2C_Bus.h"
#include "Delay.h"

/*
 * SSD1306 init sequence from the vendor example "3.1 OLED Display".
 * Unlike the example, data is streamed in one I2C transaction per page
 * segment instead of one transaction per byte, which is several times faster.
 */

#define OLED_ADDRESS	0x3C
#define OLED_COLUMNS	16

static uint8_t present;		// 0 if the display didn't answer in OLED_Init

static uint8_t OLED_WriteCommand(uint8_t Command)
{
	uint8_t ok = I2C_Bus_Start(OLED_ADDRESS, 0)
	          && I2C_Bus_Write(0x00)		// control byte: command
	          && I2C_Bus_Write(Command);

	I2C_Bus_Stop();
	return ok;
}

// Page 0..7, column 0..127 (page addressing mode)
static void OLED_SetCursor(uint8_t Page, uint8_t X)
{
	OLED_WriteCommand(0xB0 | Page);
	OLED_WriteCommand(0x10 | ((X & 0xF0) >> 4));
	OLED_WriteCommand(0x00 | (X & 0x0F));
}

static void OLED_DataStart(void)
{
	I2C_Bus_Start(OLED_ADDRESS, 0);
	I2C_Bus_Write(0x40);		// control byte: data stream follows
}

void OLED_Clear()
{
	uint8_t page, i;

	if (!present)
	{
		return;
	}

	for (page = 0; page < 8; page++)
	{
		OLED_SetCursor(page, 0);
		OLED_DataStart();
		for (i = 0; i < 128; i++)
		{
			I2C_Bus_Write(0x00);
		}
		I2C_Bus_Stop();
	}
}

void OLED_ShowString(uint8_t Line, uint8_t Column, const char *String)
{
	uint8_t half, n, i;

	if (!present || Line < 1 || Line > 4 || Column < 1 || Column > OLED_COLUMNS)
	{
		return;
	}

	// Each text line is two pages: top halves of all glyphs, then bottom halves
	for (half = 0; half < 2; half++)
	{
		OLED_SetCursor((Line - 1) * 2 + half, (Column - 1) * 8);
		OLED_DataStart();
		for (n = 0; String[n] != '\0' && Column + n <= OLED_COLUMNS; n++)
		{
			char c = String[n];
			if (c < ' ' || c > '~')
			{
				c = '?';
			}
			for (i = 0; i < 8; i++)
			{
				I2C_Bus_Write(OLED_F8x16[c - ' '][i + half * 8]);
			}
		}
		I2C_Bus_Stop();
	}
}

void OLED_Init()
{
	Delay_ms(100);				// let the module's supply settle after power-on

	// Display off. No ACK means no display: skip everything from here on.
	present = OLED_WriteCommand(0xAE);
	if (!present)
	{
		return;
	}

	OLED_WriteCommand(0xD5);	// clock divide ratio / oscillator frequency
	OLED_WriteCommand(0x80);

	OLED_WriteCommand(0xA8);	// multiplex ratio: 64 rows
	OLED_WriteCommand(0x3F);

	OLED_WriteCommand(0xD3);	// display offset 0
	OLED_WriteCommand(0x00);

	OLED_WriteCommand(0x40);	// display start line 0

	OLED_WriteCommand(0xA1);	// segment remap: normal left-right
	OLED_WriteCommand(0xC8);	// COM scan direction: normal top-bottom

	OLED_WriteCommand(0xDA);	// COM pins hardware configuration
	OLED_WriteCommand(0x12);

	OLED_WriteCommand(0x81);	// contrast
	OLED_WriteCommand(0xCF);

	OLED_WriteCommand(0xD9);	// pre-charge period
	OLED_WriteCommand(0xF1);

	OLED_WriteCommand(0xDB);	// VCOMH deselect level
	OLED_WriteCommand(0x30);

	OLED_WriteCommand(0xA4);	// display follows RAM content
	OLED_WriteCommand(0xA6);	// normal (not inverted)

	OLED_WriteCommand(0x8D);	// charge pump on
	OLED_WriteCommand(0x14);

	// Clear RAM before switching on, so no leftover content flashes up
	// (after a reset without power loss the module keeps its old picture).
	OLED_Clear();
	OLED_WriteCommand(0xAF);	// display on
}
