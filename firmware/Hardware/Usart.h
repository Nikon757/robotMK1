#ifndef _USART_H_
#define _USART_H_

#include <stdio.h>
#include "stm32f10x.h"

/*
 * USART1 (Type-C, CH340E), 115200 8N1, fully interrupt driven:
 *
 *   RX  The IRQ only drops bytes into a ring buffer. Usart1_PollPacket()
 *       assembles `@<payload>\r\n` packets from it in the main loop.
 *   TX  Writes are copied into a ring buffer that the TXE interrupt drains,
 *       so sending never blocks. A message that doesn't fit is dropped whole
 *       (no half lines on the wire).
 *
 * Everything except the IRQ handler is for the main loop only; don't call
 * the write functions from an interrupt.
 */
#define USART1_PACKET_MAX		64u		// longest payload, without '@' and line end

#define USART1_PACKET_NONE		0		// no complete packet yet
#define USART1_PACKET_READY		1		// payload holds one packet
#define USART1_PACKET_TOO_LONG	(-1)	// a packet over USART1_PACKET_MAX was dropped

void Usart1_Init(void);
uint8_t Usart1_Write(const uint8_t *Data, uint16_t Length);	// 1 = queued, 0 = dropped
uint8_t Usart1_SendString(const char *String);
void Usart1_SendByte(uint8_t Byte);
int8_t Usart1_PollPacket(char *Payload, uint8_t Size);		// Size >= USART1_PACKET_MAX + 1
#endif
