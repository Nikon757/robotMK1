#include "stm32f10x.h"                  // Device header
#include "Usart.h"

#include <string.h>

#pragma import(__use_no_semihosting)	// no ARM semihosting: printf goes to USART1

struct __FILE
{
	int handle; /* Add whatever you need here */
};

FILE __stdout;
FILE __stdin;

int fputc(int ch, FILE *f)
{
	Usart1_SendByte(ch);
	return ch;
}

void _sys_exit(int x)
{
	x = x; /* endless loop */
}

#define RX_BUF_SIZE		256u		// power of two
#define TX_BUF_SIZE		1024u		// power of two

static volatile uint8_t  Rx_Buf[RX_BUF_SIZE];
static volatile uint16_t Rx_Head;		// written by the IRQ only
static volatile uint16_t Rx_Tail;		// written by the main loop only

static volatile uint8_t  Tx_Buf[TX_BUF_SIZE];
static volatile uint16_t Tx_Head;		// written by the main loop only
static volatile uint16_t Tx_Tail;		// written by the IRQ only

// Packet assembler state (main loop only)
#define LINE_IDLE		0		// waiting for '@'
#define LINE_COLLECT	1		// inside a packet
#define LINE_DISCARD	2		// packet too long, skipping to its line end

static char    Line_Buf[USART1_PACKET_MAX];
static uint8_t Line_Len;
static uint8_t Line_State;

void Usart1_Init()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);

	//tx
	GPIO_InitTypeDef	GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);

	//rx
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);

	USART_InitTypeDef	USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART1,&USART_InitStructure);

	// RX interrupt always on; the TX interrupt is switched on by Usart1_Write
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);

	NVIC_InitTypeDef	NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;
	NVIC_Init(&NVIC_InitStructure);

	USART_Cmd(USART1,ENABLE);
}

uint8_t Usart1_Write(const uint8_t *Data, uint16_t Length)
{
	uint16_t head = Tx_Head;
	uint16_t free_space = (uint16_t)((Tx_Tail - head - 1u) & (TX_BUF_SIZE - 1u));
	uint16_t i;

	if (Length > free_space)
	{
		return 0;
	}

	for (i = 0; i < Length; i++)
	{
		Tx_Buf[head] = Data[i];
		head = (uint16_t)((head + 1u) & (TX_BUF_SIZE - 1u));
	}
	Tx_Head = head;

	// Publish the data first, then wake the IRQ. If it just switched TXE
	// off because the ring looked empty, this switches it back on.
	USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
	return 1;
}

uint8_t Usart1_SendString(const char *String)
{
	return Usart1_Write((const uint8_t *)String, (uint16_t)strlen(String));
}

void Usart1_SendByte(uint8_t Byte)
{
	Usart1_Write(&Byte, 1);
}

int8_t Usart1_PollPacket(char *Payload, uint8_t Size)
{
	while (Rx_Tail != Rx_Head)
	{
		char c = (char)Rx_Buf[Rx_Tail];
		Rx_Tail = (uint16_t)((Rx_Tail + 1u) & (RX_BUF_SIZE - 1u));

		if (c == '@')
		{
			// Start of a packet. Also resyncs after garbage or a lost line end.
			Line_State = LINE_COLLECT;
			Line_Len = 0;
		}
		else if (c == '\r' || c == '\n')
		{
			// Either ends a packet; the other one of a "\r\n" pair then
			// arrives in LINE_IDLE and is ignored.
			if (Line_State == LINE_COLLECT)
			{
				uint8_t n = (Line_Len < Size) ? Line_Len : (uint8_t)(Size - 1u);
				memcpy(Payload, Line_Buf, n);
				Payload[n] = '\0';
				Line_State = LINE_IDLE;
				return USART1_PACKET_READY;
			}
			if (Line_State == LINE_DISCARD)
			{
				Line_State = LINE_IDLE;
				return USART1_PACKET_TOO_LONG;
			}
		}
		else if (Line_State == LINE_COLLECT)
		{
			if (Line_Len < USART1_PACKET_MAX)
			{
				Line_Buf[Line_Len++] = c;
			}
			else
			{
				Line_State = LINE_DISCARD;
			}
		}
	}
	return USART1_PACKET_NONE;
}

void USART1_IRQHandler()
{
	uint16_t sr = USART1->SR;

	// Reading SR then DR clears RXNE and, if a byte was lost, the overrun
	// flag (ORE) too. ORE also triggers this IRQ, so it must never be left set.
	if (sr & (USART_SR_RXNE | USART_SR_ORE))
	{
		uint8_t byte = (uint8_t)USART1->DR;
		uint16_t next = (uint16_t)((Rx_Head + 1u) & (RX_BUF_SIZE - 1u));

		if (next != Rx_Tail)
		{
			Rx_Buf[Rx_Head] = byte;
			Rx_Head = next;
		}
		// else: ring full, byte dropped; the assembler resyncs on the next '@'
	}

	if ((USART1->CR1 & USART_CR1_TXEIE) && (sr & USART_SR_TXE))
	{
		if (Tx_Tail != Tx_Head)
		{
			USART1->DR = Tx_Buf[Tx_Tail];
			Tx_Tail = (uint16_t)((Tx_Tail + 1u) & (TX_BUF_SIZE - 1u));
		}
		else
		{
			USART1->CR1 &= (uint16_t)~USART_CR1_TXEIE;
		}
	}
}
