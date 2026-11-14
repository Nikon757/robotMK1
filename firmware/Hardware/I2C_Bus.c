#include "stm32f10x.h"                  // Device header
#include "I2C_Bus.h"
#include "Delay.h"

#define SCL_PIN		GPIO_Pin_10
#define SDA_PIN		GPIO_Pin_11

#define SCL_STRETCH_POLLS	200		// bounded wait while a slave stretches the clock

static void bus_delay(void)
{
	Delay_us(I2C_BUS_HALF_PERIOD_US);
}

// Open drain: "release" lets the pull-up take the line high
static void sda_release(void) { GPIO_SetBits(GPIOB, SDA_PIN); }
static void sda_low(void)     { GPIO_ResetBits(GPIOB, SDA_PIN); }
static uint8_t sda_read(void) { return GPIO_ReadInputDataBit(GPIOB, SDA_PIN); }
static void scl_low(void)     { GPIO_ResetBits(GPIOB, SCL_PIN); }

static void scl_release(void)
{
	uint16_t n = SCL_STRETCH_POLLS;

	GPIO_SetBits(GPIOB, SCL_PIN);
	while (!GPIO_ReadInputDataBit(GPIOB, SCL_PIN) && --n);
}

void I2C_Bus_Init()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	// Open-drain output; the input stage stays active, so the pins can be read back
	GPIO_SetBits(GPIOB, SCL_PIN | SDA_PIN);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = SCL_PIN | SDA_PIN;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	I2C_Bus_Recover();
}

void I2C_Bus_Recover()
{
	uint8_t i;

	// A reset in the middle of a read (e.g. by the watchdog) can leave a
	// slave holding SDA low. Up to 9 clocks finish its byte and free SDA.
	sda_release();
	scl_release();
	bus_delay();
	for (i = 0; i < 9 && !sda_read(); i++)
	{
		scl_low();
		bus_delay();
		scl_release();
		bus_delay();
	}
	I2C_Bus_Stop();
}

uint8_t I2C_Bus_Start(uint8_t addr7, uint8_t read)
{
	// SCL may still be low from the previous byte (repeated START)
	sda_release();
	bus_delay();
	scl_release();
	bus_delay();
	sda_low();				// START: SDA falls while SCL is high
	bus_delay();
	scl_low();

	return I2C_Bus_Write((uint8_t)((addr7 << 1) | (read ? 1u : 0u)));
}

uint8_t I2C_Bus_Write(uint8_t byte)
{
	uint8_t i, ack;

	for (i = 0; i < 8; i++)
	{
		if (byte & 0x80)
		{
			sda_release();
		}
		else
		{
			sda_low();
		}
		byte <<= 1;
		bus_delay();
		scl_release();
		bus_delay();
		scl_low();
	}

	// Ninth clock: the slave pulls SDA low to acknowledge
	sda_release();
	bus_delay();
	scl_release();
	bus_delay();
	ack = !sda_read();
	scl_low();

	return ack;
}

uint8_t I2C_Bus_Read(uint8_t ack)
{
	uint8_t i, byte = 0;

	sda_release();
	for (i = 0; i < 8; i++)
	{
		bus_delay();
		scl_release();
		bus_delay();
		byte = (uint8_t)((byte << 1) | sda_read());
		scl_low();
	}

	// Ninth clock: ACK (SDA low) asks for another byte, NACK ends the read
	if (ack)
	{
		sda_low();
	}
	bus_delay();
	scl_release();
	bus_delay();
	scl_low();
	sda_release();

	return byte;
}

void I2C_Bus_Stop()
{
	scl_low();
	sda_low();
	bus_delay();
	scl_release();
	bus_delay();
	sda_release();			// STOP: SDA rises while SCL is high
	bus_delay();
}

uint8_t I2C_Bus_WriteReg(uint8_t addr7, uint8_t reg, uint8_t value)
{
	uint8_t ok = I2C_Bus_Start(addr7, 0) && I2C_Bus_Write(reg) && I2C_Bus_Write(value);

	I2C_Bus_Stop();
	return ok;
}

uint8_t I2C_Bus_ReadRegs(uint8_t addr7, uint8_t reg, uint8_t *buf, uint8_t len)
{
	uint8_t i;

	// Write the register address, then repeated START in read mode
	if (!(I2C_Bus_Start(addr7, 0) && I2C_Bus_Write(reg) && I2C_Bus_Start(addr7, 1)))
	{
		I2C_Bus_Stop();
		return 0;
	}

	for (i = 0; i < len; i++)
	{
		buf[i] = I2C_Bus_Read(i + 1 < len);
	}
	I2C_Bus_Stop();
	return 1;
}
