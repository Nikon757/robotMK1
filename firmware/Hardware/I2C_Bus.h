#ifndef _I2C_BUS_H_
#define _I2C_BUS_H_

#include "stm32f10x.h"

/*
 * Bit-banged I2C master on PB10 (SCL) / PB11 (SDA). One bus, shared by the
 * OLED header (J9, SSD1306 at 0x3C) and the gyroscope header (J10, MPU6050
 * at 0x68). The board pulls both lines up to 5 V (4.7 kOhm); PB10/PB11 are
 * 5 V tolerant and driven open-drain.
 *
 * Speed is ~150 kHz (both chips allow 400 kHz). Every wait is bounded, so a
 * missing or stuck device can't hang the firmware. Addresses are 7-bit.
 */
#define I2C_BUS_HALF_PERIOD_US	2

void I2C_Bus_Init(void);		// pins + bus recovery
void I2C_Bus_Recover(void);		// clock out a slave stuck mid-byte, then STOP

// Low level: Start sends START (or repeated START) + address byte.
// Start/Write return 1 when the device ACKed.
uint8_t I2C_Bus_Start(uint8_t addr7, uint8_t read);
uint8_t I2C_Bus_Write(uint8_t byte);
uint8_t I2C_Bus_Read(uint8_t ack);	// ack = 1 if more bytes follow
void I2C_Bus_Stop(void);

// Register access, 1 = OK
uint8_t I2C_Bus_WriteReg(uint8_t addr7, uint8_t reg, uint8_t value);
uint8_t I2C_Bus_ReadRegs(uint8_t addr7, uint8_t reg, uint8_t *buf, uint8_t len);

#endif
