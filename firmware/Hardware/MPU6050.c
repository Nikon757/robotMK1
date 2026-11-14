#include "stm32f10x.h"                  // Device header
#include "MPU6050.h"
#include "I2C_Bus.h"
#include "Delay.h"

#define MPU6050_ADDRESS		0x68	// AD0 low, as on the GY-521

// Registers (MPU-6000/6050 register map, rev 4.2)
#define REG_SMPLRT_DIV		0x19
#define REG_CONFIG			0x1A
#define REG_GYRO_CONFIG		0x1B
#define REG_ACCEL_CONFIG	0x1C
#define REG_FIFO_EN			0x23
#define REG_INT_ENABLE		0x38
#define REG_ACCEL_XOUT_H	0x3B	// 14 bytes: accel XYZ, temperature, gyro XYZ
#define REG_USER_CTRL		0x6A
#define REG_PWR_MGMT_1		0x6B
#define REG_PWR_MGMT_2		0x6C
#define REG_WHO_AM_I		0x75

// Scale for the ranges set in MPU6050_Configure
#define ACCEL_LSB_PER_G		8192	// +-4 g
#define GYRO_LSB_PER_DPS_X10	655	// +-500 deg/s: 65.5 LSB per deg/s

static uint8_t who_am_i;

uint8_t MPU6050_Configure()
{
	uint8_t id = 0;
	uint8_t ok = I2C_Bus_WriteReg(MPU6050_ADDRESS, REG_PWR_MGMT_1, 0x01)		// awake, clock = PLL on gyro X
	          && I2C_Bus_WriteReg(MPU6050_ADDRESS, REG_PWR_MGMT_2, 0x00)		// all axes on
	          && I2C_Bus_WriteReg(MPU6050_ADDRESS, REG_CONFIG, 0x03)			// DLPF: accel 44 Hz, gyro 42 Hz
	          && I2C_Bus_WriteReg(MPU6050_ADDRESS, REG_SMPLRT_DIV, 0x00)		// 1 kHz / (1 + 0) = 1 kHz
	          && I2C_Bus_WriteReg(MPU6050_ADDRESS, REG_GYRO_CONFIG, 0x08)		// FS_SEL = 1: +-500 deg/s
	          && I2C_Bus_WriteReg(MPU6050_ADDRESS, REG_ACCEL_CONFIG, 0x08)		// AFS_SEL = 1: +-4 g
	          && I2C_Bus_WriteReg(MPU6050_ADDRESS, REG_INT_ENABLE, 0x00)		// INT pin is not wired on J10
	          && I2C_Bus_WriteReg(MPU6050_ADDRESS, REG_USER_CTRL, 0x00)		// no FIFO, no aux I2C master
	          && I2C_Bus_WriteReg(MPU6050_ADDRESS, REG_FIFO_EN, 0x00)
	          && I2C_Bus_ReadRegs(MPU6050_ADDRESS, REG_WHO_AM_I, &id, 1);

	if (ok)
	{
		who_am_i = id;
	}
	return ok;
}

uint8_t MPU6050_Init()
{
	// Full device reset, so a chip that kept running through an MCU reset
	// starts from its defaults too. The datasheet asks for 100 ms after it.
	if (!I2C_Bus_WriteReg(MPU6050_ADDRESS, REG_PWR_MGMT_1, 0x80))
	{
		return 0;
	}
	Delay_ms(100);

	return MPU6050_Configure();
}

uint8_t MPU6050_GetId()
{
	return who_am_i;
}

// Signed division rounded to nearest, so small rates don't all collapse to 0
static int32_t div_round(int32_t num, int32_t den)
{
	return (num >= 0) ? (num + den / 2) / den : (num - den / 2) / den;
}

static int16_t be16(const uint8_t *p)
{
	return (int16_t)(((uint16_t)p[0] << 8) | p[1]);
}

uint8_t MPU6050_Read(struct MPU6050_SAMPLE *Sample)
{
	uint8_t b[14];

	if (!I2C_Bus_ReadRegs(MPU6050_ADDRESS, REG_ACCEL_XOUT_H, b, sizeof b))
	{
		return 0;
	}

	Sample->ax_mg = (int16_t)div_round((int32_t)be16(&b[0]) * 1000, ACCEL_LSB_PER_G);
	Sample->ay_mg = (int16_t)div_round((int32_t)be16(&b[2]) * 1000, ACCEL_LSB_PER_G);
	Sample->az_mg = (int16_t)div_round((int32_t)be16(&b[4]) * 1000, ACCEL_LSB_PER_G);

	// Datasheet: deg C = raw / 340 + 36.53
	Sample->temp_cdeg = (int16_t)(div_round((int32_t)be16(&b[6]) * 100, 340) + 3653);

	// 0.01 deg/s = raw * 100 / 65.5 = raw * 1000 / 655
	Sample->gx_cdps = div_round((int32_t)be16(&b[8])  * 1000, GYRO_LSB_PER_DPS_X10);
	Sample->gy_cdps = div_round((int32_t)be16(&b[10]) * 1000, GYRO_LSB_PER_DPS_X10);
	Sample->gz_cdps = div_round((int32_t)be16(&b[12]) * 1000, GYRO_LSB_PER_DPS_X10);

	return 1;
}
