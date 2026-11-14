#ifndef _MPU6050_H_
#define _MPU6050_H_

#include "stm32f10x.h"

/*
 * MPU6050 (GY-521 module) on the gyroscope header J10, I2C address 0x68 on
 * the shared bus (I2C_Bus.c, call I2C_Bus_Init first).
 *
 * Set up for a ground robot: gyro +-500 deg/s, accel +-4 g, internal
 * sampling 1 kHz through the ~42 Hz digital low-pass filter. Reading it at
 * 100 Hz gets the newest filtered sample each time.
 *
 * Axes are the ones printed on the GY-521 board. Raw data only: no DMP, no
 * bias correction; the Pi does fusion and calibration.
 */
struct MPU6050_SAMPLE
{
	int16_t ax_mg, ay_mg, az_mg;			// acceleration, milli-g (1 g = 9.80665 m/s^2)
	int32_t gx_cdps, gy_cdps, gz_cdps;		// angular rate, 0.01 deg/s
	int16_t temp_cdeg;						// chip temperature, 0.01 deg C
};

uint8_t MPU6050_Init(void);			// boot only: device reset (100 ms wait) + configure; 1 = found
uint8_t MPU6050_Configure(void);	// wake + configure, no reset, no waits (~3 ms); 1 = OK
uint8_t MPU6050_GetId(void);		// WHO_AM_I from the last successful configure (0x68 genuine)
uint8_t MPU6050_Read(struct MPU6050_SAMPLE *Sample);	// one 14-byte burst, ~1 ms; 1 = OK

#endif
