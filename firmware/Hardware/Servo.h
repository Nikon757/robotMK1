#ifndef _SERVO_H_
#define _SERVO_H_

#include "stm32f10x.h"

/*
 * Servos on TIM1 hardware PWM: 50 Hz frame, 1 us resolution, no CPU load.
 *
 *   Header pin   Timer output   Servo
 *   PB13         TIM1_CH1N      camera pan
 *   PB14         TIM1_CH2N      front-wheel steering
 *   PB15         TIM1_CH3N      camera tilt (optional)
 *
 * The top servo header (PB12) and PC4 have no timer output on the
 * STM32F103, so they are no longer driven (held low). The steering servo
 * moves from the top header to PB14.
 */
typedef enum
{
	SERVO_STEER = 0,
	SERVO_PAN,
	SERVO_TILT,
	SERVO_COUNT
} servo_id;

void Servo_Init(void);
void Servo_SetPulse(servo_id id, uint16_t pulse_us);	/* clamped to the servo's limits */
/* 0..180 deg, 90 = the servo's centre (straight ahead); one degree = 2000/180 us,
 * the full travel of a standard 180-degree servo. Clamped to the servo's limits. */
void Servo_SetAngle(servo_id id, uint8_t angle_deg);
void Servo_CenterAll(void);

#endif
