#ifndef _MOTOR_H_
#define _MOTOR_H_

/* Zero all four TIM2 motor PWM channels with plain register writes:
 * no function call and no stack, so it is safe inside fault handlers. */
#define MOTOR_FORCE_STOP()  do { TIM2->CCR1 = 0; TIM2->CCR2 = 0; TIM2->CCR3 = 0; TIM2->CCR4 = 0; } while (0)

void Motor_Init(void);
void Car_Stop(void);
void Car_Forward(uint16_t Speed);
void Car_Backward(uint16_t Speed);
void Car_TurnLeft(uint16_t Speed);
void Car_TurnRight(uint16_t Speed);
void Car_TransLeft(uint16_t Speed);
void Car_TransRight(uint16_t Speed);
void Motor1_SetSpeed(uint8_t Dir, uint16_t Speed);
void Motor2_SetSpeed(uint8_t Dir, uint16_t Speed);
#endif

