#include "stm32f10x.h"                  // Device header
#include "Servo.h"

#define SERVO_PERIOD_US		20000		// 50 Hz frame

struct SERVO_LIMIT
{
	uint16_t min_us;
	uint16_t center_us;
	uint16_t max_us;
};

/*
 * Steering and tilt keep the old software-PWM driver's end stops.
 * center_us is where the servo points straight ahead, and it is what command
 * angle 90 maps to (Servo_SetAngle), so a servo horn mounted a tooth off is
 * trimmed here. If a linkage hits its stop first, tighten the limit here
 * rather than on the Pi, so a bad command can never jam a servo.
 *
 * Camera pan, measured with pi_examples/pan_sweep.py on 30 Oct 2026: straight
 * ahead at 1167 us (the horn sits about 30 degrees off); 500 us is the servo's
 * own end, about 60 degrees right; 1856 us is about 62 degrees left (the
 * user's limit minus a 3-degree margin). Usable command angles: 30..152.
 */
static const struct SERVO_LIMIT servo_limit[SERVO_COUNT] =
{
	{1200, 1500, 1900},		// SERVO_STEER, PB14
	{ 500, 1167, 1856},		// SERVO_PAN,   PB13
	{ 800, 1500, 1850},		// SERVO_TILT,  PB15
};

void Servo_Init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_TIM1, ENABLE);

	// PB13/PB14/PB15 -> TIM1_CH1N/CH2N/CH3N (default mapping, no remap)
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// Old software-PWM headers PB12 and PC4: held low, so a servo left
	// there stays limp instead of twitching on a floating pin.
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOB, GPIO_Pin_12);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOC, GPIO_Pin_4);

	// 72 MHz / 72 = 1 MHz -> 1 us per count, 20000 counts = 20 ms
	TIM_TimeBaseStructInit(&TIM_TimeBaseInitStructure);
	TIM_TimeBaseInitStructure.TIM_Prescaler = 72 - 1;
	TIM_TimeBaseInitStructure.TIM_Period = SERVO_PERIOD_US - 1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);

	// PWM1 on the complementary outputs only. With CCxE = 0 the CHxN pin
	// follows OCxREF directly: high while CNT < CCRx, so CCRx = pulse in us.
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;

	TIM_OCInitStructure.TIM_Pulse = servo_limit[SERVO_PAN].center_us;
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_Pulse = servo_limit[SERVO_STEER].center_us;
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_Pulse = servo_limit[SERVO_TILT].center_us;
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);

	// Preload: a new width takes effect at the next frame, never mid-pulse
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(TIM1, ENABLE);

	TIM_Cmd(TIM1, ENABLE);
	TIM_CtrlPWMOutputs(TIM1, ENABLE);	// advanced timer: main output enable (MOE)
}

void Servo_SetPulse(servo_id id, uint16_t pulse_us)
{
	if (id >= SERVO_COUNT)
	{
		return;
	}

	if (pulse_us < servo_limit[id].min_us)
	{
		pulse_us = servo_limit[id].min_us;
	}
	else if (pulse_us > servo_limit[id].max_us)
	{
		pulse_us = servo_limit[id].max_us;
	}

	switch (id)
	{
		case SERVO_PAN:   TIM_SetCompare1(TIM1, pulse_us); break;
		case SERVO_STEER: TIM_SetCompare2(TIM1, pulse_us); break;
		case SERVO_TILT:  TIM_SetCompare3(TIM1, pulse_us); break;
		default: break;
	}
}

void Servo_SetAngle(servo_id id, uint8_t angle_deg)
{
	int32_t offset;
	int32_t pulse_us;

	if (id >= SERVO_COUNT)
	{
		return;
	}
	if (angle_deg > 180)
	{
		angle_deg = 180;
	}

	// (angle - 90) * 2000 / 180 us around the centre, rounded half away from zero
	offset = ((int32_t)angle_deg - 90) * 2000;
	offset = (offset >= 0 ? offset + 90 : offset - 90) / 180;
	pulse_us = (int32_t)servo_limit[id].center_us + offset;
	if (pulse_us < 0)
	{
		pulse_us = 0;	// Servo_SetPulse raises it to min_us
	}
	Servo_SetPulse(id, (uint16_t)pulse_us);
}

void Servo_CenterAll()
{
	uint8_t i;

	for (i = 0; i < SERVO_COUNT; i++)
	{
		Servo_SetPulse((servo_id)i, servo_limit[i].center_us);
	}
}
