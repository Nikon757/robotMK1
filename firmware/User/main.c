/*
 * Niumo-board mapping robot firmware, v2.
 *
 *   USART1 (Type-C) command bridge from a Raspberry Pi:
 *     - servos on TIM1 hardware PWM: steering PB14, camera pan PB13, tilt PB15
 *     - 2 rear drive motors (signed power, -1000..+1000 per-mil)
 *     - 50 Hz telemetry: ms tick, cumulative L/R encoder counts
 *     - 100 Hz MPU6050 IMU (GY-521 on J10): accel, gyro, temperature
 *     - 500 ms motor watchdog, ~200 ms hardware watchdog (IWDG)
 *     - OLED (J9) for errors only: "WATCHDOG RESET" after an IWDG reset
 *
 *   The main loop never waits for long: UART RX/TX run from ring buffers,
 *   servo PWM and motor PWM are pure hardware. The one
 *   wait is the ~1 ms bit-banged IMU read every 10 ms.
 *
 *   Ackermann-project drivers still used:
 *     Motor.c     - TIM2 PWM on PA0..PA3
 *     encoder.c   - rear-wheel encoders, EXTI on PC0..PC3
 *     compute_pid.c, Vehicle_Chassis.c - available if you want closed-loop later
 *
 *   See README.md for wiring and the wire protocol.
 */

#include "stm32f10x.h"

#include <stdio.h>

#include "Delay.h"
#include "Servo.h"
#include "Motor.h"
#include "encoder.h"
#include "Usart.h"
#include "I2C_Bus.h"
#include "OLED.h"
#include "MPU6050.h"

#include "protocol.h"

/* ---- 1 kHz free-running tick (TIM6, basic timer) ----
 * Other firmwares on this board use SysTick for Delay_ms, so we keep
 * SysTick alone and dedicate TIM6 to the ms tick.
 */
volatile uint32_t g_system_ms = 0;

static void SystemTick_Init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);

    TIM_TimeBaseInitTypeDef tb;
    TIM_TimeBaseStructInit(&tb);
    tb.TIM_Prescaler     = 72 - 1;      /* 72 MHz / 72 = 1 MHz */
    tb.TIM_Period        = 1000 - 1;    /* 1 MHz / 1000 = 1 kHz -> 1 ms */
    tb.TIM_CounterMode   = TIM_CounterMode_Up;
    tb.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM6, &tb);

    TIM_ClearFlag(TIM6, TIM_FLAG_Update);
    TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);

    NVIC_InitTypeDef nv;
    nv.NVIC_IRQChannel                   = TIM6_IRQn;
    nv.NVIC_IRQChannelCmd                = ENABLE;
    nv.NVIC_IRQChannelPreemptionPriority = 3;
    nv.NVIC_IRQChannelSubPriority        = 0;
    NVIC_Init(&nv);

    TIM_Cmd(TIM6, ENABLE);
}

void TIM6_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
        g_system_ms++;
    }
}

/* ---- hardware watchdog ----
 * LSI (~40 kHz, 30..60 kHz worst case) / 32 = ~1.25 kHz; 250 counts = ~200 ms
 * (133..267 ms). If the main loop or an interrupt ever hangs, the MCU resets,
 * which stops the motors. Frozen while the core is halted in the debugger.
 */
static void Watchdog_Init(void)
{
    DBGMCU_Config(DBGMCU_IWDG_STOP, ENABLE);

    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    IWDG_SetPrescaler(IWDG_Prescaler_32);
    IWDG_SetReload(250);
    IWDG_ReloadCounter();
    IWDG_Enable();
}

/* Why did we start? Read once, then clear the flags for the next reset. */
static const char *Reset_Cause(void)
{
    const char *cause = "OTHER";

    /* Order matters: an IWDG or power-on reset also sets PINRST. */
    if      (RCC_GetFlagStatus(RCC_FLAG_IWDGRST) == SET) cause = "IWDG";
    else if (RCC_GetFlagStatus(RCC_FLAG_PORRST)  == SET) cause = "POWER";
    else if (RCC_GetFlagStatus(RCC_FLAG_SFTRST)  == SET) cause = "SOFT";
    else if (RCC_GetFlagStatus(RCC_FLAG_PINRST)  == SET) cause = "PIN";

    RCC_ClearFlag();
    return cause;
}

int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    /* Read before Reset_Cause() clears the flags. */
    uint8_t watchdog_reset = (RCC_GetFlagStatus(RCC_FLAG_IWDGRST) == SET);
    const char *reset_cause = Reset_Cause();

    /* Motors first, so they are stopped before anything else can happen. */
    Motor_Init();        /* TIM2 hardware PWM, motors stopped */
    Servo_Init();        /* TIM1 hardware PWM, all servos centered */
    Encoder_Init();      /* EXTI on PC0..PC3 */
    Usart1_Init();       /* USART1 115200, interrupt-driven RX/TX rings */

    /* PB10/PB11 I2C, shared by the OLED (J9) and the IMU (J10). */
    I2C_Bus_Init();

    /* The OLED shows errors only; blank means no error. Its writes block,
     * so it is only touched here at boot, before the watchdog starts. */
    OLED_Init();         /* ~100 ms power-up wait, then a blank screen */
    if (watchdog_reset)
    {
        /* Stays up until the next reset that isn't the watchdog's. */
        OLED_ShowString(2, 2, "WATCHDOG RESET");
    }

    uint8_t imu_found = MPU6050_Init();     /* GY-521 on J10; 100 ms reset wait */

    /* This firmware's own modules. */
    SystemTick_Init();
    Protocol_Init(imu_found);

    char boot[48];
    snprintf(boot, sizeof boot, "#BOOT NiumoMapper v2 RST=%s\r\n", reset_cause);
    Usart1_SendString(boot);

    /* Last: from here on the loop must come round at least every ~130 ms. */
    Watchdog_Init();

    while (1)
    {
        uint32_t now = g_system_ms;

        IWDG_ReloadCounter();
        Protocol_Tick(now);
    }
}
