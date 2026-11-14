#include "stm32f10x.h"
#include "protocol.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "Usart.h"
#include "Servo.h"
#include "Motor.h"
#include "encoder.h"
#include "MPU6050.h"
#include "I2C_Bus.h"

/* Packets handled per Protocol_Tick() call, so a burst can't stall the loop. */
#define PROTOCOL_MAX_PACKETS_PER_TICK  4u

static void apply_motor(int16_t left, int16_t right)
{
    if (left  >  1000) left  =  1000;
    if (left  < -1000) left  = -1000;
    if (right >  1000) right =  1000;
    if (right < -1000) right = -1000;

    /* Motor1 = left rear. Motor2 = right rear. Dir = 1 means forward. */
    Motor1_SetSpeed(left  >= 0 ? 1u : 0u, (uint16_t)abs(left));
    Motor2_SetSpeed(right >= 0 ? 1u : 0u, (uint16_t)abs(right));
}

static void halt_motors(void)
{
    apply_motor(0, 0);
}

/* ---- state ---- */
static uint8_t  s_motor_armed;          /* 1 while the motors run on an @M */
static uint32_t s_last_motor_cmd_ms;
static uint32_t s_last_telemetry_ms;
static uint8_t  s_force_report;         /* @Q: send #T and #IMU on this tick */

/* IMU: NONE until it first answers, OK while it does, LOST after
 * PROTOCOL_IMU_LOST_AFTER failed reads. NONE and LOST retry every second. */
enum { IMU_NONE = 0, IMU_OK, IMU_LOST };
static uint8_t  s_imu_state;
static uint8_t  s_imu_failures;
static uint8_t  s_imu_report;           /* send the #IMU status line on this tick */
static uint32_t s_last_imu_ms;
static uint32_t s_last_imu_retry_ms;

void Protocol_Init(uint8_t imu_found)
{
    s_motor_armed        = 0;
    s_last_motor_cmd_ms  = 0;
    s_last_telemetry_ms  = 0;
    s_force_report       = 0;

    s_imu_state          = imu_found ? IMU_OK : IMU_NONE;
    s_imu_failures       = 0;
    s_imu_report         = 1;           /* tell the Pi right after #BOOT */
    s_last_imu_ms        = 0;
    s_last_imu_retry_ms  = 0;

    halt_motors();
}

/* Replies go through the TX ring: if it is full they are dropped, never waited on. */
static void reply_ok(void)        { Usart1_SendString("#OK\r\n"); }
static void reply_err(const char *code)
{
    char line[24];
    snprintf(line, sizeof line, "#ERR %s\r\n", code);
    Usart1_SendString(line);
}

/* Dispatch a single decoded packet. `payload` is null-terminated, no framing. */
static void handle(const char *payload, uint32_t now_ms)
{
    if (payload[0] == '\0') { reply_err("EMPTY"); return; }

    switch (payload[0])
    {
        case 'H':
            halt_motors();
            s_motor_armed = 0;
            reply_ok();
            return;

        case 'R':
            Servo_CenterAll();
            reply_ok();
            return;

        case 'Q':
            s_force_report = 1;
            s_imu_report = 1;
            reply_ok();
            return;

        case 'S': {
            int n = 0, angle = 0;
            if (sscanf(payload + 1, "%d %d", &n, &angle) != 2) { reply_err("PARSE"); return; }
            if (n < 1 || n > 3 || angle < 0 || angle > 180)    { reply_err("RANGE"); return; }
            switch (n) {
                case 1: Servo_SetAngle(SERVO_STEER, (uint8_t)angle); break;   /* front-wheel steering */
                case 2: Servo_SetAngle(SERVO_PAN,   (uint8_t)angle); break;   /* camera pan  */
                case 3: Servo_SetAngle(SERVO_TILT,  (uint8_t)angle); break;   /* camera tilt */
            }
            reply_ok();
            return;
        }

        case 'M': {
            int l = 0, r = 0;
            if (sscanf(payload + 1, "%d %d", &l, &r) != 2) { reply_err("PARSE"); return; }
            if (l < -1000 || l > 1000 || r < -1000 || r > 1000) { reply_err("RANGE"); return; }
            apply_motor((int16_t)l, (int16_t)r);
            s_motor_armed = 1;
            s_last_motor_cmd_ms = now_ms;
            reply_ok();
            return;
        }

        default:
            reply_err("BADCMD");
            return;
    }
}

static void imu_send_status(void)
{
    char line[24];

    if (s_imu_state == IMU_OK)
    {
        snprintf(line, sizeof line, "#IMU OK 0x%02X\r\n", (unsigned)MPU6050_GetId());
        Usart1_SendString(line);
    }
    else
    {
        Usart1_SendString(s_imu_state == IMU_LOST ? "#IMU LOST\r\n" : "#IMU NONE\r\n");
    }
}

static void imu_tick(uint32_t now_ms)
{
    if (s_imu_state == IMU_OK)
    {
        if ((uint32_t)(now_ms - s_last_imu_ms) >= PROTOCOL_IMU_PERIOD_MS)
        {
            struct MPU6050_SAMPLE s;

            s_last_imu_ms = now_ms;
            if (MPU6050_Read(&s))
            {
                char line[80];
                int n = snprintf(line, sizeof line,
                                 "#I %lu %d %d %d %ld %ld %ld %d\r\n",
                                 (unsigned long)now_ms,
                                 s.ax_mg, s.ay_mg, s.az_mg,
                                 (long)s.gx_cdps, (long)s.gy_cdps, (long)s.gz_cdps,
                                 s.temp_cdeg);
                if (n > 0) {
                    Usart1_SendString(line);
                }
                s_imu_failures = 0;
            }
            else
            {
                I2C_Bus_Recover();      /* free the bus if a glitch left SDA held low */
                if (++s_imu_failures >= PROTOCOL_IMU_LOST_AFTER)
                {
                    s_imu_state = IMU_LOST;
                    s_last_imu_retry_ms = now_ms;
                    s_imu_report = 1;
                }
            }
        }
    }
    else if ((uint32_t)(now_ms - s_last_imu_retry_ms) >= PROTOCOL_IMU_RETRY_MS)
    {
        /* No reset and no waits here: a few register writes, ~3 ms. */
        s_last_imu_retry_ms = now_ms;
        if (MPU6050_Configure())
        {
            s_imu_state = IMU_OK;
            s_imu_failures = 0;
            s_imu_report = 1;
        }
    }

    if (s_imu_report)
    {
        s_imu_report = 0;
        imu_send_status();
    }
}

void Protocol_Tick(uint32_t now_ms)
{
    char payload[USART1_PACKET_MAX + 1];
    uint8_t budget;

    /* 1) drain inbound */
    for (budget = PROTOCOL_MAX_PACKETS_PER_TICK; budget > 0; budget--)
    {
        int8_t r = Usart1_PollPacket(payload, sizeof payload);
        if (r == USART1_PACKET_NONE) break;
        if (r == USART1_PACKET_READY) handle(payload, now_ms);
        else                          reply_err("LONG");
    }

    /* 2) motor watchdog */
    if (s_motor_armed &&
        (uint32_t)(now_ms - s_last_motor_cmd_ms) > PROTOCOL_MOTOR_TIMEOUT_MS)
    {
        halt_motors();
        s_motor_armed = 0;
    }

    /* 3) periodic telemetry */
    if (s_force_report ||
        (uint32_t)(now_ms - s_last_telemetry_ms) >= PROTOCOL_TELEMETRY_PERIOD_MS)
    {
        s_last_telemetry_ms = now_ms;

        int32_t encL = PROTOCOL_ENC_LEFT_SIGN  * Encoder1_Total_Get();
        int32_t encR = PROTOCOL_ENC_RIGHT_SIGN * Encoder2_Total_Get();

        char line[48];
        int n = snprintf(line, sizeof line,
                         "#T %lu %ld %ld\r\n",
                         (unsigned long)now_ms,
                         (long)encL, (long)encR);
        if (n > 0) {
            Usart1_SendString(line);
        }
    }

    s_force_report = 0;

    /* 4) IMU */
    imu_tick(now_ms);
}
