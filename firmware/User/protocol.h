#ifndef _PROTOCOL_H_
#define _PROTOCOL_H_

#include "stm32f10x.h"

/*
 * Wire protocol (text, framed `@<payload>\r\n` inbound / `#<payload>\r\n` outbound)
 *
 *   Pi -> Board:
 *     @S<n> <angle>         Servo n=1..3 to angle 0..180 (90 = center),
 *                             500..2500 us, then clamped per servo in Servo.c
 *                             n=1: front-wheel steering  (PB14)
 *                             n=2: camera pan, horizontal (PB13)
 *                             n=3: camera tilt, vertical  (PB15)
 *     @M <left> <right>     Motor power, signed -1000..+1000 per-mil
 *     @H                    Halt motors (servos hold). Same as @M 0 0.
 *     @R                    Reset all servos to center.
 *     @Q                    Force an immediate #T and #IMU status line.
 *
 *   Board -> Pi:
 *     #BOOT NiumoMapper v2 RST=<cause>
 *                             once at startup; cause = POWER, PIN, IWDG, SOFT, OTHER
 *     #OK
 *     #ERR <code>           BADCMD, PARSE, RANGE, EMPTY, LONG
 *     #T <ms> <encL> <encR> 50 Hz. Cumulative encoder counts since boot,
 *                             positive = wheel turning forward.
 *     #I <ms> <ax> <ay> <az> <gx> <gy> <gz> <temp>
 *                           100 Hz while the IMU answers. Accel in milli-g,
 *                             gyro in 0.01 deg/s, temperature in 0.01 deg C,
 *                             axes as printed on the GY-521 board.
 *     #IMU OK <id>          IMU found (at boot, on @Q, or when it comes back);
 *                             id = WHO_AM_I in hex, 0x68 on a genuine MPU6050.
 *     #IMU NONE             no IMU answered (at boot and on @Q).
 *     #IMU LOST             IMU stopped answering; retried every second.
 *
 * Safety: if no motor command is received for PROTOCOL_MOTOR_TIMEOUT_MS,
 *         motors are forced to 0. Servos hold their last commanded position.
 */

#define PROTOCOL_MOTOR_TIMEOUT_MS    500u
#define PROTOCOL_TELEMETRY_PERIOD_MS  20u   /* 50 Hz */
#define PROTOCOL_IMU_PERIOD_MS        10u   /* 100 Hz */
#define PROTOCOL_IMU_LOST_AFTER       50u   /* failed reads in a row (0.5 s) */
#define PROTOCOL_IMU_RETRY_MS       1000u

/*
 * Encoder sign, so that forward is positive on both sides. The vendor's
 * Ackermann program drives forward with motor 1 counting down and motor 2
 * counting up; flip a sign here if your wheels report the other way.
 */
#define PROTOCOL_ENC_LEFT_SIGN   (-1)
#define PROTOCOL_ENC_RIGHT_SIGN  (+1)

/* Call once at boot. imu_found: result of MPU6050_Init(). */
void Protocol_Init(uint8_t imu_found);

/* Call once per main-loop iteration. Handles queued commands, runs the
 * motor watchdog, and emits telemetry packets on the configured periods.
 * The only wait is the ~1 ms IMU read every 10 ms. Pass the current
 * free-running millisecond tick.
 */
void Protocol_Tick(uint32_t now_ms);

#endif
