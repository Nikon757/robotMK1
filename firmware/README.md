# Niumo-board mapping-robot firmware (v2)

STM32F103RC firmware for the YourFun "Niumo" V5 main control board, configured for
a mapping/SLAM robot driven by a Raspberry Pi over USB serial.

**What it does**

- Listens on the Type-C USB serial port (USART1, CH340E) for text commands from the Pi.
- Drives the servos with **TIM1 hardware PWM** (50 Hz, 1 µs steps): steering, camera pan,
  optional camera tilt.
- Drives **2 rear motors** with signed power (-1000..+1000 per-mil).
- Streams **50 Hz telemetry** back to the Pi: ms tick and cumulative L/R encoder counts.
- Streams the **MPU6050 IMU** (GY-521 module on the gyroscope header J10) at 100 Hz:
  acceleration, angular rate and chip temperature. *Not used on RobotMK1 since
  14 Nov 2026 (the camera's IMU is used instead); this support will be removed.*
- **500 ms motor watchdog**: if no motor command arrives in that window, the motors stop.
  (Servos hold position, so the camera stays where you pointed it.)
- **~200 ms hardware watchdog (IWDG)**: if the firmware ever hangs, the MCU resets,
  which stops the motors. Fault handlers zero the motor PWM before waiting for it.
- **OLED for errors only** (0.96" SSD1306 on the OLED header J9): blank while all is well,
  `WATCHDOG RESET` after the IWDG had to reset the board.

The main loop never waits for long: UART RX and TX run through interrupt-driven ring
buffers, servo and motor PWM are pure hardware. The one wait is the
~1 ms bit-banged IMU read every 10 ms.

**Talking to the board from a Raspberry Pi** (Python client, examples, other languages):
see [PI_GUIDE.md](../docs/PI_GUIDE.md).

**How this firmware was developed** (design decisions, problems met and how they were
solved, reference material): see [DEVELOPMENT.md](../docs/DEVELOPMENT.md). The camera-pan
calibration (method, commands, measurements, accuracy, literature) is written up in
[CAMERA_PAN_CALIBRATION.md](../docs/CAMERA_PAN_CALIBRATION.md). The plan for ROS 2 mapping,
navigation and people-aware collision avoidance is in [ROADMAP.md](../docs/ROADMAP.md). How the
drive motors are controlled and what their encoders report (`#T`) is explained in
[MOTORS_AND_ENCODERS.md](../docs/MOTORS_AND_ENCODERS.md).

**What it doesn't do (on purpose)**

- No PS2 receiver, no Bluetooth, no ESP8266, no K210. Pi-over-USB is the only
  command interface. (`ps2.c` and `Sensor.c` are in the project but excluded from the build.)
- No closed-loop motor PID yet. `compute_pid.c` and `Vehicle_Chassis.c` stay
  available for that.

---

## 1. What changed from v1

| Area | v1 | v2 |
|---|---|---|
| Encoder telemetry | Always `0 0` (read a value nothing updated) | Real cumulative counts, forward = positive on both sides |
| EC3/EC4 headers (PC6..PC9) | Floating inputs with interrupts enabled but no handler: one noise edge could hang the MCU with the motors still running | No interrupts, pull-ups on, defensive handler |
| EC1/EC2 inputs | Floating | Pull-ups, so an unplugged cable stays quiet |
| Servos | Software PWM: a TIM1 interrupt every 10 µs, 10 µs steps, jitter from other interrupts | TIM1 hardware PWM, 1 µs steps, no interrupts |
| UART RX | One shared 100-byte buffer, no bounds check; back-to-back commands could be lost | 256-byte ring, 64-char packet limit (`#ERR LONG`), accepts `\r`, `\n` or `\r\n` |
| UART TX | Blocking (~2 ms per telemetry line) | 1 KB ring drained by the TXE interrupt; a message that doesn't fit is dropped whole |
| Crash safety | None | IWDG ~200 ms, motors zeroed in fault handlers, reset cause in `#BOOT` |
| OLED | Not used | Errors only: `WATCHDOG RESET` after a watchdog reset |
| IMU | Not used | MPU6050 on J10: `#I` at 100 Hz, `#IMU` status, reconnects by itself |
| Stack | 1 KB | 2 KB (`Start/startup_stm32f10x_hd.s`) |

The wire protocol is backward compatible: `#T` keeps its exact 4-field format, and `#I`
and `#IMU` are new lines that older Pi code simply ignores.

The board does not measure the battery voltage (no divider is fitted; the firmware leaves
the AD8/AD9 pins alone).

---

## 2. Wiring changes you need to make

### 2.1 Steering servo moves to PB14

The STM32F103 has no timer output on the top servo header (PB12) or on PC4, so
hardware PWM only exists on PB13/PB14/PB15 (TIM1_CH1N/CH2N/CH3N). These are three of
the four ports in the row the manual labels PB13, PB14, PB15, PC4 (left to right).

| Servo | v1 header | **v2 header** | Timer output |
|---|---|---|---|
| Front-wheel steering | top header (PB12) | **PB14** | TIM1_CH2N |
| Camera pan | PB13 | PB13 (unchanged) | TIM1_CH1N |
| Camera tilt (optional) | PB14 | **PB15** | TIM1_CH3N |

Gray (signal) wire faces inward, as before. PB12 and PC4 are now held low: a servo
left there gets no pulses and stays limp.

### 2.2 OLED on the OLED header (J9)

Plug the kit's 0.96" I2C OLED (SSD1306, address 0x3C) into the 4-pin OLED header:
GND, 5V, SCL (PB10), SDA (PB11). It shares the I2C bus with the gyro header (J10).

The screen is for errors only, and for now there is one:

| Screen | Meaning | Clears |
|---|---|---|
| blank | No error | |
| `WATCHDOG RESET` | The firmware hung and the IWDG reset the board (`#BOOT ... RST=IWDG`) | On the next reset that isn't the watchdog's: power cycle or the RST button |

The display is written only at boot, before the watchdog starts: the driver is
bit-banged and blocking, so it never runs in the main loop. Boot takes ~100 ms longer
for the display's power-up wait. Without a display fitted the driver notices the missing
ACK and skips all writes.

### 2.3 IMU on the gyroscope header (J10)

*Not used on RobotMK1 (decision of 14 Nov 2026): leave J10 empty. The robot uses the
OAK-D Lite's built-in IMU. Without a module the board simply reports `#IMU NONE`.*

Plug the vendor's GY-521 (MPU6050) module into J10. The header's four pins are
5 V, GND, SCL (PB10), SDA (PB11), which match the first four pins of the GY-521
(VCC, GND, SCL, SDA); its other four pins (XDA, XCL, AD0, INT) stay free. The board's
own 4.7 kΩ pull-ups on this bus go to 5 V, so use only modules that are 5 V-safe on both
power and I2C pins; the GY-521 is what the vendor ships for this header.

- At start-up the firmware resets and configures the chip (gyro ±500 °/s, accel ±4 g,
  1 kHz internal sampling, ~42 Hz low-pass filter); boot takes another 100 ms for that.
- It then reads it every 10 ms and sends `#I` lines (see §4).
- If no IMU answers, the board says `#IMU NONE` and keeps trying once a second. If a
  working IMU stops answering for 0.5 s, it says `#IMU LOST` and does the same.
- The OLED (J9) and the IMU share this I2C bus (`Hardware/I2C_Bus.c`). The OLED is only
  written at boot, so they never get in each other's way.

---

## 3. How to build and flash

This folder is a complete Keil MDK project (ARM Compiler 5.06).

1. Open `project.uvprojx` in Keil µVision.
2. **Debug tab → ST-Link Debugger** → Settings → Flash Download → confirm the STM32F10x
   flash algorithm.
3. Build (F7), then **LOAD**.

Command-line build (same result):

```
C:\Keil_v5\UV4\UV4.exe -r project.uvprojx -j0 -o build.log
```

The IWDG is frozen while the core is halted in the debugger, so breakpoints don't
cause resets.

---

## 4. Wire protocol

Baud rate: **115200 8N1**. Both directions are line-based. Inbound packets start with
`@` and end with `\r`, `\n` or `\r\n`; the payload is at most 64 characters.

### Pi → Board (commands)

| Frame                       | Meaning                                         |
|-----------------------------|-------------------------------------------------|
| `@S<n> <angle>\r\n`         | Servo `n` (1..3) to angle `0..180` (90 = center). `n=1` steering, `n=2` cam pan, `n=3` cam tilt. |
| `@M <left> <right>\r\n`     | Motor power, signed `-1000..+1000` per-mil. Resets motor watchdog. |
| `@H\r\n`                    | Halt motors. Servos hold. Equivalent to `@M 0 0`. |
| `@R\r\n`                    | Reset all servos to center.                     |
| `@Q\r\n`                    | Force an immediate `#T` and `#IMU` status line. |

### Board → Pi

| Frame                                | When                          |
|--------------------------------------|-------------------------------|
| `#BOOT NiumoMapper v2 RST=<cause>\r\n` | Once at startup. Cause: `POWER`, `PIN`, `IWDG`, `SOFT`, `OTHER`. `RST=IWDG` means the watchdog caught a hang. |
| `#OK\r\n`                            | Per accepted command.         |
| `#ERR <code>\r\n`                    | `BADCMD`, `PARSE`, `RANGE`, `EMPTY`, `LONG`. |
| `#T <ms> <encL> <encR>\r\n`          | 50 Hz, unconditional.         |
| `#I <ms> <ax> <ay> <az> <gx> <gy> <gz> <temp>\r\n` | 100 Hz while the IMU answers. |
| `#IMU OK <id>\r\n` / `#IMU NONE\r\n` / `#IMU LOST\r\n` | At start-up, on `@Q`, and when the IMU appears or disappears. `<id>` is WHO_AM_I in hex (`0x68` on a genuine MPU6050). |

Field types:

- `ms`       unsigned 32-bit free-running ms tick (wraps every ~49.7 days)
- `encL`     signed 32-bit cumulative encoder count since boot, motor 1 (left rear)
- `encR`     signed 32-bit cumulative encoder count since boot, motor 2 (right rear)
- `ax ay az` acceleration in milli-g (±4000), axes as printed on the GY-521
- `gx gy gz` angular rate in 0.01 °/s (±50000)
- `temp`     IMU chip temperature in 0.01 °C

**Encoder sign.** The counts are flipped so that driving forward makes both go up.
The default follows the vendor's Ackermann program (motor 1 counts down when driving
forward). If one side counts the wrong way on your robot, flip
`PROTOCOL_ENC_LEFT_SIGN` / `PROTOCOL_ENC_RIGHT_SIGN` in `User/protocol.h`.

**Dropped output.** If the TX ring is full (for example the Pi stops reading), new
replies and telemetry lines are dropped whole rather than blocking the board.

### Example Pi-side session

```
< #BOOT NiumoMapper v2 RST=POWER
< #T 1240 0 0
< #T 1260 0 0
> @S1 95
< #OK
> @M 500 500
< #OK
< #T 1300 12 11
< #T 1320 24 23
```

---

## 5. Servo angle ranges

`Hardware/Servo.c` clamps each channel to safe values:

| Channel  | Header | Min     | Center  | Max     | Usable angles |
|----------|--------|---------|---------|---------|---------------|
| Steering | PB14   | 1200 µs | 1500 µs | 1900 µs | 63–126 |
| Cam pan  | PB13   | 500 µs  | 1167 µs | 1856 µs | 30–152 |
| Cam tilt | PB15   | 800 µs  | 1500 µs | 1850 µs | 27–122 |

Angles are relative to each servo's centre: `pulse = centre + (angle − 90) × 2000 / 180` µs,
the full travel of a standard 180° servo, so one command degree is about one real degree and
**90 always means the servo's centre** (straight ahead), as does `@R`. (v1 and early v2 used
`1000..2000 µs`, half the travel, which made the camera turn only ~18° one way.)
Out-of-range commands are rejected with `#ERR RANGE`; commands *within* `0..180` but outside
the clamp are silently capped. With a trimmed centre, one side reaches the clamp before 0 or
180.

**Camera pan, measured 30 Oct 2026 with `../pi_examples/pan_sweep.py`:**
- **Straight ahead is 1167 µs:** the servo arm sits about 30° off, so the centre is trimmed.
- **Right limit, 500 µs:** the servo's own end, about 60° right.
- **Left limit, 1856 µs:** about 62° left; the point where you stopped the sweep, minus a 3° margin.

Command angles 30–152 cover that range, and 90 points straight ahead. Re-seating the servo
arm two teeth over would bring the centre back near 1500 µs. After that, re-run
`pan_sweep.py --trim-only` and update `center_us`.

If your steering linkage hits its end-stop before reaching the clamp, **tighten the
limits in `servo_limit[]` in `Servo.c`** rather than doing it from the Pi. That way a Pi
crash can't jam the servo.

---

## 6. Pi-side reference

**[PI_GUIDE.md](../docs/PI_GUIDE.md)** is the full guide: connecting, every module, a complete
Python client (`../pi_examples/niumo_board.py`), a hardware check (`../pi_examples/hw_test.py`),
worked examples (steering check, encoder
calibration, IMU bias, dead reckoning), and C++/shell/ROS 2 notes.

The board appears on the Pi as a `/dev/ttyUSB*` port (CH340, VID:PID 1a86:7523):
`ttyUSB0`, or `ttyUSB1` if the LiDAR's adapter was found first. The udev rule in
`docs/PI_GUIDE.md` gives it the fixed name `/dev/robot_mcu`.

`robot_mapping/common/mcu.py` keeps working unchanged: it parses `#T` (same 4 fields)
and ignores `#I` and `#IMU`. Its "encoders are always 0" note no longer applies to v2.

Minimal Python loop:

```python
import serial
s = serial.Serial('/dev/robot_mcu', 115200, timeout=0.1)

def cmd(c):
    s.write(f'@{c}\r\n'.encode())

cmd('S1 95')            # steer slightly right
cmd('S2 90')            # cam pan center
cmd('M 400 400')        # crawl forward
while True:
    parts = s.readline().decode(errors='ignore').split()
    if parts[:1] == ['#T'] and len(parts) == 4:
        ms, enc_l, enc_r = map(int, parts[1:])
        print('enc', ms, enc_l, enc_r)
    # keep sending @M ... at >= 2 Hz or the watchdog will halt you
```

---

## 7. Files

```
User/
  main.c          Boot, reset cause, OLED error screen, 1 kHz tick (TIM6), IWDG, main loop
  protocol.c/.h   Command dispatcher, motor watchdog, #T / #I telemetry
  stm32f10x_it.c  Fault handlers: zero the motor PWM, then wait for the IWDG
Hardware/
  Servo.c/.h      TIM1 hardware PWM on PB13/PB14/PB15
  Usart.c/.h      USART1 with interrupt-driven RX/TX ring buffers + packet assembler
  I2C_Bus.c/.h    Bit-banged I2C on PB10/PB11 (shared by J9 and J10), bus recovery
  OLED.c/.h       SSD1306 on J9, used at boot only
  OLED_Font.h     8x16 ASCII font from the vendor OLED example
  MPU6050.c/.h    GY-521 on J10: configuration and 14-byte burst read, scaled units
  encoder.c/.h    EXTI quadrature decoding on PC0..PC3, cumulative counts
  Motor.c/.h      TIM2 PWM on PA0..PA3 (vendor driver) + MOTOR_FORCE_STOP()
Start/
  startup_stm32f10x_hd.s   vendor startup file, stack raised to 2 KB
README.md         this file
project.uvprojx   Keil project (open this); project.uvoptx holds the debugger/flash settings
```

Elsewhere in the repository:

```
../pi_examples/    Python for the Pi: niumo_board.py (client), niumo_examples.py,
                   hw_test.py (servo, motor and encoder check), pan_sweep.py (camera pan)
../docs/           PI_GUIDE, DEVELOPMENT, CAMERA_PAN_CALIBRATION, MOTORS_AND_ENCODERS, ROADMAP
```

Earlier firmware versions (v1 and the intermediate v2 builds) are archived outside the
repository.
