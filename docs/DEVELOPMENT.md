# RobotMapper firmware v2: development record

This document records how firmware v2 for the Niumo V5 control board was developed:
the starting point, the design decisions, how each part was built and checked, the
problems met on the way and how they were solved, and the reference material used.

For *using* the firmware, see [firmware/README.md](../firmware/README.md) (wiring, build, protocol) and
[PI_GUIDE.md](PI_GUIDE.md) (talking to the board from the Raspberry Pi). The camera-pan
calibration is written up in detail in [CAMERA_PAN_CALIBRATION.md](CAMERA_PAN_CALIBRATION.md).

Period covered: 23–30 October 2026.

Contents

1. [Starting point](#1-starting-point)
2. [Goals and design decisions](#2-goals-and-design-decisions)
3. [Methods and tools](#3-methods-and-tools)
4. [Development, step by step](#4-development-step-by-step)
5. [Bringing it up on the robot](#5-bringing-it-up-on-the-robot)
6. [Problems and solutions](#6-problems-and-solutions)
7. [Resulting architecture](#7-resulting-architecture)
8. [Reference material](#8-reference-material)
9. [Current state and open items](#9-current-state-and-open-items)
10. [Lessons learned](#10-lessons-learned)

---

## 1. Starting point

**The robot.** An Ackermann chassis (steered front wheels, two driven rear wheels with
encoder motors) carrying a Raspberry Pi 5 with a Hailo-8 accelerator, an OAK-D Lite depth
camera on a pan servo, and an LDROBOT STL-27L LiDAR. The Pi does the mapping; the
control board drives the motors and servos and reports the encoders.

**The control board.** YourFun "Niumo" V5 main control board with an STM32F103RCT6
(Cortex-M3, 72 MHz, 256 KB flash, 48 KB RAM, no floating-point unit). The Pi reaches it
through the board's Type-C port, which goes to a CH340E USB-to-serial chip on USART1.

**Firmware v1.** An earlier, short application layer ("RobotMapper", kept in
`RobotMapper_v1_backup/`) built on top of the vendor's example project
*"2. PS2 Control Encoder Ackermann Chassis + Robotic Arm"*. It took the vendor's
drivers (StdPeriph library, software servo PWM, TIM2 motor PWM, encoder interrupts,
USART1 parser) and added a text protocol for the Pi: `@S` servo, `@M` motor, `@H` halt,
`@R` centre, `#T` telemetry, plus a 500 ms motor watchdog.

**The question that started v2.** Should the firmware be replaced, for example with an
RTOS and new code, or kept? The wish list was:
- an OLED for information;
- battery level without extra boards;
- motor and servo control from the Pi, with motor feedback;
- an IMU later.

**What a review of v1 found.** v1 was read line by line against the board schematic and
manual. Five defects had to be fixed whichever way development went:

| # | Defect | Consequence |
|---|---|---|
| 1 | Telemetry read `Avg_PulseCount`, which nothing ever updated | `#T` always reported `0 0`: no wheel odometry |
| 2 | Interrupts enabled on the unused encoder headers EC3/EC4 (PC6–PC9, floating), with no interrupt handler | One noise edge sends the CPU into the default endless loop. The motor PWM keeps running and the 500 ms motor watchdog never runs again. There was no hardware watchdog, and the HardFault handler left the motors on. |
| 3 | Servos driven in software by a TIM1 interrupt every 10 µs (100 kHz), compiled at -O0 | An estimated 25–35 % of the CPU. Other interrupts shift the pulse edges, which makes the steering twitch. |
| 4 | Motor PWM at 100 Hz | Audible, coarse at low speed (about 20 kHz is usual for these drivers) |
| 5 | UART receive buffer of 100 bytes with no bounds check, one shared buffer, blocking transmit (~2 ms per telemetry line) | A long garbage line overwrites memory; back-to-back commands can be lost |

---

## 2. Goals and design decisions

| Decision | Choice | Reason |
|---|---|---|
| RTOS or bare metal | **Keep bare metal** (a main loop plus interrupts) | Everything on the list is light work for a 72 MHz MCU when hardware timers do the PWM. An RTOS adds stack sizing, priority rules and harder debugging, and solved nothing that was broken. (The vendor's robotic-arm example is a CubeMX + HAL + FreeRTOS project for this board, so that route stays open.) |
| Order of work | Safety fixes first, then hardware servo PWM, encoder telemetry, battery ADC, non-blocking UART | Fix what can hurt the robot before adding features |
| Servos | **TIM1 hardware PWM** on PB13/PB14/PB15 (TIM1_CH1N/CH2N/CH3N) | Zero CPU load, 1 µs resolution, no jitter. The old steering header PB12 has no timer output on this chip, so **the steering servo had to move to PB14**. |
| OLED (J9) | **Errors only**; at present just `WATCHDOG RESET` | User decision. Pi information belongs on a display driven by the Pi itself. The vendor's OLED code blocks, so it may only run at boot. |
| Battery measurement | Designed (47 kΩ / 10 kΩ divider on PB0), then **dropped** | The board has no battery sensing; it needs soldering. User decision on 25 Oct: no divider, so the code was removed. |
| IMU | Vendor's **GY-521 (MPU6050)** on the gyro header J10, **own driver**. *Dropped on 14 Nov 2026: only the OAK-D Lite's IMU is used.* | The header supplies 5 V and its I2C pull-ups go to 5 V, which the GY-521 tolerates and most newer 3.3 V-only breakouts don't. The vendor's MPU6050 code is ALIENTEK's and marked "for learning only", so it was read but not copied. |
| Sensor fusion | **On the Pi**, not on the board | The board easily has the CPU time, but the fusion belongs with the mapping stack: easier to tune, no re-flashing, and it can combine the camera and LiDAR too. The board's job is raw, time-stamped data and fast control loops. |
| Protocol | Keep the v1 text protocol, add lines | Backward compatible: older Pi code ignores lines it doesn't know |
| Link speed | Keep 115200 baud | `#T` at 50 Hz plus `#I` at 100 Hz use about 50 % of the link (under 70 % worst case) |
| Servo scale | 0..180 → **500..2500 µs** (changed 25 Oct) | One command degree ≈ one real degree on a standard servo; the old 1000..2000 µs used half the travel |

---

## 3. Methods and tools

**Development environment.**
- Keil MDK 5.37 with ARM Compiler 5.06u7, STM32 Standard Peripheral Library, C99, -O0
  (the project's existing settings).
- Every change was rebuilt from the command line and had to finish with
  **0 errors, 0 warnings**:
  `C:\Keil_v5\UV4\UV4.exe -r project.uvprojx -j0 -o build.log`
- Stack use was taken from Keil's static call-graph report (worst case 520 bytes plus
  interrupt frames), which led to raising the stack to 2 KB.
- v1 was backed up first (`RobotMapper_v1_backup/`), and the finished work was reviewed
  as a diff against it.

**How the code was designed.**
- **Datasheet-first.** Pins came from the board schematic; peripheral settings from the
  STM32F10x reference manual; IMU settings from the MPU-6000/6050 register map.
- **Hardware does the timing.** PWM comes from timers. The ADC (while it existed)
  free-ran in continuous mode. UART transmit is drained by an interrupt, and reception is
  buffered by one.
- **Nothing in the main loop waits for long.** The only deliberate wait is the ~1 ms IMU
  read every 10 ms. The OLED's blocking writes happen only at boot, before the watchdog
  starts.
- **Fail safe.** Motor watchdog, hardware watchdog, and motors forced to zero in every
  fault handler.

**How the Pi-side code was tested before the hardware existed.**
- The Python client (`niumo_board.py`), the examples and the test scripts were run on the
  PC against a **simulated board**: a fake `serial` module that produces `#T`/`#I` lines,
  reacts to commands and integrates encoder counts from motor power.
- Scenarios included lines split across reads, garbage bytes, board restarts, swapped
  encoder cables, an encoder counting backwards, a board that browns out under load, a
  silent port, a restart loop, garbled data, the LiDAR's port given instead of the
  board's, and no board plugged in.

**How the hardware was brought up.**
1. Flash with an ST-Link over SWD, checking the debug ID first.
2. Look at the raw serial output from a terminal.
3. Run an interactive test (`hw_test.py`) that moves one thing at a time, asks the
   operator what happened, and checks the encoders automatically.
4. Measure the mechanical limits with a slow sweep (`pan_sweep.py`).

---

## 4. Development, step by step

**Step 1: safety** (`encoder.c`, `stm32f10x_it.c`, `main.c`, `Motor.h`)
- Interrupts limited to EXTI lines 0–3 (EC1/EC2 on PC0–PC3). EC3/EC4 pins became inputs
  with pull-ups, and a defensive `EXTI9_5_IRQHandler` was added.
- Encoder inputs got pull-ups, so an unplugged cable stays quiet.
- Independent watchdog (IWDG): about 200 ms (133–267 ms with the LSI clock tolerance),
  frozen while the debugger halts the core.
- `MOTOR_FORCE_STOP()` (zeroes the four TIM2 compare registers) in every fault handler.
- The reset cause is read from the RCC flags and reported in `#BOOT ... RST=<cause>`.
- The motor command timeout was made explicit with an "armed" flag.

**Step 2: hardware servo PWM** (`Servo.c`)
- TIM1 at 1 MHz (72 MHz / 72) with a 20 000-count period: 50 Hz frames, 1 µs steps.
- Only the complementary outputs are enabled (CCxE = 0, CCxNE = 1), so each CHxN pin
  follows the PWM reference directly. Being an advanced timer, TIM1 also needs the main
  output enable (MOE) bit.
- Per-servo end limits are clamped in firmware. PB12 and PC4 are held low.

**Step 3: encoder telemetry** (`encoder.c`, `protocol.c`)
- X4 quadrature decoding in the EXTI handlers, cumulative `int32` counts per wheel.
- The sign is normalised so that forward is positive on both sides (left −1, right +1,
  taken from the vendor's Ackermann program). Hardware testing later confirmed it.
- `#T <ms> <left> <right>` at 50 Hz keeps v1's exact format.

**Step 4: battery ADC** (`Battery.c`). Removed on 25 Oct.
- ADC1 channel 8 (PB0) in continuous mode, 239.5-cycle sampling, 32-sample average,
  `#B <ms> <mV>` at 2 Hz.
- Removed because no divider will be fitted (§6, problem 15).

**Step 5: non-blocking UART** (`Usart.c`)
- 256-byte receive ring filled by the RX interrupt; 1 KB transmit ring drained by the
  TXE interrupt.
- A line assembler accepts `\r`, `\n` or `\r\n` and rejects lines over 64 characters
  with `#ERR LONG`.
- At most 4 commands are handled per loop pass. A message that doesn't fit the transmit
  ring is dropped whole, never waited on.

**Step 6: OLED for errors** (`OLED.c`, `OLED_Font.h`)
- Built on the vendor example's SSD1306 init sequence and 8×16 font (95 glyphs).
- Detects a missing display from the missing ACK and then skips all writes.
- Shows `WATCHDOG RESET` after an IWDG reset. Written only at boot.

**Step 7: shared I2C bus and IMU** (`I2C_Bus.c`, `MPU6050.c`, `protocol.c`)
- One bit-banged I2C driver for J9 and J10 on PB10/PB11: open-drain, about 2 µs half
  period, a bounded wait for clock stretching, and 9-clock bus recovery after a reset
  mid-transfer.
- MPU6050 settings: ±500 °/s, ±4 g, 1 kHz internal rate, DLPF ~42 Hz.
- One 14-byte burst read every 10 ms, converted on the board to milli-g, 0.01 °/s and
  0.01 °C.
- `#IMU OK/NONE/LOST` state machine: after 50 failed reads the IMU is LOST; NONE and
  LOST retry every second.
- Stack raised from 1 KB to 2 KB.

**Step 8: Pi-side documentation and client**
- `PI_GUIDE.md`: connecting, the protocol, every module, a Python client, worked
  examples (steering check, counts per metre, IMU bias, dead reckoning, CSV logging),
  shell/C++/ROS 2 notes, troubleshooting.
- `pi_examples/niumo_board.py`, `niumo_examples.py`.

**Step 9: hardware test tools** (25 Oct)
- `hw_test.py`: interactive servo, motor and encoder check. It diagnoses a silent port
  and finds the board by its USB ID.
- `pan_sweep.py`: slow camera sweep to find the safe pan range.

**Step 10: full servo travel** (25 Oct)
- Angle mapping changed to 500..2500 µs.
- Camera pan opened to the full range until measured; steering limits kept.

**Step 11: centre trim** (30 Oct)
- The angle → pulse conversion moved from `protocol.c` into `Servo_SetAngle()` in `Servo.c`
  and became relative to each servo's `center_us`. Angle 90, `@R` and start-up now all mean
  "straight ahead", even when a servo horn is mounted a tooth off.
- With every centre at 1500 µs the pulses are identical to step 10 for all 181 angles
  (checked), so steering behaves exactly as before.
- `pan_sweep.py` got numeric nudges (`-20`, `+5`, or an absolute angle such as `70`) and a
  `--trim-only` mode that prints the `center_us` value.

---

## 5. Bringing it up on the robot

| Step | What happened | Result |
|---|---|---|
| ST-Link connection | Keil showed IDCODE `0x1BA01477` | Correct for an STM32F1 (my instructions had wrongly said `0x2BA01477`) |
| First serial check | `/dev/robot_mcu` missing; board found as `1a86:7523` on `ttyUSB1` | Clean `#T` lines; uptime 393 s without a reset; encoder counts non-zero, so v2's counting worked |
| Battery line | `#B` read ~9.5 V from an unconnected pin | Battery feature dropped, code removed, rebuilt |
| `hw_test.py`, first runs | "No data", then noise | Caused by a port mix-up (§6, problem 16), not by the board |
| Port fixed with udev rule | Clean `#T` every 20 ms, no `#B` | Battery-less build confirmed running |
| `hw_test.py`, full run | Everything passed except steering | Steering servo was on the wrong header; after moving it, **all passed**: steering, pan, both motors forward and backward, both encoders with correct sign |
| Camera pan | Turned only 10–20° when asked for 90° | Servo scale fixed (§6, problem 19) |
| First pan sweep (30 Oct) | Camera hardly moved | The board still ran the previous build (§6, problem 20) |
| Pan sweep on the new build | Full 0..180 (500..2500 µs) with no contact; below 90 turns right | At 1500 µs the camera points off-centre (§6, problem 21) |
| Second sweep (30 Oct) | Straight ahead at 1167 µs; operator stopped the left side at 1889 µs | Pan set to 500 / 1167 / 1856 µs (3° margin on the left): about 60° right and 62° left of straight ahead |

---

## 6. Problems and solutions

### During development (on the PC)

| # | Problem | Cause | Solution |
|---|---|---|---|
| 1 | v1 defects (§1) | Example-project code reused without a safety review | Fixed in steps 1–5 |
| 2 | A macro in `Motor.h` was written with a literal `\n` in it | Backslashes mangled when C code was written through a shell heredoc | Macro rewritten on one line; afterwards C files were written with a file tool, not the shell |
| 3 | Some vendor files couldn't be edited | They are GBK-encoded (Chinese comments), and the editing tool only accepts UTF-8 | Backed up, then rewritten from scratch as UTF-8 (`Servo.c`, `Usart.c`) |
| 4 | Worst-case stack 520 B + interrupts against a 1 KB stack | IMU code added depth | `Stack_Size` raised to 2 KB in `startup_stm32f10x_hd.s` |
| 5 | Vendor OLED driver blocks for tens of ms | Bit-banged I2C with waits | OLED written only at boot, before the IWDG starts |
| 6 | OLED and IMU share one I2C bus; a reset mid-transfer can leave SDA held low | I2C has no reset line | 9-clock bus recovery at init and after a failed IMU read |
| 7 | Vendor MPU6050 code not reusable | ALIENTEK licence: "for learning only" | Own driver from the MPU-6000/6050 register map |
| 8 | Newer IMUs could be damaged on J10 | J10 gives 5 V, pull-ups to 5 V | Chose the 5 V-tolerant GY-521 |
| 9 | Python test failures | Mistakes in the tests themselves (module not registered before loading; a 100 ms step hit the 0.1 s gap filter) | Tests corrected; the client code was right |
| 10 | C++ example couldn't be checked | No Linux compiler on the PC | Marked "not compiled" in the guide |
| 11 | Three wrong statements in a guide draft | My errors (what a floating ADC pin reads; whether USB powers the logic; motor-direction advice) | Corrected before publishing |

### During bring-up (on the robot)

| # | Problem | Cause | Solution |
|---|---|---|---|
| 12 | Debug ID didn't match my instructions | My instructions gave the Cortex-M4 value | The board's `0x1BA01477` is correct for STM32F1; instructions corrected |
| 13 | `could not open port '/dev/robot_mcu'` | The udev rule that creates this name wasn't installed on the Pi | Rule installed; `/dev/serial/by-id/` as a fallback |
| 14 | No `#I` lines | No IMU fitted yet (it will be bought later) | Expected; the board reports `#IMU NONE` |
| 15 | `#B` showed ~9.5 V | PB0 not connected: a floating input reads anything | Battery measurement dropped by the user; code, docs and Python parts removed (code size 13 000 → 12 088 bytes) |
| 16 | `hw_test.py` saw no data; terminal showed only noise, even after reset | **The board and the LiDAR had swapped `ttyUSB0`/`ttyUSB1`** after re-enumerating (about 17:29), so the LiDAR's 921600-baud stream was being read at 115200. I first misdiagnosed it as a brownout and then as a clock fault. | Found with a baud-rate scan: at 921600 the data contained the LiDAR's 3-byte point records (e.g. `8a 05 b6` = 1418 mm). Fixed with a udev rule for both devices (`/dev/robot_mcu`, `/dev/robot_lidar`). `hw_test.py` now checks the USB ID and switches to the board's CH340 by itself. |
| 17 | Noise again on the correct port | A newly enumerated serial port opens at the Linux default of 9600 baud | `stty -F /dev/robot_mcu 115200 raw -echo` before `cat` (scripts using pyserial set the speed themselves) |
| 18 | Steering servo didn't move | Plugged into the wrong header (v1 used the top header PB12, which v2 holds low) | Moved to PB14; passed |
| 19 | Camera turned only 10–20° when 90° was intended | Firmware, not the mount: angles mapped to 1000..2000 µs (half a servo's travel) **and** the vendor's pan clamp of 1300..2200 µs. `@S2 0` gave 1300 µs, about 18°. | Mapping changed to 500..2500 µs; pan opened to 500..2500 µs; `pan_sweep.py` measures the mount's safe range for the final limits |
| 20 | During the first sweep the camera "didn't move" | The previous build was still flashed: each 5° step moved only ~2.5°, and one side stopped at the old clamp. Extra Enter presses typed during moves also skipped steps. | Flashed the servo-scale build. `pan_sweep.py` now discards keys pressed while the camera moves, and its prompt lists when to answer "too far". The build in use can't be read back over the protocol yet; a version or build-date field in `#BOOT` would make this visible. |
| 21 | At the centre (1500 µs) the camera points left (first estimated at 20°) | Mechanical: the servo arm sits about 30° off (two spline teeth). Measured in the second sweep: straight ahead at 1167 µs. | Firmware centre trim (`Servo_SetAngle`, step 11) with `center_us` = 1167 µs, so `@S2 90` and `@R` point straight ahead. Usable range is now about 60° right and 62° left. Re-seating the arm would restore a symmetric range. |

---

## 7. Resulting architecture

```
                    ┌──────────────── STM32F103RC, 72 MHz ─────────────────┐
 Raspberry Pi  USB  │ USART1 115200  RX ring ─► line assembler ─► protocol │
 ◄────────────────► │  (CH340E)      TX ring ◄─ #T 50 Hz, #I 100 Hz, #OK   │
                    │                                                      │
                    │ TIM2 PWM ──► motor drivers M1/M2       (PA0..PA3)    │
                    │ EXTI0..3 ◄── encoders EC1/EC2, X4      (PC0..PC3)    │
                    │ TIM1 CHxN ─► servos: pan PB13, steering PB14, tilt PB15
                    │ bit-bang I2C (PB10/PB11) ─► OLED J9 (boot), MPU6050 J10
                    │ TIM6 1 kHz tick · IWDG ~200 ms · fault handlers stop motors
                    └──────────────────────────────────────────────────────┘
```

**Boot order.** Motors (stopped), servos (centred), encoders, UART, I2C, OLED, IMU,
1 kHz tick, protocol, `#BOOT`, then the watchdog.

**Main loop.** Read the tick, feed the watchdog, run `Protocol_Tick`, which:
1. handles up to 4 commands;
2. runs the motor watchdog;
3. sends `#T` every 20 ms;
4. reads the IMU every 10 ms.

**Files.** See [firmware/README.md](../firmware/README.md) §7. Firmware: `User/main.c`, `User/protocol.c/.h`,
`User/stm32f10x_it.c`, `Hardware/{Servo,Usart,encoder,Motor,I2C_Bus,OLED,MPU6050}.c/.h`,
`Start/startup_stm32f10x_hd.s`. Pi side: `docs/PI_GUIDE.md`, `pi_examples/`.

**Size.** Code 12 088 B, RO 2 204 B, RW 76 B, ZI 4 076 B (latest build).

---

## 8. Reference material

### 8.1 Vendor documentation (in the resource pack)

| Document | Location | Used for |
|---|---|---|
| Main control board manual (PDF) and its English translation | `D:\downloads\A robot docs\…\Main_Control_Board_Manual_EN.md`; `C:\Users\Nikon\Desktop\отчеты\робот\Main Control Board Instruction Manual.pdf` | Header layout (servo row PB13/PB14/PB15/PC4, OLED J9, gyro J10, Type-C serial), powering the board from the battery while flashing, ST-Link + Keil flashing procedure, list of example programs |
| Board schematic "牛魔主控板原理图V1.1" (Niumo main board schematic, V1.1, 2023) | `…\3. Development Board Schematic\牛魔主控板原理图V1.1.pdf` | Every pin assignment: SERVO_1..6 = PB12, PB13, PB14, PB15, PC4, PC5; motors M1/M2 on PA0..PA3 (TIM2); encoders EC1..EC4 (HALL_1A..4B) on PC0..PC3 and PC6..PC9; I2C on PB10/PB11 with 4.7 kΩ pull-ups to 5 V; no battery divider; free ADC pins AD8/AD9 (PB0/PB1); CH340E with its RTS pin unconnected (so opening the port doesn't reset the board); BOOT0 switch; reset circuit |
| Ackermann chassis documents | `D:\downloads\A robot docs\Ackermann chassis and sofrware code\…` | Motor and encoder parameters, chassis dimensions (for the odometry part of the guide) |

### 8.2 Vendor example programs (in the resource pack, `5. Program`)

| Example | Used for |
|---|---|
| *Smart Car Experiment Programs / 2. PS2 Control Encoder Ackermann Chassis + Robotic Arm* | Base of v1 and therefore v2: project structure, StdPeriph library, TIM2 motor driver, encoder interrupt code, and the encoder direction convention (motor 1 counts down when driving forward) |
| *Development Board Basic Experiments / 3. OLED Display / 3.1 OLED Display* | SSD1306 initialisation sequence and the 8×16 ASCII font (`OLED_Font.h`) |
| *Development Board Basic Experiments / 5. MPU6050 Gyroscope Experiment* | Confirmed the J10 wiring and I2C address. Code is ALIENTEK's, "for learning only", so it was not copied. |
| *F1-V5 – Robotic Arm (4-DOF Parallel Gripper)* (CubeMX + HAL + FreeRTOS) | Reference for the RTOS question: shows an RTOS setup for this board exists if ever needed |

### 8.3 Chip and standard references

These were used from general engineering knowledge rather than opened as files during
this work. Check against the official documents if a detail matters.

| Reference | Used for |
|---|---|
| ST **RM0008**, STM32F101xx–F107xx reference manual | TIM1 complementary outputs and MOE; IWDG prescaler/reload; RCC reset flags; ADC calibration and sample time; EXTI; USART interrupts; SW-DP IDCODE `0x1BA01477` |
| ST **STM32F103xC/D/E datasheet** | Alternate functions: TIM1_CH1N/CH2N/CH3N on PB13/PB14/PB15 without remap; no timer on PB12/PC4 |
| ST **STM32F10x Standard Peripheral Library** (in `Library/`) | All peripheral set-up calls |
| InvenSense **MPU-6000/MPU-6050 Register Map and Descriptions, rev. 4.2**, and the product specification | Register addresses and values, scale factors (8192 LSB/g at ±4 g, 65.5 LSB/(°/s) at ±500 °/s), temperature formula (raw / 340 + 36.53), DLPF settings |
| Solomon Systech **SSD1306** datasheet | Display addressing (0x3C), command set |
| NXP **UM10204**, I2C-bus specification | Open-drain signalling, clock stretching, 9-clock bus recovery |
| WCH **CH340** documentation | USB ID `1a86:7523`, Linux driver behaviour |
| LDROBOT **STL-27L** protocol | 921600 baud and the point-record format; used to identify the "noise" in problem 16 |
| Hobby-servo convention (50 Hz, ~500–2500 µs for ~180°) | Servo timing and the scale change in problem 19 |
| pyserial and Linux udev documentation | `serial.tools.list_ports`, custom baud rates, `/dev/serial/by-id`, persistent device names |

### 8.4 Project files that informed the work

- `RobotMapper_v1_backup/`: the v1 sources and README (now archived outside the repository).
- `robot_mapping/` (then in the PyCharm project, now in this repository): `common/mcu.py` (the Pi's existing board client),
  `config/99-robot-usb.rules` (udev rules), `install.sh`. These were used to keep the
  protocol compatible and to confirm the mapping software only ever centres the camera.

---

## 9. Current state and open items

**Verified on the robot (25 Oct).**
- Battery-less v2 runs: steady `#T` every 20 ms, no unexpected resets.
- Steering and camera-pan servos move.
- Both motors run forward and backward.
- Both encoders count the right way.
- The udev rule (`/etc/udev/rules.d/99-robot-usb.rules`) is installed on the Pi.

**Verified on the robot (30 Oct).** The servo-scale build: the camera pan turns through
the full 500..2500 µs without touching the mount.

**Verified on the robot (30 Oct, after flashing the centre-trim build).** `@R` and `@S2 90`
point the camera straight ahead; `@S2 30` turns it about 60° right and `@S2 152` about 60°
left, with the cable clear. Pan constants: 500 / 1167 / 1856 µs. Details in
[CAMERA_PAN_CALIBRATION.md](CAMERA_PAN_CALIBRATION.md).

**Not verified on hardware.**
- IMU code: never run with a GY-521 fitted. On 14 Nov 2026 the board IMU was dropped as
  outdated: the robot uses only the OAK-D Lite's built-in IMU, and the firmware's MPU6050
  support will be removed with the next firmware change.
- `WATCHDOG RESET` on the OLED: needs a real watchdog reset to show.
- The C++ example in the guide.

**Open items.**

| Item | Notes |
|---|---|
| Optional: measure the pan angles quantitatively | Method in CAMERA_PAN_CALIBRATION.md §11 (camera sees a target; fit scale and centre) |
| Update `robot_mapping/common/mcu.py` | Real encoder counts, parse `#I`, use `/dev/robot_mcu`, send a heartbeat (`@P`) so the OLED could show `NO PI LINK` |
| Motor PWM at ~20 kHz and a closed-loop speed command | `compute_pid.c` is available in the project |
| Remove the unused MPU6050 support | Board IMU dropped on 14 Nov 2026 (camera IMU only) |
| Centre the camera pan from the left in the firmware too (`@R` and start-up) | The mount has 5–10° of hysteresis; approaching from the right lands short (CAMERA_PAN_CALIBRATION.md §9.1). The Pi-side `CameraPan` already does this. |

The project moved into the `robotMK1` repository on 14 Nov 2026. Older copies of the firmware
are archived outside it, so the earlier item "sync v2 into the other copies" no longer applies.

---

## 10. Lessons learned

- **Check the device before diagnosing it.** Problem 16 cost several rounds because
  `ttyUSB` numbers are not stable when two USB-serial adapters are attached. Use
  persistent names (`/dev/robot_mcu`), or check `ls -l /dev/serial/by-id/` first. A baud
  scan of the raw stream identified the real sender quickly.
- **Don't diagnose garbage from its byte pattern alone.** The runs of `00`/`FF`/`F0` bytes
  in problem 16 looked like a *slower* sender, and that led to the clock-fault theory, but
  the sender was *faster* (the LiDAR at 921600 baud). Two checks settle it quickly:
  - **Data volume:** far more bytes than the board ever sends points to another device.
  - **A baud-rate scan:** the rate at which the data turns into structured records
    identifies the sender.
- **Example-project limits aren't your limits.** Both the narrow servo scale and the pan
  clamp were inherited, and they looked like a mechanical problem.
- **Simulated hardware catches most software errors early.** `hw_test.py` worked on the
  robot as soon as it was pointed at the right port. The remaining failures were physical
  (a servo on the wrong header).
- **Keep the board's job small.** Hardware timers, non-blocking I/O and fail-safe
  defaults kept a bare-metal loop simple enough to verify by reading. Estimation work
  stays on the Pi.
