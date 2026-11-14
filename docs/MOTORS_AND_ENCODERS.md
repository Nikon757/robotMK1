# Motors and wheel encoders: how they work

What the drive motors on RobotMK1 do, what information comes back from them, and how to
turn that information into distance, speed and odometry. It covers the hardware, the
firmware (RobotMapper v2) and the Pi side.

Related documents:
- [PI_GUIDE.md](PI_GUIDE.md): protocol and Python client;
- [DEVELOPMENT.md](DEVELOPMENT.md): firmware history;
- [ROADMAP.md](ROADMAP.md): closed-loop speed control and odometry fusion.

Contents

1. [Short answer](#1-short-answer)
2. [The hardware](#2-the-hardware)
3. [Driving a motor: from `@M` to the wheel](#3-driving-a-motor-from-m-to-the-wheel)
4. [The encoder signal](#4-the-encoder-signal)
5. [Decoding in the firmware](#5-decoding-in-the-firmware)
6. [The `#T` telemetry line](#6-the-t-telemetry-line)
7. [From counts to distance, speed and odometry](#7-from-counts-to-distance-speed-and-odometry)
8. [Calibrating counts per revolution and per metre](#8-calibrating-counts-per-revolution-and-per-metre)
9. [What is not reported](#9-what-is-not-reported)
10. [Accuracy and error sources](#10-accuracy-and-error-sources)
11. [Checking it yourself](#11-checking-it-yourself)
12. [References](#12-references)

---

## 1. Short answer

**The motors send nothing themselves.** They are plain brushed DC gear motors. Each one
carries a small **Hall-effect encoder** that produces two square-wave signals as the shaft
turns. The control board counts the edges of those signals and sends the Pi one line every
20 ms (50 times a second):

```
#T 393240 263 0
    │      │   └── right rear wheel: encoder count since the board started
    │      └────── left rear wheel: encoder count since the board started
    └───────────── board clock: milliseconds since the board started
```

**What the numbers mean:**
- The counts are **running totals**, not speeds.
- Forward rotation adds and backward rotation subtracts, on both sides.
- Distance and speed come from the **change** in the counts between lines, divided by the
  time between them.

---

## 2. The hardware

| Part | Detail |
|---|---|
| Motors | Two "520"-type brushed DC gear motors with encoders, one per rear wheel (vendor diagram: *Interface diagram of 520 encoder motor matched with Ackermann chassis*) |
| Encoder | Hall-effect, two outputs: phase A (C1) and phase B (C2), powered from the board (3.3 or 5 V). It sits on the **motor shaft**, before the gearbox. |
| Motor connector (6 wires) | M1 (motor +), GND (encoder −), C1 (phase A), C2 (phase B), VCC (encoder +), M2 (motor −) |
| Motor drivers | Two-input H-bridge drivers on the board (DRV8870-type per the schematic), one per motor |
| Board connections | Left motor: header M1, PWM on PA0/PA1 (TIM2 channels 1 and 2). Right motor: header M2, PA2/PA3 (TIM2 channels 3 and 4). Left encoder: header EC1, PC0 (A) / PC1 (B). Right encoder: header EC2, PC2 (A) / PC3 (B). |
| Unused | Motor headers M3/M4 and encoder headers EC3/EC4 (PC6–PC9): inputs with pull-ups, no interrupts |

Swapping the two motor wires (M1/M2 on the motor connector) reverses the motor's direction.
Swapping C1/C2 reverses the direction the encoder counts.

---

## 3. Driving a motor: from `@M` to the wheel

```
Pi: "@M <left> <right>"   (−1000 … +1000 per side, per-mil of full power)
  └► protocol.c: range check, apply_motor()
       └► Motor1_SetSpeed / Motor2_SetSpeed (Hardware/Motor.c)
            direction = sign, magnitude = |value|
            |value| ≤ 50  → 0     (dead zone: too little to overcome friction)
            └► TIM2 PWM: one driver input gets the PWM, the other is held low
                 └► H-bridge → motor
```

**PWM.**
- **Frequency:** TIM2 runs at 72 MHz / 720 = 100 kHz with a period of 1000 counts, so the
  PWM is **100 Hz**.
- **Duty cycle:** the compare value (0–1000) is the duty cycle in per-mil, so `@M 500 500`
  means 50 % duty on both motors.
- **Planned change:** 100 Hz is audible and coarse at low speed. The roadmap raises it to
  about 20 kHz (ROADMAP.md §10).

**Direction.** Each motor has two driver inputs. For one direction, input A gets the PWM and
input B is held low; for the other direction it's the reverse. During the off-part of each
PWM period both inputs are low, so the motor coasts briefly.

**Open loop.** `@M` sets **power**, not speed. The same power gives different speeds
depending on:
- load and floor;
- slope;
- the supply voltage;
- the friction of each motor.

That's why the two wheels may turn at slightly different speeds for the same command, and
why the roadmap adds a speed controller on the board.

**Safety.**
- **Command watchdog:** if no `@M` arrives for 0.5 s, the motors are stopped. A program
  driving the robot must repeat `@M` at least every 0.2 s.
- **Hardware watchdog:** about 200 ms. If the firmware hangs, the board resets, which also
  stops the motors.
- **Fault handlers:** they force the motor PWM to zero.

---

## 4. The encoder signal

A small magnet disc turns with the motor shaft. Two Hall sensors next to it output square
waves, **A** and **B**, shifted by a quarter of a period (90°). That's called *quadrature*.

```
forward:      ┌──┐  ┌──┐  ┌──┐            backward:   ┌──┐  ┌──┐  ┌──┐
          A ──┘  └──┘  └──┘  └──              A ──┘  └──┘  └──┘  └──
               ┌──┐  ┌──┐  ┌──┐                    ┌──┐  ┌──┐  ┌──┐
          B ───┘  └──┘  └──┘  └─              B ─┘  └──┘  └──┘  └───
          (A leads B)                          (B leads A)
```

- **Speed** is how fast the edges come.
- **Direction** is which signal changes first.
- **Resolution.** Each period of A has 4 edges in total: A rising, B rising, A falling,
  B falling. Counting all of them gives **4 counts per encoder pulse**, called "×4"
  decoding.

**Counts per wheel revolution.** The encoder is on the motor shaft, before the gearbox, so
the count per wheel revolution includes the gear ratio:

```
counts per wheel revolution = PPR × 4 × gear ratio
```

PPR is the encoder's pulses per motor revolution, typically 11 on these motors. The gear
ratio varies between versions of the 520 motor (for example 1:30 or 1:90), and the vendor
diagram doesn't state it. Example: 11 × 4 × 30 = 1320 counts per wheel revolution. **Measure
the real value** (§8).

---

## 5. Decoding in the firmware

**Inputs.** PC0–PC3 are inputs with pull-ups. Each of the four lines (A and B of each
encoder) has an external interrupt (EXTI 0–3) that fires on **both rising and falling
edges**, at the highest priority (0), so edges are handled before anything else.

**On every edge** (`Hardware/encoder.c`):
1. The handler reads the level of the line that changed.
2. It compares it with the stored level of the other line.
3. It adds or subtracts one count:

| Edge on | `A == B` after the edge | Count |
|---|---|---|
| A | yes | −1 |
| A | no | +1 |
| B | yes | +1 |
| B | no | −1 |

This is the standard ×4 quadrature rule: every edge counts, and its direction follows from
the two levels. At start-up the firmware reads the real pin levels first, so the very first
edge is decoded correctly.

**Totals.**
- The result goes into a signed 32-bit total per wheel, `Total`, which is never reset while
  the board runs.
- 2³¹ counts is about 1.6 million wheel revolutions at 1320 counts per revolution, so it
  will never overflow in practice.
- A 32-bit read is atomic on the Cortex-M3, so the main loop can read the total while
  interrupts keep counting.

**Sign normalisation** (`User/protocol.h`).
- The left motor's raw count runs backwards when the robot drives forward. That's how the
  vendor's Ackermann program counts too, presumably because the two motors face opposite
  ways. The firmware multiplies it by −1 (`PROTOCOL_ENC_LEFT_SIGN`); the right side by +1.
- After that, **forward is positive on both wheels**. The hardware test on 25 Oct 2026
  confirmed it.
- If a wheel ever counts the wrong way, flip its sign there; no rewiring is needed.

**Leftover vendor code.** The vendor's averaging code (`PulseCount`, `Avg_PulseCount`,
`Count_FIFO`) is still in `encoder.c`, but telemetry no longer uses it. Firmware v1 sent
that average, which nothing updated, so its `#T` was always `0 0`.

---

## 6. The `#T` telemetry line

| Field | Type | Meaning |
|---|---|---|
| `#T` | text | line type: encoder telemetry |
| `ms` | unsigned 32-bit | board clock: milliseconds since start-up (1 kHz timer TIM6); wraps after ~49.7 days |
| `left` | signed 32-bit | left rear wheel: cumulative count, forward positive |
| `right` | signed 32-bit | right rear wheel: cumulative count, forward positive |

**Rate and timing.**
- The line goes out **every 20 ms (50 Hz)** from the main loop. Sending `@Q` forces one at
  once.
- The timestamp is taken when the line is built, not when the edges happen. Counts and
  timestamp belong to the same moment to within about 1 ms.
- The line reaches the Pi a few milliseconds later (USB). Use the **board clock** for time
  differences between lines, not the Pi's arrival time.

**Size.** About 17–35 characters per line, so about 1–1.8 KB/s: under a sixth of the
115200-baud link.

**After a restart.** If the board restarts, `ms` and both counts start again from 0. A drop
in `ms` therefore marks a restart. The Python client counts these in `board.reboots`, and
any calculation across a restart must start over.

---

## 7. From counts to distance, speed and odometry

Between two lines *i−1* and *i*:

```
Δt   = (ms_i − ms_{i−1}) / 1000                     seconds (normally 0.020)
Δn_L = left_i − left_{i−1}                          counts
Δn_R = right_i − right_{i−1}

Δs_L = Δn_L / counts_per_metre                      metres per wheel
Δs_R = Δn_R / counts_per_metre
v_L  = Δs_L / Δt                                    m/s
v_R  = Δs_R / Δt

Δs   = (Δs_L + Δs_R) / 2                            distance of the rear-axle centre
Δθ   = (Δs_R − Δs_L) / W                            heading change (W = rear track width)
```

The heading formula works because the **rear wheels don't steer**: in a turn the outer
wheel rolls further than the inner one. Adding up `Δs` along the heading gives wheel
odometry: `x += Δs·cos θ`, `y += Δs·sin θ`, `θ += Δθ`. ROADMAP.md §3.2 explains how this is
combined with a gyro to reduce drift.

**Smoothing.** At low speed only a few counts arrive per 20 ms, so a single-interval speed
is jumpy. Average over several lines (e.g. 100 ms) or filter it.

**Worked example.** Two lines 20 ms apart, with the left count rising by 12:
- **Count rate:** 12 / 0.020 = 600 counts/s.
- **Speed:** with an assumed 2000 counts per metre, 600 / 2000 = 0.30 m/s.

Python, with the client from `pi_examples/`:

```python
import time
from niumo_board import NiumoBoard

COUNTS_PER_METRE = 2000.0       # MEASURE (section 8)

with NiumoBoard() as board:
    prev = board.wait_for("encoders")
    while True:
        time.sleep(0.1)
        cur = board.encoders
        dt = (cur.ms - prev.ms) / 1000
        if dt > 0:
            v_l = (cur.left - prev.left) / COUNTS_PER_METRE / dt
            v_r = (cur.right - prev.right) / COUNTS_PER_METRE / dt
            print(f"left {v_l:+.3f} m/s   right {v_r:+.3f} m/s")
        prev = cur
```

---

## 8. Calibrating counts per revolution and per metre

**Counts per wheel revolution** (with the wheels off the ground):
1. Put a chalk mark on the tyre and a reference mark on the chassis.
2. Read the count: start a terminal on `/dev/robot_mcu` (§11), or use the Python client.
3. Turn the wheel by hand **exactly 10 revolutions** forward and read the count again.
   10 turns averages out the error of lining up the marks.
4. Counts per revolution = change / 10. Do both wheels; they should agree within about 1 %.
5. Divide by 4 × PPR to get the gear ratio, if you need it.

**Counts per metre** (on the floor; this is what odometry needs):
- **Fast, but optimistic:** counts per revolution / (π × wheel diameter). Tyres compress
  under load, so the real rolling diameter is smaller than the measured one.
- **Better:** drive straight a measured distance (3–5 m, tape measure), then divide the
  average count change by the distance. `calibrate_counts_per_meter()` in
  `pi_examples/niumo_examples.py` does this. Repeat 3 times and average.

**Track width `W`** (for the heading formula):
- Measure it between the centres of the rear tyres' contact patches.
- Then refine it with the UMBmark square test [4]: drive a square clockwise and
  counter-clockwise and correct from the error at the end.

---

## 9. What is not reported

| Not sent | Why / how to get it |
|---|---|
| Wheel speed | Not computed on the board; derive it from counts and time (§7). The roadmap adds measured speeds once the board controls speed. |
| Motor current, temperature, stall | The drivers have no feedback connection to the microcontroller. A stall shows up only as "power applied, counts not changing". |
| Confirmation that a wheel turned | `@M` gets `#OK`, which only means the command was accepted. The encoder counts are the proof of motion. |
| The applied power | Not echoed; the Pi knows what it sent. A watchdog stop isn't reported either: the counts simply stop changing. |
| Encoder faults (unplugged, broken wire) | Inputs with pull-ups stay quiet, so the count just doesn't change. `hw_test.py` detects this: "no counts" while the wheel turns. |

---

## 10. Accuracy and error sources

| Source | Effect | Size / handling |
|---|---|---|
| Resolution | Smallest measurable movement: 1 count | At ~2000 counts/m that's 0.5 mm. Negligible for odometry. |
| Gearbox backlash | The encoder is before the gearbox, so the wheel can move a little without counts, mainly when reversing | Small; matters for precise reversing |
| Wheel slip and skid | Counts without motion, or motion without counts | The largest error, especially in turns and on smooth floors. Fuse with a gyro or SLAM (ROADMAP.md §6.1). |
| Diameter and track errors | Systematic distance and heading drift | Calibration (§8); UMBmark [4] |
| Uneven floor | Path longer than the straight-line distance | Small indoors |
| Missed edges | Lost counts at extreme edge rates or with electrical noise | Each edge is one short interrupt at top priority. At a few thousand edges per second per motor this is far below the processor's limit. Pull-ups and short cables keep noise out. |
| Time base | Speed error from timing | The board clock comes from the crystal-controlled 72 MHz; timing error is negligible compared with slip |

Odometry error grows with distance driven. That's why it is only a short-term estimate and
gets corrected by SLAM or localisation [1], [2].

---

## 11. Checking it yourself

**Watch the raw stream** (stop it with Ctrl+]):

```bash
python3 -m serial.tools.miniterm /dev/robot_mcu 115200 --eol CRLF --echo
```

Turn a wheel by hand. Its number changes: up when the wheel turns forward, down when it
turns backward.

**Without miniterm:** `stty` sets the speed, which a newly connected port needs.

```bash
stty -F /dev/robot_mcu 115200 raw -echo
```

```bash
timeout 3 cat /dev/robot_mcu
```

**Full automatic check** (rear wheels off the ground) runs each motor forward and backward
and checks the encoder sign and count:

```bash
python3 hw_test.py
```

---

## 12. References

Check editions, page numbers and access dates against your copies before citing.

[1] R. Siegwart, I. R. Nourbakhsh, and D. Scaramuzza, *Introduction to Autonomous Mobile
Robots*, 2nd ed. Cambridge, MA, USA: MIT Press, 2011. Wheel encoders, wheeled-robot
kinematics, odometry error.

[2] J. Borenstein, H. R. Everett, and L. Feng, *Where am I? Sensors and Methods for Mobile
Robot Positioning*. University of Michigan, 1996. Optical and magnetic encoders,
quadrature, odometry.

[3] S. Thrun, W. Burgard, and D. Fox, *Probabilistic Robotics*. Cambridge, MA, USA: MIT Press,
2005. Odometry motion model and its uncertainty.

[4] J. Borenstein and L. Feng, "Measurement and correction of systematic odometry errors in
mobile robots," *IEEE Transactions on Robotics and Automation*, vol. 12, no. 6,
pp. 869–880, 1996. UMBmark.

[5] STMicroelectronics, *RM0008 Reference manual: STM32F101xx–STM32F107xx*. Sections on
external interrupts (EXTI), GPIO inputs with pull-ups, general-purpose timer PWM (TIM2).

[6] Texas Instruments, *DRV8870 3.6-A Brushed DC Motor Driver (PWM Control)*, datasheet. H-bridge
input logic (drive, coast, brake). Check the driver part number on your board first.

[7] Vendor documentation: *Interface diagram of 520 encoder motor matched with Ackermann
chassis* (motor connector), board manual and schematic (headers M1/M2, EC1/EC2).

[8] Project sources: `firmware/Hardware/encoder.c` (decoding), `Hardware/Motor.c`
(PWM), `User/protocol.c` and `protocol.h` (`#T`, sign normalisation, motor watchdog),
`pi_examples/niumo_board.py` (client).
