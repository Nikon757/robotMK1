# Camera pan servo calibration

How the camera pan of the RobotMK1 robot was calibrated, 25–30 October 2026: why it was
needed, the theory, the tools and commands, the measurements, the results, how accurate
they are, and the literature. Written to be reused in the thesis. The wider development
history is in [DEVELOPMENT.md](DEVELOPMENT.md), problems 19–21.

Contents

1. [Purpose](#1-purpose)
2. [The system](#2-the-system)
3. [Background: position control of an RC servo](#3-background-position-control-of-an-rc-servo)
4. [Initial state and diagnosis](#4-initial-state-and-diagnosis)
5. [Method](#5-method)
6. [Procedure and commands](#6-procedure-and-commands)
7. [Measurements](#7-measurements)
8. [Result](#8-result)
9. [Verification](#9-verification)
10. [Accuracy and limitations](#10-accuracy-and-limitations)
11. [Suggested quantitative extension](#11-suggested-quantitative-extension)
12. [References](#12-references)

---

## 1. Purpose

The OAK-D Lite depth camera sits on a servo-driven pan platform: a Yahboom pan-tilt bracket
made for the Astra Pro camera, with a custom mount for the OAK-D Lite. The calibration had
three goals:

1. **Command angle = physical angle.** One command degree should turn the camera about one
   degree, so the Pi can point the camera predictably.
2. **A true "straight ahead".** The mapping software centres the camera (`@R`) before mapping,
   and its camera–robot transform (the extrinsic calibration) assumes the camera then looks
   along the robot's forward axis. An error here becomes a heading error of the whole camera
   point cloud relative to the LiDAR and the robot body [4], [8].
3. **Safe limits.** The firmware must never drive the camera into the mount, or pull its USB
   cable tight. The limits are enforced on the control board, so even a faulty program on
   the Pi can't exceed them.

---

## 2. The system

| Part | Detail |
|---|---|
| Servo | Hobby (RC) position servo on the Yahboom pan platform, plugged into the board's servo header PB13. Model not recorded; see [13]. |
| Controller | STM32F103RCT6 on the Niumo V5 board, running the RobotMapper v2 firmware |
| PWM generation | Hardware timer TIM1, complementary output CH1N on PB13 [1], [2]. Prescaler 72 gives a 1 MHz count (1 µs resolution); period 20 000 counts gives a 50 Hz frame. The compare register CCR1 holds the pulse width in µs, preloaded so a new width starts at the next frame. |
| Host | Raspberry Pi 5, connected over USB serial (CH340, 115200 baud). The board's port appears as `/dev/robot_mcu` via a udev rule. |
| Command | `@S2 <angle>`, angle 0..180 (integer degrees); `@R` centres all servos |

**Command chain:**

```
Pi:  "@S2 <angle>\r\n"  ──USB serial──►  protocol.c: parse, range check 0..180
     ──► Servo_SetAngle(SERVO_PAN, angle)                          (Hardware/Servo.c)
         pulse = center_us + round((angle − 90) × 2000 / 180)
         pulse = clamp(pulse, min_us, max_us)
     ──► TIM1->CCR1 = pulse   ──►  50 Hz pulse train on PB13  ──►  servo
```

The per-servo constants live in one table in `Hardware/Servo.c`:

```c
static const struct SERVO_LIMIT servo_limit[SERVO_COUNT] =
{
	{1200, 1500, 1900},		// SERVO_STEER, PB14
	{ 500, 1167, 1856},		// SERVO_PAN,   PB13   <- result of this calibration
	{ 800, 1500, 1850},		// SERVO_TILT,  PB15
};
```

---

## 3. Background: position control of an RC servo

**Pulse width sets the position.** An RC servo is a closed-loop position actuator: a DC motor
and gearbox, with a potentiometer on the output shaft as feedback. The controller sends one
pulse every 20 ms (50 Hz). The servo's electronics compare the pulse **width** with the
potentiometer position and drive the motor until they match. Only the width carries
information; the frame rate just has to stay in the servo's accepted range.

**The pulse-to-angle relation is approximately linear.**
- **Common convention:** about 1500 µs is the middle of travel. Roughly 1000–2000 µs covers
  the "standard" range, and many servos accept about 500–2500 µs for about 180° [13].
- **Nominal scale in this work:** 180° over 2000 µs, i.e.

  `k = 180° / 2000 µs = 0.09 °/µs`   (equivalently 11.1 µs per degree).

- **Real servos differ:** individual units differ in scale and in where their own end stops
  are. This is why the scale is only *nominal* here (see §10–11).

**Mechanical zero vs electrical zero.** The servo arm (horn) is pressed onto a splined output
shaft, so it can only be fitted in discrete steps of 360° / *N*, where *N* is the number of
spline teeth. For example, 25 teeth gives 14.4° per tooth and 24 teeth gives 15°. A horn, or
a mount on it, fitted a tooth or two off leaves the camera pointing off-axis at 1500 µs.
There are two ways to correct it:
- **Mechanically:** re-seat the horn. This keeps the range symmetric.
- **In firmware:** move the electrical centre (`center_us`). This is what was done here.

**End stops.** Beyond its mechanical travel the servo stalls and draws a large current. On
this robot, a USB reconnection once coincided with the steering servo moving (DEVELOPMENT.md,
problem 16), so large servo currents are worth avoiding. The limits must stay inside both the
servo's own travel and the mount's clearance.

---

## 4. Initial state and diagnosis

**Symptom.** A command meant to turn the camera 90° turned it only 10–20°.

**Cause: firmware, not the mount.** Two inherited settings reduced the travel:

| Setting | Value (inherited from the vendor's example) | Effect |
|---|---|---|
| Angle → pulse mapping | 0..180 → 1000..2000 µs | Only half of a 180° servo's travel: one command degree ≈ 0.5° |
| Pan clamp | 1300..2200 µs (made for the Astra Pro bracket) | Below the centre, the pulse could not go under 1300 µs |

**Worked example.** `@S2 0` gives 1000 µs, clamped to 1300 µs. That is 200 µs from centre,
and 200 µs × 0.09 °/µs ≈ **18°**, which matches the observed 10–20°.

**Changes made before calibrating (25 Oct):**
1. The mapping became 0..180 → 500..2500 µs (`k` ≈ 0.09 °/µs, about one degree per command
   degree).
2. The pan limits were opened to 500..2500 µs, so the sweep could find the real limits.

Steering kept its old limits, which protect the linkage.

---

## 5. Method

The calibration is an **operator-observed incremental sweep**, in three parts:

1. **Centre (straight-ahead) determination.** The camera is moved until it visibly points
   along the robot's forward axis. The pulse width there becomes `center_us`.
2. **Limit search on each side.** From the centre, the camera is turned in fixed command
   steps. After each step the operator inspects the mount and answers:
   - **Enter:** continue;
   - **`s`:** this is as far as it should go;
   - **`b`:** too far, then step back.

   "Too far" means any of: the camera or mount touches something, the USB cable gets tight,
   the servo buzzes (it's straining), or the camera stops moving while the command still
   changes (the servo has reached its own end stop).
3. **Safety margin.** Where the operator stopped the sweep, the limit is pulled back by a
   margin (3 command degrees ≈ 33 µs). This leaves room for overshoot, backlash and cable
   flex. Where the sweep reached the end of the command range without a problem, that end
   is used as it is.

**Design choices in the tool (`pi_examples/pan_sweep.py`):**

| Choice | Value | Reason |
|---|---|---|
| Step size | 5 command degrees (`--step`) | Coarse enough for a quick sweep, fine enough to stop before contact |
| Motion speed | 30 °/s, sent as 1° commands every 1/30 s | Slow approach, so the operator can react before a collision |
| Settling time | 0.3 s after each step | The servo reaches the new position before the question is asked |
| Keyboard flush | Keys pressed during a move are discarded (`termios.tcflush`) | Every step needs a deliberate answer after looking. In the first run, extra key presses had skipped steps. |
| Restart detection | The board's millisecond counter going backwards marks a restart | A servo stall can brown out the board; the last safe angle is then taken as the limit |
| Port selection | `/dev/robot_mcu`, else the board's USB ID (1a86:7523) | The LiDAR and the board can swap `ttyUSB` numbers (DEVELOPMENT.md, problem 16) |
| Output | Range in command degrees and µs, plus the ready-made `servo_limit[]` line | Results go straight into the firmware without manual conversion |

**Conversion used by the tool.** It is the same as the firmware's, so the printed µs are
exactly what the board sends:

```
pulse(angle) = center_us + sign(o) · floor((|o| + 90) / 180),   o = (angle − 90) · 2000
```

This is integer rounding half away from zero. With `center_us` = 1500 µs it gives the same
pulse as the earlier formula `500 + round(angle · 2000 / 180)` for all 181 angles; this was
checked by computing both for every angle.

---

## 6. Procedure and commands

All commands below are the ones actually used. The Windows paths are the folders the files
were in at the time; since 14 Nov 2026 they live in the `robotMK1` repository
(`pi_examples/`, `firmware/`).

### 6.1 Build and flash the firmware (development PC, Windows)

```bash
C:\Keil_v5\UV4\UV4.exe -r project.uvprojx -j0 -o build.log
```

Every build had to finish with 0 errors and 0 warnings. Flashing was done with an ST-Link
over SWD, using Keil µVision: F7 to build, F8 to download, with the board powered from its
battery.

### 6.2 Copy the tool to the Pi

```bash
scp "C:\Users\Nikon\Desktop\отчеты\робот\rpv2\pi_examples\pan_sweep.py" ivan@RobotMK1.local:~/Downloads/pi_examples/
```

`pan_sweep.py` needs `niumo_board.py` and `hw_test.py` in the same folder, plus pyserial
[12].

### 6.3 Quick check of which firmware is running

```bash
stty -F /dev/robot_mcu 115200 raw -echo
printf '@S2 90\r\n' > /dev/robot_mcu; sleep 1; printf '@S2 30\r\n' > /dev/robot_mcu; sleep 2; printf '@S2 90\r\n' > /dev/robot_mcu
```

At `30`, the old mapping turns the camera about 18°; the new one turns it about 60°. The
`stty` line is needed because a newly connected serial port opens at 9600 baud.

### 6.4 Sweep and centre

```bash
cd ~/Downloads/pi_examples && python3 pan_sweep.py
```

The tool first asks for the centre:
- **Enter:** accept;
- **`+` / `-`:** nudge by 1°;
- **`+10` / `-20`:** nudge by that many degrees;
- **a number such as `70`:** go to that angle.

It then sweeps side 1 (towards 0) and side 2 (towards 180), and asks which way side 1 turned.

To measure the centre alone:

```bash
python3 pan_sweep.py --trim-only
```

Once a centre is set in the firmware, pass it with `--center-us 1167`, so that the µs values
the tool prints match what the board sends.

### 6.5 Write the result into the firmware

1. Edit `servo_limit[]` in `Hardware/Servo.c`.
2. Rebuild (§6.1) and flash.

### 6.6 Verify

```bash
stty -F /dev/robot_mcu 115200 raw -echo
printf '@R\r\n' > /dev/robot_mcu; sleep 2; printf '@S2 30\r\n' > /dev/robot_mcu; sleep 2; printf '@S2 152\r\n' > /dev/robot_mcu; sleep 2; printf '@S2 90\r\n' > /dev/robot_mcu
```

---

## 7. Measurements

| Run | Date | Firmware | What was done | Observation | Used? |
|---|---|---|---|---|---|
| 1 | 30 Oct | previous build (1000..2000 µs mapping, 1300..2200 µs clamp) | full sweep | Camera barely moved. That's consistent with the old firmware: ~2.5° per 5° step, and a stop at the 1300 µs clamp on side 1. | **No.** Wrong firmware; it showed the need for the check in §6.3 |
| 2 | 30 Oct | 500..2500 µs mapping, limits open, centre 1500 µs | full sweep, no centre trim | Side 1 (→ 0, 500 µs): camera turned **right**. On both sides the operator went to the end of the range (0 and 180) without stopping, so no contact was reported. At 1500 µs the camera pointed noticeably **left** (first estimate 20°). | Yes: direction, and that the servo's whole range clears the mount |
| 3 | 30 Oct | same | full sweep, observing the centre | Straight ahead at command **60 = 1167 µs**. Side 1 to 0 (500 µs) clear. Side 2 stopped by the operator at **125 = 1889 µs** (`s`, "as far as it should go"). | Yes: centre and left limit |
| 4 | 30 Oct | final values (§8) | verification commands §6.6 | All four positions as intended | Acceptance |

In run 3 the tool reported:

```
Measured range: 0..125   centre: 90
Angles below the centre turn the camera right, above it left.
Safe range (with a 3-degree margin where you stopped it): 0..122  =  500..1856 us
```

The centre in that printout is 90, because the trim step was skipped (Enter at 90). The
operator identified the straight-ahead position as command 60 during side 1 of the same run.

---

## 8. Result

**Firmware constants:** `{ 500, 1167, 1856 }` for SERVO_PAN.

| Quantity | Pulse | Command angle | Nominal physical angle from straight ahead (k = 0.09 °/µs) |
|---|---|---|---|
| Straight ahead (`center_us`) | 1167 µs | 90 (also `@R` and start-up) | 0° |
| Right limit (`min_us`) | 500 µs | 30 | 60° right (−667 µs) |
| Left limit (`max_us`) | 1856 µs | 152 | 62° left (+689 µs) |
| Operator's left stop, before margin | 1889 µs | — | 65° left |
| Electrical offset of the mount | 1167 − 1500 = −333 µs | — | ≈ 30° (about two spline teeth on a 24/25-tooth spline) |

**How to derive the limits:**
- **Right limit:** the servo's own end of travel (500 µs). The camera reaches it without
  contact.
- **Left limit:** the operator's stop at 1889 µs, minus the 3° margin, giving 1856 µs.
- **Command angles:** from the inverse of the mapping:
  `angle = 90 + (pulse − 1167) · 180 / 2000`. That gives 30 for 500 µs and 152 for 1856 µs.
  Commands outside 30..152 are capped at the limits.

**Sign convention.** A lower command turns the camera right, a higher one left. So
`angle − 90` is, nominally, the pan angle in degrees **with left positive**. That matches the
ROS convention of a counter-clockwise (left) positive yaw about an upward z axis [6], which
makes it easy to use in a camera transform.

---

## 9. Verification

After flashing the final build, the commands in §6.6 were run (30 Oct). The camera went:
1. straight ahead (`@R`);
2. about 60° right (`@S2 30`);
3. about 60° left, without the cable tightening (`@S2 152`);
4. back to straight ahead (`@S2 90`).

The operator confirmed that everything worked as intended. The firmware build had 0 errors
and 0 warnings.

### 9.1 Direction-dependent error (found 17 Nov 2026)

**Symptom.** Running script 05 (left, right, centre), the camera didn't return fully to the
centre after the far-right position.

**Tests.** Each test commands a start position, waits 2 s, then sends `@S2 90` (1167 µs):

| Start command | Approach to the centre | Result |
|---|---|---|
| `@S2 30` (far right, 500 µs) | from the right, turning left | stopped **5–10° right** of centre |
| `@S2 60` (30° right, no end stop involved) | from the right, turning left | stopped **5–10° right** of centre |
| `@S2 152` (far left) | from the left, turning right | stopped **exactly** on centre |

**Interpretation.** The error depends on the direction of approach, not on the end stop: the
pan mechanism has 5–10° of **hysteresis**. Possible sources:
- play between the servo arm and the camera mount;
- friction;
- the camera's USB cable pulling against the servo near its target.

The centre (1167 µs) had been found in run 3 while the camera was turning **right**, i.e.
approaching from the left. So it is correct only for that approach direction.

**Correction (software).** `modules/camera/pan.py` makes every movement end in the
calibration direction, turning right. A move that would arrive from the right first goes
past the target to the left (`approach_overshoot_deg`), then comes back. This is the standard
way to make a positioning mechanism with backlash repeatable: always approach the target from
the same side.

**Second iteration (17 Nov 2026).** The first version came back 12° past the target in a slow
sweep, 1 command degree every 33 ms. Script 05 then ended **3–5° off** the position the
camera had before the script. The successful test above differs in two ways: the camera came
back in **one jump** (a single `@S2 90`), and from 62° away rather than 12°. A likely cause is
the servo's deadband under load: each 1° step is too small a position error for the loaded
servo to overcome friction and the cable, so it stops short of the end of a slow sweep. A
12° return also left little margin over the 5–10° of play. The module now reproduces the
successful test: it sweeps slowly to 20° past the target (about twice the play), pauses
0.3 s, then sends the target in a single command. Whether this removes the error must be
checked on the robot. The direction of the remaining error (left or right of centre) tells
the two causes apart: short of the target (left of centre) points to the deadband, past it
(right of centre) to the play not being fully taken up.

**Not affected by this correction:** commands sent straight to the board, such as `@R`, or
the start-up centring in the firmware. They still land 5–10° off when coming from the right.
Before mapping, centre the camera through `CameraPan`, not with `@R`. Doing the same approach
in the firmware is an open item (DEVELOPMENT.md §9). A firmer mount, with less play, would
reduce the hysteresis itself.

---

## 10. Accuracy and limitations

These points should be stated honestly in the thesis:

1. **Centre resolution.** In run 3, the centre was read off a sweep with 5° steps. The true
   straight-ahead position therefore lies within about **±2.5 command degrees (±28 µs) of
   1167 µs**, plus the operator's visual judgement. The visual check in §9 found no visible
   offset. For a tighter value, run `pan_sweep.py --trim-only --center-us 1167`, which has
   1° steps.
2. **The physical angles are nominal.** The 60°/62° figures assume the nominal scale
   k = 0.09 °/µs. The actual scale of this servo was not measured with an instrument. Real
   servos can differ by several percent, and the relation can be slightly non-linear near the
   ends.
3. **Observation was visual and qualitative.** "Straight ahead" and "far enough" were the
   operator's judgement. There was no protractor, target or image measurement. This is
   adequate for setting safe limits and a working centre, but not for a precise angle model.
4. **Hysteresis: measured roughly, corrected in software.** Approaching from the right leaves
   the camera 5–10° short; approaching from the left is exact (§9.1). The pan module now
   always approaches from the left. Its exact size and repeatability were not measured;
   §11 describes how.
5. **Not characterised:**
   - the servo's deadband and repeatability;
   - temperature and load effects;
   - behaviour while the robot moves.
6. **Pulse timing.** The pulse width itself is accurate to the timer resolution (1 µs), with
   the crystal-based clock error, which is negligible compared with the items above [1].
7. **Reason for the left stop.** The operator chose the left limit at 1889 µs ("as far as it
   should go"), although run 2 had found no contact up to 2500 µs. Record the reason (for
   example, cable slack or field of view) in the thesis.

The measurement uncertainty can be framed according to the GUM [10]:
- **Type B (resolution):** step size / √12 ≈ 1.4° for 5° steps.
- **Combined with:** the operator's alignment judgement.

---

## 11. Suggested quantitative extension

If the thesis needs measured rather than nominal angles, the camera can measure its own pan
angle. This uses the pinhole camera model and the intrinsic calibration [9], which the OAK-D
Lite stores from the factory [11].

1. **Set up a target.** Put a narrow vertical target (a pole, or the edge of a checkerboard)
   straight ahead at a known distance, and keep the robot still.
2. **Record the target's image position.** For a set of commands (for example every 10°
   between 30 and 152), record the target's pixel column *u* in the colour image. Approach
   each angle from both sides, to see hysteresis.
3. **Convert pixels to an angle.** The target's bearing in the camera frame is
   `α = atan((u − c_x) / f_x)`, where `f_x` and `c_x` are the camera's intrinsics. The camera's
   pan is then `θ = −α`, for a target that stays straight ahead.
4. **Fit a linear model.** Fit `θ = k · (pulse − p₀)` by least squares:
   - **`p₀`** is the measured centre (compare it with 1167 µs);
   - **`k`** is the true scale (compare it with 0.09 °/µs);
   - **the residuals** show non-linearity, and the two approach directions show hysteresis.
5. **Report the results** with uncertainties [10]:
   - `k`, `p₀`, the residual RMS and the hysteresis;
   - optionally a corrected angle mapping if `k` differs much from nominal.

Camera-to-LiDAR extrinsic calibration [8] would then build on a known, repeatable centre
position.

---

## 12. References

Check the edition, revision and access date of each item against the copy you use, and
convert to your required style. For Russian theses that is usually GOST R 7.0.100-2018 for
the bibliographic description and GOST R 7.0.5-2008 for references.

[1] STMicroelectronics, *RM0008 Reference manual: STM32F101xx, STM32F102xx, STM32F103xx,
STM32F105xx and STM32F107xx advanced Arm-based 32-bit MCUs*. Sections on the advanced-control
timers (TIM1/TIM8): PWM mode, complementary outputs, main output enable; and on the clock tree.

[2] STMicroelectronics, *STM32F103xC, STM32F103xD, STM32F103xE high-density performance line
Arm-based 32-bit MCU*, datasheet DS5792. Pin alternate functions: TIM1_CH1N on PB13.

[3] YourFun, *Main Control Board Instruction Manual* (Niumo V5) and board schematic
*牛魔主控板原理图 V1.1* (2023). Servo header layout and pin assignment. Vendor documentation
supplied with the board.

[4] R. Siegwart, I. R. Nourbakhsh, and D. Scaramuzza, *Introduction to Autonomous Mobile
Robots*, 2nd ed. Cambridge, MA, USA: MIT Press, 2011. Sensors, actuators, and sensor
placement on mobile robots.

[5] J. J. Craig, *Introduction to Robotics: Mechanics and Control*, 3rd ed. Upper Saddle
River, NJ, USA: Pearson Prentice Hall, 2005. Coordinate frames and transforms, for describing
the camera mount.

[6] T. Foote and M. Purvis, "REP 103: Standard Units of Measure and Coordinate Conventions,"
ROS Enhancement Proposals, 2010. Online: https://www.ros.org/reps/rep-0103.html. Sign
convention for yaw.

[7] M. Labbé and F. Michaud, "RTAB-Map as an open-source lidar and visual simultaneous
localization and mapping library for large-scale and long-term online operation," *Journal of
Field Robotics*, vol. 36, no. 2, pp. 416–446, 2019. The mapping system that uses the camera
pose.

[8] Q. Zhang and R. Pless, "Extrinsic calibration of a camera and laser range finder
(improves camera calibration)," in *Proc. IEEE/RSJ Int. Conf. Intelligent Robots and Systems
(IROS)*, 2004, pp. 2301–2306. Camera–LiDAR extrinsics.

[9] Z. Zhang, "A flexible new technique for camera calibration," *IEEE Transactions on Pattern
Analysis and Machine Intelligence*, vol. 22, no. 11, pp. 1330–1334, 2000. Pinhole model and
intrinsic calibration.

[10] JCGM 100:2008, *Evaluation of measurement data – Guide to the expression of uncertainty
in measurement (GUM)*. Joint Committee for Guides in Metrology, 2008. Type A/B uncertainty.

[11] Luxonis, *OAK-D Lite* product documentation and DepthAI API documentation. Online:
https://docs.luxonis.com (access date needed). Factory calibration and intrinsics.

[12] pySerial documentation. Online: https://pyserial.readthedocs.io (access date needed).
Serial access and port enumeration on the Pi.

[13] Datasheet of the pan servo fitted to the Yahboom pan-tilt platform. **To be added:**
read the model from the servo label. It gives the pulse range, travel and spline count that
§3 refers to.

[14] Project sources: `firmware/Hardware/Servo.c` (`servo_limit[]`, `Servo_SetAngle`),
`pi_examples/pan_sweep.py`, and [DEVELOPMENT.md](DEVELOPMENT.md).
