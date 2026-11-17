# Baseline test of the robot, 17 November 2026

This is the record of the first test of RobotMK1 driven by the Raspberry Pi software. It
covers the software written for the test, how the test was run, every problem met, and how
each was handled. It is written to be reused in the thesis.

Earlier work is covered in other documents:
- firmware bring-up: [DEVELOPMENT.md](DEVELOPMENT.md);
- camera pan calibration: [CAMERA_PAN_CALIBRATION.md](CAMERA_PAN_CALIBRATION.md);
- encoders and odometry: [MOTORS_AND_ENCODERS.md](MOTORS_AND_ENCODERS.md).

Times are local (UTC+5). They are approximate, taken from the session log and the commit
history.

Contents

1. [Summary](#1-summary)
2. [Starting point](#2-starting-point)
3. [Software prepared for the test](#3-software-prepared-for-the-test)
4. [Procedure](#4-procedure)
5. [Problems and how they were resolved](#5-problems-and-how-they-were-resolved)
6. [State at the end of the day](#6-state-at-the-end-of-the-day)
7. [Lessons learned](#7-lessons-learned)
8. [Next steps](#8-next-steps)
9. [Appendix: commands used](#9-appendix-commands-used)

---

## 1. Summary

**Purpose.** The test checked, one function at a time, that the Pi can drive the robot
through the control board. The functions were:
- camera pan;
- encoder calibration;
- an arc turn;
- a multi-point turn;
- straight driving;
- driving with the camera stream.

It is the baseline that the later ROS 2 mapping and navigation work builds on
([ROADMAP.md](ROADMAP.md)).

**Outcome.**

| Function | Script | Result on 17 Nov | Status |
|---|---|---|---|
| Camera pan: left, right, centre | `05_camera_pan.py` | Runs every time. The return to centre was off by 5–10° at first, then 3–5° | Third version of the centring awaits a test |
| Encoder calibration | `00_calibrate_encoders.py` | Ran: the driving scripts accepted its result | The values are only in the Pi's `config/robot.yaml`, not in Git |
| 45° left arc turn | `01_turn_left_45.py` | Turns left. The front wheels did not end straight | Two fixes, awaiting a test |
| 180° multi-point turn | `02_turn_around.py` | Completed once in 4 legs. The encoders reported 181.5°; the robot really turned about 210°. A later run stopped with a direction error | Fixes and a new calibration, awaiting a test |
| Straight drive, 0.5 m | `03_forward.py` | No result reported | Still to test |
| Drive while streaming the camera | `04_drive_and_stream.py` | Did not start: DepthAI not found | Fixed in `a6da3c0`, awaiting a test |

**Main findings.**
1. **The steering direction was the reverse of the default guess.** Command 126 turns
   the wheels right. The scripts' direction check caught it within about 8 cm of travel and
   stopped the robot.
2. **The camera pan has 5–10° of direction-dependent hysteresis.** Approached from the left
   it stops on target; from the right it stops short. A slow, small-step final approach also
   leaves an error of 3–5°.
3. **Encoder-only heading underestimates sharp turns by about 16 %.** The effective track for
   the heading is about 0.143 m, not the measured 0.165 m.
4. **The steering servo can't fully turn the front wheels while the robot stands still.**
   Steering has to change while the robot rolls.
5. **The software environment matters.** DepthAI exists only in the project's virtual
   environment, and the camera needs its own udev rule.
6. **Calibration values kept only on the Pi are at risk.** The Pi's files were lost once
   during the day, and the local values blocked `git pull`.

---

## 2. Starting point

**Hardware.**
- Raspberry Pi 5 with Raspberry Pi OS Trixie (Python 3.13).
- Niumo V5 control board (STM32F103) with firmware v2.
- Ackermann chassis: servo steering at the front, two rear motors with quadrature encoders.
- OAK-D Lite camera on a pan servo.
- STL-27L LiDAR and Hailo-8, not used in this test.

**Measured geometry (14 Nov).**
- Wheel diameter: 95 mm.
- Tyre width: 35 mm.
- Outer length: 280 mm, giving a wheelbase of about 0.185 m.
- Outer rear width: 200 mm, giving a rear track of about 0.165 m.

**Already verified.**
- **25 Oct, firmware v2 on the robot.**
  - Steady `#T` telemetry every 20 ms.
  - Both motors run both ways.
  - The encoders count the right way.
  - Both servos move.
- **30 Oct, camera pan calibration.** Constants 500 / 1167 / 1856 µs; command 90 points the
  camera straight ahead.

**Decided before the test (14 Nov).**
- The board's own IMU socket stays unused; only the OAK-D Lite's built-in IMU will be used.
- The project lives in the `robotMK1` repository.

**Not yet known on the morning of 17 Nov.**
- Which steering command turns left.
- Whether steering command 90 drives straight.
- The encoder counts per metre.

---

## 3. Software prepared for the test

The software was written on 17 Nov from 17:06 and committed by the operator at 17:31 as
`669653b` ("Первые тесты робота"). The same commit removed the old `robot_mapping/` folder.
The operator had reorganized the project into the following layout:

| Path | Contents |
|---|---|
| `scripts/` | One file per test, numbered so the menu finds it |
| `main.py` | Console menu only: lists the numbered scripts and runs the chosen one with its options (`3 --distance 1.0`, `h 3` for help) |
| `modules/engines/` | `board.py` (serial client), `config.py` (settings), `motion.py` (odometry and driving) |
| `modules/camera/` | `pan.py` (pan servo), `stream.py` (OAK-D MJPEG stream) |
| `config/robot.yaml` | Every robot setting: geometry, calibration, steering and camera limits, motor power |
| `config/99-robot-usb.rules` | udev rules: fixed names `/dev/robot_mcu` and `/dev/robot_lidar`, and access to the OAK-D |
| `setup_venv.sh`, `requirements.txt`, `pyproject.toml` | Virtual environment `.venv` with DepthAI, OpenCV, Jupyter Lab |
| `docs/INSTALL_PI.md` | Installing everything on Raspberry Pi OS |

### 3.1 The test scripts

| No. | Script | What it does |
|---|---|---|
| 00 | `00_calibrate_encoders.py` | **Default mode:** with the wheels lifted, the operator turns each rear wheel 5 times by hand; the script writes `counts_per_wheel_rev` and `counts_per_metre`. **`--drive`:** refines the result from a measured drive on the floor. |
| 01 | `01_turn_left_45.py` | Drives forward on an arc until the heading has changed by 45° to the left |
| 02 | `02_turn_around.py` | Turns 180° like a car in a narrow street: forward with full left lock, back with full right lock, legs of up to 0.30 m |
| 03 | `03_forward.py` | Drives 0.5 m straight, holding its heading from the encoders and slowing for the last 10 cm |
| 04 | `04_drive_and_stream.py` | Starts the camera stream (MJPEG over HTTP, port 8080, encoded on the camera), prints the address, centres the camera, drives forward |
| 05 | `05_camera_pan.py` | Turns the camera to the far left, the far right, then back to the centre |

### 3.2 Safety mechanisms

Every driving script stops the motors and explains why when one of these happens:

| Event | Detection |
|---|---|
| Ctrl+C | Caught; the motors stop at once |
| The program dies | The board's watchdog stops the motors 0.5 s after the last motor command |
| The wheels stall | No encoder change for 1.5 s while powered |
| The board restarts | The board's millisecond clock in `#T` jumps backwards |
| The robot turns the wrong way | The heading changes against the commanded direction early in a turn |
| The robot drifts while driving straight | Heading off by more than 20° |
| The calibration is missing | Scripts 01–04 refuse to start |

### 3.3 Verification before the robot: a simulator

Every change was first run against a simulated robot (a test harness outside the
repository). The simulator replaces the board's serial port and models:
- **Motion:** a kinematic bicycle model with the robot's wheelbase and track;
- **Motors:** a dead zone of 50 per-mil;
- **Encoders:** 4400 counts per metre;
- **The board's watchdog;**
- **The steering servo:** a finite slew rate;
- **Failure cases on demand:**
  - a stall;
  - swapped steering;
  - one weak motor;
  - side-slip that shrinks the effective track;
  - a steering servo too weak to turn standing tyres;
  - a steering servo that stops working;
- **The camera:** a fake DepthAI that serves JPEG frames, so the HTTP stream can be checked.

At 17:28, 15 scenarios passed; by the end of the day there were 18.

The simulator checks the logic and the order of commands. It does not model:
- play or deadband in the servos;
- tyre scrub;
- the real effect of the cable on the camera.

Those effects only showed up on the robot (section 5).

---

## 4. Procedure

### 4.1 Planned order

[INSTALL_PI.md §5](INSTALL_PI.md) gives this order, from the least to the most risky step:
1. **`05` camera pan.** The wheels don't move; it confirms that the board answers.
2. **`00` calibration.** Wheels lifted; scripts 01–04 need its result.
3. **`03` forward 0.5 m.** Check the distance with a tape measure.
4. **`01` and `02`, the turns.** These confirm the steering direction.
5. **`04` drive and stream.** Uses the camera and the driving together.

### 4.2 How each test was run

- **Running the scripts.** The operator started each script from the menu (`python main.py`)
  on the Pi, with the robot on the floor and free space around it.
- **Observing the robot.** The robot's behaviour was observed by eye. The script's printed
  output was copied into the session.
- **Isolating a problem.** Where a script misbehaved, single board commands were sent
  directly, without the Python code. The serial port was set up first:

  ```bash
  stty -F /dev/robot_mcu 115200 raw -echo
  ```

  Then commands were sent with `printf`, for example
  `printf '@S2 152\r\n' > /dev/robot_mcu`. This separates the firmware and mechanics from
  the script logic.
- **The fix cycle.** Each fix followed the same steps:
  1. a diagnosis from the symptom;
  2. the change on the PC;
  3. a simulator run of all scenarios;
  4. the change transferred to the Pi, by copying files or by `git pull`;
  5. the test repeated on the robot.

### 4.3 Chronology

"T" marks a test on the robot.

| Time | Event |
|---|---|
| 17:06 | Request: the five test scripts, the menu, a virtual environment and the install guide |
| 17:28 | Software complete; 15 simulator scenarios pass |
| 17:31 | Commit `669653b` by the operator |
| 18:13 | **T1** script 05: the camera stays at the far right instead of returning (§5.1) |
| 18:16 | DepthAI permission warning for the OAK-D (§5.2) |
| 18:19 | **T2** script 05 after the first fix: all steps complete, no connection error |
| 18:23 | **T3** direct test `@S2 30` → `@S2 90`: stops 5–10° right of centre |
| 18:26 | **T4, T5** direct tests: from `152` exact; from `60` 5–10° right |
| 18:28 | Fix: every pan move ends turning right, with a 12° overshoot |
| 18:34 | **T6** script 02: "The robot turned the wrong way" after about 8 cm (§5.3) |
| 18:35 | Steering values swapped: `left: 63`, `right: 126` |
| 18:39 | **T7** script 01: turns left; the front wheels stay turned at the end (§5.4) |
| 18:41 | Fix: the wheels are always straightened when a script ends |
| 18:43 | **T8** script 01: straight at the start, slightly left at the end |
| 18:45 | Fix: steering eases off while the robot rolls out of the turn |
| 18:46 | **T9** script 02: 4 legs, "+181.5 degrees", 0.15 m from the start; observed about 210° (§5.5) |
| 18:49 | Fix: effective track for the heading, new calibration `00 --track` |
| 19:05 | **T10** script 02: "turned the wrong way" again (§5.6) |
| 19:08 | Fix: direction judged after the wheels roll; longer settle for large steering swings |
| 19:10 | Commit `8e4e8ab` ("Правки для теста") by the operator |
| 19:15 | The project files on the Pi were lost; virtual environment rebuilt with `setup_venv.sh` (§5.7) |
| 19:16 | **T11** script 04: `AttributeError: module 'depthai' has no attribute 'Pipeline'` (§5.8) |
| 19:18 | First diagnosis: a project folder hiding the library. Partly withdrawn at 19:20 |
| 19:22 | Revised diagnosis: the script ran outside the virtual environment; `main.py` fixed |
| 19:23 | Commit `a6da3c0` |
| 19:26 | **T12** script 04: same error; the traceback line numbers show the Pi still runs the old code |
| 19:27 | `git pull` on the Pi refused: local changes in `config/robot.yaml` (§5.9) |
| 19:32 | **T13** script 05: returns to within 3–5° of the start position (§5.1) |
| 19:35 | Fix: final approach as a single jump from 20° away (not committed yet) |

---

## 5. Problems and how they were resolved

Each problem is described in the same order:
- **Symptom:** what was observed.
- **Investigation:** how the cause was narrowed down.
- **Cause.**
- **Resolution.**
- **Status:** at the end of the day.

### 5.1 The camera did not return to the centre

**Symptom (T1).** Script 05 turned the camera to the far left, then the far right, and left
it there.

**First hypothesis.** The right limit (command 30, 500 µs) is the end of the servo's own
travel. A servo pressed against its end stop draws a high current. That could trip the servo
supply's protection, or cause a USB dropout that loses the later commands.

**First changes.**
- The scripts stop 3° inside each limit (`camera_pan.end_margin_deg: 3`).
- The final position is sent once more after each move.
- A lost board connection now gives a plain message instead of a Python traceback.

**T2 result.** All steps completed with no connection error. So the commands did reach the
board, and the dropout theory was rejected.

**Isolating the mechanics (T3–T5).** Direct commands, each with a 2 s pause at the start
position:

| Start command | Approach to the centre | Result |
|---|---|---|
| `@S2 30` (far right) | from the right | **5–10° right** of centre |
| `@S2 60` (no end stop involved) | from the right | **5–10° right** of centre |
| `@S2 152` (far left) | from the left | **exactly** on centre |

**Cause.** The pan mechanism has 5–10° of **direction-dependent hysteresis**. The end stop is
not involved: the short move from 60 showed the same error. Likely sources:
- play between the servo arm and the camera mount;
- friction;
- the camera's USB cable pulling.

The centre had been calibrated on 30 Oct while approaching from the left, so it is correct
only for that direction.

**Second version (18:28).** Every move in `modules/camera/pan.py` ends turning right, the
calibration direction. A move that would arrive from the right first goes 12° past the target
to the left, then sweeps back slowly in 1° steps. At the start of a script, the camera is
centred the same way.

**T13 result.** The camera came back to within **3–5°** of its start position. The direction
of the error was not recorded. It is also not confirmed that the Pi already had the updated
code (see §5.9).

**Third version (19:35, not yet committed).** It reproduces T4, the only case that landed
exactly:
1. The camera sweeps slowly to 20° left of the target. 20° is about twice the measured play.
2. It pauses for 0.3 s.
3. It covers the rest of the way with **one** command.

The likely cause of the remaining error is the servo's deadband under load. Each 1° step is
too small a position error for the loaded servo to overcome friction, so a slow sweep ends a
little short. The 12° return also left little margin over 5–10° of play.

**Status.** The third version passes the simulator but is untested on the robot. The
direction of any remaining error tells the two causes apart:
- **left of centre (short):** the servo's deadband;
- **right of centre:** the play is not fully taken up.

**Still to do.**
- **Firmware centring.** The board's own centring (`@R` and at start-up) still goes straight
  to 1167 µs, so it can land off-centre. This is an open item for the next firmware update
  (DEVELOPMENT.md §9).
- **Mechanical fix.** Tightening the servo arm and the mount would reduce the play itself.

### 5.2 DepthAI: insufficient permissions for the OAK-D

**Symptom.** DepthAI printed: "Insufficient permissions to communicate with X_LINK_UNBOOTED
device … Make sure udev rules are set".

**Cause.** The udev rule file installed on 14 Nov had only two rules, for the board and the
LiDAR. The rule giving users access to the camera (USB vendor `03e7`) was missing.

**Resolution.**
1. The full rule file is in the repository as `config/99-robot-usb.rules`.
2. It is copied to `/etc/udev/rules.d/`, and the rules are reloaded.
3. The camera is re-plugged, so the new permissions apply.

**Status.** The fix was installed. The camera's start-up has not been seen since, because
script 04 failed earlier for another reason (§5.8).

### 5.3 The steering turned the wrong way

**Symptom (T6).** Script 02 stopped after about 8 cm with "The robot turned the wrong way:
swap steering.left and steering.right".

**Cause.** The steering direction was unknown when the scripts were written, so
`left: 126` was a guess, marked as such. On this robot, command 126 turns the wheels
**right**. The encoders were ruled out: the 25 Oct hardware test had confirmed that the
left encoder belongs to the left wheel.

**Resolution.** `steering.left: 63` and `steering.right: 126`. On the Pi, the two lines were
swapped in place with `sed`, so the encoder calibration in the same file was kept.

**Status.** Verified: T9 turned the right way and completed.

**Note for the thesis.** The direction check worked as designed. A configuration error was
caught after a few centimetres and stopped the robot, instead of producing a full manoeuvre
in the wrong direction.

### 5.4 The front wheels did not end straight after a turn

**Symptom (T7, T8).** After script 01, the wheels stayed turned. After the first fix they
started straight but ended slightly to the left.

**Causes.**
1. **A gap in the code.** The steering was straightened only after a *successful* turn, not
   after an error stop or Ctrl+C.
2. **The steering servo can't turn standing tyres.** The script stopped the robot first and
   straightened the wheels afterwards. A standing tyre grips the floor, so the servo couldn't
   turn the wheels all the way back.

**Resolution.**
- `Robot.close()` straightens the steering however a script ends.
- Turns now ease the steering off while the robot is still rolling, the way a car leaves a
  bend:
  1. full steering until 15° before the target heading;
  2. then steadily less steering;
  3. the stop within 1° of the target.

  On the simulator, the steering is back at about 88 (straight = 90) when the motors stop,
  and the turns end at 44° for 45°, 29° for 30° right, and 179° for 180°.

**Status.** Untested on the robot. Whether command 90 really drives straight
(`steering.center`) is still unmeasured; the test is in §9.

### 5.5 The heading was under-reported in the 180° turn

**Symptom (T9).** Script 02 reported "+181.5 degrees … in 4 legs; ended 0.15 m from the
start". The operator estimated the real turn at about **210°**.

**Cause.** Odometry computes the heading as the difference between the rear wheels' travel
divided by the track. The measured track (0.165 m, tyre centre to tyre centre) assumes each
tyre touches the floor at its centre. With 35 mm-wide tyres, and side-slip in full-lock turns,
the **effective** track is smaller. Going by the estimate:

  0.165 m × 181.5° / 210° ≈ **0.143 m**

That is an under-report of about 16 %.

**Resolution.**
- **New setting.** `chassis.odometry_track_m` is used for the heading. The measured
  `rear_track_m` is kept for sharing power between the wheels in a turn.
- **New calibration mode, `00_calibrate_encoders.py --track`:**
  1. the robot turns 180°;
  2. the operator measures the real angle against a tape line on the floor;
  3. the script saves `odometry_track_m = track used × encoder angle / real angle`.
- **Saving settings.** `save_value()` can now add a key that is missing from an older config
  file. Floats are written in full precision.

On the simulator, with slip matching the robot, the 180° command gave **207°** before the
calibration and **179°** after it. Details are in MOTORS_AND_ENCODERS.md §8.

**Status.** The calibration has not been run on the robot yet.

**Limitation.** The effective track depends on slip, and slip varies with the manoeuvre and
the floor. A heading that doesn't depend on slip needs the camera's gyroscope, fused with
the wheels in the ROS 2 stage.

### 5.6 A false "wrong way" stop in the multi-point turn

**Symptom (T10).** Script 02 again stopped with "turned the wrong way", although the
steering had been swapped and T9 had completed.

**Investigation.** Two explanations fit, and the data cannot tell them apart:
1. **The steering didn't finish its swing.** Between legs the steering swings from full
   left to full right while the robot stands still. By §5.4, the servo may not finish that
   swing, so the next leg starts with the wheels still turned the old way. The direction
   check judged the first 8 cm and took this for swapped steering.
2. **The steering swap had been lost.** The Pi's project files were lost around this time
   (§5.7). The commit restored from GitHub (`669653b`) still contained the old guess,
   `left: 126`. If the files were restored before T10, the steering really was swapped again.

**Resolution (covers explanation 1).**
- **Settle time.** It scales with the size of the swing: 0.6 s for a full left-to-right
  change, instead of 0.3 s.
- **Direction check.** The direction is judged between 5 and 12 cm into each leg, after the
  wheels have finished swinging.
- **Error message.** A wrong turn on a leg after leg 1 is now reported as "Leg N didn't turn
  the way leg 1 did". It no longer suggests swapping the settings.
- **Leg output.** Script 02 prints the heading after each leg.

On the simulator:
- with a weak steering servo, the turn completes at 179° in 4 legs;
- with a servo that stops working after leg 1, the run stops with the leg-specific message;
- with swapped steering, the run is still caught on leg 1.

**Status.** Untested on the robot. After the `git pull` in §5.9, the Pi has `left: 63` from
the repository, so explanation 2 no longer applies.

### 5.7 The project files on the Pi were lost

**Symptom.** The operator reported that the project files on the Pi had been deleted by the
system. The exact cause is unknown.

**Recovery.**
- The project was restored from GitHub.
- `setup_venv.sh` rebuilt the virtual environment. Its log showed DepthAI 2.33.0.0 installed,
  and the final import check passed.
- The PC copy and GitHub were complete: the operator had committed both rounds of changes.

**Risk.** The encoder calibration from script 00 lived only in the Pi's `config/robot.yaml`,
so it could have been lost with the files. It is not known whether the calibration was run
again before the later tests.

### 5.8 Script 04: `module 'depthai' has no attribute 'Pipeline'`

**Symptom (T11).** Script 04 failed when creating the camera pipeline.

**First diagnosis (partly wrong).** The project has a `depthai/` folder at its root. The
scripts put the project root at the *front* of Python's search path, so they could import
`modules`. A non-empty `depthai/` folder would then hide the installed library.
- The search order was changed: the project root now goes at the *end* of the path, so
  installed libraries always win.
- This was still a valid improvement. But the operator found the folder empty, so it did not
  explain this error.

**Real cause.** The script ran with the system Python, not the project's virtual
environment.
- DepthAI is installed only in `.venv`.
- Outside it, Python's only match for `depthai` was the empty `depthai/` folder, which it
  accepts as an empty package. That gives exactly this error.
- The other scripts worked because the system Python also has pyserial and PyYAML.
- `.venv` seemed to be missing only because folders starting with a dot are hidden.

**Resolution (commit `a6da3c0`).**
- `main.py` runs every script with `.venv`'s Python when it exists, whether or not the
  environment was activated.
- The camera module checks the import. If DepthAI is missing, it says that the script is
  running outside the virtual environment and how to fix it.

**T12.** The same error appeared again. The traceback pointed to `stream.py` lines 48 and
101; in `a6da3c0`, those calls are on lines 57 and 109. So the Pi was still running the old
code. Comparing traceback line numbers with the repository is a quick way to check which
version a remote machine runs.

**Status.** Fixed in the repository; not yet confirmed on the robot.

### 5.9 `git pull` refused on the Pi

**Symptom.**

```
Updating 669653b..a6da3c0
error: Your local changes to the following files would be overwritten by merge:
	config/robot.yaml
```

**Cause.** The Pi's `config/robot.yaml` had local edits:
- most likely the encoder calibration;
- possibly the `sed` steering swap from §5.3.

The incoming commits changed the same file (the steering swap, the camera settings). Git
refuses rather than overwrite local values.

**Resolution.**
1. Back up the Pi's file.
2. Restore the repository version and pull.
3. Copy back only the three calibration values with a short script that uses `save_value()`.

The repository already contains the steering swap. For the third pan version, which changes
`robot.yaml` again in a different section, `git stash && git pull && git stash pop` merges
cleanly.

**Status.** The procedure was given to the operator. Its result was not reported before T13.

**Structural fix (open).** Keep the calibration in its own file, for example
`config/calibration.yaml`, written only by the robot and committed from the Pi. Then pulling
never collides with measured values, and the values are backed up in Git.

---

## 6. State at the end of the day

**Verified on the robot.**
- The Pi controls the board over `/dev/robot_mcu`: motors, steering servo and pan servo all
  respond to the scripts.
- The menu and the script structure work on the Pi.
- **Steering direction:** `left: 63`, `right: 126`.
- **Multi-point turn:** script 02 completed 4 legs after the swap, turning the right way.
- **Pan hysteresis:** characterised by direct commands (§5.1 table).
- **Safety checks:** the direction check stopped the robot on a real configuration error.

**Changed, but not yet tested on the robot.**
- The third version of the pan centring (§5.1). Not committed.
- Steering eased off while rolling out of a turn; wheels always straightened at exit (§5.4).
- The effective-track calibration `00 --track` (§5.5).
- The direction check after 5 cm, the longer settle for large swings, and the leg output
  (§5.6).
- `main.py` using `.venv`, and the DepthAI error message (§5.8).
- The OAK-D udev rule (§5.2).

**Open.**
- **Steering:** measure the straight-ahead command (`steering.center`) and the angle at full
  lock (`steering.full_lock_deg`).
- **Script 03:** check the 0.5 m drive against a tape measure.
- **Script 04:** stream while driving.
- **Calibration:** keep the values in Git (§5.9).
- **Firmware:**
  - pan centring from the left;
  - a version number in `#BOOT`;
  - an `@P` heartbeat;
  - removal of the MPU6050 code;
  - 20 kHz motor PWM and a speed loop.

**Repository.**

| Commit | Contents |
|---|---|
| `669653b` | Test software, menu, virtual environment, install guide |
| `8e4e8ab` | Pan approach, steering swap, wheels straightened at exit, effective track |
| `a6da3c0` | Multi-point turn checks, `.venv` in `main.py`, search-path order |
| Not committed | Pan centring v3 (`modules/camera/pan.py`, `approach_overshoot_deg: 20`), this report |

---

## 7. Lessons learned

1. **Plausibility checks pay off on day one.** An unknown sign (the steering direction) was
   guessed. Because a check guarded that guess, the robot stopped after 8 cm instead of
   driving off in the wrong direction.
2. **Mechanics dominate servo accuracy.** The firmware sets pulse widths to 1 µs, yet the
   camera's position varied by 5–10°, depending on how it got there.
   - **Approach from one side.** Ending every move from the same side is the standard remedy
     for backlash.
   - **Small steps can stop short.** Under load, very small steps can stop short, so the
     final approach should be a single move.
3. **Odometry from the wheels alone depends on slip.** A geometric track width that is
   correct on paper under-reported a sharp turn by 16 %. A per-robot calibration fixes it
   only for similar manoeuvres; the gyroscope is the general fix.
4. **Ackermann steering needs rolling wheels.** Steering a standing car-like robot is
   unreliable. Steer while moving, and judge the direction only after the wheels have rolled.
5. **Test on hardware, not only in simulation.** The simulator caught logic and ordering
   errors before every change reached the robot. Each physical effect of the day (hysteresis,
   deadband, tyre scrub, slip) appeared only on the hardware.
6. **Keep the environment part of the program.** A script that silently depends on an
   activated virtual environment will eventually run without it. The entry point now picks
   the environment itself, and the error message names the cause.
7. **Keep the robot's own data apart from shared settings.** Calibration kept only on the Pi
   was put at risk twice in one day: once by lost files, once by a `git pull` conflict.
8. **Check which version runs before diagnosing.** `git log --oneline -1` on the robot, or
   comparing traceback line numbers, avoids debugging code that is no longer current.
9. **Diagnoses can be wrong; record them anyway.** Two first explanations (the end-stop
   current, the shadowing folder) were rejected by follow-up tests. Each test was chosen to
   tell the explanations apart.

---

## 8. Next steps

1. **Update the Pi.**
   1. Commit and push on the PC.
   2. On the Pi: `git stash && git pull && git stash pop`.
   3. Check with `git log --oneline -1`.
2. **Script 05.** Check whether the camera returns to its start position. If not, note which
   side it ends on and roughly how far.
3. **Steering centre.**
   1. Send `@S1 63` → `@S1 90`, then `@S1 126` → `@S1 90`.
   2. Check that the wheels are straight both times.
   3. Adjust `steering.center` if needed.
4. **Calibration.**
   1. Run `00` (wheels lifted) again if the values were lost.
   2. Run `00 --track` with a tape line on the floor.
   3. Commit the values.
5. **Script 02.** Run it, keep the per-leg output, and measure the final angle.
6. **Scripts 01 and 03.** Measure the turn angle and the distance.
7. **Script 04.** Open the stream in a browser while the robot drives.
8. **Calibration file.** Move the calibration into its own file (§5.9).

---

## 9. Appendix: commands used

**Serial port set-up for direct commands** (a new tty starts at 9600 baud):

```bash
stty -F /dev/robot_mcu 115200 raw -echo
```

**Pan hysteresis tests (§5.1):**

```bash
printf '@S2 30\r\n'  > /dev/robot_mcu; sleep 2; printf '@S2 90\r\n' > /dev/robot_mcu
printf '@S2 60\r\n'  > /dev/robot_mcu; sleep 2; printf '@S2 90\r\n' > /dev/robot_mcu
printf '@S2 152\r\n' > /dev/robot_mcu; sleep 2; printf '@S2 90\r\n' > /dev/robot_mcu
```

**Steering centre test (§5.4, still to run):**

```bash
printf '@S1 63\r\n'  > /dev/robot_mcu; sleep 1; printf '@S1 90\r\n' > /dev/robot_mcu
printf '@S1 126\r\n' > /dev/robot_mcu; sleep 1; printf '@S1 90\r\n' > /dev/robot_mcu
```

**Steering swap in place on the Pi (§5.3):**

```bash
sed -i 's/^  left: 126/  left: 63 /; s/^  right: 63 /  right: 126/' config/robot.yaml
```

**udev rules, including the OAK-D (§5.2):**

```bash
sudo cp config/99-robot-usb.rules /etc/udev/rules.d/99-robot-usb.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

**Virtual environment (§5.7, §5.8):**

```bash
bash setup_venv.sh
source .venv/bin/activate && python main.py
```

**Updating the Pi while keeping its calibration (§5.9):**

```bash
cp config/robot.yaml ~/robot.yaml.pi-backup && git checkout -- config/robot.yaml && git pull
.venv/bin/python - <<'EOF'
import yaml
from modules.engines import save_value
old = yaml.safe_load(open("/home/ivan/robot.yaml.pi-backup"))["chassis"]
for key in ("counts_per_wheel_rev", "counts_per_metre", "odometry_track_m"):
    if old.get(key) is not None:
        save_value("chassis", key, old[key])
EOF
```

**Script options used:**

| Command in the menu | Meaning |
|---|---|
| `0` | Encoder calibration by hand, wheels lifted |
| `0 --track` | Effective-track calibration with a 180° turn |
| `1` | 45° left arc |
| `2` | 180° multi-point turn (`--leg 0.30`) |
| `3` | Forward 0.5 m (`--distance`) |
| `4` | Stream on port 8080 and drive forward |
| `5` | Camera pan left, right, centre (`--speed`, `--pause`) |
