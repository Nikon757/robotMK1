# Talking to the Niumo board from a Raspberry Pi

This guide is for the **RobotMapper firmware v2** on the YourFun "Niumo" V5 board
(STM32F103RC). It covers how a Raspberry Pi reaches each module on the board, with a
complete Python client, worked examples, and notes for other languages.

**The key idea:** the Pi never talks to the board's chips directly. Every module (motors,
servos, encoders, IMU) is handled by the firmware, and the Pi reaches all of
them through one USB serial link using short text lines. Anything that can open a serial
port can drive the robot: Python, C++, a shell script, a ROS 2 node.

Contents

1. [Connecting](#1-connecting)
2. [First contact from a terminal](#2-first-contact-from-a-terminal)
3. [The protocol in brief](#3-the-protocol-in-brief)
4. [Module by module](#4-module-by-module)
5. [Python client](#5-python-client)
6. [Worked examples](#6-worked-examples)
7. [Other languages](#7-other-languages)
8. [How this relates to robot_mapping's mcu.py](#8-how-this-relates-to-robot_mappings-mcupy)
9. [Troubleshooting](#9-troubleshooting)

---

## 1. Connecting

| Item | Value |
|---|---|
| Cable | Board's Type-C port → any USB port on the Pi |
| USB bridge | CH340 (USB ID `1a86:7523`) |
| Device | `/dev/ttyUSB0` or `/dev/ttyUSB1` (the LiDAR is the other one), or `/dev/robot_mcu` with the udev rule below |
| Settings | 115200 baud, 8N1, no flow control |

**Stable device name.** The `robot_mapping` project already ships a udev rule
(`robot_mapping/config/99-robot-usb.rules`, installed by `install.sh`) that creates
`/dev/robot_mcu`:

```
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="robot_mcu", MODE="0666"
```

To install it now, together with the LiDAR's rule (same file name as `install.sh` uses, so
running that later simply replaces it):

```bash
sudo tee /etc/udev/rules.d/99-robot-usb.rules > /dev/null <<'EOF'
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="robot_mcu", MODE="0666"
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="robot_lidar", MODE="0666"
EOF
sudo udevadm control --reload-rules && sudo udevadm trigger
```

**Don't rely on `ttyUSB0`/`ttyUSB1`.** The board and the LiDAR swap these numbers whenever
they reconnect. Reading the LiDAR's port at 115200 baud gives a flood of garbage that looks
like a broken board.

Without it, find the board's port with `ls -l /dev/serial/by-id/` (the `1a86` entry; the
LiDAR's CP2102 is the `Silicon_Labs` one). That `by-id` path works as a stable name too.
The number after `ttyUSB` depends on which adapter Linux found first. Add yourself to the
`dialout` group once:

```bash
sudo usermod -aG dialout $USER
```

(log out and back in afterwards).

**Opening the port does not reset the board.** Unlike an Arduino, the CH340's control
lines aren't wired to the STM32, so you can connect and disconnect freely while the
robot runs.

**Power.** Keep the robot battery switched on: the motors and servos run from it, even
when USB alone is enough to light up the board's logic.

---

## 2. First contact from a terminal

pyserial includes a small terminal program:

```bash
python3 -m serial.tools.miniterm /dev/robot_mcu 115200 --eol CRLF --echo
```

Type `@Q` and press Enter. You should see something like:

```
#OK
#T 81230 0 0
#IMU OK 0x68
#I 81231 -14 -6 1004 -112 61 -35 3091
...
```

Then try `@S2 60` (camera pan) and `@R` (centre all servos). Leave with **Ctrl+]**.

`picocom -b 115200 --echo /dev/robot_mcu` works too (Enter sends a carriage return, which
the board accepts; exit with Ctrl+A, Ctrl+X).

---

## 3. The protocol in brief

Everything is plain ASCII, one message per line.

**Pi → board:** a line starting with `@`, ending with `\r`, `\n` or `\r\n`, at most 64
characters after the `@`. Every command gets exactly one reply: `#OK` or `#ERR <code>`.

| Command | Meaning |
|---|---|
| `@M <left> <right>` | Rear motor power, -1000..1000 per side. Must be repeated within 0.5 s. |
| `@H` | Halt the motors (servos keep their position) |
| `@S<n> <angle>` | Servo `n` to `angle` 0..180: 1 = steering, 2 = camera pan, 3 = camera tilt |
| `@R` | Centre all servos |
| `@Q` | Send `#T` and the `#IMU` status line right away |

**Board → Pi:**

| Line | When | Content |
|---|---|---|
| `#BOOT NiumoMapper v2 RST=<cause>` | once, at start-up | why it started: `POWER`, `PIN` (RST button), `IWDG` (watchdog), `SOFT`, `OTHER` |
| `#OK` / `#ERR <code>` | after each command | see the error table below |
| `#T <ms> <encL> <encR>` | 50 Hz | encoder counts since boot |
| `#I <ms> <ax> <ay> <az> <gx> <gy> <gz> <temp>` | 100 Hz, if an IMU answers | accel in mg, gyro in 0.01 °/s, temperature in 0.01 °C |
| `#IMU OK <id>` / `#IMU NONE` / `#IMU LOST` | at start-up, on `@Q`, on change | IMU state |

`<ms>` is the board's own clock: milliseconds since it started (it wraps after ~49.7 days).
All numbers are decimal integers, separated by single spaces.

| Error | Meaning |
|---|---|
| `BADCMD` | unknown command letter |
| `PARSE` | arguments missing or not numbers |
| `RANGE` | a number outside the allowed range (nothing was changed) |
| `EMPTY` | a bare `@` |
| `LONG` | more than 64 characters; the line was dropped |

**Keep reading.** The board sends about 5–6 KB/s, roughly half of what 115200 baud carries.
If the Pi stops reading, the board drops whole lines rather than waiting, so after a pause
you simply resume with fresh data.

---

## 4. Module by module

### 4.1 Drive motors (M1 rear left, M2 rear right)

```
@M 400 400      both rear wheels forward at 40 %
@M -300 -300    reverse at 30 %
@H              stop
```

* **Units.** Per-mil of the PWM duty cycle: 1000 is full power. Values with `|power| <= 50`
  are treated as 0 by the motor driver code, and a heavier robot may need 150–250 before it
  starts rolling. Find your own minimum.
* **Direction.** Positive = forward. If a wheel turns the wrong way for a positive value,
  swap that motor's two power wires (M1 and M2 on the motor's connector, see the vendor's
  motor diagram). The encoder signs don't depend on this.
* **Motor watchdog.** If no `@M` arrives for 0.5 s, the board stops the motors by itself.
  While driving, send `@M` at least every 0.2 s (5 Hz or more). This is deliberate: if your
  program crashes or the cable comes out, the robot stops.
* **Turning (Ackermann).** Steering is done by the front servo. In a turn the outer rear
  wheel has to cover more ground, so give it more power:
  `outer / inner = (R + W/2) / (R - W/2)`, with `R = L / tan(steering angle)`, `L` the
  wheelbase and `W` the rear track width (measure both on your chassis). Driving both sides
  at the same power also works, with some tyre scrub.

### 4.2 Steering and camera servos

```
@S1 90     steering straight
@S1 70     steer one way      (which way depends on the linkage; check once)
@S2 120    camera pan, about 30 degrees from the middle
@S3 100    camera tilt (only if a tilt servo is fitted)
@R         all to centre (90)
```

The angle maps linearly to a pulse width around each servo's centre:
`pulse = centre + (angle − 90) × 2000 / 180` µs. So **90 is always the servo's centre**
(straight ahead, same as `@R`), and one command degree is about one real degree on a standard
180° servo. Each servo also has hard limits in the firmware (`servo_limit[]` in
`Hardware/Servo.c`), so a wrong command can't drive a linkage into its end stop:

| Servo | Header | Limits | Centre | Usable angles |
|---|---|---|---|---|
| 1 steering | PB14 | 1200–1900 µs | 1500 µs | 63–126 |
| 2 camera pan | PB13 | 500–1856 µs | 1167 µs | 30–152 |
| 3 camera tilt | PB15 | 800–1850 µs | 1500 µs | 27–122 |

**Camera pan on this robot** (measured with `pi_examples/pan_sweep.py`, 30 Oct 2026):
- **Straight ahead:** `@S2 90` or `@R` (1167 µs; the servo arm sits about 30° off, so the
  firmware's centre is trimmed).
- **Direction:** angles below 90 turn the camera right, above 90 left.
- **Range:** `@S2 30` is about 60° right (the servo's own end), `@S2 152` about 62° left (the
  limit found in the sweep). Angles beyond that are capped.
- **If the mount changes,** run `python3 pan_sweep.py --trim-only --center-us 1167` to
  re-measure the centre. The full sweep finds new limits, and its microsecond values need the
  same `--center-us`.

Angles outside the usable range are capped silently; angles outside 0..180 get `#ERR RANGE`.

* **The command angle is not the wheel angle.** Find, once, which command makes the wheels
  straight (often 85–95 rather than exactly 90) and the commands for full left and full
  right, then map your steering angle onto that range in your code.
* Servos are not covered by the motor watchdog: they hold the last position. The camera stays
  where you pointed it.
* The `robot_mapping` calibration assumes the camera pan is centred (`@S2 90`).

### 4.3 Wheel encoders (EC1, EC2)

`#T <ms> <encL> <encR>` arrives 50 times per second. The counts:

* start at 0 when the board starts and are never reset while it runs;
* go **up when the wheel turns forward**, on both sides;
* count all four edges of the A and B signals (×4 decoding).

To get distance, take the difference between two readings and divide by
**counts per metre**. In theory that is

```
counts_per_metre = encoder_lines × 4 × gear_ratio / (π × wheel_diameter_m)
```

The kit's 520 gear motors come in several gear ratios and the encoder line count varies, so
check the label on the motor, or simply calibrate it (example 6.3): drive straight, measure
the distance with a tape, divide.

Speed = Δcounts / counts_per_metre / Δt, with Δt from the board's `ms` field (more precise
than host time, which includes USB jitter).

If one side counts down when driving forward, flip `PROTOCOL_ENC_LEFT_SIGN` or
`PROTOCOL_ENC_RIGHT_SIGN` in `User/protocol.h` and reflash.

Rear-wheel encoders alone give a poor heading on an Ackermann chassis: the wheels slip in
turns. Use them for distance and the IMU gyro for heading (example 6.5).

### 4.4 IMU: MPU6050 on the gyroscope header (J10)

> **Not used on RobotMK1.** On 14 Nov 2026 the board IMU was dropped: the robot uses the
> OAK-D Lite's built-in IMU (`/oak/imu/data` via depthai; see ROADMAP.md §4.2). This section
> documents the firmware's MPU6050 support, which will be removed with the next firmware
> change. Examples 6.4, 6.5 and 6.6 use it, so they apply only if a GY-521 is fitted.

Plug the vendor's GY-521 module into J10 (pins VCC, GND, SCL, SDA; the module's other four
pins stay free). The firmware finds it at start-up and reports:

```
#IMU OK 0x68            found (0x68 = genuine MPU6050 ID)
#I 81231 -14 -6 1004 -112 61 -35 3091
```

| Field | Unit | Range | To SI |
|---|---|---|---|
| `ax ay az` | milli-g | ±4000 | × 9.80665 / 1000 → m/s² |
| `gx gy gz` | 0.01 °/s | ±50000 | × π / 18000 → rad/s |
| `temp` | 0.01 °C | | ÷ 100 → °C |

* **Rate and filtering.** 100 lines per second. The chip samples at 1 kHz through its own
  ~42 Hz low-pass filter, which adds about 5 ms of delay.
* **Axes** are the ones printed on the GY-521. How they relate to the robot depends on how
  the module sits in J10 (it may stand upright). At rest the accelerometer reads about
  **+1000 mg on the axis that points up**; the gyro around that axis is the yaw rate.
  `find_up_axis()` in example 6.4 works this out for you.
* **Gyro bias.** A stationary MPU6050 reads a few °/s, not zero, and the offset changes
  with temperature. Average the gyro for a few seconds while the robot stands still and
  subtract that (example 6.4). Redo it when the robot has warmed up or stands still again.
* **No orientation output.** The firmware sends raw rates and accelerations only (the chip's
  "DMP" fusion engine is not used). Do the fusion on the Pi: integrate the gyro for heading
  (example 6.5), or feed `#I` into a filter such as Madgwick, or into `robot_localization`
  under ROS 2.
* **Status.** `#IMU NONE`: nothing answered at start-up; the board keeps trying every second,
  so plugging the module in later works (still, plug it in with the power off).
  `#IMU LOST`: it stopped answering (loose connector); `#I` lines stop until it is back,
  then you get `#IMU OK` again.

The OAK-D Lite camera has its own IMU, which `robot_mapping` already uses. The MPU6050's
advantage is that its samples share the board clock with the encoder counts.

### 4.5 OLED display (J9)

The board's OLED is **not controlled by the Pi**. It shows firmware errors only:

| Screen | Meaning |
|---|---|
| blank | no error |
| `WATCHDOG RESET` | the firmware hung and the hardware watchdog restarted the board; stays until the next power cycle or RST press |

If you want to show Pi information on a screen, connect a separate display to the Pi's own
I2C pins (GPIO2/GPIO3, powered from the Pi's **3.3 V** pin) and drive it from Python, e.g.
with the `luma.oled` library.

### 4.6 Watchdogs and restarts

* **Motor watchdog (0.5 s)**: see 4.1.
* **Hardware watchdog (~0.2 s)**: if the firmware ever hangs, the STM32 restarts. The board
  then sends `#BOOT ... RST=IWDG` and shows `WATCHDOG RESET` on the OLED.
* **Recognising a restart on the Pi.** The `ms` field jumps back to a small number (the
  client below counts this in `board.reboots`). After a restart:
  * encoder counts start again from 0, so take new reference values;
  * the servos are back at centre, so re-send steering and camera positions;
  * the motors are stopped until the next `@M`.

### 4.7 What the Pi can't reach (yet)

These parts of the board are not used by firmware v2, so there is no command for them:

* buzzer (PC12), LED (PD2), buttons KEY1/KEY2 (PB5/PB4);
* motor outputs M3/M4 and encoder headers EC3/EC4;
* the ADC inputs AD8/AD9 (PB0/PB1); there is no battery measurement;
* servo headers PB12 and PC4 (held low);
* the PS2, Bluetooth (UART4), ESP8266 and SPI/NRF headers.

Each would need a small firmware addition and a new command.

---

## 5. Python client

One file, `niumo_board.py` (also in the repository's `pi_examples/` folder, ready to copy
to the Pi), with no dependencies besides pyserial
(`sudo apt install python3-serial`). A background thread reads and parses everything; your
code reads the newest values or registers callbacks. It works with Python 3.9 and later.
It was tested against a simulated board (split lines, garbage, restarts), not yet against
the real one.

```python
#!/usr/bin/env python3
"""Client for the Niumo V5 board running the RobotMapper firmware (v2).

Needs pyserial:  sudo apt install python3-serial   (or: pip install pyserial)

    from niumo_board import NiumoBoard
    with NiumoBoard("/dev/robot_mcu") as board:
        board.wait_for("encoders")
        print(board.encoders, board.imu, board.imu_status)
"""
from __future__ import annotations

import math
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Callable, Optional

import serial

G = 9.80665                       # m/s^2 per g
CDPS_TO_RADS = math.pi / 180 / 100  # 0.01 deg/s -> rad/s


@dataclass
class Encoders:
    t: float     # host time (time.monotonic()) when the line arrived
    ms: int      # board clock, ms since the board started
    left: int    # cumulative counts since boot; driving forward makes both go up
    right: int


@dataclass
class Imu:
    t: float
    ms: int
    ax_mg: int       # acceleration, milli-g
    ay_mg: int
    az_mg: int
    gx_cdps: int     # angular rate, 0.01 deg/s
    gy_cdps: int
    gz_cdps: int
    temp_cdeg: int   # chip temperature, 0.01 deg C

    @property
    def accel(self) -> tuple[float, float, float]:
        """Acceleration in m/s^2 (includes gravity)."""
        return (self.ax_mg * G / 1000, self.ay_mg * G / 1000, self.az_mg * G / 1000)

    @property
    def gyro(self) -> tuple[float, float, float]:
        """Angular rate in rad/s."""
        return (self.gx_cdps * CDPS_TO_RADS, self.gy_cdps * CDPS_TO_RADS, self.gz_cdps * CDPS_TO_RADS)

    @property
    def temp_c(self) -> float:
        return self.temp_cdeg / 100


class NiumoBoard:
    """Talks to the board over USB serial. A background thread parses everything
    the board sends; the newest values are in .encoders and .imu.

    Motors: the board stops them 0.5 s after the last drive() call, so call
    drive() at least every 0.2 s while moving (or use drive_for()).
    """

    def __init__(self, port: str = "/dev/robot_mcu", baud: int = 115200,
                 on_encoders: Optional[Callable[[Encoders], None]] = None,
                 on_imu: Optional[Callable[[Imu], None]] = None):
        self._ser = serial.Serial(port, baud, timeout=0.1)
        self._ser.reset_input_buffer()
        self._write_lock = threading.Lock()

        # Optional callbacks; they run on the reader thread, so keep them short.
        self.on_encoders = on_encoders
        self.on_imu = on_imu

        self.encoders: Optional[Encoders] = None
        self.imu: Optional[Imu] = None
        self.imu_status = "unknown"     # "OK 0x68", "NONE" or "LOST"
        self.boot_line: Optional[str] = None   # last "#BOOT ..." seen, e.g. "... RST=IWDG"
        self.reboots = 0                # board restarts noticed while connected
        self.errors: deque[str] = deque(maxlen=100)   # "#ERR ..." replies and bad lines

        self._last_ms: Optional[int] = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._read_loop, name="niumo-rx", daemon=True)
        self._thread.start()
        self.query()                    # ask for #T and #IMU right away

    # ---- commands -------------------------------------------------------
    def send(self, payload: str) -> None:
        """Send one raw command, e.g. send("S2 90"). Framing is added here."""
        with self._write_lock:
            self._ser.write(f"@{payload}\r\n".encode("ascii"))

    def drive(self, left: int, right: int) -> None:
        """Motor power per side, -1000..1000 (per-mil). |power| <= 50 does not move."""
        left = max(-1000, min(1000, int(left)))
        right = max(-1000, min(1000, int(right)))
        self.send(f"M {left} {right}")

    def drive_for(self, left: int, right: int, seconds: float, period: float = 0.1) -> None:
        """Drive for a fixed time, repeating the command so the board's 0.5 s
        motor watchdog never fires, then halt (also on Ctrl+C)."""
        end = time.monotonic() + seconds
        try:
            while time.monotonic() < end:
                self.drive(left, right)
                time.sleep(period)
        finally:
            self.halt()

    def halt(self) -> None:
        self.send("H")

    def steer(self, angle: int) -> None:
        """Front wheels, 0..180 deg (90 = straight; clamped by the firmware)."""
        self.send(f"S1 {int(angle)}")

    def cam_pan(self, angle: int) -> None:
        self.send(f"S2 {int(angle)}")

    def cam_tilt(self, angle: int) -> None:
        self.send(f"S3 {int(angle)}")

    def center_servos(self) -> None:
        self.send("R")

    def query(self) -> None:
        """Ask for an immediate #T and #IMU status line."""
        self.send("Q")

    def wait_for(self, name: str, timeout: float = 1.0):
        """Block until .encoders / .imu has a value, then return it."""
        end = time.monotonic() + timeout
        while getattr(self, name) is None:
            if time.monotonic() > end:
                raise TimeoutError(f"no {name} data from the board within {timeout} s")
            time.sleep(0.01)
        return getattr(self, name)

    def close(self) -> None:
        try:
            self.halt()
        except serial.SerialException:
            pass
        self._stop.set()
        self._thread.join(timeout=1)
        self._ser.close()

    def __enter__(self) -> "NiumoBoard":
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    # ---- receiving ------------------------------------------------------
    def _read_loop(self) -> None:
        buf = b""
        while not self._stop.is_set():
            try:
                chunk = self._ser.read(self._ser.in_waiting or 1)
            except serial.SerialException as exc:
                self.errors.append(f"serial error: {exc}")
                time.sleep(0.5)
                continue
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                self._handle(raw.decode("ascii", "replace").strip(), time.monotonic())
            if len(buf) > 512:          # noise without line ends; drop it
                buf = b""

    def _handle(self, line: str, t: float) -> None:
        parts = line.split()
        if not parts:
            return
        kind = parts[0]
        try:
            if kind == "#T" and len(parts) == 4:
                ms, left, right = (int(p) for p in parts[1:])
                if self._last_ms is not None and ms < self._last_ms:
                    self.reboots += 1   # board clock went backwards: it restarted
                self._last_ms = ms
                self.encoders = Encoders(t, ms, left, right)
                if self.on_encoders:
                    self.on_encoders(self.encoders)
            elif kind == "#I" and len(parts) == 9:
                self.imu = Imu(t, *(int(p) for p in parts[1:]))
                if self.on_imu:
                    self.on_imu(self.imu)
            elif kind == "#IMU":
                self.imu_status = " ".join(parts[1:])
                if not self.imu_status.startswith("OK"):
                    self.imu = None
            elif kind == "#BOOT":
                self.boot_line = line
            elif kind == "#ERR":
                self.errors.append(line)
            # "#OK" needs no action
        except ValueError:
            self.errors.append(f"unparsable line: {line!r}")


if __name__ == "__main__":
    import sys

    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/robot_mcu"
    with NiumoBoard(port) as board:
        board.wait_for("encoders")
        print("IMU:", board.imu_status)
        while True:
            e, i = board.encoders, board.imu
            print(f"enc L={e.left:>8} R={e.right:>8}  "
                  + (f"gyro_z={i.gz_cdps / 100:7.2f} deg/s  az={i.az_mg:5d} mg" if i else "no IMU"))
            time.sleep(0.5)
```

Run it directly for a live readout:

```bash
python3 niumo_board.py /dev/robot_mcu
```

```
IMU: OK 0x68
enc L=       0 R=       0  gyro_z=  -0.35 deg/s  az= 1004 mg
```

Using it from your own code:

```python
from niumo_board import NiumoBoard

with NiumoBoard("/dev/robot_mcu") as board:
    board.wait_for("encoders")          # first #T has arrived
    board.center_servos()
    board.steer(90)
    board.drive_for(400, 400, 1.5)      # forward for 1.5 s, then halt
    print(board.encoders.left, board.encoders.right)
    print(board.errors)                 # any #ERR replies
```

Callbacks run on the reader thread, once per line, so keep them short (store the value,
push it into a queue):

```python
def on_imu(imu):
    print(imu.ms, imu.gyro[2])          # yaw-ish rate in rad/s (if z points up)

board = NiumoBoard("/dev/robot_mcu", on_imu=on_imu)
```

---

## 6. Worked examples

**Start with the hardware check.** `pi_examples/hw_test.py` (copy it next to
`niumo_board.py`) moves each servo and each motor in turn, asks what you saw, and checks
the encoder counts by itself. No IMU needed. Lift the rear wheels off the ground first.

```bash
python3 hw_test.py                  # /dev/robot_mcu, else finds the board by its USB ID
python3 hw_test.py --power 450      # if the motors don't start at the default 350
```

Then measure how far the camera may turn on your mount (see 4.2):

```bash
python3 pan_sweep.py
```

`hw_test.py` ends with an OK/FAIL list and says what to fix for each failure: a swapped motor or
encoder cable, an encoder counting the wrong way (the sign in `User/protocol.h`), or the
board restarting under load (power supply).

The helpers below live in `niumo_examples.py` (in the repository's `pi_examples/`), next to `niumo_board.py`. Like the client,
they were tested with simulated data.

```python
"""Worked examples from PI_GUIDE.md (kept here so they can be tested)."""
from __future__ import annotations

import math
import time

from niumo_board import Encoders, Imu, NiumoBoard


# ---- IMU: which axis points up, and gyro bias ---------------------------
def find_up_axis(imu: Imu) -> tuple[str, int]:
    """At rest the accelerometer reads +1 g (~+1000 mg) on the axis pointing UP.
    Returns e.g. ("z", +1) for a module lying flat, chip side up."""
    a = {"x": imu.ax_mg, "y": imu.ay_mg, "z": imu.az_mg}
    axis = max(a, key=lambda k: abs(a[k]))
    return axis, (1 if a[axis] > 0 else -1)


def gyro_bias(board: NiumoBoard, seconds: float = 3.0) -> dict[str, float]:
    """Average the gyro while the robot stands completely still.
    Returns the bias per axis in 0.01 deg/s; subtract it from every sample."""
    samples: list[Imu] = []
    board.on_imu = samples.append
    try:
        time.sleep(seconds)
    finally:
        board.on_imu = None
    if len(samples) < seconds * 50:     # expect ~100 per second
        raise RuntimeError(f"only {len(samples)} IMU samples in {seconds} s - is the IMU connected?")
    n = len(samples)
    return {ax: sum(getattr(s, f"g{ax}_cdps") for s in samples) / n for ax in "xyz"}


# ---- odometry: encoders for distance, gyro for heading -------------------
class DeadReckoning:
    """2-D pose from rear-wheel encoders (distance) and the gyro (heading).

    counts_per_meter: calibrate it (see the guide).  up: result of find_up_axis().
    bias: result of gyro_bias().  Heading is in radians, counter-clockwise positive.
    """

    def __init__(self, counts_per_meter: float, up: tuple[str, int], bias: dict[str, float]):
        self.counts_per_meter = counts_per_meter
        self.axis, self.sign = up
        self.bias = bias[self.axis]
        self.x = self.y = self.heading = 0.0
        self._enc: Encoders | None = None
        self._imu_ms: int | None = None

    def on_imu(self, imu: Imu) -> None:
        if self._imu_ms is not None:
            dt = (imu.ms - self._imu_ms) / 1000
            if 0 < dt < 0.1:                 # skip gaps and board restarts
                rate_cdps = getattr(imu, f"g{self.axis}_cdps") - self.bias
                self.heading += self.sign * math.radians(rate_cdps / 100) * dt
        self._imu_ms = imu.ms

    def on_encoders(self, enc: Encoders) -> None:
        if self._enc is not None and enc.ms >= self._enc.ms:
            counts = ((enc.left - self._enc.left) + (enc.right - self._enc.right)) / 2
            d = counts / self.counts_per_meter
            self.x += d * math.cos(self.heading)
            self.y += d * math.sin(self.heading)
        self._enc = enc


# ---- encoder calibration ------------------------------------------------
def calibrate_counts_per_meter(board: NiumoBoard, power: int = 300, seconds: float = 2.0) -> float:
    """Drive straight, then type the distance you measured with a tape."""
    board.steer(90)
    start = board.wait_for("encoders")
    board.drive_for(power, power, seconds)
    time.sleep(0.5)                      # let the robot stop and the counts settle
    end = board.encoders
    meters = float(input("Distance travelled in metres: "))
    counts = ((end.left - start.left) + (end.right - start.right)) / 2
    return counts / meters
```

### 6.1 Steering check

Find the command that makes the wheels point straight, and the ends of the range:

```python
import time
from niumo_board import NiumoBoard

with NiumoBoard() as board:
    for angle in (90, 75, 105, 63, 126, 90):     # 63 and 126 are full lock
        board.steer(angle)
        input(f"steering at {angle} - look at the wheels, then press Enter")
```

### 6.2 Drive a slow square

```python
from niumo_board import NiumoBoard

with NiumoBoard() as board:
    for _ in range(4):
        board.steer(90)
        board.drive_for(350, 350, 1.0)      # straight
        board.steer(63)                     # full lock one way
        board.drive_for(350, 350, 1.2)      # adjust time for a 90 degree turn
    board.steer(90)
```

### 6.3 Calibrate counts per metre

Put the robot on the floor with about 2 m of free space in front of it:

```python
from niumo_board import NiumoBoard
from niumo_examples import calibrate_counts_per_meter

with NiumoBoard() as board:
    cpm = calibrate_counts_per_meter(board, power=300, seconds=2.0)
    print(f"counts per metre: {cpm:.0f}")   # write it into your config
```

Repeat it two or three times and average.

### 6.4 IMU: up axis and gyro bias

The robot must stand still:

```python
from niumo_board import NiumoBoard
from niumo_examples import find_up_axis, gyro_bias

with NiumoBoard() as board:
    imu = board.wait_for("imu")
    up = find_up_axis(imu)
    bias = gyro_bias(board, seconds=3.0)
    print("up axis:", up)                    # e.g. ('z', 1) or ('y', -1)
    print("gyro bias, 0.01 deg/s:", bias)
```

### 6.5 Dead reckoning: encoders for distance, gyro for heading

```python
import math, time
from niumo_board import NiumoBoard
from niumo_examples import DeadReckoning, find_up_axis, gyro_bias

COUNTS_PER_METER = 2000.0                    # from example 6.3

with NiumoBoard() as board:
    up = find_up_axis(board.wait_for("imu"))
    bias = gyro_bias(board)                  # robot still!
    odo = DeadReckoning(COUNTS_PER_METER, up, bias)
    board.on_imu = odo.on_imu
    board.on_encoders = odo.on_encoders

    board.steer(90)
    board.drive_for(350, 350, 2.0)
    time.sleep(0.5)
    print(f"x={odo.x:.2f} m  y={odo.y:.2f} m  heading={math.degrees(odo.heading):.1f} deg")
```

### 6.6 Log everything to CSV

```python
import csv, time
from niumo_board import NiumoBoard

with open("imu_log.csv", "w", newline="") as f, NiumoBoard() as board:
    out = csv.writer(f)
    out.writerow(["host_t", "ms", "ax_mg", "ay_mg", "az_mg", "gx_cdps", "gy_cdps", "gz_cdps", "temp_cdeg"])
    board.on_imu = lambda i: out.writerow([f"{i.t:.4f}", i.ms, i.ax_mg, i.ay_mg, i.az_mg,
                                           i.gx_cdps, i.gy_cdps, i.gz_cdps, i.temp_cdeg])
    time.sleep(30)
    board.on_imu = None
```

---

## 7. Other languages

The protocol is the whole interface: open the port at 115200 8N1 in raw mode, write
`@...\r\n` lines, split what comes back at `\n`, and split each line at spaces.

### 7.1 Shell

Good for quick tests:

```bash
stty -F /dev/robot_mcu 115200 raw -echo
cat /dev/robot_mcu &                  # print everything the board sends
printf '@Q\r\n'     > /dev/robot_mcu
printf '@S2 60\r\n' > /dev/robot_mcu  # camera pan
printf '@R\r\n'     > /dev/robot_mcu
kill %1
```

Driving from a shell is awkward because of the motor watchdog: repeat the command, e.g.
`for i in $(seq 10); do printf '@M 300 300\r\n' > /dev/robot_mcu; sleep 0.1; done`.

### 7.2 C++ (Linux)

POSIX `termios`, no libraries. Prints encoder counts and the IMU's z rate. (Not compiled
or run yet: no Linux toolchain was available when this was written.)

```cpp
// niumo_read.cpp - build: g++ -O2 -std=c++17 -o niumo_read niumo_read.cpp
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cstdio>
#include <sstream>
#include <string>

static int open_port(const char *path)
{
    int fd = open(path, O_RDWR | O_NOCTTY);
    if (fd < 0) { perror(path); return -1; }

    termios tio{};
    tcgetattr(fd, &tio);
    cfmakeraw(&tio);                    // 8N1, no echo, no line editing
    cfsetispeed(&tio, B115200);
    cfsetospeed(&tio, B115200);
    tio.c_cflag |= CLOCAL | CREAD;
    tio.c_cflag &= ~CRTSCTS;            // no hardware flow control
    tio.c_cc[VMIN] = 1;                 // read() waits for at least one byte
    tio.c_cc[VTIME] = 0;
    tcsetattr(fd, TCSANOW, &tio);
    tcflush(fd, TCIFLUSH);
    return fd;
}

static void send_cmd(int fd, const std::string &payload)
{
    const std::string frame = "@" + payload + "\r\n";
    if (write(fd, frame.data(), frame.size()) < 0) perror("write");
}

static void handle_line(const std::string &line)
{
    std::istringstream in(line);
    std::string kind;
    in >> kind;
    if (kind == "#T") {
        unsigned long ms; long left, right;
        if (in >> ms >> left >> right)
            std::printf("enc  %8lu ms  L=%ld R=%ld\n", ms, left, right);
    } else if (kind == "#I") {
        unsigned long ms; long ax, ay, az, gx, gy, gz, temp;
        if (in >> ms >> ax >> ay >> az >> gx >> gy >> gz >> temp)
            std::printf("imu  %8lu ms  gz=%7.2f deg/s  az=%5ld mg\n", ms, gz / 100.0, az);
    } else if (!kind.empty() && kind != "#OK") {
        std::printf("%s\n", line.c_str());      // #IMU, #BOOT, #ERR
    }
}

int main(int argc, char **argv)
{
    const int fd = open_port(argc > 1 ? argv[1] : "/dev/robot_mcu");
    if (fd < 0) return 1;
    send_cmd(fd, "Q");

    std::string line;
    char buf[256];
    ssize_t n;
    while ((n = read(fd, buf, sizeof buf)) > 0) {
        for (ssize_t i = 0; i < n; i++) {
            if (buf[i] == '\r') continue;
            if (buf[i] != '\n') { line += buf[i]; continue; }
            handle_line(line);
            line.clear();
        }
    }
    close(fd);
    return 0;
}
```

To drive from C++, send `@M` from a timer at 5–10 Hz, exactly as in Python.

### 7.3 ROS 2

`robot_mapping` runs ROS 2 Jazzy inside Docker. A bridge node (not written yet) would read
the serial lines, as `niumo_board.py` does, and publish standard messages:

| Board line | ROS 2 message | Conversion |
|---|---|---|
| `#I` | `sensor_msgs/msg/Imu` | `angular_velocity` = value × π / 18000 (rad/s); `linear_acceleration` = value × 9.80665 / 1000 (m/s²); `orientation_covariance[0] = -1` (no orientation) |
| `#T` | `nav_msgs/msg/Odometry` after integration (example 6.5) | counts / counts_per_metre |
| `@M`, `@S1` | subscribe to `geometry_msgs/msg/Twist` (`cmd_vel`) | speed → motor power, turn rate → steering angle; resend at ≥ 5 Hz |

For timestamps, use the Pi's clock when a line arrives (minus ~5 ms for the IMU's filter
delay), and the board's `ms` field for intervals between samples.

### 7.4 Anything else

Node.js (`serialport`), Rust (`serialport` crate), Go (`go.bug.st/serial`) and others all
work the same way. Remember the three rules: one line per message, `@M` at least every
0.2 s while moving, and keep reading.

---

## 8. How this relates to robot_mapping's mcu.py

`robot_mapping/common/mcu.py` is the client the mapping software already uses. It opens the
port, centres the servos at start-up and reads `#T`. It keeps working with firmware v2
unchanged, because `#T` has the same format and it ignores the new lines.

It doesn't yet read `#I` or `#IMU`, and its comment saying the encoder counts are
always 0 no longer applies. `niumo_board.py` in this guide is a separate, more complete
client; the two can be merged later.

---

## 9. Troubleshooting

| Symptom | What to check |
|---|---|
| `/dev/robot_mcu` missing | `lsusb` shows `1a86:7523`? Udev rule installed (section 1)? Meanwhile use the port from `ls -l /dev/serial/by-id/`. |
| `/dev/ttyUSB*` appears, then disappears | The `brltty` package can grab CH340 adapters: `sudo apt remove brltty`. |
| `Permission denied` | `dialout` group (log out and back in), or the udev rule's `MODE="0666"`. |
| Nothing arrives | Battery switched on? Board flashed with v2? Send `@Q`; any `#OK`? |
| Garbled text | Baud rate must be 115200. |
| Motors run briefly, then stop | Motor watchdog: send `@M` at least every 0.2 s. |
| Motors don't move at small power | `abs(power) <= 50` is treated as 0; most robots need 150+ to start. |
| A servo doesn't move | Steering must be on PB14, pan on PB13, tilt on PB15; the angle may be past the limits in 4.2. |
| Encoders stay at 0 | Encoder cables in EC1/EC2? Wheels actually turning? |
| One encoder counts down going forward | Flip its sign in `User/protocol.h`. |
| `#IMU NONE` | GY-521 in J10, pins in the right order (VCC, GND, SCL, SDA)? |
| `#IMU OK` with an id other than `0x68` | A clone or a related chip (e.g. MPU6500). Basic readings usually still work; check that az ≈ ±1000 mg at rest. |
| Gyro drifts when standing still | Normal; subtract the bias (example 6.4). |
| `ms` jumped back to a small number | The board restarted; the `#BOOT` line says why. `RST=IWDG` means a firmware hang: please report it. |
| `#ERR LONG` | Line longer than 64 characters, or a missing line end. |
