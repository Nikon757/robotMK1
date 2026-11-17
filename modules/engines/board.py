"""Serial link to the Niumo V5 control board running the RobotMapper firmware (v2).

The same client as pi_examples/niumo_board.py, plus open_board(), which finds the board's
USB port by itself. Protocol: firmware/README.md; usage: docs/PI_GUIDE.md.

    from modules.engines import open_board
    with open_board() as board:
        print(board.wait_for("encoders"))
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


BOARD_USB_ID = (0x1A86, 0x7523)     # the board's CH340; the LiDAR's CP2102 is 10c4:ea60
DEFAULT_PORT = "/dev/robot_mcu"     # made by config/99-robot-usb.rules


def find_board_port(requested: Optional[str] = None) -> Optional[str]:
    """The board's serial port. ttyUSB numbers swap between the board and the LiDAR when
    they reconnect, so a requested port that turns out to be another USB adapter is
    replaced by the board's CH340, found by its USB ID."""
    import os
    from serial.tools import list_ports

    ports = {p.device: p for p in list_ports.comports()}
    board = next((dev for dev, p in ports.items() if (p.vid, p.pid) == BOARD_USB_ID), None)
    requested = requested or DEFAULT_PORT
    if not os.path.exists(requested):
        return board
    other = ports.get(os.path.realpath(requested))
    if other is not None and (other.vid, other.pid) != BOARD_USB_ID:
        return board
    return requested


def open_board(port: Optional[str] = None, timeout: float = 2.0) -> NiumoBoard:
    """Connect to the board and wait for its first encoder line."""
    found = find_board_port(port)
    if found is None:
        raise RuntimeError("Can't find the control board: no /dev/robot_mcu and no CH340 "
                           "(USB 1a86:7523). Is its USB cable plugged in? `lsusb` should list it.")
    board = NiumoBoard(found)
    try:
        board.wait_for("encoders", timeout=timeout)
    except TimeoutError:
        board.close()
        raise RuntimeError(f"No data from the board on {found} within {timeout:g} s. "
                           "Battery switched on? Run pi_examples/hw_test.py for a diagnosis.") from None
    return board
