"""Serial link to the Niumo V5 (STM32F103) board running the RobotMapper firmware.

Protocol (see ../firmware/README.md):
    Pi -> board    @S<n> <angle>   servo n=1 steering, 2 cam pan, 3 cam tilt (0..180, 90 = centre)
                   @M <l> <r>      motors, -1000..1000 per-mil, must be repeated at >= 2 Hz
                   @H              halt motors
                   @R              centre all servos
                   @Q              force a telemetry packet
    board -> Pi    #BOOT ..., #OK, #ERR <code>, #T <ms> <encL> <encR>   (50 Hz)

KNOWN FIRMWARE LIMITATION (documented, not fixed here): the firmware never calls
``Encoder_Updata()``, so encL / encR are always 0.  Wheel odometry is therefore
unavailable; the mappers never rely on it.  ``Mcu.encoders_alive`` turns True
only if non-zero counts are ever seen.
"""
from __future__ import annotations

import threading
import time
from typing import Optional

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None

CH340 = (0x1A86, 0x7523)


def find_port(preferred: str | None = None) -> Optional[str]:
    import os
    if preferred and os.path.exists(preferred):
        return preferred
    if list_ports is None:
        return None
    for p in list_ports.comports():
        if (p.vid or 0, p.pid or 0) == CH340:
            return p.device
    return None


class Mcu:
    def __init__(self, port: str | None, baud: int = 115200, timeout_boot: float = 1.0):
        if serial is None:
            raise RuntimeError("pyserial is not installed")
        self.port = find_port(port)
        if not self.port:
            raise RuntimeError(f"MCU port not found (wanted {port})")
        self._ser = serial.Serial(self.port, baud, timeout=0.05)
        time.sleep(0.3)
        self._ser.reset_input_buffer()
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self.telemetry = None            # (t_host, ms, encL, encR)
        self.telemetry_count = 0
        self.errors = []
        self.encoders_alive = False
        self._th = threading.Thread(target=self._run, name="mcu", daemon=True)
        self._th.start()
        self.cmd("Q")
        t0 = time.monotonic()
        while self.telemetry is None and time.monotonic() - t0 < timeout_boot:
            time.sleep(0.02)
        self.alive = self.telemetry is not None

    def _run(self):
        buf = b""
        while not self._stop.is_set():
            try:
                data = self._ser.read(self._ser.in_waiting or 1)
            except Exception:
                time.sleep(0.1)
                continue
            if not data:
                continue
            buf += data
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                s = line.decode(errors="ignore").strip()
                if s.startswith("#T"):
                    parts = s.split()
                    if len(parts) == 4:
                        try:
                            ms, el, er = int(parts[1]), int(parts[2]), int(parts[3])
                        except ValueError:
                            continue
                        with self._lock:
                            self.telemetry = (time.monotonic(), ms, el, er)
                            self.telemetry_count += 1
                            if el or er:
                                self.encoders_alive = True
                elif s.startswith("#ERR"):
                    self.errors.append(s)

    def cmd(self, c: str):
        with self._lock:
            self._ser.write(f"@{c}\r\n".encode())

    def center_servos(self):
        self.cmd("R")

    def halt(self):
        self.cmd("H")

    def steer(self, angle: int):
        self.cmd(f"S1 {int(angle)}")

    def cam_pan(self, angle: int):
        self.cmd(f"S2 {int(angle)}")

    def cam_tilt(self, angle: int):
        self.cmd(f"S3 {int(angle)}")

    def drive(self, left: int, right: int):
        self.cmd(f"M {int(left)} {int(right)}")

    def latest(self):
        with self._lock:
            return self.telemetry

    def close(self):
        self._stop.set()
        try:
            self.halt()
        except Exception:
            pass
        try:
            self._th.join(0.5)
            self._ser.close()
        except Exception:
            pass

    def __enter__(self):
        return self

    def __exit__(self, *a):
        self.close()
