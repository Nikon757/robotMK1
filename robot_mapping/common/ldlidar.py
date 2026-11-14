"""LDROBOT STL-27L / LD19 / LD06 serial driver (pure Python, pyserial).

Wire format (identical for the three sensors, only the baud rate and point
rate differ):

    byte  0      0x54                       header
    byte  1      0x2C                       VerLen: type=1 (upper 3 bits), 12 points (lower 5 bits)
    bytes 2-3    speed        uint16 LE     deg/s
    bytes 4-5    start_angle  uint16 LE     0.01 deg
    bytes 6-41   12 x { distance uint16 LE (mm), intensity uint8 }
    bytes 42-43  end_angle    uint16 LE     0.01 deg
    bytes 44-45  timestamp    uint16 LE     ms, wraps at 30000
    byte  46     crc8                       CRC-8, poly 0x4D, init 0, over bytes 0..45

The sensor angle increases CLOCKWISE when viewed from above (0 deg roughly at
the cable / connector).  Angles inside a packet are linearly interpolated
between start_angle and end_angle.

Frame conventions produced by :func:`scan_to_points`:
    bearing = wrap(forward_angle_deg - raw_angle)      (clockwise sensor)
    bearing = wrap(raw_angle - forward_angle_deg)      (clockwise: false)
so bearing 0 = robot forward, positive = left (REP-103), and the returned
xy points are in the LiDAR frame (x forward, y left).

Standalone test:
    python -m common.ldlidar --port /dev/robot_lidar --baud 921600
"""
from __future__ import annotations

import argparse
import glob
import struct
import sys
import threading
import time
from dataclasses import dataclass
from typing import Optional

import numpy as np

try:
    import serial  # pyserial
    from serial.tools import list_ports
except ImportError:  # keep the module importable for offline tools / replay
    serial = None
    list_ports = None

HEADER = 0x54
VERLEN = 0x2C
POINTS_PER_PACKET = 12
PACKET_LEN = 47
_PT = struct.Struct("<HB")
_HDR = struct.Struct("<BBHH")
_TAIL = struct.Struct("<HHB")

BAUD_BY_MODEL = {"STL27L": 921600, "LD19": 230400, "LD06": 230400, "LD14P": 230400}
USB_BRIDGES = {(0x10C4, 0xEA60): "CP210x", (0x1A86, 0x7523): "CH340", (0x1A86, 0x55D4): "CH9102"}


def _make_crc_table(poly: int = 0x4D) -> bytes:
    table = []
    for i in range(256):
        c = i
        for _ in range(8):
            c = ((c << 1) ^ poly) & 0xFF if c & 0x80 else (c << 1) & 0xFF
        table.append(c)
    return bytes(table)


CRC_TABLE = _make_crc_table()


def crc8(data: bytes) -> int:
    c = 0
    t = CRC_TABLE
    for b in data:
        c = t[(c ^ b) & 0xFF]
    return c


@dataclass
class Packet:
    speed_dps: float
    start_angle: float
    end_angle: float
    timestamp_ms: int
    distances_mm: np.ndarray      # (12,)
    intensities: np.ndarray       # (12,)

    def angles(self) -> np.ndarray:
        span = (self.end_angle - self.start_angle) % 360.0
        step = span / (POINTS_PER_PACKET - 1)
        return (self.start_angle + step * np.arange(POINTS_PER_PACKET)) % 360.0


def parse_packet(buf: bytes) -> Optional[Packet]:
    """Parse one 47-byte packet; returns None on bad header / CRC."""
    if len(buf) != PACKET_LEN or buf[0] != HEADER or buf[1] != VERLEN:
        return None
    if crc8(buf[:-1]) != buf[-1]:
        return None
    _, _, speed, start = _HDR.unpack_from(buf, 0)
    pts = np.frombuffer(buf, dtype=np.dtype([("d", "<u2"), ("i", "u1")]), count=POINTS_PER_PACKET, offset=6)
    end, ts, _ = _TAIL.unpack_from(buf, 42)
    return Packet(speed, start / 100.0, end / 100.0, ts,
                  pts["d"].astype(np.float32), pts["i"].astype(np.float32))


def build_packet(speed_dps: float, start_angle: float, end_angle: float, distances_mm, intensities,
                 timestamp_ms: int = 0) -> bytes:
    """Inverse of parse_packet (used by tests and the synthetic recorder)."""
    body = _HDR.pack(HEADER, VERLEN, int(speed_dps), int(round(start_angle * 100)) % 36000)
    for d, i in zip(distances_mm, intensities):
        body += _PT.pack(int(d) & 0xFFFF, int(i) & 0xFF)
    body += _TAIL.pack(int(round(end_angle * 100)) % 36000, int(timestamp_ms) % 30000, 0)[:-1]
    return body + bytes([crc8(body)])


@dataclass
class LidarScan:
    """One full revolution of raw sensor data."""
    t: float                                   # host monotonic time at end of revolution (s)
    angles_deg: np.ndarray                     # raw sensor angles, 0..360, clockwise
    ranges_m: np.ndarray
    intensities: np.ndarray
    speed_dps: float = 0.0

    def __len__(self) -> int:
        return len(self.angles_deg)


class PacketStream:
    """Feeds raw bytes, yields Packet objects (resynchronises on the header)."""

    def __init__(self):
        self._buf = bytearray()
        self.bad_crc = 0

    def feed(self, data: bytes):
        self._buf += data
        out = []
        buf = self._buf
        while True:
            i = buf.find(HEADER)
            if i < 0:
                buf.clear()
                break
            if i > 0:
                del buf[:i]
            if len(buf) < PACKET_LEN:
                break
            if buf[1] != VERLEN:
                del buf[:1]
                continue
            pkt = parse_packet(bytes(buf[:PACKET_LEN]))
            if pkt is None:
                self.bad_crc += 1
                del buf[:1]
                continue
            out.append(pkt)
            del buf[:PACKET_LEN]
        return out


class ScanAssembler:
    """Groups packets into full 360-degree scans (split when the angle wraps)."""

    def __init__(self):
        self._ang, self._rng, self._int = [], [], []
        self._last_angle = None
        self._speed = 0.0

    def push(self, pkt: Packet, t: float) -> Optional[LidarScan]:
        angles = pkt.angles()
        done = None
        # start angles grow monotonically within a revolution; any decrease = new revolution
        if self._last_angle is not None and angles[0] + 1e-3 < self._last_angle and self._ang:
            done = LidarScan(t,
                             np.concatenate(self._ang),
                             np.concatenate(self._rng) / 1000.0,
                             np.concatenate(self._int),
                             self._speed)
            self._ang, self._rng, self._int = [], [], []
        self._ang.append(angles)
        self._rng.append(pkt.distances_mm)
        self._int.append(pkt.intensities)
        self._last_angle = float(angles[0])
        self._speed = pkt.speed_dps
        return done


def scan_to_points(scan: LidarScan, forward_angle_deg: float = 0.0, fov_deg: float = 360.0,
                   clockwise: bool = True, min_range: float = 0.05, max_range: float = 25.0,
                   min_intensity: float = 0.0):
    """Apply the FOV crop and convert to LiDAR-frame xy.

    Returns (points_xy (N,2), bearings_rad (N,), ranges_m (N,)) with bearing 0 = forward,
    positive = counter-clockwise (left).
    """
    raw = scan.angles_deg
    bearing = (forward_angle_deg - raw) if clockwise else (raw - forward_angle_deg)
    bearing = (bearing + 180.0) % 360.0 - 180.0
    r = scan.ranges_m
    keep = (np.abs(bearing) <= fov_deg / 2.0) & (r >= min_range) & (r <= max_range)
    if min_intensity > 0:
        keep &= scan.intensities >= min_intensity
    b = np.radians(bearing[keep])
    rr = r[keep]
    xy = np.stack([rr * np.cos(b), rr * np.sin(b)], axis=1)
    return xy, b, rr


def find_port(preferred: str | None = None) -> Optional[str]:
    """Return a serial port that looks like the LiDAR USB bridge."""
    import os
    if preferred and os.path.exists(preferred):
        return preferred
    if list_ports is None:
        return None
    cands = []
    for p in list_ports.comports():
        tag = USB_BRIDGES.get((p.vid or 0, p.pid or 0))
        if tag == "CP210x":
            cands.insert(0, p.device)
        elif tag:
            cands.append(p.device)
    if cands:
        return cands[0]
    for pat in ("/dev/ttyUSB*", "/dev/ttyACM*"):
        g = sorted(glob.glob(pat))
        if g:
            return g[0]
    return None


class LDLidar:
    """Background reader thread.  ``latest()`` returns the newest LidarScan."""

    def __init__(self, port: str | None, baud: int = 921600, probe_timeout: float = 3.0):
        if serial is None:
            raise RuntimeError("pyserial is not installed (pip install pyserial)")
        self.port = find_port(port)
        if not self.port:
            raise RuntimeError(f"LiDAR port not found (wanted {port}); check the USB cable and udev rules")
        self.baud = baud
        self._ser = serial.Serial(self.port, baud, timeout=0.05)
        self._stream = PacketStream()
        self._asm = ScanAssembler()
        self._lock = threading.Lock()
        self._scan: Optional[LidarScan] = None
        self._cv = threading.Condition(self._lock)
        self._stop = threading.Event()
        self.scans = 0
        self.packets = 0
        self._th = threading.Thread(target=self._run, name="ldlidar", daemon=True)
        self._th.start()
        t0 = time.monotonic()
        while self.packets == 0 and time.monotonic() - t0 < probe_timeout:
            time.sleep(0.05)
        if self.packets == 0:
            self.close()
            raise RuntimeError(f"no LiDAR packets on {self.port} @ {baud} baud "
                               f"(STL-27L = 921600, LD19/LD06 = 230400; is the sensor powered and spinning?)")

    def _run(self):
        while not self._stop.is_set():
            try:
                data = self._ser.read(self._ser.in_waiting or 1)
            except Exception:
                time.sleep(0.1)
                continue
            if not data:
                continue
            now = time.monotonic()
            for pkt in self._stream.feed(data):
                self.packets += 1
                scan = self._asm.push(pkt, now)
                if scan is not None:
                    with self._cv:
                        self._scan = scan
                        self.scans += 1
                        self._cv.notify_all()

    def latest(self) -> Optional[LidarScan]:
        with self._lock:
            return self._scan

    def wait_scan(self, timeout: float = 1.0) -> Optional[LidarScan]:
        with self._cv:
            n = self.scans
            self._cv.wait_for(lambda: self.scans != n, timeout=timeout)
            return self._scan

    @property
    def bad_crc(self) -> int:
        return self._stream.bad_crc

    def close(self):
        self._stop.set()
        try:
            self._th.join(0.5)
        except RuntimeError:
            pass
        try:
            self._ser.close()
        except Exception:
            pass

    def __enter__(self):
        return self

    def __exit__(self, *a):
        self.close()


def _main(argv=None):
    ap = argparse.ArgumentParser(description="LDROBOT LiDAR serial test")
    ap.add_argument("--port", default=None)
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--seconds", type=float, default=5.0)
    a = ap.parse_args(argv)
    with LDLidar(a.port, a.baud) as ld:
        print(f"port {ld.port} @ {ld.baud}")
        t0 = time.monotonic()
        while time.monotonic() - t0 < a.seconds:
            s = ld.wait_scan(1.0)
            if s is None:
                print("timeout")
                continue
            valid = s.ranges_m > 0
            print(f"scan #{ld.scans}: {len(s)} pts, {valid.sum()} valid, speed {s.speed_dps:.0f} deg/s, "
                  f"range {s.ranges_m[valid].min() if valid.any() else 0:.2f}..{s.ranges_m.max():.2f} m, "
                  f"bad crc {ld.bad_crc}")
    return 0


if __name__ == "__main__":
    sys.exit(_main())
