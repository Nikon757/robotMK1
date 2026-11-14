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
