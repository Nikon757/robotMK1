"""Relative heading from the OAK IMU while the camera is fixed to the chassis."""
from __future__ import annotations

import time

import numpy as np


MAX_SAMPLE_GAP_S = 0.15
MAX_DRIVING_SAMPLE_AGE_S = 0.08
IMU_STREAM = "demo_imu"


class OakYaw:
    @staticmethod
    def add_to_pipeline(dai, pipeline, rate_hz: int) -> None:
        imu = pipeline.create(dai.node.IMU)
        imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, rate_hz)
        imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, rate_hz)
        imu.setBatchReportThreshold(1)
        imu.setMaxBatchReports(10)
        out = pipeline.create(dai.node.XLinkOut)
        out.setStreamName(IMU_STREAM)
        imu.out.link(out.input)

    def __init__(self, dai, device):
        self._queue = device.getOutputQueue(IMU_STREAM, maxSize=64, blocking=False)
        self._clock = dai.Clock.now
        self._up = None
        self._bias = None
        self._previous = None

    def _packets(self):
        for batch in self._queue.tryGetAll():
            yield from batch.packets

    @staticmethod
    def _vector(report) -> np.ndarray:
        return np.array((report.x, report.y, report.z), dtype=float)

    def _require_fresh(self, report, max_age_s: float = MAX_SAMPLE_GAP_S) -> None:
        if report is None or abs((self._clock() - report.getTimestamp()).total_seconds()) > max_age_s:
            raise RuntimeError("OAK gyroscope readings went stale. Check the camera's USB connection.")

    def calibrate(self, duration_s: float) -> None:
        """Estimate the vertical axis and gyro bias with the robot on level ground at rest."""
        self._queue.tryGetAll()
        started = self._clock()
        deadline = time.monotonic() + duration_s
        accel = {}
        gyro = {}
        latest = None
        while time.monotonic() < deadline:
            for packet in self._packets():
                a, g = packet.acceleroMeter, packet.gyroscope
                if a.getTimestamp() >= started:
                    accel[a.getTimestampDevice()] = self._vector(a)
                if g.getTimestamp() >= started:
                    gyro[g.getTimestampDevice()] = self._vector(g)
                    latest = g
            time.sleep(0.01)
        if len(accel) < 8 or len(gyro) < 8:
            raise RuntimeError("Too few OAK IMU samples for calibration. Check the camera's USB connection.")
        self._require_fresh(latest)
        mean_accel = np.mean(list(accel.values()), axis=0)
        gyro_samples = np.array(list(gyro.values()))
        gravity = np.linalg.norm(mean_accel)
        if not 8.0 < gravity < 12.0 or not np.max(np.linalg.norm(gyro_samples, axis=1)) < 0.15:
            raise RuntimeError("Keep the robot stationary with the camera centred during IMU calibration.")
        self._up = mean_accel / gravity
        self._bias = gyro_samples.mean(axis=0)
        self._previous = latest

    def delta(self, reset: bool = False) -> float:
        """Integrate fresh angular rate in radians. Reset discards motion between route legs."""
        if self._up is None:
            raise RuntimeError("Calibrate the OAK gyroscope before driving.")
        reports = [packet.gyroscope for packet in self._packets()]
        previous = self._previous
        if reset:
            latest = reports[-1] if reports else previous
            self._require_fresh(latest)
            self._previous = latest
            return 0.0

        angle = 0.0
        for report in reports:
            dt = (report.getTimestampDevice() - previous.getTimestampDevice()).total_seconds()
            if dt <= 0:
                continue
            if dt > MAX_SAMPLE_GAP_S:
                raise RuntimeError("OAK gyroscope samples were lost during motion.")
            mean_rate = (self._vector(previous) + self._vector(report)) / 2 - self._bias
            angle += float(mean_rate @ self._up) * dt
            previous = report
        self._require_fresh(previous, MAX_DRIVING_SAMPLE_AGE_S)
        if not np.isfinite(angle):
            raise RuntimeError("Invalid angular rate from the OAK gyroscope.")
        self._previous = previous
        return angle
