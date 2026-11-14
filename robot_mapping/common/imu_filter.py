"""IMU orientation for the OAK-D IMU (BMI270 raw accel/gyro or BNO086 rotation vector).

* Gyro bias is estimated from the first ``bias_samples`` samples (robot must be still).
* Gravity auto-alignment: the mean accelerometer vector at rest is rotated by the
  configured ``camera.imu_rpy_deg`` into the body frame; whatever residual tilt is
  left is removed with a small correction rotation.  This makes the *vertical axis*
  right even if the configured IMU orientation is off, which is what the planar
  (3-DoF) mapper needs: the yaw rate is measured about the true vertical.
* Orientation is tracked with a Madgwick (IMU-only) filter, or taken straight from
  the BNO086 rotation vector when the sample carries a quaternion.

The output ``R_world_body`` is gravity-aligned; yaw is relative to the first
frame (Madgwick) or magnetic north (BNO086).  Only *relative* rotations between
two timestamps are used by the mapper, so the yaw reference does not matter.
"""
from __future__ import annotations

import math
from collections import deque
from typing import Optional

import numpy as np

from .transforms import R_to_rpy, orthonormalize, quat_to_R, rotation_between, rpy_to_R

G = 9.80665


class ImuOrientation:
    def __init__(self, imu_rpy_deg=(0.0, 90.0, 0.0), beta: float = 0.05, bias_samples: int = 200,
                 history_s: float = 3.0, auto_align: bool = True):
        self.R_body_imu = rpy_to_R(*imu_rpy_deg)       # body <- imu (configured)
        self.R_fix = np.eye(3)                           # gravity correction, body <- body'
        self.beta = beta
        self.bias = np.zeros(3)
        self.bias_samples = bias_samples
        self.auto_align = auto_align
        self._acc = []
        self._gyr = []
        self.ready = False
        self.q = np.array([1.0, 0.0, 0.0, 0.0])         # world <- body
        self.t_last: Optional[float] = None
        self._hist = deque()
        self._history_s = history_s
        self.tilt_deg = 0.0                              # residual tilt found at start-up
        self.gravity_body = None                         # mean accel in body frame at start-up
        self.samples = 0

    # ---------------------------------------------------------------- setup
    def _finish_calibration(self):
        acc = np.mean(self._acc, axis=0)
        gyr = np.mean(self._gyr, axis=0)
        self.bias = gyr
        g_body = self.R_body_imu @ acc
        self.gravity_body = g_body
        n = np.linalg.norm(g_body)
        if n < 1e-3:
            self.tilt_deg = 0.0
        else:
            up = g_body / n
            self.tilt_deg = math.degrees(math.acos(max(-1.0, min(1.0, float(up[2])))))
            if self.auto_align and self.tilt_deg > 0.5:
                self.R_fix = rotation_between(up, [0.0, 0.0, 1.0])
        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        self.ready = True
        self._acc, self._gyr = [], []

    def R_body_from_imu(self) -> np.ndarray:
        return orthonormalize(self.R_fix @ self.R_body_imu)

    # ---------------------------------------------------------------- update
    def push(self, t: float, gyro, accel, quat=None):
        """Feed one sample (IMU frame).  Returns True once orientation is valid."""
        self.samples += 1
        gyro = np.asarray(gyro, float)
        accel = np.asarray(accel, float)
        if not self.ready:
            self._acc.append(accel)
            self._gyr.append(gyro)
            if len(self._acc) >= self.bias_samples:
                self._finish_calibration()
                self.t_last = t
            return False
        Rbi = self.R_body_from_imu()
        if quat is not None:
            R_wi = quat_to_R(quat)
            R_wb = R_wi @ Rbi.T
            self.q = _R_to_q(R_wb)
        else:
            dt = 0.0 if self.t_last is None else max(0.0, min(0.1, t - self.t_last))
            w = Rbi @ (gyro - self.bias)
            a = Rbi @ accel
            self._madgwick(w, a, dt)
        self.t_last = t
        R = self.R_world_body()
        self._hist.append((t, R))
        while self._hist and t - self._hist[0][0] > self._history_s:
            self._hist.popleft()
        return True

    def _madgwick(self, w, a, dt):
        q0, q1, q2, q3 = self.q
        gx, gy, gz = w
        # rate of change from gyro
        qDot0 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz)
        qDot1 = 0.5 * (q0 * gx + q2 * gz - q3 * gy)
        qDot2 = 0.5 * (q0 * gy - q1 * gz + q3 * gx)
        qDot3 = 0.5 * (q0 * gz + q1 * gy - q2 * gx)
        an = np.linalg.norm(a)
        if an > 1e-6 and abs(an - G) < 0.35 * G:      # skip accel correction while accelerating hard
            ax, ay, az = a / an
            # gradient descent step (Madgwick 2010, IMU variant)
            _2q0, _2q1, _2q2, _2q3 = 2 * q0, 2 * q1, 2 * q2, 2 * q3
            _4q0, _4q1, _4q2 = 4 * q0, 4 * q1, 4 * q2
            _8q1, _8q2 = 8 * q1, 8 * q2
            q0q0, q1q1, q2q2, q3q3 = q0 * q0, q1 * q1, q2 * q2, q3 * q3
            s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay
            s1 = _4q1 * q3q3 - _2q3 * ax + 4 * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az
            s2 = 4 * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az
            s3 = 4 * q1q1 * q3 - _2q1 * ax + 4 * q2q2 * q3 - _2q2 * ay
            sn = math.sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3)
            if sn > 1e-12:
                s0, s1, s2, s3 = s0 / sn, s1 / sn, s2 / sn, s3 / sn
                qDot0 -= self.beta * s0
                qDot1 -= self.beta * s1
                qDot2 -= self.beta * s2
                qDot3 -= self.beta * s3
        q = np.array([q0 + qDot0 * dt, q1 + qDot1 * dt, q2 + qDot2 * dt, q3 + qDot3 * dt])
        self.q = q / np.linalg.norm(q)

    # ---------------------------------------------------------------- queries
    def R_world_body(self) -> np.ndarray:
        return quat_to_R(self.q)

    def R_at(self, t: float) -> Optional[np.ndarray]:
        """Orientation closest in time to t (None if no history)."""
        if not self._hist:
            return None
        best = min(self._hist, key=lambda h: abs(h[0] - t))
        return best[1]

    def delta_R(self, t0: float, t1: float) -> Optional[np.ndarray]:
        """Rotation of the body between t0 and t1: R_b0_b1 (pose of body(t1) in body(t0))."""
        R0, R1 = self.R_at(t0), self.R_at(t1)
        if R0 is None or R1 is None:
            return None
        return R0.T @ R1

    def rpy_deg(self):
        return R_to_rpy(self.R_world_body())

    def yaw(self) -> float:
        R = self.R_world_body()
        return math.atan2(R[1, 0], R[0, 0])

    def describe(self) -> str:
        r, p, y = R_to_rpy(self.R_body_from_imu())
        return (f"gyro bias [{self.bias[0]:+.4f} {self.bias[1]:+.4f} {self.bias[2]:+.4f}] rad/s, "
                f"start-up tilt {self.tilt_deg:.1f} deg, effective imu->body rpy "
                f"[{r:.1f}, {p:.1f}, {y:.1f}]")


def _R_to_q(R: np.ndarray) -> np.ndarray:
    from .transforms import R_to_quat
    return R_to_quat(R)


def estimate_imu_to_body(acc_level: np.ndarray, acc_nose_down: np.ndarray) -> tuple:
    """Two-pose alignment used by tools/calib_camera.py --imu.

    acc_level     : mean accelerometer vector (IMU frame) with the camera level, looking forward
    acc_nose_down : mean accelerometer vector after pitching the camera nose DOWN by 20..60 deg
    Returns (R_body_imu, rpy_deg, tilt_between_poses_deg).
    """
    u1 = acc_level / np.linalg.norm(acc_level)             # body +z expressed in IMU coords
    u2 = acc_nose_down / np.linalg.norm(acc_nose_down)
    ang = math.degrees(math.acos(max(-1.0, min(1.0, float(np.dot(u1, u2))))))
    axis = np.cross(u1, u2)                                # = -y_body for a nose-down pitch
    if np.linalg.norm(axis) < 1e-6:
        raise ValueError("the two poses are identical; tilt the camera more")
    y_b = -axis / np.linalg.norm(axis)
    z_b = u1
    x_b = np.cross(y_b, z_b)
    x_b /= np.linalg.norm(x_b)
    y_b = np.cross(z_b, x_b)
    R_body_imu = orthonormalize(np.stack([x_b, y_b, z_b], axis=0))   # rows = body axes in IMU coords
    return R_body_imu, R_to_rpy(R_body_imu), ang
