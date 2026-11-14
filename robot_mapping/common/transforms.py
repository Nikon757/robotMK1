"""Small rigid-transform toolbox (numpy only).

Conventions
-----------
* Homogeneous 4x4 matrices ``T`` map points from the *child* frame into the
  *parent* frame:  p_parent = T @ p_child.   ``T_a_b`` reads "pose of b in a".
* Robot / body frames are REP-103 (x forward, y left, z up).
* Camera *optical* frames are OpenCV style (x right, y down, z forward).
* Euler angles: roll about x, pitch about y, yaw about z,
  composed as ``R = Rz(yaw) @ Ry(pitch) @ Rx(roll)`` (degrees in the config).
"""
from __future__ import annotations

import math

import numpy as np

# body <- optical.  body_x = optical_z, body_y = -optical_x, body_z = -optical_y
R_BODY_FROM_OPTICAL = np.array([[0.0, 0.0, 1.0],
                                [-1.0, 0.0, 0.0],
                                [0.0, -1.0, 0.0]])
R_OPTICAL_FROM_BODY = R_BODY_FROM_OPTICAL.T


def rot_x(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=float)


def rot_y(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=float)


def rot_z(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=float)


def rpy_to_R(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    return rot_z(math.radians(yaw_deg)) @ rot_y(math.radians(pitch_deg)) @ rot_x(math.radians(roll_deg))


def R_to_rpy(R: np.ndarray) -> tuple:
    """Inverse of rpy_to_R, returns degrees (roll, pitch, yaw)."""
    sy = max(-1.0, min(1.0, -R[2, 0]))
    pitch = math.asin(sy)
    if abs(math.cos(pitch)) > 1e-6:
        roll = math.atan2(R[2, 1], R[2, 2])
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:  # gimbal lock
        roll = 0.0
        yaw = math.atan2(-R[0, 1], R[1, 1])
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def make_T(R: np.ndarray | None = None, t=None) -> np.ndarray:
    T = np.eye(4)
    if R is not None:
        T[:3, :3] = R
    if t is not None:
        T[:3, 3] = np.asarray(t, dtype=float).reshape(3)
    return T


def inv_T(T: np.ndarray) -> np.ndarray:
    R = T[:3, :3]
    t = T[:3, 3]
    Ti = np.eye(4)
    Ti[:3, :3] = R.T
    Ti[:3, 3] = -R.T @ t
    return Ti


def _g(m, k):
    if hasattr(m, "get"):
        return float(m.get(k, 0.0))
    return float(getattr(m, k, 0.0))


def T_from_mount(m) -> np.ndarray:
    """Pose of a sensor *body* frame in base_link from a config mount block."""
    return make_T(rpy_to_R(_g(m, "roll"), _g(m, "pitch"), _g(m, "yaw")),
                  [_g(m, "x"), _g(m, "y"), _g(m, "z")])


def T_optical_from_mount(m) -> np.ndarray:
    """Pose of a camera *optical* frame in base_link from a config mount block."""
    return T_from_mount(m) @ make_T(R_BODY_FROM_OPTICAL)


def transform_points(T: np.ndarray, pts: np.ndarray) -> np.ndarray:
    pts = np.asarray(pts, dtype=float)
    return pts @ T[:3, :3].T + T[:3, 3]


def rotation_angle_deg(R: np.ndarray) -> float:
    c = (np.trace(R) - 1.0) / 2.0
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def wrap_angle(a: float) -> float:
    """Wrap radians to [-pi, pi)."""
    return (a + math.pi) % (2 * math.pi) - math.pi


def wrap_deg(a):
    """Wrap degrees to [-180, 180) (works on arrays)."""
    return (np.asarray(a, dtype=float) + 180.0) % 360.0 - 180.0


# ---------------------------------------------------------------- SE(2)

def se2_from_T(T: np.ndarray) -> np.ndarray:
    """(x, y, yaw) of a planar pose from a 4x4 matrix (yaw from the x-axis projection)."""
    yaw = math.atan2(T[1, 0], T[0, 0])
    return np.array([T[0, 3], T[1, 3], yaw])


def T_from_se2(p, z: float = 0.0) -> np.ndarray:
    return make_T(rot_z(float(p[2])), [float(p[0]), float(p[1]), z])


def project_to_se2(T: np.ndarray) -> np.ndarray:
    """Force a 6-DoF pose onto the plane (z=0, roll=pitch=0)."""
    return T_from_se2(se2_from_T(T))


def se2_relative(p_i, p_j) -> np.ndarray:
    """Pose of j expressed in frame i, both given as (x, y, yaw)."""
    c, s = math.cos(p_i[2]), math.sin(p_i[2])
    dx, dy = p_j[0] - p_i[0], p_j[1] - p_i[1]
    return np.array([c * dx + s * dy, -s * dx + c * dy, wrap_angle(p_j[2] - p_i[2])])


def se2_compose(p_i, rel) -> np.ndarray:
    """World pose of j given world pose of i and rel = pose of j in i."""
    c, s = math.cos(p_i[2]), math.sin(p_i[2])
    return np.array([p_i[0] + c * rel[0] - s * rel[1],
                     p_i[1] + s * rel[0] + c * rel[1],
                     wrap_angle(p_i[2] + rel[2])])


# ------------------------------------------------------------ quaternions

def quat_to_R(q) -> np.ndarray:
    """q = (w, x, y, z)."""
    w, x, y, z = (float(v) for v in q)
    n = math.sqrt(w * w + x * x + y * y + z * z) or 1.0
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def R_to_quat(R: np.ndarray) -> np.ndarray:
    """Returns (w, x, y, z)."""
    m = R
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    q = np.array([w, x, y, z])
    return q / np.linalg.norm(q)


def rotation_between(a, b) -> np.ndarray:
    """Smallest rotation R with R @ a parallel to b (both 3-vectors)."""
    a = np.asarray(a, float); a = a / np.linalg.norm(a)
    b = np.asarray(b, float); b = b / np.linalg.norm(b)
    v = np.cross(a, b)
    c = float(np.dot(a, b))
    if c < -1 + 1e-9:                      # opposite: rotate 180 deg about any perpendicular axis
        p = np.array([1.0, 0, 0]) if abs(a[0]) < 0.9 else np.array([0, 1.0, 0])
        v = np.cross(a, p); v /= np.linalg.norm(v)
        return 2 * np.outer(v, v) - np.eye(3)
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + vx + vx @ vx / (1 + c)


def orthonormalize(R: np.ndarray) -> np.ndarray:
    u, _, vt = np.linalg.svd(R)
    d = np.sign(np.linalg.det(u @ vt))
    return u @ np.diag([1, 1, d]) @ vt


def exp_so3(w) -> np.ndarray:
    """Rodrigues: rotation vector (rad) -> matrix."""
    w = np.asarray(w, float)
    th = np.linalg.norm(w)
    if th < 1e-12:
        return np.eye(3)
    k = w / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * K @ K
