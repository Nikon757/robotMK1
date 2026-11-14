#!/usr/bin/env python3
"""Generate a synthetic *raw recording* (same layout as ``--raw``) of a textured box room,
so the fusion mapper can be tested end-to-end without any hardware:

    python tools/make_synthetic_raw.py --out maps/synth/raw [--frames 300]
    python run.py fusion --replay maps/synth/raw --out maps/synth --record

The robot drives a rectangle on the floor and returns to the start, which
exercises visual odometry, LiDAR ICP, keyframes, loop closure and graph
optimisation.  Ground truth poses are written to ``truth.txt`` (TUM format).
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from common.config import load_config  # noqa: E402
from common.ldlidar import LidarScan  # noqa: E402
from common.oakd import Frame, ImuSample  # noqa: E402
from common.recorder import RawRecorder  # noqa: E402
from common.transforms import (R_to_quat, T_from_mount, T_from_se2, T_optical_from_mount, rpy_to_R)  # noqa: E402

G = 9.80665


class BoxWorld:
    """Room [x0,x1]x[y0,y1]x[0,zc] with a few axis-aligned boxes; procedural textures."""

    def __init__(self, seed: int = 0):
        self.room = (-1.0, 5.0, -2.0, 2.0, 2.4)
        # boxes: (x0, x1, y0, y1, z0, z1)
        self.boxes = [(1.2, 1.8, -1.7, -1.2, 0.0, 0.5), (4.1, 4.6, 0.6, 1.4, 0.0, 0.9), (-0.4, 0.0, 1.2, 1.9, 0.0, 1.2)]
        rng = np.random.default_rng(seed)
        self.palette = rng.integers(40, 230, (4096, 3)).astype(np.uint8)

    def planes(self):
        x0, x1, y0, y1, zc = self.room
        big = 1e9
        P = [  # axis, value, normal sign (unused), bounds (other two axes)
            (2, 0.0, (x0, x1, y0, y1)),   # floor
            (2, zc, (x0, x1, y0, y1)),    # ceiling
            (0, x0, (y0, y1, 0.0, zc)), (0, x1, (y0, y1, 0.0, zc)),
            (1, y0, (x0, x1, 0.0, zc)), (1, y1, (x0, x1, 0.0, zc)),
        ]
        for bx0, bx1, by0, by1, bz0, bz1 in self.boxes:
            P += [(0, bx0, (by0, by1, bz0, bz1)), (0, bx1, (by0, by1, bz0, bz1)),
                  (1, by0, (bx0, bx1, bz0, bz1)), (1, by1, (bx0, bx1, bz0, bz1)),
                  (2, bz1, (bx0, bx1, by0, by1))]
        return P

    def cast(self, o: np.ndarray, d: np.ndarray):
        """o (3,), d (N,3) unnormalised directions -> t (N,) (inf if no hit), hit points (N,3)."""
        n = len(d)
        t_best = np.full(n, np.inf)
        others = {0: (1, 2), 1: (0, 2), 2: (0, 1)}
        for axis, val, (a0, a1, b0, b1) in self.planes():
            da = d[:, axis]
            with np.errstate(divide="ignore", invalid="ignore"):
                t = (val - o[axis]) / da
            ok = np.isfinite(t) & (t > 1e-6) & (t < t_best)
            if not ok.any():
                continue
            ia, ib = others[axis]
            pa = o[ia] + t * d[:, ia]
            pb = o[ib] + t * d[:, ib]
            ok &= (pa >= a0) & (pa <= a1) & (pb >= b0) & (pb <= b1)
            t_best = np.where(ok, t, t_best)
        pts = o + t_best[:, None] * d
        return t_best, pts

    def color(self, pts: np.ndarray) -> np.ndarray:
        cell = np.floor(pts / 0.25).astype(np.int64)
        h = (cell[:, 0] * 73856093) ^ (cell[:, 1] * 19349663) ^ (cell[:, 2] * 83492791)
        col = self.palette[np.abs(h) % len(self.palette)].astype(np.int16)
        # fine dots inside cells give ORB plenty of corners
        f = np.floor(pts / 0.05).astype(np.int64)
        h2 = (f[:, 0] * 2654435761 + f[:, 1] * 97 + f[:, 2] * 1013) % 7
        col = np.where((h2 == 0)[:, None], col - 60, col)
        return np.clip(col, 0, 255).astype(np.uint8)


def trajectory(n: int, fps: float):
    """Rectangle 3.0 x 1.6 m starting at (0.5, -0.8), CCW, returning to start; (x, y, yaw) per frame."""
    corners = [(0.5, -0.8), (3.5, -0.8), (3.5, 0.8), (0.5, 0.8), (0.5, -0.8)]
    speed = 0.35                       # m/s
    turn = math.radians(45)            # rad/s
    poses = []
    t = 0.0
    dt = 1.0 / fps
    yaw = 0.0
    x, y = corners[0]
    for k in range(4):
        (x0, y0), (x1, y1) = corners[k], corners[k + 1]
        yaw_t = math.atan2(y1 - y0, x1 - x0)
        while abs(((yaw_t - yaw) + math.pi) % (2 * math.pi) - math.pi) > 1e-3:
            d = ((yaw_t - yaw) + math.pi) % (2 * math.pi) - math.pi
            yaw += np.clip(d, -turn * dt, turn * dt)
            poses.append((t, x, y, yaw)); t += dt
        L = math.hypot(x1 - x0, y1 - y0)
        s = 0.0
        while s < L:
            s = min(L, s + speed * dt)
            x, y = x0 + (x1 - x0) * s / L, y0 + (y1 - y0) * s / L
            poses.append((t, x, y, yaw)); t += dt
    # finally turn back to the initial heading so the last view overlaps the first (loop closure)
    while abs(((0.0 - yaw) + math.pi) % (2 * math.pi) - math.pi) > 1e-3:
        d = ((0.0 - yaw) + math.pi) % (2 * math.pi) - math.pi
        yaw += np.clip(d, -turn * dt, turn * dt)
        poses.append((t, x, y, yaw)); t += dt
    for _ in range(int(fps)):                       # hold still for a second at the end
        poses.append((t, x, y, yaw)); t += dt
    if n and len(poses) > n:
        poses = poses[:n]
    return poses


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="synthetic raw recording generator")
    ap.add_argument("--out", default=str(ROOT / "maps" / "synth" / "raw"))
    ap.add_argument("--frames", type=int, default=0, help="limit the number of frames (0 = full loop)")
    ap.add_argument("--config", default=None)
    ap.add_argument("--seed", type=int, default=0)
    a = ap.parse_args(argv)
    cfg = load_config(a.config)
    W, H, fps = int(cfg.camera.width), int(cfg.camera.height), float(cfg.camera.fps)
    fx = fy = 0.78 * W
    K = np.array([[fx, 0, W / 2], [0, fy, H / 2], [0, 0, 1.0]])
    world = BoxWorld(a.seed)
    T_bc = T_optical_from_mount(cfg.camera.mount)
    T_bl = T_from_mount(cfg.lidar.mount)
    R_bi = rpy_to_R(*cfg.camera.imu_rpy_deg)
    R_ib = R_bi.T
    rng = np.random.default_rng(a.seed)

    meta = {"K": K.tolist(), "width": W, "height": H, "fps": fps, "imu": "synthetic-BMI270", "rotation_vector": False,
            "usb_speed": "SYNTH", "mxid": "synthetic", "config": cfg.to_dict()}
    rec = RawRecorder(a.out, meta)
    poses = trajectory(a.frames, fps)
    print(f"[synth] {len(poses)} frames, {len(poses) / fps:.1f} s, out = {a.out}")

    us, vs = np.meshgrid(np.arange(W), np.arange(H))
    dirs_opt = np.stack([(us - K[0, 2]) / fx, (vs - K[1, 2]) / fy, np.ones_like(us, dtype=float)], axis=-1).reshape(-1, 3)
    lidar_raw = np.arange(0.0, 360.0, 0.5)
    fwd = float(cfg.lidar.forward_angle_deg)
    cw = bool(cfg.lidar.clockwise)
    rel = (fwd - lidar_raw) if cw else (lidar_raw - fwd)
    bearing = np.radians((rel + 180.0) % 360.0 - 180.0)
    imu_dt = 1.0 / float(cfg.camera.get("imu_rate_hz", 100))
    truth = open(Path(a.out) / "truth.txt", "w", encoding="utf-8")
    truth.write("# TUM: t x y z qx qy qz qw (base_link ground truth)\n")
    t_imu = 0.0
    prev_yaw = poses[0][3]
    next_scan_t = 0.0
    t0 = time.time()
    for i, (t, x, y, yaw) in enumerate(poses):
        T_wb = T_from_se2((x, y, yaw))
        q = R_to_quat(T_wb[:3, :3])
        truth.write(f"{t:.6f} {x:.4f} {y:.4f} 0.0 {q[1]:.6f} {q[2]:.6f} {q[3]:.6f} {q[0]:.6f}\n")
        # camera
        T_wc = T_wb @ T_bc
        o = T_wc[:3, 3]
        d = dirs_opt @ T_wc[:3, :3].T
        tt, pts = world.cast(o, d)
        depth = np.where(np.isfinite(tt), tt, 0.0)
        depth = depth * (1 + rng.normal(0, 0.004, depth.shape))
        depth_mm = np.clip(depth * 1000, 0, 65535).astype(np.uint16).reshape(H, W)
        depth_mm[depth_mm > 6000] = 0
        col = world.color(pts)
        col[~np.isfinite(tt)] = 0
        rgb = col.reshape(H, W, 3)[:, :, ::-1].copy()          # store BGR like the camera
        rgb = np.clip(rgb.astype(np.int16) + rng.integers(-6, 7, rgb.shape), 0, 255).astype(np.uint8)
        rec.add_frame(Frame(t, rgb, depth_mm, i + 1))
        # imu samples between frames
        yaw_rate = ((yaw - prev_yaw + math.pi) % (2 * math.pi) - math.pi) * fps
        prev_yaw = yaw
        samples = []
        while t_imu <= t + 1e-9:
            w_body = np.array([0.0, 0.0, yaw_rate])
            a_body = T_wb[:3, :3].T @ np.array([0.0, 0.0, G])
            samples.append(ImuSample(t_imu, R_ib @ w_body + rng.normal(0, 0.002, 3) + np.array([0.001, -0.002, 0.0015]),
                                     R_ib @ a_body + rng.normal(0, 0.02, 3)))
            t_imu += imu_dt
        rec.add_imu(samples)
        # lidar at 10 Hz
        if t >= next_scan_t:
            next_scan_t += 0.1
            T_wl = T_wb @ T_bl
            ol = T_wl[:3, 3]
            dl = np.stack([np.cos(bearing), np.sin(bearing), np.zeros_like(bearing)], axis=1) @ T_wl[:3, :3].T
            tl, _ = world.cast(ol, dl)
            r = np.where(np.isfinite(tl), tl, 0.0) * (1 + rng.normal(0, 0.003, tl.shape))
            r[np.abs(bearing) > math.pi / 2] = 0.06        # rear half blocked by the robot body
            rec.add_scan(LidarScan(t, lidar_raw.copy(), r, np.full_like(r, 200.0), 3600.0))
        if i % 50 == 0:
            print(f"[synth] frame {i}/{len(poses)}  ({time.time() - t0:.0f} s)")
    truth.close()
    rec.close()
    print(f"[synth] done: {rec.n_frames} frames, {rec.n_imu} imu samples, {rec.n_scans} scans -> {a.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
