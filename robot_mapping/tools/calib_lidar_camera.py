#!/usr/bin/env python3
"""LiDAR <-> camera extrinsic refinement and camera-mount check (no targets needed).

    python run.py calib-lidar-camera            # measure and print suggested robot.yaml values
    python run.py calib-lidar-camera --apply    # ... and write them

Setup
    Put the robot 1.5-2.5 m in front of a flat wall, with something that breaks the
    symmetry inside the camera view: a vertical pole (broom handle) ~1 m ahead and
    ~0.5 m to one side, a box, or simply a room corner.  Robot still, camera centred.

What is measured
    1. Camera mount check: the floor plane is fitted in the depth image.  Its tilt gives the
       camera pitch/roll error, its height gives the camera z error (camera.mount.*).
    2. LiDAR mount refinement: depth points in a horizontal slice at the LiDAR's height are
       compared with the LiDAR scan by 2-D ICP.  The resulting planar correction is applied
       to lidar.mount.x / y / yaw.  (z / roll / pitch of the LiDAR stay as measured by hand.)

Expected accuracy: 1-3 cm and ~1 deg at 2 m (limited by stereo depth noise).
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from common.config import load_config, save_config_values  # noqa: E402
from common.ldlidar import scan_to_points  # noqa: E402
from common.pointcloud import backproject, voxel_downsample  # noqa: E402
from common.transforms import (T_from_mount, T_optical_from_mount, se2_from_T, T_from_se2, transform_points,  # noqa: E402
                               rotation_between, R_to_rpy, rpy_to_R)
from mappers.fusion.scan_match import icp_pair  # noqa: E402


def fit_plane_ransac(pts: np.ndarray, iters: int = 200, thr: float = 0.02, rng=None):
    """Returns (normal (3,), d, inlier_mask) with n.p + d = 0."""
    rng = rng or np.random.default_rng(0)
    best = (None, None, np.zeros(len(pts), bool))
    n_pts = len(pts)
    for _ in range(iters):
        i = rng.choice(n_pts, 3, replace=False)
        p0, p1, p2 = pts[i]
        n = np.cross(p1 - p0, p2 - p0)
        nn = np.linalg.norm(n)
        if nn < 1e-9:
            continue
        n /= nn
        d = -float(n @ p0)
        dist = np.abs(pts @ n + d)
        inl = dist < thr
        if inl.sum() > best[2].sum():
            best = (n, d, inl)
    n, d, inl = best
    if n is None or inl.sum() < 50:
        return None, None, inl
    # refine with all inliers (PCA)
    q = pts[inl]
    c = q.mean(axis=0)
    _, _, vt = np.linalg.svd(q - c, full_matrices=False)
    n = vt[2]
    if n[2] < 0:
        n = -n
    d = -float(n @ c)
    return n, d, np.abs(pts @ n + d) < thr


def capture(cfg, frames: int = 15):
    from common.sensors import LiveHub
    hub = LiveHub(cfg, use_lidar=True, use_mcu=True)
    if not hub.has_lidar:
        hub.close()
        raise RuntimeError("LiDAR not available")
    print(f"capturing {frames} frames + scans ... keep the robot still")
    depths, scans = [], []
    rgb = None
    while len(depths) < frames:
        f = hub.get_frame(1.0)
        if f is None:
            continue
        depths.append(f.depth.astype(np.float32))
        rgb = f.rgb
        s = hub.latest_scan()
        if s is not None and (not scans or s is not scans[-1]):
            scans.append(s)
    K = hub.K
    hub.close()
    D = np.stack(depths)
    D[D == 0] = np.nan
    depth = np.nanmedian(D, axis=0)
    depth = np.nan_to_num(depth, nan=0.0).astype(np.uint16)
    return depth, rgb, K, scans


def analyse(cfg, depth, K, scans, verbose=True) -> dict:
    T_bc = T_optical_from_mount(cfg.camera.mount)
    T_bl = T_from_mount(cfg.lidar.mount)
    L = cfg.lidar
    out = {}

    pts_c, pix = backproject(depth, K, stride=2, min_mm=250, max_mm=6000)
    pts_b = transform_points(T_bc, pts_c)
    h = depth.shape[0]

    # ---- 1. camera mount check from the floor plane (lower 45 % of the image)
    low = pix[:, 1] > 0.55 * h
    n, d, inl = fit_plane_ransac(pts_b[low]) if low.sum() > 200 else (None, None, None)
    if n is not None and inl.sum() > 300:
        tilt = math.degrees(math.acos(min(1.0, float(n[2]))))
        # rotation that would make the floor normal vertical, expressed as camera roll/pitch corrections
        R_fix = rotation_between(n, [0, 0, 1.0])
        R_cam_new = R_fix @ rpy_to_R(*[float(cfg.camera.mount.get(k, 0.0)) for k in ("roll", "pitch", "yaw")])
        roll, pitch, yaw = R_to_rpy(R_cam_new)
        cam_z_err = -d  # floor plane n.p + d = 0 -> floor height in base frame = -d (should be 0)
        out["floor"] = {"tilt_deg": tilt, "floor_height_m": float(-d), "inliers": int(inl.sum()),
                        "camera_roll_deg": roll, "camera_pitch_deg": pitch,
                        "camera_z_m": float(cfg.camera.mount.z) - float(cam_z_err)}
        if verbose:
            print(f"\n[floor] plane fitted with {inl.sum()} points: tilt {tilt:.2f} deg, floor at z = {-d:+.3f} m")
            print(f"        -> camera.mount.roll {roll:+.2f}  pitch {pitch:+.2f}  (currently "
                  f"{float(cfg.camera.mount.get('roll', 0)):+.2f} / {float(cfg.camera.mount.get('pitch', 0)):+.2f})")
            print(f"        -> camera.mount.z {out['floor']['camera_z_m']:.3f}  (currently {float(cfg.camera.mount.z):.3f})")
            if tilt > 1.0 or abs(d) > 0.03:
                print("        apply these first (the LiDAR slice below depends on them), then re-run")
    elif verbose:
        print("\n[floor] could not fit the floor plane - is the floor visible in the lower part of the image?")

    # ---- 2. LiDAR slice vs camera slice
    lz = float(L.mount.z)
    band = np.abs(pts_b[:, 2] - lz) < 0.04
    cam_slice = pts_b[band][:, :2]
    if len(cam_slice) < 100:
        raise RuntimeError(f"only {len(cam_slice)} depth points at the LiDAR height {lz:.2f} m - move closer to the wall / "
                           "check camera.mount.z and lidar.mount.z")
    lid = []
    for s in scans:
        xy, _, _ = scan_to_points(s, L.forward_angle_deg, L.fov_deg, L.clockwise, L.min_range_m, 6.0)
        lid.append(xy)
    lid = np.concatenate(lid)
    lid_b = lid @ T_bl[:2, :2].T + T_bl[:2, 3]
    # keep only LiDAR points inside the camera's horizontal field of view (else ICP fitness is meaningless)
    hfov = 2.0 * math.atan(depth.shape[1] / (2.0 * K[0, 0]))
    cam_xy = T_bc[:2, 3]
    bearing = np.arctan2(lid_b[:, 1] - cam_xy[1], lid_b[:, 0] - cam_xy[0])
    lid_b = lid_b[np.abs(bearing) < hfov / 2 - math.radians(3)]
    cam_ds, _, _ = voxel_downsample(np.column_stack([cam_slice, np.zeros(len(cam_slice))]), 0.02)
    lid_ds, _, _ = voxel_downsample(np.column_stack([lid_b, np.zeros(len(lid_b))]), 0.02)
    res = icp_pair(lid_ds[:, :2], cam_ds[:, :2], init=[0.0, 0.0, 0.0], voxel=0.02, max_corr=0.3, iters=60,
                   min_fitness=0.4)
    T_corr = T_from_se2(res.pose)
    T_new = T_corr @ T_bl
    p_new = se2_from_T(T_new)
    out["lidar"] = {"fitness": res.fitness, "rmse_m": res.rmse, "ok": bool(res.ok),
                    "correction": {"dx_m": float(res.pose[0]), "dy_m": float(res.pose[1]),
                                   "dyaw_deg": math.degrees(float(res.pose[2]))},
                    "mount_x": float(p_new[0]), "mount_y": float(p_new[1]),
                    "mount_yaw_deg": math.degrees(float(p_new[2])),
                    "cam_points": int(len(cam_ds)), "lidar_points": int(len(lid_ds))}
    if verbose:
        print(f"\n[lidar] {len(lid_ds)} LiDAR pts vs {len(cam_ds)} camera pts at z={lz:.2f} m: ICP fitness "
              f"{res.fitness:.2f}, rmse {res.rmse * 100:.1f} cm")
        print(f"        correction: dx {res.pose[0] * 100:+.1f} cm  dy {res.pose[1] * 100:+.1f} cm  "
              f"dyaw {math.degrees(res.pose[2]):+.2f} deg")
        print(f"        -> lidar.mount.x {p_new[0]:.3f}  y {p_new[1]:.3f}  yaw {math.degrees(p_new[2]):+.2f}   "
              f"(currently {float(L.mount.x):.3f} / {float(L.mount.y):.3f} / {float(L.mount.get('yaw', 0)):+.2f})")
        if not res.ok:
            print("        WARNING: low overlap - the LiDAR and the camera slice do not see the same things. "
                  "Check forward_angle_deg (lidar-view --find-forward) and the mount heights.")
        elif abs(res.pose[0]) > 0.3 or abs(res.pose[1]) > 0.3 or abs(res.pose[2]) > math.radians(20):
            print("        WARNING: large correction - re-measure the mounts with a tape measure first.")
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="LiDAR<->camera extrinsic refinement")
    ap.add_argument("--config", default=None)
    ap.add_argument("--frames", type=int, default=15)
    ap.add_argument("--apply", action="store_true", help="write lidar.mount.x/y/yaw (and camera roll/pitch/z) into robot.yaml")
    ap.add_argument("--apply-camera", action="store_true", help="with --apply: also write the camera mount corrections")
    ap.add_argument("--save-capture", default=None, help="save depth/scans to an .npz for offline analysis")
    ap.add_argument("--load-capture", default=None, help="analyse a saved .npz instead of live sensors")
    a = ap.parse_args(argv)
    cfg = load_config(a.config)
    if a.load_capture:
        from common.ldlidar import LidarScan
        z = np.load(a.load_capture, allow_pickle=True)
        depth, K = z["depth"], z["K"]
        scans = [LidarScan(float(t), ang, r, i, 0.0) for t, ang, r, i in
                 zip(z["scan_t"], z["scan_angles"], z["scan_ranges"], z["scan_intens"])]
    else:
        depth, rgb, K, scans = capture(cfg, a.frames)
        if a.save_capture:
            np.savez_compressed(a.save_capture, depth=depth, K=K, scan_t=[s.t for s in scans],
                                scan_angles=[s.angles_deg for s in scans], scan_ranges=[s.ranges_m for s in scans],
                                scan_intens=[s.intensities for s in scans])
    res = analyse(cfg, depth, K, scans)
    if a.apply:
        upd = {"lidar.mount.x": round(res["lidar"]["mount_x"], 3), "lidar.mount.y": round(res["lidar"]["mount_y"], 3),
               "lidar.mount.yaw": round(res["lidar"]["mount_yaw_deg"], 2)}
        if a.apply_camera and "floor" in res:
            upd.update({"camera.mount.roll": round(res["floor"]["camera_roll_deg"], 2),
                        "camera.mount.pitch": round(res["floor"]["camera_pitch_deg"], 2),
                        "camera.mount.z": round(res["floor"]["camera_z_m"], 3)})
        p = save_config_values(upd, a.config)
        print(f"\nwritten to {p}: {upd}")
    else:
        print("\nre-run with --apply (and --apply-camera) to write the values into config/robot.yaml")
    return 0


if __name__ == "__main__":
    sys.exit(main())
