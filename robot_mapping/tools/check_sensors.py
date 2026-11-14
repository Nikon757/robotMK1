#!/usr/bin/env python3
"""Sanity check of all three devices before mapping:  python run.py check

* OAK-D Lite : opens the pipeline, prints intrinsics, IMU type, USB speed, frame rate,
               depth statistics and the gravity vector as seen in the *body* frame
               (should be ~ [0, 0, +9.8] when the camera is level; if not -> calib-imu)
* LiDAR      : port, packet rate, scan rate, points per scan, range statistics,
               number of points inside the configured forward FOV
* STM32 board: telemetry rate; warns that encoders are always 0 (firmware limitation)
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from common.config import load_config  # noqa: E402
from common.transforms import rpy_to_R  # noqa: E402


def check_camera(cfg, seconds: float) -> bool:
    print("\n=== OAK-D Lite ===")
    try:
        from common.oakd import OakCamera, list_devices
    except Exception as e:  # noqa: BLE001
        print(f"  FAIL: cannot import depthai ({e})")
        return False
    devs = list_devices()
    print(f"  devices found: {devs if devs else 'NONE'}")
    if not devs:
        print("  -> check the USB3 cable (blue port), udev rule 03e7 in config/99-robot-usb.rules, and power.")
        return False
    try:
        cam = OakCamera(cfg)
    except Exception as e:  # noqa: BLE001
        print(f"  FAIL: cannot start the pipeline: {e}")
        return False
    info = cam.info()
    print(f"  mxid {info['mxid']}  usb {info['usb_speed']}  imu {info['imu']}  rotation-vector {info['rotation_vector']}")
    if info["usb_speed"] not in ("SUPER", "SUPER_PLUS"):
        print("  WARNING: not on USB3 (SUPER). Frame rate will be limited. Use a blue port + USB3 cable.")
    K = cam.K
    print(f"  intrinsics @ {cam.width}x{cam.height}: fx {K[0,0]:.1f} fy {K[1,1]:.1f} cx {K[0,2]:.1f} cy {K[1,2]:.1f}")
    if cam.T_cam_imu is not None:
        print("  EEPROM has an IMU->camera extrinsic (will be printed as rpy in calib-imu)")
    n = 0
    t0 = time.monotonic()
    depths = []
    acc = []
    gyr = []
    while time.monotonic() - t0 < seconds:
        f = cam.get_frame(1.0)
        if f is None:
            continue
        n += 1
        d = f.depth[f.depth > 0]
        if len(d):
            depths.append((len(d) / f.depth.size, np.median(d) / 1000.0))
        for s in cam.get_imu():
            acc.append(s.accel)
            gyr.append(s.gyro)
    dt = time.monotonic() - t0
    print(f"  frames: {n} in {dt:.1f} s = {n / max(dt, 1e-6):.1f} fps  (configured {cam.fps})")
    if depths:
        cov = np.mean([c for c, _ in depths])
        med = np.median([m for _, m in depths])
        print(f"  depth: {cov * 100:.0f}% valid pixels, median distance {med:.2f} m")
        if cov < 0.3:
            print("  WARNING: little valid depth - too close to a wall, too dark, or a textureless scene?")
    else:
        print("  WARNING: no depth data received")
    if acc:
        a = np.mean(acc, axis=0)
        g = np.mean(gyr, axis=0)
        R = rpy_to_R(*cfg.camera.imu_rpy_deg)
        ab = R @ a
        print(f"  imu: {len(acc)} samples in {dt:.1f} s = {len(acc) / dt:.0f} Hz")
        print(f"  accel raw (imu frame)  [{a[0]:+.2f} {a[1]:+.2f} {a[2]:+.2f}] m/s^2 |a| = {np.linalg.norm(a):.2f}")
        print(f"  accel in body frame    [{ab[0]:+.2f} {ab[1]:+.2f} {ab[2]:+.2f}]  <- expect ~[0, 0, +9.8] when level")
        print(f"  gyro bias (imu frame)  [{g[0]:+.4f} {g[1]:+.4f} {g[2]:+.4f}] rad/s  (robot must be still)")
        tilt = np.degrees(np.arccos(np.clip(ab[2] / max(np.linalg.norm(ab), 1e-6), -1, 1)))
        if tilt > 8:
            print(f"  WARNING: gravity is {tilt:.0f} deg off the body z-axis with the configured camera.imu_rpy_deg.")
            print("           Run  python run.py calib-imu  and/or check that the camera is level.")
        else:
            print(f"  imu orientation OK (gravity {tilt:.1f} deg from vertical)")
    else:
        print("  WARNING: no IMU samples (is this the IMU version of the OAK-D Lite?)")
    cam.close()
    return n > 0


def check_lidar(cfg, seconds: float) -> bool:
    print("\n=== LiDAR (LDROBOT) ===")
    try:
        from common.ldlidar import LDLidar, find_port, scan_to_points
    except Exception as e:  # noqa: BLE001
        print(f"  FAIL: {e}")
        return False
    port = find_port(str(cfg.serial.lidar_port))
    print(f"  port: {port or 'NOT FOUND'} (configured {cfg.serial.lidar_port}, baud {cfg.serial.lidar_baud})")
    if not port:
        print("  -> lsusb should show a Silicon Labs CP210x (10c4:ea60); check config/99-robot-usb.rules")
        return False
    try:
        ld = LDLidar(port, int(cfg.serial.lidar_baud))
    except Exception as e:  # noqa: BLE001
        print(f"  FAIL: {e}")
        return False
    t0 = time.monotonic()
    scans = 0
    pts_fov = []
    rng = []
    while time.monotonic() - t0 < seconds:
        s = ld.wait_scan(1.0)
        if s is None:
            continue
        scans += 1
        L = cfg.lidar
        xy, _, r = scan_to_points(s, L.forward_angle_deg, L.fov_deg, L.clockwise, L.min_range_m, L.max_range_m)
        pts_fov.append(len(xy))
        if len(r):
            rng.append((r.min(), np.median(r), r.max()))
    dt = time.monotonic() - t0
    print(f"  scans: {scans} in {dt:.1f} s = {scans / max(dt, 1e-6):.1f} Hz, packets {ld.packets}, bad CRC {ld.bad_crc}")
    last = ld.latest()
    if last is not None:
        print(f"  points per revolution: {len(last)}, motor speed {last.speed_dps:.0f} deg/s")
        print(f"  inside the forward {cfg.lidar.fov_deg:.0f} deg FOV: {int(np.mean(pts_fov)) if pts_fov else 0} points")
        if rng:
            r = np.array(rng)
            print(f"  range (FOV): min {r[:,0].min():.2f}  median {np.median(r[:,1]):.2f}  max {r[:,2].max():.2f} m")
    ld.close()
    return scans > 0


def check_mcu(cfg, seconds: float) -> bool:
    print("\n=== STM32 main board (Niumo V5) ===")
    try:
        from common.mcu import Mcu, find_port
    except Exception as e:  # noqa: BLE001
        print(f"  FAIL: {e}")
        return False
    port = find_port(str(cfg.serial.mcu_port))
    print(f"  port: {port or 'NOT FOUND'} (configured {cfg.serial.mcu_port})")
    if not port:
        print("  (not required for mapping)")
        return False
    try:
        m = Mcu(port, int(cfg.serial.mcu_baud))
    except Exception as e:  # noqa: BLE001
        print(f"  FAIL: {e}")
        return False
    time.sleep(seconds)
    t = m.latest()
    print(f"  telemetry: {m.telemetry_count} packets in {seconds:.0f} s = {m.telemetry_count / seconds:.0f} Hz (expect 50)")
    if t:
        print(f"  last: ms {t[1]}  encL {t[2]}  encR {t[3]}")
    if not m.encoders_alive:
        print("  note: encoders report 0 - expected with the current firmware (Encoder_Updata() never called);")
        print("        wheel odometry is not used by the mappers.")
    if m.errors:
        print(f"  errors: {m.errors[:3]}")
    m.center_servos()
    m.close()
    return m.telemetry_count > 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="RobotMK1 sensor check")
    ap.add_argument("--config", default=None)
    ap.add_argument("--seconds", type=float, default=4.0)
    ap.add_argument("--skip-camera", action="store_true")
    ap.add_argument("--skip-lidar", action="store_true")
    ap.add_argument("--skip-mcu", action="store_true")
    a = ap.parse_args(argv)
    cfg = load_config(a.config)
    print(f"config: {cfg.to_dict().get('_path')}")
    ok = {}
    if not a.skip_camera:
        ok["camera"] = check_camera(cfg, a.seconds)
    if not a.skip_lidar:
        ok["lidar"] = check_lidar(cfg, a.seconds)
    if not a.skip_mcu:
        ok["mcu"] = check_mcu(cfg, min(a.seconds, 3.0))
    print("\n=== summary ===")
    for k, v in ok.items():
        print(f"  {k:7s} {'OK' if v else 'FAIL / missing'}")
    return 0 if ok.get("camera", True) else 1


if __name__ == "__main__":
    sys.exit(main())
