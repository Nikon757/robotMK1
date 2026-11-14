#!/usr/bin/env python3
"""OAK-D Lite calibration helpers.

    python run.py calib-camera                 # dump the factory calibration (EEPROM) to config/oak_calib.json
    python run.py calib-camera --distance 1.0  # compare depth at the image centre with a tape measure
    python run.py calib-imu                    # two-pose IMU alignment -> camera.imu_rpy_deg in robot.yaml

Depth check: point the camera squarely at a flat, textured wall at a known distance
(0.8-2 m works best), enter that distance, and the tool reports the median depth in a
central window and the error.  A systematic error > 3 % means the stereo calibration
is off -> recalibrate with Luxonis' calibrate.py (see CALIBRATION.md).

IMU alignment (needed because the IMU orientation differs between OAK-D Lite batches):
    pose A: camera level, looking forward, robot still      (~3 s)
    pose B: camera nose tilted DOWN by 20-60 deg, still     (~3 s)
From the two gravity vectors the tool computes the rotation IMU -> camera body and writes
camera.imu_rpy_deg + camera.imu_verified: true (with --apply).
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from common.config import load_config, save_config_values  # noqa: E402
from common.imu_filter import estimate_imu_to_body  # noqa: E402
from common.transforms import R_to_rpy, rpy_to_R  # noqa: E402


def dump_calibration(cfg, out_path: Path) -> dict:
    import depthai as dai
    with dai.Device() as dev:
        calib = dev.readCalibration()
        w, h = int(cfg.camera.width), int(cfg.camera.height)
        info = {"mxid": dev.getMxId(), "eeprom_version": None, "rgb": {}, "left": {}, "right": {}, "stereo": {}}
        try:
            info["imu"] = str(dev.getConnectedIMU())
        except Exception:
            pass
        for name, sock in (("rgb", dai.CameraBoardSocket.CAM_A), ("left", dai.CameraBoardSocket.CAM_B),
                           ("right", dai.CameraBoardSocket.CAM_C)):
            try:
                ww, hh = (w, h) if name == "rgb" else (640, 480)
                info[name]["K"] = calib.getCameraIntrinsics(sock, ww, hh)
                info[name]["size"] = [ww, hh]
                info[name]["distortion"] = list(calib.getDistortionCoefficients(sock))
                info[name]["fov_deg"] = calib.getFov(sock)
            except Exception as e:  # noqa: BLE001
                info[name]["error"] = str(e)
        try:
            info["stereo"]["baseline_cm"] = calib.getBaselineDistance()
            info["stereo"]["T_left_right"] = calib.getCameraExtrinsics(dai.CameraBoardSocket.CAM_B, dai.CameraBoardSocket.CAM_C)
            info["stereo"]["T_rgb_left"] = calib.getCameraExtrinsics(dai.CameraBoardSocket.CAM_A, dai.CameraBoardSocket.CAM_B)
        except Exception as e:  # noqa: BLE001
            info["stereo"]["error"] = str(e)
        try:
            ext = np.array(calib.getImuToCameraExtrinsics(dai.CameraBoardSocket.CAM_A), dtype=float)
            info["imu_to_rgb_extrinsic"] = ext.tolist()
            if not np.allclose(ext[:3, :3], 0):
                info["imu_to_rgb_rpy_deg"] = list(R_to_rpy(ext[:3, :3]))
        except Exception:
            info["imu_to_rgb_extrinsic"] = None
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(info, indent=2, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o)),
                        encoding="utf-8")
    return info


def depth_check(cfg, true_distance: float, seconds: float = 3.0) -> dict:
    from common.oakd import OakCamera
    with OakCamera(cfg) as cam:
        meds = []
        t0 = time.monotonic()
        while time.monotonic() - t0 < seconds:
            f = cam.get_frame(1.0)
            if f is None:
                continue
            h, w = f.depth.shape
            win = f.depth[h // 2 - 20:h // 2 + 20, w // 2 - 30:w // 2 + 30]
            v = win[win > 0]
            if len(v) > 100:
                meds.append(np.median(v) / 1000.0)
    if not meds:
        raise RuntimeError("no valid depth in the centre window - is there a textured surface in front?")
    med = float(np.median(meds))
    err = med - true_distance
    return {"measured_m": med, "true_m": true_distance, "error_m": err, "error_pct": 100.0 * err / true_distance,
            "samples": len(meds), "jitter_m": float(np.std(meds))}


def collect_accel(cam, seconds: float) -> np.ndarray:
    acc = []
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        cam.get_frame(0.2)
        for s in cam.get_imu():
            acc.append(s.accel)
        time.sleep(0.005)
    if len(acc) < 20:
        raise RuntimeError("no IMU samples received")
    a = np.array(acc)
    if np.std(np.linalg.norm(a, axis=1)) > 0.6:
        raise RuntimeError("the camera was moving during the measurement - keep it still and retry")
    return a.mean(axis=0)


def imu_alignment(cfg, apply: bool, config_path) -> dict:
    from common.oakd import OakCamera
    with OakCamera(cfg, enable_rotation_vector=False) as cam:
        print(f"IMU: {cam.imu_type}")
        input("Pose A - camera LEVEL, looking forward, robot still.  Press Enter to measure ...")
        a1 = collect_accel(cam, 3.0)
        print(f"   accel A = [{a1[0]:+.2f} {a1[1]:+.2f} {a1[2]:+.2f}]  |a| = {np.linalg.norm(a1):.2f}")
        input("Pose B - tilt the camera NOSE DOWN by 20-60 deg (pitch only, no roll/yaw), hold still.  Enter ...")
        a2 = collect_accel(cam, 3.0)
        print(f"   accel B = [{a2[0]:+.2f} {a2[1]:+.2f} {a2[2]:+.2f}]  |a| = {np.linalg.norm(a2):.2f}")
    R, rpy, ang = estimate_imu_to_body(a1, a2)
    if ang < 12:
        raise RuntimeError(f"only {ang:.1f} deg between the poses - tilt more (20-60 deg) and retry")
    ab = R @ a1
    res = {"imu_rpy_deg": [round(float(v), 2) for v in rpy], "tilt_between_poses_deg": round(float(ang), 1),
           "gravity_in_body_after": [round(float(v), 2) for v in ab]}
    print(f"\nIMU -> camera body rotation: roll {rpy[0]:.1f}  pitch {rpy[1]:.1f}  yaw {rpy[2]:.1f} deg "
          f"(tilt between poses {ang:.1f} deg)")
    print(f"gravity in body frame with this rotation: [{ab[0]:+.2f} {ab[1]:+.2f} {ab[2]:+.2f}]  (expect ~[0 0 +9.8])")
    old = rpy_to_R(*cfg.camera.imu_rpy_deg)
    from common.transforms import rotation_angle_deg
    print(f"difference to the configured camera.imu_rpy_deg {list(cfg.camera.imu_rpy_deg)}: "
          f"{rotation_angle_deg(old.T @ R):.1f} deg")
    if apply:
        p = save_config_values({"camera.imu_rpy_deg": res["imu_rpy_deg"], "camera.imu_verified": True}, config_path)
        print(f"written camera.imu_rpy_deg and camera.imu_verified: true to {p}")
    else:
        print("re-run with --apply to write it into config/robot.yaml")
    return res


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="OAK-D Lite calibration helpers")
    ap.add_argument("--config", default=None)
    ap.add_argument("--out", default=str(ROOT / "config" / "oak_calib.json"))
    ap.add_argument("--distance", type=float, default=None, help="depth check against a known wall distance (m)")
    ap.add_argument("--imu", action="store_true", help="two-pose IMU alignment")
    ap.add_argument("--apply", action="store_true", help="write results into robot.yaml (with --imu)")
    a = ap.parse_args(argv)
    cfg = load_config(a.config)
    if a.imu:
        imu_alignment(cfg, a.apply, a.config)
        return 0
    if a.distance is not None:
        r = depth_check(cfg, a.distance)
        print(f"depth at centre: {r['measured_m']:.3f} m  (true {r['true_m']:.3f} m)  error {r['error_m']*100:+.1f} cm "
              f"= {r['error_pct']:+.1f} %  jitter {r['jitter_m']*100:.1f} cm  ({r['samples']} frames)")
        if abs(r["error_pct"]) > 3:
            print("-> systematic error > 3 %: consider recalibrating the stereo pair (CALIBRATION.md, section 1.3)")
        else:
            print("-> depth calibration OK")
        return 0
    info = dump_calibration(cfg, Path(a.out))
    rgb = info.get("rgb", {})
    print(f"calibration written to {a.out}")
    if "K" in rgb:
        K = np.array(rgb["K"])
        print(f"rgb {rgb['size']}: fx {K[0,0]:.1f} fy {K[1,1]:.1f} cx {K[0,2]:.1f} cy {K[1,2]:.1f}  hfov {rgb.get('fov_deg', 0):.1f} deg")
    if "baseline_cm" in info.get("stereo", {}):
        print(f"stereo baseline: {info['stereo']['baseline_cm']:.2f} cm")
    if info.get("imu_to_rgb_rpy_deg"):
        print(f"EEPROM IMU->RGB rpy: {np.round(info['imu_to_rgb_rpy_deg'], 1)} (optical frame; calib-imu is still recommended)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
