#!/usr/bin/env python3
"""RobotMK1 mapping launcher.

    python run.py                       interactive menu
    python run.py rtabmap --out maps/lab --record --viz
    python run.py fusion  --out maps/lab --no-record --hailo --viz
    python run.py fusion  --replay maps/lab/raw --out maps/lab_replay
    python run.py check | lidar-view [--find-forward --apply] | calib-camera | calib-imu |
                  calib-lidar-camera [--apply] | export --ply ... | synth

Every sub-command forwards its remaining arguments to the underlying script
(``python run.py fusion --help`` shows that script's options).
"""
from __future__ import annotations

import os
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent

COMMANDS = {
    "rtabmap":            ("mappers/rtabmap/run_rtabmap.py",   "RTAB-Map SLAM (ROS 2 Jazzy in Docker): camera + LiDAR + IMU, loop closure"),
    "fusion":             ("mappers/fusion/fusion_mapper.py",  "Fusion mapper (pure Python, Hailo-8 optional): RGB-D + LiDAR + IMU"),
    "check":              ("tools/check_sensors.py",           "Check that the OAK-D, the LiDAR and the STM32 board respond"),
    "lidar-view":         ("tools/lidar_view.py",              "Live LiDAR view / measure lidar.forward_angle_deg (--find-forward)"),
    "calib-camera":       ("tools/calib_camera.py",            "Dump OAK-D calibration / depth check against a tape measure (--distance)"),
    "calib-imu":          ("tools/calib_camera.py",            "Two-pose IMU alignment -> camera.imu_rpy_deg"),
    "calib-lidar-camera": ("tools/calib_lidar_camera.py",      "Refine LiDAR<->camera extrinsics and the camera mount from a wall"),
    "export":             ("tools/export_map.py",              "Convert a map (.ply / .db) -> PLY/PCD + occupancy grid + preview"),
    "synth":              ("tools/make_synthetic_raw.py",      "Generate a synthetic raw recording for testing the fusion mapper"),
}
EXTRA_ARGS = {"calib-imu": ["--imu"]}


def run(cmd: str, args: list) -> int:
    script, _ = COMMANDS[cmd]
    argv = [sys.executable, str(ROOT / script)] + EXTRA_ARGS.get(cmd, []) + args
    env = dict(os.environ)
    env.setdefault("PYTHONUNBUFFERED", "1")
    try:
        return subprocess.call(argv, cwd=str(ROOT), env=env)
    except KeyboardInterrupt:
        return 130


def ask(prompt: str, default: str = "") -> str:
    s = input(f"{prompt}{f' [{default}]' if default else ''}: ").strip()
    return s or default


def yes(prompt: str, default: bool = True) -> bool:
    s = input(f"{prompt} [{'Y/n' if default else 'y/N'}]: ").strip().lower()
    if not s:
        return default
    return s in ("y", "yes")


def menu() -> int:
    order = ["rtabmap", "fusion", "check", "lidar-view", "calib-camera", "calib-imu", "calib-lidar-camera", "export", "synth"]
    print("\nRobotMK1 mapping\n" + "=" * 60)
    for i, k in enumerate(order, 1):
        print(f"  {i:2d}) {k:20s} {COMMANDS[k][1]}")
    print("   0) quit")
    choice = ask("select", "1")
    if choice in ("0", "q", ""):
        return 0
    try:
        cmd = order[int(choice) - 1]
    except (ValueError, IndexError):
        print("invalid choice")
        return 1
    args = []
    if cmd in ("rtabmap", "fusion"):
        default_out = f"maps/{time.strftime('%Y%m%d_%H%M%S')}"
        out = ask("output directory", default_out)
        record = yes("save the map on exit (map recording)", True)
        args += ["--out", out, "--record" if record else "--no-record"]
        if yes("show a live view window (needs the desktop)", True):
            args.append("--viz")
        if cmd == "fusion":
            if yes("use the Hailo-8 for dynamic-object removal / labels", False):
                args.append("--hailo")
            if record and yes("also record raw sensor data for later replay (--raw)", False):
                args.append("--raw")
        else:
            odom = ask("odometry: vo (visual) or icp (LiDAR)", "vo")
            args += ["--odom", odom]
            if record and yes("also record a rosbag of the raw topics (--bag)", False):
                args.append("--bag")
        if yes("hand-held / not on the floor (6-DoF, no planar constraint)", False):
            args.append("--6dof")
        print("\nhot-keys while mapping:  p = pause/resume map building   q = stop and save\n")
    elif cmd == "lidar-view":
        if yes("measure the forward angle (box straight ahead) instead of the live view", False):
            args.append("--find-forward")
            if yes("write the result into config/robot.yaml", True):
                args.append("--apply")
    elif cmd == "calib-camera":
        d = ask("known wall distance for a depth check in metres (empty = just dump the calibration)", "")
        if d:
            args += ["--distance", d]
    elif cmd in ("calib-imu", "calib-lidar-camera"):
        if yes("write the results into config/robot.yaml", False):
            args.append("--apply")
    elif cmd == "export":
        p = ask("input (.ply or .db)")
        args += ["--db", p] if p.endswith(".db") else ["--ply", p]
    elif cmd == "synth":
        args += ["--out", ask("output directory", "maps/synth/raw")]
    print(f"\n$ python run.py {cmd} {' '.join(args)}\n")
    return run(cmd, args)


def main(argv=None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    if not argv:
        return menu()
    cmd = argv[0]
    if cmd in ("-h", "--help", "help"):
        print(__doc__)
        for k, (_, d) in COMMANDS.items():
            print(f"  {k:20s} {d}")
        return 0
    if cmd not in COMMANDS:
        print(f"unknown command '{cmd}'. Known: {', '.join(COMMANDS)}")
        return 2
    return run(cmd, argv[1:])


if __name__ == "__main__":
    sys.exit(main())
