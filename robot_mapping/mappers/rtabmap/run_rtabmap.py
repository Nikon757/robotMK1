#!/usr/bin/env python3
"""Host-side runner for the RTAB-Map stack (ROS 2 Jazzy inside Docker) on the Raspberry Pi.

    python mappers/rtabmap/run_rtabmap.py --out maps/room1 --record [--viz] [--bag] [--odom vo|icp]
    python mappers/rtabmap/run_rtabmap.py --build            # build the Docker image (once, ~30 min)
    python mappers/rtabmap/run_rtabmap.py --shell            # interactive shell in the container (debug)

What it does
    1. checks docker, the image, the USB devices (OAK via /dev/bus/usb, LiDAR serial port)
    2. runs the container with the launch file  mappers/rtabmap/mapping.launch.py
       (this folder and config/ are bind-mounted, so edits need no rebuild)
    3. hot-keys while mapping:   p = pause / resume map building   q = stop
    4. on stop:  Nav2 map_saver -> graceful shutdown (RTAB-Map writes rtabmap.db) ->
       rtabmap-export (camera cloud + LiDAR cloud + poses) -> occupancy grid + preview PNG
Outputs (--record):  <out>/rtabmap.db  cloud.ply  scan.ply  *poses*.txt  occupancy.pgm/.yaml  preview.png
"""
from __future__ import annotations

import argparse
import glob
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from common.config import load_config  # noqa: E402

HERE = Path(__file__).resolve().parent
IMAGE = "robotmk1/rtabmap:jazzy"
CONTAINER = "robotmk1_rtabmap"


def sh(cmd, check=True, capture=False, timeout=None, **kw):
    if capture:
        return subprocess.run(cmd, check=check, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                              timeout=timeout, **kw)
    return subprocess.run(cmd, check=check, timeout=timeout, **kw)


def docker_ok() -> bool:
    if shutil.which("docker") is None:
        print("docker is not installed. Run  ./install.sh --docker   (or see SETUP_AND_RUN.md)")
        return False
    r = sh(["docker", "info"], check=False, capture=True)
    if r.returncode != 0:
        print("docker is installed but not usable by this user:\n" + r.stdout.strip().splitlines()[-1])
        print("-> sudo usermod -aG docker $USER  and log out / in, or check that the docker service is running")
        return False
    return True


def image_exists(image: str) -> bool:
    return sh(["docker", "image", "inspect", image], check=False, capture=True).returncode == 0


def build_image(image: str, no_cache: bool = False) -> int:
    cmd = ["docker", "build", "-t", image] + (["--no-cache"] if no_cache else []) + [str(HERE)]
    print("[build] " + " ".join(cmd))
    print("[build] this takes 20-40 minutes on a Pi 5 the first time ...")
    return sh(cmd, check=False).returncode


def find_lidar_port(cfg) -> str | None:
    from common.ldlidar import find_port
    return find_port(str(cfg.serial.lidar_port))


def oak_present() -> bool:
    r = sh(["lsusb"], check=False, capture=True)
    return r.returncode == 0 and "03e7:" in r.stdout


def base_docker_args(cfg, out: Path | None, viz: bool, lidar_port: str | None, extra_env=None) -> list:
    args = ["docker", "run", "--rm", "--name", CONTAINER, "--privileged", "--network", "host", "--ipc", "host",
            "-v", "/dev/bus/usb:/dev/bus/usb",
            "-v", f"{HERE}:/robot:ro",
            "-v", f"{ROOT / 'config'}:/robot_config:ro",
            "-e", "ROBOT_CONFIG=/robot_config/robot.yaml",
            "-e", f"ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '0')}"]
    if lidar_port:
        args += ["--device", f"{os.path.realpath(lidar_port)}:/dev/robot_lidar"]
    if out is not None:
        args += ["-v", f"{out}:/maps"]
    if viz:
        args += ["-e", f"DISPLAY={os.environ.get('DISPLAY', ':0')}", "-e", "QT_X11_NO_MITSHM=1",
                 "-v", "/tmp/.X11-unix:/tmp/.X11-unix"]
        if os.environ.get("XAUTHORITY"):
            args += ["-e", f"XAUTHORITY={os.environ['XAUTHORITY']}", "-v", f"{os.environ['XAUTHORITY']}:{os.environ['XAUTHORITY']}:ro"]
    for k, v in (extra_env or {}).items():
        args += ["-e", f"{k}={v}"]
    return args


def ros_in_container(args: list, timeout: float = 30.0):
    """docker exec <container> /entrypoint.sh <args>"""
    return sh(["docker", "exec", CONTAINER, "/entrypoint.sh"] + args, check=False, capture=True, timeout=timeout)


def export_outputs(image: str, out: Path, cfg) -> list:
    """After the container stopped: rtabmap-export + occupancy grid + preview."""
    files = []
    db = out / "rtabmap.db"
    if not db.exists():
        print(f"[export] no database at {db} - nothing to export")
        return files
    base = ["docker", "run", "--rm", "-v", f"{out}:/maps", image]
    jobs = [
        ("camera cloud", ["rtabmap-export", "--output", "cloud", "--output_dir", "/maps", "--voxel", "0.02",
                          "--max_range", "6", "--decimation", "2", "--poses", "/maps/rtabmap.db"]),
        ("lidar cloud", ["rtabmap-export", "--scan", "--output", "scan", "--output_dir", "/maps", "/maps/rtabmap.db"]),
    ]
    for label, cmd in jobs:
        print(f"[export] {label}: {' '.join(cmd)}")
        r = sh(base + cmd, check=False, capture=True, timeout=900)
        tail = "\n".join(r.stdout.strip().splitlines()[-3:])
        print("   " + tail.replace("\n", "\n   "))
        if r.returncode != 0:
            print(f"[export] WARNING: {label} export failed (see above). You can export manually with "
                  f"rtabmap-databaseViewer or rtabmap-export inside the container (--shell).")
    files += [Path(p) for p in glob.glob(str(out / "*.ply"))]
    files += [Path(p) for p in glob.glob(str(out / "*poses*.txt"))]

    # occupancy grid + preview from the exported clouds (pure python, no ROS needed)
    try:
        import numpy as np
        from common.occupancy import grid_from_cloud, render_preview
        from common.pointcloud import read_ply
        cam = [p for p in files if p.suffix == ".ply" and p.name.startswith("cloud")]
        scan = [p for p in files if p.suffix == ".ply" and p.name.startswith("scan")]
        xyz = np.zeros((0, 3), np.float32)
        rgb = None
        if cam:
            d = read_ply(cam[0])
            xyz, rgb = d["xyz"], d.get("rgb")
        m = cfg.mapping
        parts = [xyz]
        if scan:
            s = read_ply(scan[0])["xyz"]
            # LiDAR points are at the sensor height: force them into the obstacle band
            s = s.copy()
            s[:, 2] = 0.5 * (float(m.ground_z_m) + float(m.obstacle_max_z_m))
            parts.append(s)
        allpts = np.concatenate(parts) if parts else xyz
        poses = None
        pf = [p for p in files if p.suffix == ".txt"]
        if pf:
            try:
                arr = np.loadtxt(pf[0], comments="#", ndmin=2)
                poses = arr[:, 1:4] if arr.shape[1] >= 4 else None
            except Exception:
                poses = None
        if len(allpts):
            grid = grid_from_cloud(allpts, float(m.grid_resolution_m), float(m.ground_z_m), float(m.obstacle_max_z_m),
                                   poses_xy=poses)
            pgm, yml = grid.save(out / "occupancy")
            files += [pgm, yml]
            files.append(render_preview(out / "preview.png", xyz, rgb, grid, poses, title=out.name))
    except Exception as e:  # noqa: BLE001
        print(f"[export] occupancy/preview generation failed: {e}")
    return files


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="RTAB-Map (ROS 2 in Docker) mapping runner")
    ap.add_argument("--config", default=None)
    ap.add_argument("--out", default=None, help="output directory (default maps/<timestamp>)")
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--record", dest="record", action="store_true", default=True)
    g.add_argument("--no-record", dest="record", action="store_false", help="run everything, save nothing")
    ap.add_argument("--viz", action="store_true", help="open rtabmap_viz (X11 display on the Pi)")
    ap.add_argument("--bag", action="store_true", help="also record a rosbag of the raw topics into <out>/bag")
    ap.add_argument("--odom", choices=["vo", "icp"], default="vo", help="visual (default) or LiDAR ICP odometry")
    ap.add_argument("--6dof", dest="six_dof", action="store_true", help="disable the planar constraint (hand-held)")
    ap.add_argument("--use-imu", action="store_true", help="gravity constraints (only after calib-imu)")
    ap.add_argument("--localization", action="store_true", help="reuse <out>/rtabmap.db without adding nodes")
    ap.add_argument("--image", default=IMAGE)
    ap.add_argument("--build", action="store_true", help="build the Docker image and exit")
    ap.add_argument("--no-cache", action="store_true")
    ap.add_argument("--shell", action="store_true", help="open a bash shell in the container and exit")
    ap.add_argument("--export-only", action="store_true", help="only run the export step on <out>/rtabmap.db")
    ap.add_argument("--lidar-port", default=None)
    a = ap.parse_args(argv)

    cfg = load_config(a.config)
    if not docker_ok():
        return 2
    if a.build:
        return build_image(a.image, a.no_cache)
    if not image_exists(a.image):
        print(f"[docker] image {a.image} not found.")
        ans = input("Build it now? (20-40 min) [Y/n] ").strip().lower()
        if ans in ("", "y", "yes"):
            if build_image(a.image, a.no_cache) != 0:
                return 1
        else:
            return 1

    out = Path(a.out).resolve() if a.out else (ROOT / "maps" / time.strftime("%Y%m%d_%H%M%S")).resolve()
    if a.export_only:
        for f in export_outputs(a.image, out, cfg):
            print("  ", f)
        return 0

    lidar_port = a.lidar_port or find_lidar_port(cfg)
    if not lidar_port:
        print("[devices] WARNING: LiDAR serial port not found (no /dev/robot_lidar, no CP210x/CH340 tty). "
              "Mapping will run with the camera only; the LiDAR node will complain.")
    else:
        print(f"[devices] LiDAR: {lidar_port}")
    if not oak_present():
        print("[devices] WARNING: no Luxonis device (03e7:xxxx) in lsusb - check the OAK-D cable/port")

    if a.shell:
        cmd = base_docker_args(cfg, out if a.record else None, a.viz, lidar_port) + ["-it", a.image, "bash"]
        return sh(cmd, check=False).returncode

    use_imu = a.use_imu or bool(cfg.camera.get("imu_verified", False))
    force_3dof = bool(cfg.mapping.force_3dof) and not a.six_dof
    if a.record:
        out.mkdir(parents=True, exist_ok=True)
    launch = ["ros2", "launch", "/robot/mapping.launch.py",
              f"db_path:={'/maps/rtabmap.db' if a.record else '/tmp/rtabmap.db'}",
              f"delete_db:={'false' if a.localization else 'true'}",
              f"viz:={'true' if a.viz else 'false'}", f"odom:={a.odom}",
              f"force_3dof:={'true' if force_3dof else 'false'}", f"use_imu:={'true' if use_imu else 'false'}",
              f"bag:={'true' if (a.bag and a.record) else 'false'}", "bag_dir:=/maps/bag",
              f"localization:={'true' if a.localization else 'false'}", "lidar_port:=/dev/robot_lidar"]
    if a.viz:
        sh(["xhost", "+local:"], check=False, capture=True)
    cmd = base_docker_args(cfg, out if a.record else None, a.viz, lidar_port) + [a.image] + launch
    print("[docker] " + " ".join(cmd))
    print(f"[run] record={'ON -> ' + str(out) if a.record else 'OFF'}  odom={a.odom}  3dof={force_3dof}  "
          f"imu={use_imu}  viz={a.viz}\n[run] hot-keys:  p = pause/resume map building   q = stop and save\n")

    sh(["docker", "rm", "-f", CONTAINER], check=False, capture=True)
    proc = subprocess.Popen(cmd)
    from common.keys import KeyListener
    keys = KeyListener()
    paused = False
    try:
        while proc.poll() is None:
            for k in keys.pop():
                if k == "q":
                    raise KeyboardInterrupt
                if k == "p":
                    paused = not paused
                    srv = "/rtabmap/pause" if paused else "/rtabmap/resume"
                    print(f"[keys] {'PAUSE' if paused else 'RESUME'} -> ros2 service call {srv}")
                    r = ros_in_container(["ros2", "service", "call", srv, "std_srvs/srv/Empty"], timeout=20)
                    if r.returncode != 0:
                        print("   " + r.stdout.strip().splitlines()[-1] if r.stdout.strip() else "   (no output)")
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("\n[stop] shutting down ...")
    finally:
        keys.stop()
        if proc.poll() is None:
            if a.record:
                print("[stop] saving the occupancy grid with nav2 map_saver (up to 40 s) ...")
                r = ros_in_container(["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", "/maps/occupancy_rtabmap",
                                      "--ros-args", "-r", "map:=/rtabmap/map", "-p", "map_subscribe_transient_local:=true"],
                                     timeout=45)
                print("   " + (r.stdout.strip().splitlines()[-1] if r.stdout.strip() else "(no output)"))
            print("[stop] stopping the container (RTAB-Map writes its database on shutdown) ...")
            sh(["docker", "stop", "-t", "40", CONTAINER], check=False, capture=True)
            try:
                proc.wait(timeout=60)
            except subprocess.TimeoutExpired:
                proc.kill()
    if a.record:
        print("[export] exporting point clouds from the database ...")
        files = export_outputs(a.image, out, cfg)
        print("[export] outputs in " + str(out) + ":\n  " + "\n  ".join(p.name for p in sorted(set(files))))
    else:
        print("[export] --no-record: nothing saved")
    return 0


if __name__ == "__main__":
    sys.exit(main())
