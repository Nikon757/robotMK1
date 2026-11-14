# robot_mapping — point-cloud mapping for RobotMK1

Mapping software for the RobotMK1 platform: Raspberry Pi 5 (8 GB, Raspberry Pi OS) with an
**OAK-D Lite** (RGB-D + IMU), an **LDROBOT D800 / STL-27L** 2-D LiDAR and a **Hailo-8**
accelerator, on the Niumo V5 Ackermann chassis (firmware in `../firmware`).

It produces a **3-D point-cloud map** (`.ply`) plus a **2-D occupancy grid** (Nav2 `.pgm/.yaml`)
that the robot can later use for localisation and navigation.

Two mappers are included; both are started from the same console launcher:

| Mapper | What it is | Needs |
|---|---|---|
| `rtabmap` | RTAB-Map graph SLAM (ROS 2 Jazzy in Docker): RGB-D visual odometry, LiDAR-refined links, loop closure, optimisation, occupancy grid. The reference solution — use it for the real maps. | Docker image (built once) |
| `fusion` | Pure-Python RGB-D + LiDAR + IMU SLAM (numpy / scipy / OpenCV): ORB-PnP odometry primed by the IMU, 2-D scan ICP, voxel map, loop closure with SE(2) pose-graph optimisation, **Hailo-8 YOLOv8** removes people/animals from the map and labels objects. Lighter, fully hackable, no ROS. | nothing beyond `install.sh` (Hailo optional) |

## Contents

```
run.py                   console launcher: choose the script, the output folder, map recording on/off
install.sh               one-shot installer for the Pi (apt, venv, udev, optional Docker + Hailo)
requirements.txt         pip dependencies
GUIDE_RU.md              пошаговый гайд по запуску на роботе (по-русски)
SETUP_AND_RUN.md         installation and start-up instructions, outputs, troubleshooting
CALIBRATION.md           camera / IMU / LiDAR calibration, LiDAR<->camera extrinsics, the 180 deg FOV
DEPENDENCIES.md          every apt / pip / Docker / Hailo dependency and version, known limitations

config/robot.yaml        ports, sensor mounts (positions/angles), LiDAR forward angle + FOV, mapping params
config/99-robot-usb.rules   udev rules: /dev/robot_lidar, /dev/robot_mcu, OAK permissions

common/                  shared code
  ldlidar.py             STL-27L / LD19 serial driver (CRC-8, scan assembly, forward-FOV crop)
  oakd.py                OAK-D Lite: synchronised RGB + aligned depth + IMU (DepthAI v2)
  imu_filter.py          gyro bias, gravity alignment, Madgwick orientation
  mcu.py                 STM32 board link (servo centring, telemetry)
  sensors.py / recorder.py   live sensor hub and raw recording / replay
  pointcloud.py          back-projection, voxel map, PLY/PCD I/O
  occupancy.py           occupancy grid (Nav2 format) + preview image
  transforms.py, config.py, keys.py

mappers/rtabmap/         RTAB-Map stack
  Dockerfile, entrypoint.sh       ROS 2 Jazzy + rtabmap_ros + depthai_ros + ldlidar driver
  mapping.launch.py               all ROS nodes, TF tree, parameters from robot.yaml
  scan_fov_filter.py              keeps only the forward 180 deg of the LiDAR scan
  params/oak.yaml, params/rtabmap.yaml
  run_rtabmap.py                  host-side runner (docker run, hot-keys, export on exit)
mappers/fusion/          pure-Python mapper
  fusion_mapper.py                main loop, keyframes, map, export, live view
  vo.py, scan_match.py, pose_graph.py, hailo_detector.py

tools/
  check_sensors.py       does everything respond?  (python run.py check)
  lidar_view.py          live LiDAR plot; measures lidar.forward_angle_deg
  calib_camera.py        OAK calibration dump, depth check, IMU alignment (calib-imu)
  calib_lidar_camera.py  LiDAR<->camera extrinsics + camera mount from a wall
  export_map.py          .ply / .db -> PLY, PCD, occupancy grid, preview
  make_synthetic_raw.py  synthetic recording to test the fusion mapper without hardware

maps/                    output folder (one sub-folder per run)
models/                  put the Hailo-8 YOLOv8 .hef here
```

## Quick start

```bash
./install.sh --all                  # once; then log out / in
source .venv/bin/activate
python run.py check                 # all three devices respond?
python run.py lidar-view --find-forward --apply    # LiDAR forward angle (box in front)
python run.py calib-imu --apply     # IMU orientation (two poses)
python run.py                       # interactive menu -> RTAB-Map or fusion mapper
```

Outputs land in `maps/<name>/`: `map.ply` / `cloud.ply` (3-D map), `occupancy.pgm` + `.yaml`
(Nav2), `preview.png`, `poses.txt`, and for RTAB-Map the `rtabmap.db` database.
See **SETUP_AND_RUN.md** for details and **CALIBRATION.md** before the first real map.
