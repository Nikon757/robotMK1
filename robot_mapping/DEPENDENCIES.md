# Dependencies

Target platform: Raspberry Pi 5 (8 GB), Raspberry Pi OS 64-bit **Trixie** (Debian 13, Python 3.13).
Bookworm (Python 3.11) works the same way. `install.sh` installs everything below.

## 1. System packages (apt)

| Package | Why |
|---|---|
| `python3 python3-venv python3-dev python3-pip` | the venv for the Python scripts |
| `libusb-1.0-0 udev usbutils` | OAK-D USB access, `lsusb`, udev rules |
| `libgl1 libglib2.0-0 libxcb-xinerama0 libxcb-cursor0 libxkbcommon-x11-0 libfontconfig1 libdbus-1-3` | runtime libs for the `opencv-python` (Qt GUI) and matplotlib windows |
| `x11-xserver-utils` | `xhost` (rtabmap_viz from Docker) |
| `git curl ca-certificates` | Docker install script, cloning |
| `hailo-all` (optional, `--hailo`) | Hailo-8 kernel driver, HailoRT runtime, `hailortcli`, Python bindings (`hailo_platform`) for the *system* Python — this is why the venv uses `--system-site-packages` |
| `docker-ce` (optional, `--docker`, via get.docker.com) | runs the ROS 2 / RTAB-Map stack |

udev: `config/99-robot-usb.rules` → `/etc/udev/rules.d/` (LiDAR CP210x `10c4:ea60` → `/dev/robot_lidar`,
Niumo board CH340 `1a86:7523` → `/dev/robot_mcu`, Luxonis `03e7` permissions). User must be in
`dialout` (serial) and `docker`.

## 2. Python packages (pip, `requirements.txt`)

| Package | Version | Used for |
|---|---|---|
| `numpy` | ≥ 1.26, < 3 | everything |
| `scipy` | ≥ 1.11 | cKDTree (ICP), sparse least-squares (pose graph) |
| `opencv-python` | ≥ 4.10, < 5 | ORB features, PnP RANSAC, image I/O, live windows (use `opencv-python-headless` without a desktop) |
| `depthai` | ≥ 2.30, < 3 | OAK-D Lite (DepthAI **v2** API; v3 changed the API and is not used) |
| `pyserial` | ≥ 3.5 | LiDAR and STM32 serial ports |
| `PyYAML` | ≥ 6 | `robot.yaml` |
| `matplotlib` | ≥ 3.7 | LiDAR live plot, `preview.png` |

All have `cp313` / `aarch64` wheels on PyPI (checked September 2026). **Open3D is deliberately
not used**: it has no aarch64 wheel for Python ≥ 3.12, so the fusion mapper implements its own
voxel map, ICP and pose-graph optimisation with numpy/scipy.

## 3. Docker image `robotmk1/rtabmap:jazzy` (`mappers/rtabmap/Dockerfile`)

Base `ros:jazzy-ros-base` (Ubuntu 24.04, arm64) plus apt packages from packages.ros.org:

| Package | Version (Sept 2026) | Role |
|---|---|---|
| `ros-jazzy-rtabmap-ros` | 0.23.x | rtabmap_slam, rtabmap_odom, rtabmap_sync, rtabmap_viz, `rtabmap-export` |
| `ros-jazzy-depthai-ros` | 2.12.x | `depthai_ros_driver` for the OAK-D Lite (RGB, aligned depth, IMU, TF/URDF) |
| `ros-jazzy-imu-filter-madgwick` | 2.1.x | orientation from the raw BMI270 accel/gyro |
| `ros-jazzy-nav2-map-server` | 1.3.x | `map_saver_cli` (occupancy grid) |
| `ros-jazzy-rmw-cyclonedds-cpp` | | DDS with good large-message behaviour |
| `ros-jazzy-rosbag2*` | | `--bag` recording (mcap) |
| `ldlidar_stl_ros2` | git master, built from source | LDROBOT LD06 / LD19 / **STL-27L** driver |

Image size ≈ 3 GB; build 20–40 min on the Pi 5. The launch file, parameters and `robot.yaml` are
bind-mounted (`/robot`, `/robot_config`), the output folder is mounted at `/maps`; the container runs
`--privileged --network host` with `/dev/bus/usb` and the LiDAR device passed through.

## 4. Hailo-8 (optional, fusion mapper `--hailo`)

* Runtime: `sudo apt install hailo-all` (Raspberry Pi repository), reboot, check
  `hailortcli fw-control identify`. Python API used: `hailo_platform` (HailoRT 4.x
  `VDevice / HEF / InferVStreams`).
* Model: a YOLOv8 detection `.hef` compiled for **Hailo-8** (26 TOPS, *not* the 8L variant of the
  AI Kit) with NMS post-processing on chip, e.g. `yolov8s.hef` from the Hailo Model Zoo
  (https://github.com/hailo-ai/hailo_model_zoo → "Object Detection" → Hailo-8 → yolov8s).
  Save as `models/yolov8s.hef` (`hailo.hef_path` in `robot.yaml`).
* Output format assumed: per class an `(N, 5)` array `[y_min, x_min, y_max, x_max, score]`
  (normalised) — the standard Hailo NMS output. If the model has no on-chip NMS the detector
  raises and the mapper continues without masking.

What the Hailo does in the pipeline: COCO classes listed in `hailo.dynamic_classes` (person, cat,
dog, …) are masked out of the depth image before integration, so moving people do not leave
ghosts in the map; classes in `hailo.label_classes` are written into the `label` property of
`map.ply` (COCO id + 1) for later semantic use. Detection runs every `hailo.every_n_frames` frames
(~5 Hz at 15 fps; the Hailo-8 itself does > 100 fps, the limit is the Pi's pre-processing).

## 5. Hardware / firmware assumptions and known limitations

* LiDAR: LDROBOT D800 kit = **STL-27L**, 921600 baud, 25 m, 21.6 k points/s, 10 Hz, the 47-byte
  `0x54 0x2C` packet format (CRC-8, poly 0x4D). LD19/LD06 work with `lidar.model` and
  `serial.lidar_baud: 230400`.
* OAK-D Lite "IMU version": BMI270 (raw accel/gyro, no on-chip fusion) on most units; BNO086 on some
  (rotation vector used automatically). USB 3 required for 15 fps at 640×360 + depth.
* Firmware v1 (`../firmware` now holds v2, which reports real counts; this package is not yet updated for it): the `#T <ms> <encL> <encR>` telemetry **always reports encoder counts
  0** because `Encoder_Updata()` is never called in `main.c`/`protocol.c` (only from the unused
  `Vehicle_Chassis.c`), and even then `Avg_PulseCount` is a 4-sample moving average of pulses per
  update, not a cumulative count. Wheel odometry is therefore unavailable and **not used**. Fix for
  later: call `Encoder_Updata()` from the 1 kHz `TIM6` tick (or every 10 ms) and report accumulated
  `PulseCount` sums; then feed `odom` to RTAB-Map (`rgbd_odometry guess_frame_id`) or to the fusion
  mapper as a translation prior.
* The camera pan/tilt servos are centred at start (`@R`) and must not be moved during mapping.
* The fusion mapper's pose graph is planar (SE(2)); with `--6dof` (hand-held) it runs odometry
  only, without graph optimisation.
