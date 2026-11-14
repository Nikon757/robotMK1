# Setup and start-up instructions

Target: Raspberry Pi 5 (8 GB), Raspberry Pi OS 64-bit (Trixie, Python 3.13; Bookworm also works),
desktop attached or SSH with X forwarding. All commands are run inside `robot_mapping/` on the Pi.

## 1. Copy the folder to the Pi

```bash
# from the PC (PowerShell / Git Bash), adjust user@host
scp -r robot_mapping pi@robot.local:~/
ssh pi@robot.local
cd ~/robot_mapping
```

## 2. Install

```bash
chmod +x install.sh
./install.sh --all          # apt libs, udev rules, python venv, Docker + RTAB-Map image, Hailo runtime
```

`--all` takes 30–60 minutes (the Docker image build is the long part). Smaller variants:

| Command | Installs |
|---|---|
| `./install.sh` | apt libs, udev rules, `.venv` with the Python packages → fusion mapper + all tools |
| `./install.sh --docker --build-image` | … + Docker + the `robotmk1/rtabmap:jazzy` image (RTAB-Map) |
| `./install.sh --hailo` | … + `hailo-all` (Hailo-8 runtime; reboot afterwards) |

Then **log out and back in** (group membership for `dialout`, `docker`) and

```bash
source .venv/bin/activate
```

Hailo model: download a YOLOv8 `.hef` compiled for **Hailo-8** (not Hailo-8L) from the Hailo Model
Zoo (`yolov8s.hef`) and put it at `models/yolov8s.hef` (path in `config/robot.yaml → hailo.hef_path`).
See DEPENDENCIES.md for the link.

## 3. Plug in the sensors

* OAK-D Lite → a **blue USB 3** port with a USB 3 cable (USB 2 halves the frame rate).
* LiDAR USB adapter and the Niumo board (Type-C) → any USB port.
* Check the udev symlinks and the camera:

```bash
ls -l /dev/robot_lidar /dev/robot_mcu      # both should exist
lsusb | grep -i -E "03e7|10c4|1a86"        # OAK (03e7), LiDAR bridge (10c4), STM32 board (1a86)
python run.py check
```

`check` prints the camera intrinsics, IMU type (BMI270 or BNO086), USB speed, frame rate, depth
statistics, the gravity vector in the robot frame, LiDAR scan rate / points in the forward FOV,
and the STM32 telemetry rate. Fix anything marked WARNING/FAIL before mapping.

If a symlink is missing, find the real IDs with `lsusb` and edit `config/99-robot-usb.rules`,
then `sudo udevadm control --reload-rules && sudo udevadm trigger`. The scripts also fall back
to auto-detecting `/dev/ttyUSB*` by USB vendor ID.

## 4. Calibrate (first time, and after moving a sensor)

Follow **CALIBRATION.md**. Minimum before the first map:

```bash
python run.py lidar-view --find-forward --apply    # which raw LiDAR angle points forward
python run.py calib-imu --apply                    # IMU orientation (two poses, 1 minute)
python run.py calib-lidar-camera --apply           # LiDAR<->camera offsets from a wall (optional but recommended)
```

and measure the sensor positions in `config/robot.yaml` (`camera.mount`, `lidar.mount`) with a ruler.

## 5. Run a mapping session

### Interactive

```bash
python run.py
```

The menu asks which mapper to run, where to save (`maps/<timestamp>` by default), whether to
**record the map** (yes/no), and mapper-specific options (live view, Hailo, raw recording, odometry).

### Command line

```bash
# RTAB-Map (Docker).  First start pulls nothing but takes ~20 s to boot the camera.
python run.py rtabmap --out maps/lab --record --viz
python run.py rtabmap --out maps/lab --no-record          # dry run, nothing written
python run.py rtabmap --out maps/lab --record --odom icp  # LiDAR ICP odometry instead of visual
python run.py rtabmap --out maps/lab --record --bag       # also record a rosbag (mcap) of the raw topics

# Fusion mapper (pure Python)
python run.py fusion --out maps/lab --record --viz --hailo
python run.py fusion --out maps/lab --record --raw        # also record raw sensor data for replay
python run.py fusion --replay maps/lab/raw --out maps/lab_replay --record   # offline re-run (also works on a PC)
```

Common flags: `--record` / `--no-record` (map recording on/off), `--out DIR`, `--viz` (live
window on the desktop), `--6dof` (hand-held, not on the floor — disables the planar
constraint), `--config FILE`.

### While mapping

* `p` — pause / resume map building (tracking continues, nothing is added to the map).
  In the RTAB-Map runner this calls the `/rtabmap/pause` and `/rtabmap/resume` services.
* `q` or `Ctrl+C` — stop. The map is finalised and exported (if recording).
* Status line (fusion): fps, pose, VO inliers, ICP fitness (`*` = LiDAR correction applied),
  keyframes, loop closures, voxels, detections, PAUSED/REC.

The robot does not need to drive: for a first map carry it slowly around the room, or rotate it in
place. Keep the camera **centred** (the launcher sends `@R` to the board); do not use the pan/tilt
servos while mapping — the extrinsics assume the camera does not move relative to the chassis.
Move slowly (≤ 0.3 m/s, ≤ 30 deg/s), keep textured surfaces in view, avoid pointing at blank walls
closer than 30 cm, and finish by coming back to the start looking in the start direction (this
gives the loop closure a chance and removes drift).

## 6. Outputs

`maps/<name>/`

| File | Produced by | Meaning |
|---|---|---|
| `rtabmap.db` | rtabmap | full RTAB-Map database (images, scans, graph) — keep it; it is the map for later localisation with `--localization` |
| `cloud.ply` | rtabmap | coloured 3-D point cloud from the camera (`rtabmap-export`) |
| `scan.ply` | rtabmap | LiDAR points assembled with the optimised poses |
| `occupancy_rtabmap.pgm/.yaml` | rtabmap | RTAB-Map's own occupancy grid (Nav2 map_saver) |
| `map.ply` | fusion | coloured 3-D voxel map; `label` property = COCO class id + 1 (0 = none) |
| `lidar_map.ply` | fusion | LiDAR points at the sensor height |
| `poses.txt`, `keyframes.json` | fusion | keyframe trajectory (TUM format) |
| `occupancy.pgm` + `occupancy.yaml` | both | 2-D occupancy grid for Nav2 `map_server` (0 = occupied, 254 = free, 205 = unknown) |
| `preview.png` | both | top view of the cloud + grid + trajectory (look at this first) |
| `stats.json` | fusion | counts: frames, keyframes, loop closures, VO losses, voxels |
| `raw/`, `bag/` | `--raw` / `--bag` | raw sensor data for offline re-processing |

Post-processing / conversion:

```bash
python run.py export --ply maps/lab/map.ply --voxel 0.05 --max-z 1.8 --pcd   # crop ceiling, PCD, grid, preview
python run.py export --db maps/lab/rtabmap.db                                # re-export from a database
```

View `.ply` files on the PC with MeshLab or CloudCompare; `preview.png` needs no tools.

## 7. Testing without the robot

```bash
python run.py synth --out maps/synth/raw            # synthetic textured room, ~35 s drive around a loop
python run.py fusion --replay maps/synth/raw --out maps/synth --record
```

This exercises the whole fusion pipeline (VO, LiDAR ICP, keyframes, loop closure, graph
optimisation, export) and runs on the PC as well as on the Pi.

## 8. Troubleshooting

| Symptom | What to do |
|---|---|
| `check`: no OAK device | blue USB 3 port, USB 3 cable, `lsusb` shows `03e7`; udev rule installed; unplug/replug |
| OAK on USB2 (`usb HIGH`) | change port/cable; or run with `--usb2` (lower fps) |
| `no LiDAR packets` | baud 921600 for STL-27L (230400 for LD19/LD06); LiDAR spinning? `ls -l /dev/robot_lidar` |
| LiDAR points appear rotated / behind | `lidar.forward_angle_deg` wrong → `lidar-view --find-forward`; `clockwise` flag |
| Map tilted / floor not flat (fusion) | `camera.mount.pitch` / `roll` wrong → `calib-lidar-camera` prints the corrections |
| `[imu] WARNING: configured camera.imu_rpy_deg looks wrong` | run `calib-imu --apply` |
| VO `lost` most of the time | too dark / textureless / too fast; depth mostly invalid (`check` shows %); lower `camera.fps` |
| RTAB-Map: `Did not receive data since 5 seconds` | topics/QoS: `python run.py rtabmap --shell` then `ros2 topic hz /oak/rgb/image_rect /scan /imu/data` |
| RTAB-Map: odometry lost (red screen in viz) | move slower; try `--odom icp`; check depth quality |
| Docker permission denied | `sudo usermod -aG docker $USER` then log out / in |
| `rtabmap_viz` does not open | on the Pi desktop run `xhost +local:` and start with `--viz` from a desktop terminal (`DISPLAY=:0`) |
| Nothing written | you used `--no-record`; or the process was killed (use `q` / Ctrl+C once and wait for export) |

Logs of the ROS nodes are printed in the terminal; add `--shell` to poke around inside the container
(`ros2 topic list`, `ros2 run tf2_tools view_frames`, `rtabmap-databaseViewer /maps/rtabmap.db`).

## 9. Known limitation: wheel odometry

Firmware v1 always reported `encL = encR = 0` in the `#T` telemetry (v2, in `../firmware`, sends real counts; this package is not yet updated to use them) —
`Encoder_Updata()` is never called (only `Vehicle_Chassis.c`, which is not part of the build,
calls it). Both mappers therefore work **without wheel odometry** (visual + IMU + LiDAR). When the
firmware is fixed (call `Encoder_Updata()` from the 1 kHz tick and accumulate the counts), wheel
odometry can be fed to RTAB-Map as `guess_frame_id` / `odom` — see DEPENDENCIES.md.
