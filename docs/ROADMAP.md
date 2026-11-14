# RobotMK1 roadmap: ROS 2 mapping, navigation and people-aware collision avoidance

A plan for the next stages of RobotMK1:
- **mapping and navigation** with ROS 2;
- **people detection and tracking** with the camera, so the robot avoids collisions with
  moving people;
- **how to use the LiDAR and the camera's IMU** in that system;
- **a project structure** to hold it all.

It builds on what exists today (firmware v2, `robot_mapping`), lists the work in phases
with acceptance criteria, and ends with an evaluation plan and literature for the thesis.

Configuration snippets in this document are **starting points**. Parameter names follow
ROS 2 Jazzy / Nav2 Jazzy as far as I know them; check each against the documentation of the
installed version before relying on it. Values marked **MEASURE** must come from the robot.

**Update 14 Nov 2026:** the robot uses **only the OAK-D Lite's built-in IMU**. The separate
board IMU (GY-521) was dropped as outdated. This document reflects that decision (§4.2).

Contents

1. [Starting point](#1-starting-point)
2. [Target system](#2-target-system)
3. [Foundations: kinematics, odometry, frames, time](#3-foundations-kinematics-odometry-frames-time)
4. [Sensors in ROS 2](#4-sensors-in-ros-2)
5. [Mapping](#5-mapping)
6. [Localisation and navigation](#6-localisation-and-navigation)
7. [People detection, tracking and collision avoidance](#7-people-detection-tracking-and-collision-avoidance)
8. [Phased roadmap](#8-phased-roadmap)
9. [Project structure](#9-project-structure)
10. [Firmware work this needs](#10-firmware-work-this-needs)
11. [Evaluation plan for the thesis](#11-evaluation-plan-for-the-thesis)
12. [Risks and mitigations](#12-risks-and-mitigations)
13. [First steps](#13-first-steps)
14. [References](#14-references)

---

## 1. Starting point

| Area | State (30 Oct 2026) |
|---|---|
| **Chassis** | Ackermann: servo-steered front wheels, two rear drive motors with encoders. Car-like: it cannot turn on the spot. |
| **Control board** | STM32F103 running firmware v2, verified on the robot ([DEVELOPMENT.md](DEVELOPMENT.md)). It provides: <ul><li>raw motor power (`@M`), stopped by a 0.5 s watchdog;</li><li>steering and camera-pan servos (`@S`), with the pan calibrated ([CAMERA_PAN_CALIBRATION.md](CAMERA_PAN_CALIBRATION.md));</li><li>cumulative encoder counts at 50 Hz (`#T`).</li></ul>The firmware's support for a GY-521 board IMU (`#I`) is unused and will be removed: only the camera's IMU is used. |
| **Main computer** | Raspberry Pi 5 (8 GB), Raspberry Pi OS Trixie, Hailo-8 accelerator (26 TOPS) |
| **Sensors** | <ul><li>**OAK-D Lite:** RGB, stereo depth, built-in IMU.</li><li>**LDROBOT STL-27L 2-D LiDAR:** only the forward 180° is usable; the rear is blocked by the camera and body.</li></ul> |
| **Software** | `robot_mapping` (now in the `robotMK1` repository): <ul><li>**RTAB-Map** in a ROS 2 Jazzy Docker container: OAK-D + LiDAR, visual or ICP odometry, an optional IMU gravity constraint, map export.</li><li>**A pure-Python "fusion" mapper** that uses Hailo YOLOv8 to remove people from maps.</li><li>**Calibration tools:** LiDAR forward angle, LiDAR↔camera, IMU alignment.</li></ul>Tested on synthetic data only. |
| **Missing** | <ul><li>a ROS driver for the board;</li><li>wheel odometry;</li><li>sensor fusion (EKF);</li><li>Nav2;</li><li>people tracking;</li><li>closed-loop wheel speed on the board.</li></ul> |

---

## 2. Target system

```
                         ┌────────────────────────── Raspberry Pi 5 (ROS 2 Jazzy, Docker) ─────────────────────────────┐
  STM32 board  ◄─serial─►│ robotmk1_base ──/wheel/odom──┐                                                              │
  (motors, servos,       │   ▲   │  /oak/imu (camera)───┤►robot_localization EKF──/odometry/filtered, TF odom→base     │
   encoders)             │   │   └──/joint_states──►robot_state_publisher──TF (laser, camera pan, imu frames)          │
                         │   │ /cmd_vel                                                                                │
  STL-27L LiDAR ─USB────►│ ldlidar_stl_ros2──/scan_raw──►laser_filters──/scan──┬──►slam_toolbox / RTAB-Map──map, TF    │
                         │                                                    ├──►Nav2 costmaps (obstacle layer)       │
                         │                                                    ├──►Collision Monitor                    │
                         │                                                    └──►LiDAR people clusters ─┐             │
  OAK-D Lite ─USB3──────►│ depthai_ros_driver──RGB, depth, /oak/imu──┬──►RTAB-Map (RGB-D)                 │            │
                         │                                          └──►person detector (Hailo-8 or OAK NN)│           │
  Hailo-8 ─PCIe─────────►│        ──/people/detections (3-D)──►person tracker◄────────────────────────────┘            │
                         │                    ──/people/tracks──►people → costmap layer + Collision Monitor            │
                         │ Nav2: planner (Smac Hybrid-A*), controller (MPPI or RPP), BT navigator,                     │
                         │       velocity smoother ──► Collision Monitor ──/cmd_vel──► robotmk1_base                   │
                         └─────────────────────────────────────────────────────────────────────────────────────────────┘
                                   RViz2 / Foxglove on a laptop over Wi-Fi (same ROS_DOMAIN_ID)
```

**Principles:**
- **The board does fast control and safety:** speed loop, watchdogs, servo limits.
- **The Pi does estimation and decisions:** odometry fusion, SLAM, planning, perception
  (DEVELOPMENT.md §2).
- **Use standard ROS 2 packages where they exist** [3] (Nav2, slam_toolbox, RTAB-Map,
  robot_localization, laser_filters). Write your own only for the robot-specific parts:
  the board driver, people tracking, and the people costmap.
- **Safety in layers:**
  1. board watchdog (0.5 s, already in place);
  2. Nav2 Collision Monitor (independent of planning);
  3. people-aware costmaps (planning);
  4. physical emergency stop (add one).

**Main topics:**

| Topic | Type | From → to |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` (Nav2 can also use `TwistStamped`) | Collision Monitor → `robotmk1_base` |
| `/wheel/odom` | `nav_msgs/Odometry` | `robotmk1_base` → EKF |
| `/oak/imu/data` | `sensor_msgs/Imu` | depthai → RTAB-Map (gravity); through the pan gate (§4.2) → EKF as `/imu/for_ekf` |
| `/odometry/filtered` | `nav_msgs/Odometry` | EKF → SLAM, Nav2 |
| `/scan` | `sensor_msgs/LaserScan` | laser_filters → SLAM, costmaps, Collision Monitor, people |
| `/oak/rgb/image_rect`, `/oak/stereo/image_raw` | `sensor_msgs/Image` | depthai → RTAB-Map, detector |
| `/people/detections` | `vision_msgs/Detection3DArray` | detector → tracker |
| `/people/tracks` | `robotmk1_msgs/TrackedPersonArray` (own message) | tracker → costmap, Collision Monitor, RViz |
| `/people/points` | `sensor_msgs/PointCloud2` | tracker (current + predicted positions) → Collision Monitor, costmap |
| `/joint_states` | `sensor_msgs/JointState` | `robotmk1_base` (steering, camera pan) → `robot_state_publisher` |
| `/camera_pan/command` | `std_msgs/Float64` (rad, left positive) | perception or teleop → `robotmk1_base` |
| `/map` | `nav_msgs/OccupancyGrid` | SLAM or map_server → Nav2 |

---

## 3. Foundations: kinematics, odometry, frames, time

### 3.1 Ackermann model

Use the kinematic bicycle model [28], with the reference point at the centre of the rear
axle. That is
where `base_link` already is in `robot.yaml`.

| Symbol | Meaning | Source |
|---|---|---|
| `L` | wheelbase (rear axle → front axle) | MEASURE |
| `W` | rear track width (between rear wheel contact points) | MEASURE |
| `δ` | front steering angle (virtual centre wheel) | steering calibration (below) |
| `δ_max` | largest steering angle | MEASURE at full lock (commands 63 and 126) |
| `v` | forward speed of the rear-axle centre | encoders |

**Equations:**
- **Yaw rate:** `ω = v · tan(δ) / L`.
- **Inverse (`cmd_vel` → steering):** `δ = atan(L · ω / v)`, clamped to ±`δ_max`.
- **Minimum turning radius:** `R_min = L / tan(δ_max)`. Nav2's planner and controller need it.
- **Rear wheel speeds in a turn:** `v_left = v · (1 − W·tan(δ) / (2L))`,
  `v_right = v · (1 + W·tan(δ) / (2L))`.

**Pure rotation is impossible.** If `cmd_vel` asks for `ω ≠ 0` with `v ≈ 0`, the driver must
not spin the wheels. Keep the last steering angle and output zero speed, and configure Nav2
so it never asks for spinning (§6.3).

**Steering calibration** works the same way as the camera calibration:
1. For a set of `@S1` commands, measure the real wheel angle. A protractor works, or the
   turning circle: drive a slow full circle at a fixed command and measure its radius *R*;
   then `δ = atan(L / R)`.
2. Fit command → `δ`. The centre (straight driving) is probably not exactly 90.
3. Store the result as a lookup table in the driver's parameters.

### 3.2 Wheel odometry

From the cumulative counts in `#T` (left and right rear wheels):

```
Δs_l = Δcounts_l / counts_per_metre      Δs_r = Δcounts_r / counts_per_metre
Δs   = (Δs_l + Δs_r) / 2
Δθ   = (Δs_r − Δs_l) / W                 ← rear wheels don't steer, so their difference gives yaw
       (alternatives: Δs·tan(δ)/L from the steering model, or ∫ gyro z from an IMU)
x += Δs·cos(θ + Δθ/2),  y += Δs·sin(θ + Δθ/2),  θ += Δθ
```

There are three independent sources of yaw:
1. **The encoder difference.** It drifts with wheel slip and with errors in `W`.
2. **The steering model.** It drifts with steering error and slip.
3. **The gyro.** It drifts slowly with bias.

The EKF combines them (§6.1).

**Calibration:**
- **`counts_per_metre`:** drive straight and measure with a tape, as in PI_GUIDE §6.3.
- **`W` and the systematic errors:** use the UMBmark square test [12]: drive a 4 × 4 m square
  clockwise and counter-clockwise, then correct from the end-point errors.

### 3.3 Coordinate frames (REP 103 / REP 105 [8], [9])

```
map ──(SLAM / localisation)──► odom ──(EKF)──► base_link ──┬─(fixed)──► laser
                                                          ├─(fixed)──► camera_pan_base
                                                          │            └─(revolute, /joint_states)──► camera_link ─► oak frames, oak_imu_frame
                                                          └─(revolute)──► front_left/right_steer (visualisation only)
```

- **`base_link`:** on the floor under the centre of the rear axle, as in `robot.yaml`.
  x points forward, y left, z up.
- **Camera pan as a joint.** A revolute joint whose angle is published from the commanded
  pan: `yaw ≈ (command − 90)°`, left positive, using the nominal servo scale. That matches
  REP 103 and CAMERA_PAN_CALIBRATION.md §8. The servo has no position sensor, so the
  joint angle is the *commanded* angle, published after a short settling delay.
- **All sensor poses live in one place:** the URDF/xacro, generated from today's `robot.yaml`
  values. The calibration tools write their results there.

### 3.4 Time

**The board.**
- `#T` carries the board's millisecond clock.
- The driver stamps messages with the Pi time of arrival, minus a constant latency (USB +
  parsing, ~2–5 ms). It uses the board clock for exact intervals between samples.
- A running fit `t_pi ≈ a + b · t_board` removes jitter (PI_GUIDE §7.3).

**The Pi.** Keep it on NTP (chrony). If RViz runs on a laptop, keep the laptop synchronised
too.

**Sensor delays.** The LiDAR and the OAK-D stamp their own data in their ROS drivers. Check
the delays once by turning the robot quickly in front of a wall: the scan and the depth
image should line up in RViz.

---

## 4. Sensors in ROS 2

### 4.1 LiDAR (STL-27L)

| Item | Plan |
|---|---|
| **Driver** | `ldlidar_stl_ros2`, already built from source in the Docker image; publishes `/scan_raw` in frame `laser`. Pin the commit in a `.repos` file. |
| **Field of view** | Keep only the forward 180°, as `scan_fov_filter.py` does today. In ROS 2 use `laser_filters` with `LaserScanAngularBoundsFilter`, and add `LaserScanRangeFilter` (0.10–12 m) and a box filter that removes returns from the robot's own body. |
| **Mounting** | 0.15 m above the floor, in front of the camera. It sees legs, furniture legs and walls, but nothing lower than 0.15 m and nothing above that plane. Depth from the camera fills that gap in the costmap (§6.2). |
| **SLAM** | 2-D scan matching in slam_toolbox; scan-refined links and loop closures in RTAB-Map (`Reg/Strategy` 1 or 2) |
| **Localisation** | AMCL or RTAB-Map localisation. With only 180°, AMCL converges more slowly: give it a good initial pose (§6.1). |
| **Obstacles** | Nav2 obstacle layer with ray tracing: clears free space, marks hits. Collision Monitor stop/slow-down zones. |
| **People** | Leg-height clusters: segment the scan into clusters, keep those of human-leg size, and associate them with camera detections (§7.3). The LiDAR then gives accurate range at the scan rate across the whole 180°, even where the camera can't see. |
| **No rear view** | The robot is blind when reversing. Reverse only slowly and briefly (Nav2 BackUp behaviour with a short distance), or not at all (`DUBIN` motion model, §6.3). A rear sensor would be a hardware extension. |
| **Calibration** | `lidar-view --find-forward` (forward angle) and `calib_lidar_camera.py` (LiDAR↔camera extrinsics [10]) already exist. Move them into the calibration package (§9) and write their results into the URDF. |

### 4.2 IMU (built into the OAK-D Lite)

The robot uses **one IMU: the BMI270 inside the OAK-D Lite** (decision of 14 Nov 2026). A
separate board IMU (GY-521 / MPU6050) was dropped as outdated.

| | OAK-D Lite IMU |
|---|---|
| **Data** | `/oak/imu/data` from depthai: accelerometer and gyroscope; rate set in `robot.yaml` (`imu_rate_hz`, 100 Hz) |
| **Clock** | Camera clock, synchronised to the Pi by depthai |
| **Uses** | **Yaw rate for wheel odometry** in the EKF, especially in turns, where wheels slip. **Gravity direction** for RTAB-Map (`Optimizer/GravitySigma`). The camera's tilt. |
| **Mounting** | Inside the camera, so it **turns with the camera pan** |

**The pan problem.** When the camera pans, the gyroscope measures the robot's turn rate
**plus** the pan rate. `robot_localization` rotates IMU data into `base_link` using TF,
but it cannot remove the extra rotation of a moving joint. So the EKF must only see the
gyro while the camera is still:

- **Rule:** keep the camera centred (`@R`) during mapping and normal driving.
- **Pan gate:** a small node, part of `robotmk1_base`, republishes `/oak/imu/data` as
  `/imu/for_ekf` only while the pan servo is at rest. The driver knows when it commands a
  pan, and allows a settling time after it. While the camera moves, the EKF runs on wheel
  odometry alone.
- **Optional refinement:** subtract the commanded pan rate from the gyro z-axis instead of
  dropping samples. It's only worth doing if active camera panning while driving becomes
  important (§4.3).

**Calibration and set-up:**
1. **Gyro bias.** Average 3–5 s at standstill on every start. Optionally re-estimate it
   whenever the wheels report zero speed (a zero-velocity update).
2. **Orientation.** The IMU's axes relative to the camera body depend on the production
   batch of the OAK-D Lite. The existing `calib-imu` tool measures them and writes
   `imu_rpy_deg`; that rotation goes into the URDF (`camera_link` → `oak_imu_frame`).
3. **Covariances.** Measure the noise at rest (standard deviation of each axis) and put it
   in the `Imu` message's covariance fields, so the EKF weighs the IMU correctly.
4. **Orientation filter.** `imu_filter_madgwick` produces orientation (already in the
   RTAB-Map launch file), but for a 2-D robot the EKF only needs the **yaw rate**. Don't
   fuse absolute yaw from a 6-axis IMU: it has no magnetometer, so the yaw drifts.

### 4.3 Camera (OAK-D Lite) and the pan servo

| Use | How |
|---|---|
| **RGB-D for mapping** | `depthai_ros_driver`, RGB 640×360 at 15 fps with depth aligned to RGB (as today); RTAB-Map RGB-D odometry and loop closure |
| **Depth for obstacles** | Nav2 voxel or obstacle layer fed from the depth image converted to a point cloud (`depth_image_proc`, or the cloud RTAB-Map produces). It sees obstacles above and below the LiDAR plane: table tops, low boxes. Use a reduced resolution and rate to save CPU. |
| **People** | §7 |
| **Active looking** (optional) | `/camera_pan/command` to look into a turn, or to keep a tracked person in view. **During mapping the camera stays centred** (`@R`), because the pan angle comes from the command, not a sensor, so it is less accurate than a fixed mount. |
| **Calibration** | Intrinsics: the OAK factory calibration [11]. Mount: CAMERA_PAN_CALIBRATION.md; `calib_lidar_camera.py` for the extrinsics. |

**Note:** the OAK-D Lite has **passive** stereo, with no IR projector. Depth fails on plain,
untextured walls and in the dark. The LiDAR covers that weakness, and the camera covers the
LiDAR's single plane.

---

## 5. Mapping

Two SLAM systems, both already available as ROS 2 Jazzy packages. **Use both, and compare
them in the thesis.**

| | **RTAB-Map** [4] (already set up) | **slam_toolbox** [5] |
|---|---|---|
| Input | RGB-D + LiDAR scan + odometry (+ IMU gravity) | 2-D scan + odometry |
| Output | 3-D point cloud + 2-D occupancy grid + database | 2-D occupancy grid + pose graph (serialised) |
| Strengths | 3-D map; visual loop closure works with a narrow LiDAR FOV; one tool for mapping and localisation | Light on CPU, the standard Nav2 companion, good with clean 2-D scans |
| Weak spot here | CPU load on the Pi; passive-stereo depth gaps | Only 180° of scan: fewer features, weaker loop closures |
| Role | Primary: the 3-D map for the thesis, plus a grid for Nav2 | Baseline or alternative 2-D map for Nav2, and an ablation |

**Changes to the existing RTAB-Map set-up:**
1. **Odometry input.** Replace `odom:=vo` with the EKF output (`/odometry/filtered`:
   wheels + IMU). RTAB-Map then only does mapping and loop closure. Keep visual odometry as
   an option, for comparison.
2. **Move the launch file.** `mapping.launch.py` moves into `robotmk1_bringup` (§9). Frames
   come from the URDF (robot_state_publisher), not hand-built static transforms.
3. **Masking people.** Keep the idea from the fusion mapper: pixels in person boxes are not
   integrated into the map. RTAB-Map can take a mask, or the depth image can be masked by
   the people pipeline before RTAB-Map sees it.
4. **Saving maps.** Save the grid for Nav2 with `nav2_map_server` `map_saver_cli`, already
   used in `run_rtabmap.py`. Keep the `.db` for RTAB-Map localisation.

**Mapping procedure** (for repeatable thesis runs):
1. Drive manually with a joystick or keyboard teleop at ≤ 0.3 m/s.
2. Close loops deliberately: return through known places.
3. Record everything with `ros2 bag record` (MCAP). Maps can then be rebuilt offline with
   different parameters, which makes comparisons fair: same data, different algorithm.

---

## 6. Localisation and navigation

### 6.1 Localisation

| Layer | Package | Output |
|---|---|---|
| Local, continuous | `robot_localization` EKF [6] (wheel odometry + camera-IMU yaw rate) | `odom → base_link`, `/odometry/filtered` |
| Global, in a known map | **RTAB-Map localisation mode** (`Mem/IncrementalMemory: false`), or **AMCL** [7] on the 2-D grid | `map → odom` |

Starting EKF configuration (2-D, Ackermann):

```yaml
ekf_filter_node:
  ros__parameters:
    frequency: 30.0
    two_d_mode: true
    publish_tf: true
    map_frame: map
    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom
    # [x y z  roll pitch yaw  vx vy vz  vroll vpitch vyaw  ax ay az]
    odom0: /wheel/odom
    odom0_config: [false, false, false,  false, false, false,
                   true,  true,  false,  false, false, true,
                   false, false, false]       # vy = 0 is a real constraint for a car-like robot
    imu0: /imu/for_ekf                        # camera IMU through the pan gate (§4.2)
    imu0_config:  [false, false, false,  false, false, false,
                   false, false, false,  false, false, true,
                   false, false, false]       # yaw rate only
    imu0_remove_gravitational_acceleration: true
```

The covariances in `/wheel/odom` and the IMU message decide the weighting. Set the wheel
yaw-rate variance higher than the gyro's, because wheels slip in turns.

**AMCL vs RTAB-Map localisation:**
- **AMCL** needs a decent initial pose. With 180° of scan it can mistake symmetric corridors
  for each other.
- **RTAB-Map** relocalises from images (it can recognise a place it has seen), which
  matters for a forward-only LiDAR.

Try both: comparing them is a good thesis experiment.

### 6.2 Costmaps

| Costmap | Layers |
|---|---|
| **Global** | static layer (the map) + obstacle layer (`/scan`) + inflation |
| **Local** (rolling, ~4 × 4 m) | obstacle layer (`/scan`) + voxel or obstacle layer (camera depth points) + **people layer** (§7.5) + inflation |

**Footprint:** a polygon for the real chassis, relative to the rear axle (MEASURE: length,
width, overhang in front of the front axle and behind the rear axle). The planner checks
the full footprint at every pose, which matters for a car-like robot.

### 6.3 Nav2 for an Ackermann robot [1], [2]

| Component | Choice | Why |
|---|---|---|
| **Planner** | `nav2_smac_planner::SmacPlannerHybrid` (Hybrid-A*) [13] | Produces paths a car can drive: they respect the minimum turning radius |
| **Controller** | `nav2_mppi_controller::MPPIController` with `motion_model: "Ackermann"` [15]. Alternative: Regulated Pure Pursuit (RPP) [14], [16]. | MPPI handles car kinematics and costs directly; RPP is simpler, easy to tune, and a good baseline. Comparing them is a thesis experiment. |
| **Velocity smoother** | `nav2_velocity_smoother` | Limits acceleration; protects the gearbox and keeps odometry clean |
| **Safety** | `nav2_collision_monitor` | Stop/slow-down polygons on `/scan` and the people points, independent of the planner |
| **Behaviours** | BackUp (short and slow), Wait, DriveOnHeading; **no Spin** | Spinning in place is impossible for this chassis. Remove it from the behaviour tree. |

Starting points:

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_smac_planner::SmacPlannerHybrid"
      motion_model_for_search: "DUBIN"     # forward only; "REEDS_SHEPP" if reversing is allowed
      minimum_turning_radius: 0.60         # MEASURE: L / tan(delta_max)
      angle_quantization_bins: 72
      allow_unknown: true

controller_server:
  ros__parameters:
    controller_frequency: 20.0
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "nav2_mppi_controller::MPPIController"
      motion_model: "Ackermann"
      AckermannConstraints:
        min_turning_r: 0.60                # same value as above
      vx_max: 0.40                         # start slow indoors
      vx_min: -0.15                        # 0.0 to forbid reversing (no rear view)
      wz_max: 1.0
```

**Behaviour tree.** Start from Nav2's default `navigate_to_pose` tree and remove the Spin
recovery. Add a "wait for the path to clear" step before re-planning when a person blocks
the path (§7.5).

### 6.4 Collision Monitor (safety layer)

The Collision Monitor sits between the velocity smoother and the robot. It cuts or scales
`/cmd_vel` when points enter zones in front of the robot. Its sources are the LiDAR scan
and the people points (current and predicted positions).

```yaml
collision_monitor:
  ros__parameters:
    base_frame_id: "base_link"
    odom_frame_id: "odom"
    cmd_vel_in_topic: "cmd_vel_smoothed"
    cmd_vel_out_topic: "cmd_vel"
    polygons: ["Stop", "Slow"]
    Stop:   {type: "polygon", action_type: "stop",
             points: "[[0.55, 0.18], [0.55, -0.18], [-0.05, -0.18], [-0.05, 0.18]]"}   # MEASURE footprint + margin
    Slow:   {type: "polygon", action_type: "slowdown", slowdown_ratio: 0.3,
             points: "[[1.20, 0.40], [1.20, -0.40], [-0.05, -0.40], [-0.05, 0.40]]"}
    observation_sources: ["scan", "people"]
    scan:   {type: "scan", topic: "/scan"}
    people: {type: "pointcloud", topic: "/people/points", min_height: 0.0, max_height: 2.0}
```

The exact parameter names, for example how `points` is written, changed between Nav2
releases. Copy the structure from the Jazzy Collision Monitor documentation.

---

## 7. People detection, tracking and collision avoidance

Goal: see people, estimate where they are **and where they are going**, and adapt:
- **plan around them;**
- **slow down near them;**
- **stop** if a collision is imminent.

The literature calls this human-aware or social navigation [20], [21].

### 7.1 Pipeline

```
 RGB (+ depth) ──► 1. detect persons ──► 2. 3-D position ──► 3. track (Kalman + association) ──► 4. predict ──► 5. act
                     (Hailo-8 YOLO or       (depth in box,        (odom frame, constant            (1–3 s)      ├─ costmap people layer
                      OAK spatial NN)        → base_link → odom)   velocity, IDs)                               ├─ Collision Monitor points
 LiDAR /scan ──► leg-height clusters ───────────────────────────────┘ (association adds range + 180° coverage) └─ speed limits / wait
```

### 7.2 Detection: two options

| | **A. Hailo-8 + YOLOv8** [17] | **B. On-camera spatial detection (OAK-D Lite VPU)** |
|---|---|---|
| Where it runs | Hailo-8 (26 TOPS) on the Pi's PCIe | Myriad X inside the camera |
| Output | 2-D boxes; 3-D position computed on the Pi from aligned depth | Boxes **with X, Y, Z** computed on the camera (spatial detection network); depthai-ros can publish them as `vision_msgs/Detection3DArray` |
| Models | Larger (YOLOv8s/m), segmentation, pose (keypoints → heading of a person) | Small models only (tiny YOLO, MobileNet-SSD) |
| Effort | Needs HailoRT inside the Docker container, at the same version as the host driver; `hailo_detector.py` exists as a starting point | Least code: configuration in depthai-ros |
| Recommendation | Phase 6b: better accuracy, plus pose for body heading | **Start here** (Phase 6a): fastest route to working 3-D person positions |

Run detection at 5–10 Hz on 640×360 frames. People move slowly compared with that rate, and
the tracker fills the gaps between frames.

### 7.3 From boxes to 3-D tracks

1. **3-D position** (option A). Take the **median depth** in the central 50 % of the box
   (avoids background at the edges). Back-project the box centre with the camera intrinsics:
   `X = (u − c_x) · Z / f_x`, `Y = (v − c_y) · Z / f_y`. Transform the point from the camera
   frame to `odom` using TF at the image timestamp. Working in `odom` removes the robot's own
   motion, so a person standing still stays still in the track.
2. **LiDAR association** (optional, improves range and coverage).
   - Cluster `/scan` points by distance jumps.
   - Keep clusters 5–25 cm wide: legs at 0.15 m height.
   - Pair leg clusters up to 0.4 m apart.
   - Match them to camera tracks by distance.
   - A person outside the camera's ~70° view (but inside the LiDAR's 180°) stays tracked from
     LiDAR alone, as an "unconfirmed" track. Classic leg detection: [18].
3. **Tracking.**
   - **State:** one Kalman filter [22] per person, with a constant-velocity model (x, y, vx, vy)
     in `odom`.
   - **Association:** each cycle, match detections to tracks with the Hungarian algorithm [23]
     on gated distances (SORT-style [19]). ByteTrack's idea of also using low-confidence
     detections [24] reduces broken tracks behind obstacles.
   - **Track life cycle:** *tentative* (needs 3 hits), *confirmed*, *lost* (kept for 1–2 s for
     occlusions), then deleted.
4. **Prediction.** For each confirmed track publish the current position plus positions
   0.5, 1.0, 1.5 and 2.0 s ahead (`p + v·t`), with uncertainty growing with `t`.

**Own message** (`robotmk1_msgs/TrackedPerson`):

```
uint32 id
geometry_msgs/PoseWithCovariance pose        # in odom
geometry_msgs/TwistWithCovariance velocity
float32 confidence
uint8 state                                  # 0 tentative, 1 confirmed, 2 lost
bool seen_by_camera
bool seen_by_lidar
```

### 7.4 Collision avoidance: three layers, from slow to fast

| Layer | Mechanism | Reaction time | What it does |
|---|---|---|---|
| **Plan** | People layer in the costmaps (§7.5) | about 1 s (re-planning) | Paths keep a social distance from people and avoid where they are heading |
| **Control** | MPPI cost or RPP regulation on the people layer; speed limit near people (e.g. ≤ 0.2 m/s within 1.5 m) | 50 ms (controller cycle) | Slows down and steers away |
| **Safety** | Collision Monitor on `/scan` + `/people/points` (§6.4) | at the scan or tracker rate | Stops or slows the robot whatever the planner thinks |
| (Hardware) | Board motor watchdog 0.5 s; **physical emergency stop** (add) | immediate | The last line |

### 7.5 People layer in the costmap

| Variant | How | Notes |
|---|---|---|
| **Simple (start here)** | Publish current and predicted person positions as a `PointCloud2` (`/people/points`); add it as an observation source of an obstacle layer; let the inflation layer make the safety zone | No C++ needed; works with Nav2 as is |
| **Social layer (thesis contribution)** | Own `nav2_costmap_2d` layer plugin (C++), adding a **Gaussian cost around each person, stretched in the direction of motion**. Personal-space radius from proxemics [25]: ~0.45–1.2 m. | Planned paths keep comfortable distances and pass behind rather than in front of walking people. Community social-costmap plugins exist; check whether one supports Jazzy before writing your own. |

Behaviour when the path is blocked: wait a few seconds (a behaviour-tree node), then
re-plan. Don't push through.

### 7.6 Using the pan servo for people (optional)

- **Look into turns:** pan the camera towards the planned path's curvature.
- **Keep a tracked person in view:** the person closest to the path gets priority.
- The pan angle reaches TF through `/joint_states`, so detections are transformed correctly
  while the camera turns.
- Limit the pan speed: images during fast motion are blurred.

### 7.7 Ethics and safety for experiments with people

- Run tests at low speed (≤ 0.3 m/s), with an operator holding the emergency stop.
- Use only participants who have agreed. Avoid storing identifiable images where possible:
  keep only tracks, or blur faces in recordings.
- Mention this in the thesis methods section.

---

## 8. Phased roadmap

Effort is a rough guide for one person part-time. Each phase ends with a demonstrable
result and acceptance criteria. Those criteria are also the thesis measurements.

| Phase | Content | Acceptance criteria | Effort |
|---|---|---|---|
| **0. Firmware and calibration** (done) | Firmware v2, hardware test, camera pan calibration | All passed (DEVELOPMENT.md) | — |
| **1. ROS 2 base** | <ul><li>Workspace (§9), Docker image with Nav2, slam_toolbox, robot_localization, laser_filters.</li><li>`robotmk1_base` driver; URDF.</li><li>Teleop (keyboard, joystick); steering calibration (§3.1).</li></ul> | <ul><li>Drive by teleop.</li><li>TF tree complete in RViz.</li><li>Board restarts handled.</li><li>No dropped `#T`.</li></ul> | 2–3 weeks |
| **2. Firmware speed control** | 20 kHz motor PWM, wheel-speed PID, `@V` command, heartbeat `@P`, version field (§10) | Speed step response settles within 0.3 s; ±5 % speed error at 0.2–0.5 m/s | 1–2 weeks |
| **3. Odometry and IMU** | <ul><li>Wheel odometry and its calibration (counts/m, UMBmark [12]).</li><li>Camera IMU into the EKF through the pan gate (§4.2).</li><li>EKF.</li></ul> | <ul><li>Straight 5 m: < 2 % distance error.</li><li>Square test: end-point error reported, and improved by the gyro.</li></ul> | 2 weeks |
| **4. Sensors and mapping** | <ul><li>LiDAR via laser_filters; OAK via depthai-ros.</li><li>Extrinsics into the URDF.</li><li>RTAB-Map on EKF odometry; slam_toolbox; bag recording.</li></ul> | Maps of two indoor areas with both SLAM systems; loop closures; trajectory error against reference points (§11) | 3 weeks |
| **5. Navigation** | <ul><li>Nav2: Smac Hybrid-A*, MPPI (and RPP for comparison), costmaps with the depth layer, Collision Monitor, behaviour tree without Spin.</li><li>Localisation: RTAB-Map and AMCL.</li></ul> | ≥ 90 % of 20 goal runs reached without collision or intervention; stops before static obstacles in a corridor | 3–4 weeks |
| **6a. People: detection and tracking** | OAK spatial detection (or Hailo), 3-D positions, tracker, `/people/tracks`, RViz markers | On recorded data: detection precision and recall; tracking MOTA/IDF1 [26], [27]; position error vs tape-measured positions | 2–3 weeks |
| **6b. People: avoidance** | <ul><li>People points into the costmap and Collision Monitor; speed limit near people.</li><li>Optional: social layer, LiDAR leg association, Hailo YOLOv8, active pan.</li></ul> | <ul><li>Crossing and head-on scenarios (§11): no contact.</li><li>Minimum distance ≥ 0.5 m.</li><li>Stopping distance measured at 0.2/0.3/0.4 m/s.</li></ul> | 3–4 weeks |
| **7. Integration and thesis experiments** | Final experiments, data analysis, figures, write-up | All §11 metrics with repeated trials | 3–4 weeks |
| (Optional) **Simulation** | Gazebo (gz sim, Harmonic for Jazzy) with an Ackermann steering plugin and the URDF | Nav2 configuration tuned in simulation before the robot | 1–2 weeks, in parallel with phase 5 |

**Order and dependencies:**
- Phase 2 can run in parallel with phase 1, since it's firmware.
- Phases 3 → 4 → 5 are sequential.
- Phase 6a can start after phase 4, working on recorded bags, in parallel with phase 5.
- Phase 6b needs phases 5 and 6a.

---

## 9. Project structure

One Git repository for everything: `robotMK1` (GitHub `Nikon757/robotMK1`), created on
14 Nov 2026. It already holds `firmware/`, `docs/`, `pi_examples/` and `robot_mapping/`;
the other folders below arrive with the ROS 2 work.

```
robotMK1/
├── firmware/                         Keil project and the firmware README
├── docs/                             PI_GUIDE, DEVELOPMENT, CAMERA_PAN_CALIBRATION, MOTORS_AND_ENCODERS, ROADMAP
│   └── thesis/                       figures, tables, experiment notes
├── docker/
│   ├── Dockerfile                    ROS 2 Jazzy + Nav2 + slam_toolbox + rtabmap_ros + depthai_ros
│   │                                 + robot_localization + laser_filters + HailoRT (matching host driver)
│   ├── compose.yaml                  network_mode: host; devices: /dev/robot_mcu, /dev/robot_lidar,
│   │                                 /dev/hailo0, USB for the OAK; volumes: ros_ws, maps, bags
│   └── entrypoint.sh
├── robotmk1.repos                    vcstool list: ldlidar_stl_ros2 (pinned commit), other source deps
├── ros_ws/src/
│   ├── robotmk1_description/         urdf/robotmk1.urdf.xacro (all mounts; from robot.yaml), rviz/, meshes/
│   ├── robotmk1_msgs/                msg/TrackedPerson.msg, TrackedPersonArray.msg
│   ├── robotmk1_base/                board driver (Python first): serial ↔ ROS, Ackermann, odometry,
│   │                                 IMU, joint states, heartbeat, diagnostics, reboot handling
│   ├── robotmk1_bringup/             launch/: robot, sensors, teleop, mapping_rtabmap, mapping_slam_toolbox,
│   │                                 localization, navigation, perception, record (bags)
│   │                                 config/: ekf.yaml, laser_filters.yaml, depthai.yaml, rtabmap.yaml,
│   │                                 slam_toolbox.yaml, nav2.yaml, collision_monitor.yaml
│   ├── robotmk1_navigation/          behavior_trees/*.xml (no Spin), maps/, waypoint lists for experiments
│   ├── robotmk1_perception/          person_detector (depthai / hailo backends), leg_detector (LiDAR),
│   │                                 person_tracker (Kalman + Hungarian), people_publisher (points, markers),
│   │                                 camera_pan_controller
│   ├── robotmk1_people_layer/        (C++, optional) nav2_costmap_2d plugin: social/predictive cost
│   └── robotmk1_tools/               calibration (steering, odometry/UMBmark, IMU, LiDAR forward angle,
│                                     LiDAR↔camera), evaluation (trajectory error, MOT metrics, stop distance),
│                                     bag export
├── pi_examples/                      today's standalone scripts (hw_test, pan_sweep, niumo_board)
└── robot_mapping/                    existing non-ROS mapper: kept as a baseline and for the thesis comparison
```

**Where today's code goes:**

| Today | Becomes |
|---|---|
| `robot_mapping/common/mcu.py`, `pi_examples/niumo_board.py` | `robotmk1_base` (parser, commands, reboot detection reused) |
| `robot_mapping/config/robot.yaml` (mounts) | `robotmk1_description` xacro properties |
| `mappers/rtabmap/mapping.launch.py`, `params/*.yaml` | `robotmk1_bringup/launch/mapping_rtabmap.launch.py`, `config/rtabmap.yaml` |
| `mappers/rtabmap/scan_fov_filter.py` | `laser_filters` configuration (`LaserScanAngularBoundsFilter`) |
| `mappers/rtabmap/Dockerfile` | `docker/Dockerfile` (more packages) |
| `mappers/fusion/hailo_detector.py` | `robotmk1_perception` Hailo backend |
| `tools/*.py` (calibration, export) | `robotmk1_tools` (as ROS nodes or scripts reading bags) |
| `common/ldlidar.py`, `common/oakd.py` | replaced by the ROS drivers; kept in `robot_mapping` for the non-ROS mapper |

**Working practice:**
- **Develop on a PC** with recorded bags: `ros2 bag play` replays a real run, so perception and
  mapping can be tuned without the robot.
- **Use the same Docker image** on the PC (x86) and the Pi (arm64) via a multi-arch build, or
  build on the Pi.
- **RViz or Foxglove on the laptop.** Set the same `ROS_DOMAIN_ID` on both machines, use Cyclone
  DDS (already chosen), and check Wi-Fi multicast. For a heavy RViz, view only compressed
  images and downsampled clouds.
- **Tests:**
  - unit tests for the driver's parsing and kinematics (the simulated board from today's tests
    carries over);
  - `launch_testing` for start-up;
  - one recorded "golden" bag per phase.
- **Parameters** live in YAML under version control. Every thesis experiment records the Git
  commit and parameter files with its bag.

---

## 10. Firmware work this needs

| Change | Why | Phase |
|---|---|---|
| Motor PWM 100 Hz → ~20 kHz | Smooth low-speed torque, no audible whine | 2 |
| Wheel-speed PID on the board, new command `@V <left_mm_s> <right_mm_s>` (`compute_pid.c` exists) | Nav2 sends velocities; a 1 kHz loop on the board tracks them far better than the Pi could over USB | 2 |
| Wheel speeds in `#T` (or a new `#V` line) | Lets the driver check speed tracking and tune the PID | 2 |
| Heartbeat `@P`, with `NO PI LINK` on the OLED | Detects a dead Pi or link; stops motors independently of `@M` | 1–2 |
| Firmware version and build date in `#BOOT` and on `@Q` | Problem 20 in DEVELOPMENT.md: the running build couldn't be identified | 1 |
| Steering in calibrated units (optional) | Keep the lookup in the driver (simpler), or move it into `servo_limit[]` | 1 |
| Remove the unused GY-521 / MPU6050 support (`#I`, `#IMU`, the IMU part of the I2C code) | Board IMU dropped on 14 Nov 2026; the camera's IMU is used instead | 1 |
| Emergency-stop input (optional): a button on a free GPIO cuts motor PWM in firmware, plus a separate power switch | Safety layer independent of all software | 5–6 |
| Raise the link to 230400 or 460800 baud (only if needed) | More telemetry (per-wheel speeds, IMU) still fits at 115200 today | as needed |

---

## 11. Evaluation plan for the thesis

| What | Metric | How measured | Reference |
|---|---|---|---|
| Odometry | Distance error %; UMBmark end-point error, clockwise and counter-clockwise; drift per metre with and without gyro | Tape measure, floor marks | [12] |
| SLAM / mapping | Absolute trajectory error (ATE) at reference points; map consistency (wall thickness, straightness); loop-closure count; CPU and RAM use | Floor markers at measured positions; RTAB-Map vs slam_toolbox on the **same bag** | [4], [5] |
| Localisation | Convergence time and success from a known or unknown start; position error at markers | AMCL vs RTAB-Map localisation | [7] |
| Navigation | Success rate, path length, time to goal, interventions, minimum obstacle distance | 20+ runs per configuration (MPPI vs RPP) | [1], [14], [15] |
| Detection | Precision and recall at 1–6 m; 3-D position error vs tape-measured positions | Labelled frames from bags | [17] |
| Tracking | MOTA, IDF1, ID switches | Recorded scenes with 1–3 people, hand-labelled ground truth | [26], [27] |
| People avoidance | Minimum robot–person distance; time-to-collision at the reaction; stopping distance vs speed; stops and slowdowns per run | Scenarios: person crossing, approaching head-on, standing in a corridor, overtaking; with and without the people layer (ablation) | [20], [21] |
| System | End-to-end latency (person appears → robot reacts); detector and tracker frame rates | Timestamps in bags | — |

**Repeat every experiment** (≥ 5 trials per condition) and report mean ± standard deviation.
Uncertainties follow the GUM approach from CAMERA_PAN_CALIBRATION.md §10.

---

## 12. Risks and mitigations

| Risk | Effect | Mitigation |
|---|---|---|
| **Pi 5 CPU load** (RTAB-Map + Nav2 + depth processing) | Lag, dropped frames, late reactions | Detection on the Hailo or the OAK; low camera resolution and rate; RTAB-Map detection rate 1 Hz; local costmap mainly from the LiDAR; measure early with `top` and `ros2 topic hz` |
| **HailoRT in Docker** (driver and library versions must match) | Detector won't start | Spike it early in phase 1. Fall back to OAK on-camera detection (option B). |
| **180° LiDAR, no rear view** | Blind reversing; weaker AMCL | Forward-only planning (`DUBIN`), slow and short BackUp only; RTAB-Map visual relocalisation |
| **Ackermann tuning in Nav2** | Oscillation, missed goals in tight spaces | Measure `R_min` honestly; start in simulation; RPP baseline before MPPI; generous goal tolerances |
| **Passive stereo depth** | Holes on plain walls, false obstacles | LiDAR for walls; depth-confidence threshold; clear the voxel layer by ray tracing |
| **Wheel slip** (small wheels, smooth floors) | Odometry drift | Gyro yaw in the EKF; loop closures; slow acceleration (velocity smoother) |
| **Camera pan is open-loop** | Wrong camera pose if the servo lags | Centred during mapping; settling delay before using the published angle; slow pan |
| **USB disturbances** (servo or motor current, see DEVELOPMENT.md problem 16) | Lost sensors mid-run | Consider powering the servos from the second power board, away from the Pi's supply; udev names; drivers that reconnect; watch `dmesg` |
| **Scope** (thesis time) | Unfinished work | Phases deliver standalone results; 6b's social layer and active pan are optional extras |

---

## 13. First steps

1. **Repository.** Done on 14 Nov 2026 (`robotMK1`). Next in the firmware: a version number
   in `#BOOT` (removes an existing pain point) and removing the unused board-IMU code.
2. **Docker image.** Extend it with `navigation2`, `nav2_bringup`, `slam_toolbox`,
   `robot_localization`, `laser_filters`, `teleop_twist_keyboard`, `xacro` and
   `robot_state_publisher`. Test HailoRT inside it.
3. **Base driver.** Write `robotmk1_base`, version 1:
   - `/cmd_vel` → `@M` + `@S1` with the Ackermann conversion;
   - `#T` → `/wheel/odom`;
   - `/joint_states`, heartbeat, reboot handling.

   Unit-test it with the simulated board.
4. **URDF.** Write it from `robot.yaml`, measuring `L`, `W`, `δ_max` and the footprint. Check
   the TF tree in RViz.
5. **Calibrate and record.** Calibrate steering and `counts_per_metre`, then record a first
   teleop bag with all sensors. That bag becomes the test data for the next phases.

---

## 14. References

Check editions, page numbers and access dates against the copies you use, and convert to
your required citation style.

**ROS 2, navigation and mapping**

[1] S. Macenski, F. Martín, R. White, and J. Ginés Clavero, "The Marathon 2: A Navigation
System," in *Proc. IEEE/RSJ IROS*, 2020. (Nav2)

[2] Nav2 documentation, https://docs.nav2.org (Smac Planner, MPPI, Regulated Pure Pursuit,
Collision Monitor, costmap plugins, behaviour trees). Access date needed.

[3] S. Macenski, T. Foote, B. Gerkey, C. Lalancette, and W. Woodall, "Robot Operating System 2:
Design, architecture, and uses in the wild," *Science Robotics*, vol. 7, no. 66, eabm6074,
2022.

[4] M. Labbé and F. Michaud, "RTAB-Map as an open-source lidar and visual simultaneous
localization and mapping library for large-scale and long-term online operation," *Journal
of Field Robotics*, vol. 36, no. 2, pp. 416–446, 2019.

[5] S. Macenski and I. Jambrecic, "SLAM Toolbox: SLAM for the dynamic world," *Journal of Open
Source Software*, vol. 6, no. 61, p. 2783, 2021.

[6] T. Moore and D. Stouch, "A generalized extended Kalman filter implementation for the Robot
Operating System," in *Intelligent Autonomous Systems 13 (IAS-13)*, Springer, 2016,
pp. 335–348. (robot_localization)

[7] S. Thrun, W. Burgard, and D. Fox, *Probabilistic Robotics*. Cambridge, MA, USA: MIT Press,
2005. (EKF, particle-filter localisation / AMCL, occupancy grids)

[8] T. Foote and M. Purvis, "REP 103: Standard Units of Measure and Coordinate Conventions,"
2010. https://www.ros.org/reps/rep-0103.html

[9] W. Meeussen, "REP 105: Coordinate Frames for Mobile Platforms," 2010.
https://www.ros.org/reps/rep-0105.html

[10] Q. Zhang and R. Pless, "Extrinsic calibration of a camera and laser range finder
(improves camera calibration)," in *Proc. IEEE/RSJ IROS*, 2004, pp. 2301–2306.

[11] Luxonis, DepthAI and depthai-ros documentation, https://docs.luxonis.com. Access date
needed.

[12] J. Borenstein and L. Feng, "Measurement and correction of systematic odometry errors in
mobile robots," *IEEE Transactions on Robotics and Automation*, vol. 12, no. 6, pp. 869–880,
1996. (UMBmark)

**Planning and control for car-like robots**

[13] S. Macenski, M. Booker, and J. Wallace, "Open-source, cost-aware kinematically feasible
planning for mobile and surface robotics," arXiv:2401.13078, 2024. (Smac planners)

[14] S. Macenski, S. Singh, F. Martín, and J. Ginés, "Regulated pure pursuit for robot path
tracking," *Autonomous Robots*, vol. 47, pp. 685–694, 2023.

[15] G. Williams, P. Drews, B. Goldfain, J. M. Rehg, and E. A. Theodorou, "Aggressive driving
with model predictive path integral control," in *Proc. IEEE ICRA*, 2016. (MPPI)

[16] R. C. Coulter, "Implementation of the Pure Pursuit Path Tracking Algorithm," Carnegie
Mellon University, Robotics Institute, Tech. Rep. CMU-RI-TR-92-01, 1992.

**Perception and tracking**

[17] G. Jocher, A. Chaurasia, and J. Qiu, *Ultralytics YOLOv8*, software, 2023.
https://github.com/ultralytics/ultralytics (Also cite the original YOLO paper: J. Redmon,
S. Divvala, R. Girshick, and A. Farhadi, "You Only Look Once: Unified, real-time object
detection," in *Proc. IEEE CVPR*, 2016.)

[18] K. O. Arras, Ó. Martínez Mozos, and W. Burgard, "Using boosted features for the detection
of people in 2D range data," in *Proc. IEEE ICRA*, 2007.

[19] A. Bewley, Z. Ge, L. Ott, F. Ramos, and B. Upcroft, "Simple online and realtime tracking,"
in *Proc. IEEE ICIP*, 2016. (SORT)

[22] R. E. Kalman, "A new approach to linear filtering and prediction problems," *Journal of
Basic Engineering*, vol. 82, no. 1, pp. 35–45, 1960.

[23] H. W. Kuhn, "The Hungarian method for the assignment problem," *Naval Research Logistics
Quarterly*, vol. 2, pp. 83–97, 1955.

[24] Y. Zhang et al., "ByteTrack: Multi-object tracking by associating every detection box," in
*Proc. ECCV*, 2022.

[26] K. Bernardin and R. Stiefelhagen, "Evaluating multiple object tracking performance: The
CLEAR MOT metrics," *EURASIP Journal on Image and Video Processing*, 2008. (MOTA)

[27] E. Ristani, F. Solera, R. Zou, R. Cucchiara, and C. Tomasi, "Performance measures and a
data set for multi-target, multi-camera tracking," in *Proc. ECCV Workshops*, 2016. (IDF1)

**Human-aware navigation**

[20] T. Kruse, A. K. Pandey, R. Alami, and A. Kirsch, "Human-aware robot navigation: A survey,"
*Robotics and Autonomous Systems*, vol. 61, no. 12, pp. 1726–1743, 2013.

[21] C. Mavrogiannis et al., "Core challenges of social robot navigation: A survey," *ACM
Transactions on Human-Robot Interaction*, vol. 12, no. 3, 2023.

[25] E. T. Hall, *The Hidden Dimension*. Garden City, NY, USA: Doubleday, 1966. (Proxemics)

**General**

[28] R. Siegwart, I. R. Nourbakhsh, and D. Scaramuzza, *Introduction to Autonomous Mobile
Robots*, 2nd ed. Cambridge, MA, USA: MIT Press, 2011. (Kinematics of wheeled robots,
sensors, localisation)
