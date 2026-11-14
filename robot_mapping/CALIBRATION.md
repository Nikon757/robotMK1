# Calibration guide

Everything the mappers need to know about the geometry lives in `config/robot.yaml`. This file
explains what each item means, how to measure it, and which tool refines it.

## 0. Frames and the mount blocks

All frames follow ROS REP-103: **x forward, y left, z up**, angles in degrees.

* `base_link` — the robot frame: on the **floor**, under the centre of the **rear axle**.
* `camera.mount` — position of the RGB camera's optical centre (the middle lens of the OAK-D Lite)
  in `base_link`, and its orientation. `pitch > 0` = camera looking down.
* `lidar.mount` — position of the LiDAR's rotation axis (centre of the black puck) in `base_link`.
  On RobotMK1 the LiDAR sits **slightly in front of and below the camera**, e.g.
  `camera: x 0.05, z 0.22` and `lidar: x 0.12, z 0.15` (defaults; measure yours).

Measure x/y/z with a ruler to the nearest 5 mm; that is enough as a starting point because the
tools below refine the angles and the x/y offset.

```
              z ^
                |   camera (x_c, z_c)  <- optical centre, middle lens
                |     [oak]
                |    ___/
                |   (lidar) (x_l, z_l)  <- puck centre, x_l > x_c, z_l < z_c
   rear axle    |____|_____________________> x (forward)
   base_link  --O----+--------------
```

## 1. OAK-D Lite

### 1.1 Intrinsics and stereo (factory)

The OAK-D Lite ships with a factory calibration stored in its EEPROM (intrinsics of all three
cameras, distortion, stereo extrinsics). Both mappers read it automatically. To see it:

```bash
python run.py calib-camera            # -> config/oak_calib.json
```

### 1.2 Depth check (2 minutes)

Point the camera squarely at a flat, textured wall at a measured distance (0.8–2 m):

```bash
python run.py calib-camera --distance 1.20
```

The tool prints the median depth in the image centre and the error. Up to ±3 % is normal for
stereo depth; a larger *systematic* error means the stereo calibration is off (camera was dropped,
lens moved) → recalibrate (1.3). Also check that the depth image is well filled (`run.py check`
prints the percentage of valid pixels; > 60 % on a normal indoor scene).

### 1.3 Recalibration (only if 1.2 fails)

Use Luxonis' calibration script with a printed ChArUco board:

```bash
git clone https://github.com/luxonis/depthai.git && cd depthai
python3 install_requirements.py
python3 calibrate.py -s <square size in cm> -brd OAK-D-LITE -db   # follow the on-screen instructions
```

It writes the new calibration into the EEPROM; the mappers pick it up automatically.

### 1.4 IMU orientation (important — batch dependent)

The BMI270 in the OAK-D Lite is mounted differently from the OAK-D, and its axis mapping differs
between production batches, so the rotation "IMU → camera body" cannot be hard-coded. The
mappers use the IMU for (a) the rotation prior between frames and (b) gravity alignment, so the
vertical axis must be right.

```bash
python run.py calib-imu --apply
```

Procedure (1 minute): pose A — camera level, looking forward, robot still (3 s); pose B — tilt the
camera **nose down** by 20–60° (pitch only), hold still (3 s). The two gravity vectors give the
full rotation, which is written to `camera.imu_rpy_deg` together with `camera.imu_verified: true`.
Verification: `python run.py check` must show *accel in body frame ≈ [0, 0, +9.8]* with the camera
level.

The fusion mapper additionally auto-corrects the vertical axis from gravity at start-up (robot must be
still for the first 2 s), so a slightly wrong value is harmless there; RTAB-Map uses the IMU only if
`imu_verified` is true (`Optimizer/GravitySigma`), otherwise it maps without it.

Gyro bias is measured at every start (the first ~1.5 s must be motionless).

### 1.5 Camera mount angles (from the floor)

`calib-lidar-camera` (section 3) fits the floor plane in the depth image and prints the camera
`roll`, `pitch` and `z` that make the floor horizontal and at z = 0. Apply them with
`--apply --apply-camera`. A wrong pitch makes walls lean in the 3-D map and the floor appear as
an obstacle in the occupancy grid.

## 2. LiDAR (STL-27L)

The LiDAR needs no intrinsic calibration (range/angle are factory calibrated, ±15 mm). What has
to be established is *where it points* and *where it sits*.

### 2.1 Forward angle

The raw angle 0° of the sensor is arbitrary (near the cable). The angle that points along the
robot's +x axis is `lidar.forward_angle_deg`:

```bash
python run.py lidar-view --find-forward --apply
```

Put a box or book 30–60 cm straight ahead of the robot (centred on the chassis axis, nothing
else closer than ~1 m). The tool averages 20 revolutions, finds the nearest cluster and stores its
raw angle. Check with the live view (`python run.py lidar-view`): forward is at the top of the polar
plot, and points on the left of the robot must appear on the left (positive bearing). If left/right
are mirrored the sensor is mounted upside down → set `lidar.clockwise: false`.

### 2.2 The 180° field of view

Because the LiDAR sits low and in front of the camera, the rear half of each revolution hits the
camera and the chassis. `lidar.fov_deg: 180` keeps only bearings within ±90° of forward; the rest
is discarded **in both mappers** (Python driver `scan_to_points`, ROS node `scan_fov_filter.py`,
which sets those beams to NaN). Consequences:

* The occupancy grid and the LiDAR map are only observed *ahead* of the robot; areas behind it are
  filled in as the robot turns or as the camera sees them (`Grid/Sensor: 2` in RTAB-Map also uses the
  depth camera for the grid).
* Loop-closure refinement with ICP still works (it needs overlap, not 360°).
* If the blocked sector is larger or asymmetric, reduce `fov_deg` (e.g. 160) — check in
  `lidar-view` that no points of the robot itself (range < 0.15 m at fixed angles) survive the crop,
  or raise `lidar.min_range_m`.

### 2.3 Level check

In `lidar-view`, a flat wall must appear as a straight line at constant range. If the line bends
away (range grows towards the sides) or returns from the floor appear on one side, the puck is
tilted: shim it level and set `lidar.mount.roll/pitch` only for a residual ≤ 1°.

## 3. LiDAR ↔ camera extrinsics (and the camera mount)

After the ruler values are in `robot.yaml`, refine them from real data:

```bash
python run.py calib-lidar-camera                  # measure, print suggestions
python run.py calib-lidar-camera --apply --apply-camera
```

Setup: robot 1.5–2.5 m in front of a flat wall, with something that breaks the symmetry in the camera
view (a vertical pole / broom handle 1 m ahead and 0.5 m to one side, a box, or a room corner).
Robot still, camera centred.

What it does:

1. **Floor plane** in the lower part of the depth image → camera `roll`, `pitch`, `z`.
   Apply these first if the tilt is > 1° or the floor height is off by > 3 cm, then re-run.
2. **Horizontal slice** of the depth cloud at the LiDAR's height (±4 cm) vs. the LiDAR scan
   (only the part inside the camera's field of view): 2-D ICP gives the planar correction
   (Δx, Δy, Δyaw) of the LiDAR mount → `lidar.mount.x`, `y`, `yaw`. Expected accuracy 1–3 cm and
   ~1° at 2 m (stereo-depth limited). ICP fitness should be > 0.8; if it is low, the two sensors do
   not see the same thing: check `forward_angle_deg` and the mount heights.

The LiDAR `z`, `roll`, `pitch` are not observable from a single wall and stay as measured by hand.

## 4. Time synchronisation

Camera and IMU share the OAK's clock (synchronised to the Pi's monotonic clock by DepthAI);
LiDAR scans are stamped on arrival. At mapping speeds (≤ 0.3 m/s) the resulting error is below
1 cm and is ignored. In ROS the same holds (`approx_sync` with 50–200 ms windows).

## 5. Checklist before the first real map

1. `python run.py check` — three devices OK, gravity ≈ [0, 0, 9.8], > 60 % valid depth.
2. `robot.yaml` mounts measured with a ruler.
3. `lidar-view --find-forward --apply`, then live view: forward at the top, left is left.
4. `calib-imu --apply`.
5. `calib-lidar-camera --apply --apply-camera` (re-run until the floor tilt is < 1°).
6. Short test map with `--no-record --viz`: walls should be straight in the top-down view and the
   LiDAR points (red in the fusion view) should lie on the camera points of the walls.
