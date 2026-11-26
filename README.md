# robotMK1

A mapping robot on an Ackermann chassis:
- a Raspberry Pi 5 with a Hailo-8 accelerator;
- an OAK-D Lite depth camera on a pan servo;
- an LDROBOT STL-27L LiDAR;
- a Niumo V5 control board (STM32F103) driving the motors and servos.

## Quick start on the Pi

```bash
cd ~/robotMK1 && bash setup_venv.sh          # once: virtual environment + packages
source .venv/bin/activate && python main.py  # console menu with the scripts
```

The full setup (USB names, camera, Hailo-8, Jupyter Lab, ROS 2 in Docker) is in
[docs/INSTALL_PI.md](docs/INSTALL_PI.md).

## Track demonstration

From `python main.py`, enter `6 --dry-run` to inspect the route or `6 --camera-only`
to check the depth overlay without opening the motor controller. Enter `6` to run
the full demonstration. The script prints the browser address for the live feed
before asking you to start.

Set the straight-leg distances at the top of [script 06](scripts/06_track_demo.py)
from the physical course. Measure each straight leg between the steering arcs
and allow room for the robot to sweep through each turn. The supplied distances
are starting values for rehearsal. Use the encoder distance and turn calibrations
from script 00 for distance and for heading when using wheel counts.
`PAUSE_S` controls the timed stop and `DEPTH_*` controls the camera inspection.

The demo uses the OAK-D Lite gyro for heading and the rear encoders for distance.
Keep the robot stationary and level while the camera gyro calibrates. Calibration
runs with the camera centred before departure and after the inspection. Lost gyro
samples stop the run. If your camera has no IMU, select `6 --heading wheels` to use
encoder heading. [OAK-D Lite IMU specifications](https://docs.luxonis.com/hardware/products/OAK-D%20Lite)

The demo reduces motor power during the last `TURN_SLOWDOWN_DEG` degrees of each
turn. `TURN_FINISH_POWER_FRACTION` sets the minimum power during that slowdown.
`TURN_STOP_LEAD_DEG` allows a few degrees for coasting. A 90° left turn with the gyro
uses `TURN_GYRO_LEFT_90_STOP_LEAD_DEG` based on the measured 9.9° of rollout. The
steering and motor power taper earlier by the same change in lead. In wheel-heading
mode, a 90° right turn uses `TURN_RIGHT_90_STOP_LEAD_DEG`. If the settled turn misses its
target, the error shows the heading when the Pi requested a halt, the settled
heading, and the count change for each wheel. The Pi repeats the halt command
while checking that the readings settle.

Run `6 --turn-test 90` in a clear area to test the left turn by itself and compare
the gyro and wheel estimates with the physical angle. Add `--heading wheels` to
run the same test using the board alone. Negative angles turn right.

At the second stop, the camera pans right. The detector marks the largest connected
patch of in-range stereo depth inside the inspection area and shows its median
distance. Place the target inside the configured distance band and keep nearby
background surfaces outside that band. If the detector times out, the run stops.
Once a box appears, press Enter in the terminal to recenter the camera and drive
the final measured leg. The bypass and finish use the configured route distances.
RGB and depth are paired on the camera before the Pi draws the overlay.
[Luxonis DepthAI v2 alignment example](https://github.com/luxonis/depthai-python/blob/v2.33.0.0/examples/StereoDepth/rgb_depth_aligned.py)

## Repository layout

| Path | Contents |
|---|---|
| [`main.py`](main.py) | Console menu: lists the scripts in `scripts/` and runs the one you pick |
| [`scripts/`](scripts/) | One file per task, numbered. **00** encoder calibration (run first), **01** 45° left turn, **02** turn around by driving back and forth, **03** drive forward, **04** drive while streaming the camera, **05** camera pan left-right-centre, **06** track demonstration with depth inspection. |
| [`modules/`](modules/) | Python code the scripts and notebooks import: `engines` (board link, motors, steering, encoders, odometry), `camera` (OAK-D stream, pan servo), `lidar` and `npu` (to come) |
| [`config/`](config/) | `robot.yaml`: all robot settings (geometry, calibration, limits, power); `99-robot-usb.rules`: fixed USB device names |
| [`firmware/`](firmware/) | Control-board firmware v2 (Keil MDK project); its [README](firmware/README.md) covers wiring, building, flashing and the serial protocol |
| [`docs/`](docs/) | Documentation (below) |
| [`ros2/`](ros2/) | ROS 2 mapping and navigation (to come; see the roadmap) |
| [`mapping_scripts/`](mapping_scripts/), [`depthai/`](depthai/) | Reserved for mapping and DepthAI work |
| [`pi_examples/`](pi_examples/) | Earlier standalone Pi scripts: board client, `hw_test.py`, `pan_sweep.py` |
| `requirements.txt`, `pyproject.toml`, `setup_venv.sh` | The Python virtual environment |

The earlier mapping package `robot_mapping/` was removed from the working tree. It is still in
the Git history (commit a9deefa), for example its ROS 2 Dockerfile.

## Documentation

| Document | What it covers |
|---|---|
| [docs/INSTALL_PI.md](docs/INSTALL_PI.md) | Installing everything on Raspberry Pi OS |
| [firmware/README.md](firmware/README.md) | Firmware features, wiring, build and flash, wire protocol, servo ranges |
| [docs/PI_GUIDE.md](docs/PI_GUIDE.md) | Talking to the board from the Pi: protocol, every module, examples |
| [docs/MOTORS_AND_ENCODERS.md](docs/MOTORS_AND_ENCODERS.md) | Motor control and encoder telemetry: signals, decoding, `#T`, odometry, calibration |
| [docs/CAMERA_PAN_CALIBRATION.md](docs/CAMERA_PAN_CALIBRATION.md) | How the camera pan servo was calibrated |
| [docs/DEVELOPMENT.md](docs/DEVELOPMENT.md) | Development record of firmware v2: decisions, problems and solutions, references |
| [docs/BASELINE_TEST_2026-11-17.md](docs/BASELINE_TEST_2026-11-17.md) | First test of the robot with the Pi scripts: procedure, problems found, fixes, open items |
| [docs/ROADMAP.md](docs/ROADMAP.md) | Next stages: ROS 2 mapping, Nav2, people tracking, evaluation plan |

## Status (November 2026)

- **Firmware v2:** verified on the robot. The steering, the camera pan (calibrated), both motors
  and both encoders all work.
- **Scripts 00–05:** tested against a simulated robot; not yet run on the real one. The encoder
  calibration (script 00) is the first step on the robot.
- **Inertial data:** comes from the camera's built-in IMU; there is no separate board IMU.
- **Next steps:** run the scripts on the robot, steering calibration, firmware speed control,
  then ROS 2 (mapping with slam_toolbox and RTAB-Map, navigation with Nav2).
