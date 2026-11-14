# robotMK1

A mapping robot on an Ackermann chassis:
- a Raspberry Pi 5 with a Hailo-8 accelerator;
- an OAK-D Lite depth camera on a pan servo;
- an LDROBOT STL-27L LiDAR;
- a Niumo V5 control board (STM32F103) driving the motors and servos.

## Repository layout

| Folder | Contents |
|---|---|
| [`firmware/`](firmware/) | Control-board firmware v2 (Keil MDK project); its [README](firmware/README.md) covers wiring, building, flashing and the serial protocol |
| [`docs/`](docs/) | Project documentation (below) |
| [`pi_examples/`](pi_examples/) | Standalone Python for the Pi: board client, hardware test, camera-pan sweep |
| [`robot_mapping/`](robot_mapping/) | Mapping software: RTAB-Map in ROS 2 Docker, and a pure-Python mapper |

ROS 2 navigation packages will be added as described in the roadmap.

## Documentation

| Document | What it covers |
|---|---|
| [firmware/README.md](firmware/README.md) | Firmware features, wiring, build and flash, wire protocol, servo ranges |
| [docs/PI_GUIDE.md](docs/PI_GUIDE.md) | Talking to the board from the Raspberry Pi: protocol, every module, Python client, examples |
| [docs/MOTORS_AND_ENCODERS.md](docs/MOTORS_AND_ENCODERS.md) | Motor control and encoder telemetry: signals, decoding, `#T`, odometry, calibration |
| [docs/CAMERA_PAN_CALIBRATION.md](docs/CAMERA_PAN_CALIBRATION.md) | How the camera pan servo was calibrated: method, commands, results, accuracy |
| [docs/DEVELOPMENT.md](docs/DEVELOPMENT.md) | Development record of firmware v2: decisions, problems and solutions, references |
| [docs/ROADMAP.md](docs/ROADMAP.md) | Next stages: ROS 2 mapping, Nav2, people tracking, project structure, evaluation plan |
| [robot_mapping/README.md](robot_mapping/README.md) | The mapping software and how to run it |

## Status (November 2026)

- **Firmware v2:** verified on the robot. The steering, the camera pan (calibrated), both
  motors and both encoders all work.
- **Mapping software:** written; not yet run on the real robot.
- **Inertial data:** comes from the camera's built-in IMU; there is no separate board IMU.
- **Next steps:** chassis calibration (encoders, steering), the first real map, firmware
  speed control, then the ROS 2 base. See [docs/ROADMAP.md](docs/ROADMAP.md).
