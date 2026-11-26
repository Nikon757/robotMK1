#!/usr/bin/env python3
"""Drive the demonstration track and show a depth-detected obstacle in the live stream."""
from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parents[1]))

from modules.camera import CameraPan, MjpegServer
from modules.engines import MotionError, Robot, load_config
from modules.script_tools import run


# ---- Settings to adjust at the track -----------------------------------------------
# Straight legs are measured between the starts and ends of the steering arcs.
# Turn angles are changes in heading. Positive is left and negative is right.
# Distances are rehearsal values. Set them from the actual course before driving.
BEFORE_PAUSE = (
    ("drive", 0.50, "Approach the first obstacle"),
    ("turn", -45.0, "Turn right to start the detour"),
    ("drive", 0.60, "Follow the diagonal"),
    ("turn", 45.0, "Return to the original heading"),
    ("drive", 0.25, "Approach the corner past the obstacle"),
    ("turn", 90.0, "Turn left past the obstacle"),
    ("drive", 0.35, "Cross to the left edge of the course"),
    ("turn", -90.0, "Turn right toward the top straight"),
    ("drive", 0.25, "Reach the top corner"),
    ("turn", -90.0, "Turn right onto the top straight"),
    ("drive", 0.55, "Reach the timed stop"),
)
AFTER_PAUSE = (
    ("drive", 0.55, "Continue along the top straight"),
    ("turn", -90.0, "Turn right onto the final straight"),
    ("drive", 0.55, "Reach the inspection stop"),
)
FINISH = (("drive", 0.80, "Continue to the finish line"),)
PAUSE_S = 5.0                         # 5 to 10 seconds on the top straight
DRIVE_POWER = None                    # None uses drive.power in config/robot.yaml
TURN_STEER = 1.0                      # fraction of full steering lock
TURN_SLOWDOWN_DEG = 30.0               # start reducing motor power this far from the target
TURN_FINISH_POWER_FRACTION = 0.45      # minimum fraction of drive power at the end of a turn
TURN_STOP_LEAD_DEG = 4.0              # default lead to allow for coasting
TURN_GYRO_LEFT_90_STOP_LEAD_DEG = 10.0 # measured rollout after halt was 9.9°
TURN_RIGHT_90_STOP_LEAD_DEG = 7.0     # 90° right turns in wheel-heading mode only
MAX_TURN_DISTANCE_M = 1.4            # stop if an arc reaches this length early
TURN_TOLERANCE_DEG = 5.0             # allowed settled heading error
MOTION_TIMEOUT_S = 20.0              # per leg or turn
IMU_RATE_HZ = 100
IMU_CALIBRATION_S = 1.0

# Pan positions and stream settings come from config/robot.yaml.
DEPTH_MIN_M = 0.35
DEPTH_MAX_M = 1.20
DEPTH_ROI = (0.18, 0.18, 0.82, 0.78)  # left, top, right, bottom as fractions of the image
DEPTH_MIN_PIXELS = 600               # connected pixels in a 640 × 360 frame
DEPTH_CONFIRM_FRAMES = 3
DETECTION_TIMEOUT_S = 12.0
CAMERA_START_TIMEOUT_S = 12.0


# ---- Route execution --------------------------------------------------------------
def describe_step(step):
    action, value, label = step
    if action == "drive":
        return f"{label} ({value:g} m)"
    side = "left" if value > 0 else "right"
    return f"{label} ({abs(value):g}° {side})"


def show_route() -> None:
    for step in BEFORE_PAUSE:
        print(" ", describe_step(step))
    print(f"  Stop for {PAUSE_S:g} s")
    for step in AFTER_PAUSE:
        print(" ", describe_step(step))
    print("  Stop, pan right, show the obstacle box, wait for Enter, and centre the camera")
    for step in FINISH:
        print(" ", describe_step(step))
    print("  Stop at the finish")


def wait_for_enter(prompt: str) -> None:
    try:
        input(prompt)
    except EOFError as exc:
        raise RuntimeError("The demonstration requires an interactive terminal") from exc


def require_live_video(camera, timeout: float = 2.0) -> None:
    last_id, _ = camera.wait_frame(timeout=0)
    _, frame = camera.wait_frame(last_id, timeout=timeout)
    if frame is None or not camera.running:
        raise RuntimeError("No live picture from the OAK-D Lite. Check its USB connection.")


def drive_turn(robot: Robot, angle_deg: float, label: str) -> None:
    gyro_heading = robot.heading_delta is not None
    stop_lead = TURN_STOP_LEAD_DEG
    approach_lead = 0.0
    if gyro_heading and angle_deg == 90.0:
        stop_lead = TURN_GYRO_LEFT_90_STOP_LEAD_DEG
        approach_lead = max(0.0, stop_lead - TURN_STOP_LEAD_DEG)
    elif not gyro_heading and angle_deg == -90.0:
        stop_lead = TURN_RIGHT_90_STOP_LEAD_DEG
    start_heading = robot.pose.heading
    start_enc = robot.board.encoders
    try:
        end = robot.turn_by(angle_deg, steer=TURN_STEER, power=DRIVE_POWER,
                            max_distance_m=MAX_TURN_DISTANCE_M, timeout=MOTION_TIMEOUT_S,
                            slowdown_deg=TURN_SLOWDOWN_DEG, finish_power_fraction=TURN_FINISH_POWER_FRACTION,
                            stop_lead_deg=stop_lead, approach_lead_deg=approach_lead,
                            settled_tolerance_deg=TURN_TOLERANCE_DEG)
    except RuntimeError as exc:
        raise MotionError(f"{label}. {exc}") from exc
    turned = math.degrees(end.heading - start_heading)
    if gyro_heading and start_enc is not None:
        enc = robot.board.encoders
        count_diff = (enc.right - start_enc.right) - (enc.left - start_enc.left)
        wheel_turn = math.degrees(count_diff / (robot.odometry.counts_per_metre * robot.odometry.track_m))
        print(f"Turned {turned:+.1f}° by gyro, {wheel_turn:+.1f}° by wheel encoders.", flush=True)
    else:
        print(f"Turned {turned:+.1f}° by wheel encoders.", flush=True)


def follow_route(robot: Robot, camera, steps) -> None:
    for step in steps:
        require_live_video(camera)
        action, value, label = step
        print(describe_step(step), flush=True)
        if action == "drive":
            if value > 0:
                robot.forward(value, power=DRIVE_POWER, timeout=MOTION_TIMEOUT_S)
            continue

        drive_turn(robot, value, label)


# ---- Stationary inspection --------------------------------------------------------
def inspect_obstacle(camera, finish_prompt: str) -> None:
    camera.inspect(True)
    print("Looking for the obstacle in the depth image ...", flush=True)
    obstacle = camera.wait_for_obstacle(DETECTION_TIMEOUT_S)
    if obstacle is None:
        require_live_video(camera)
        raise RuntimeError(f"No obstacle found in {DETECTION_TIMEOUT_S:g} s. "
                           "Check the target's placement and the DEPTH settings.")
    print(f"Obstacle detected at {obstacle.distance_m:.2f} m. See the box in the stream.")
    wait_for_enter(finish_prompt)
    require_live_video(camera)
    camera.inspect(False)


def calibrate_gyro(yaw) -> None:
    print("Calibrating the OAK gyro. Keep the robot still on level ground.", flush=True)
    yaw.calibrate(IMU_CALIBRATION_S)


def drive_demo(cfg, camera) -> None:
    show_route()
    wait_for_enter("Confirm the measured route is clear, then press Enter to start. Ctrl+C stops the run. ")
    require_live_video(camera)
    yaw = camera.yaw
    with Robot(config=cfg, heading_delta=yaw.delta if yaw else None) as robot:
        pan = CameraPan(robot.board, cfg)
        if yaw is not None:
            calibrate_gyro(yaw)
        follow_route(robot, camera, BEFORE_PAUSE)
        robot.stop()
        print(f"Stopped for {PAUSE_S:g} s ...", flush=True)
        time.sleep(PAUSE_S)

        follow_route(robot, camera, AFTER_PAUSE)
        robot.stop()
        print("Stopped for inspection. Camera turning right ...", flush=True)
        pan.look_right()
        inspect_obstacle(camera, "Press Enter to centre the camera and drive to the finish. ")
        pan.look_center()
        if yaw is not None:
            calibrate_gyro(yaw)
        follow_route(robot, camera, FINISH)
        robot.stop()
        print("Finished. The robot is stopped.")


def test_turn(cfg, angle: float, yaw=None) -> None:
    print(f"Single {angle:+g}° test turn. Leave room for the arc and for the robot to roll to a stop.")
    wait_for_enter("Press Enter once the test area is clear. Ctrl+C stops the run. ")
    with Robot(config=cfg, heading_delta=yaw.delta if yaw else None) as robot:
        if yaw is not None:
            CameraPan(robot.board, cfg)
            calibrate_gyro(yaw)
        drive_turn(robot, angle, "Test turn")
        print("The robot is stopped.")


# ---- Command line and setup -------------------------------------------------------
def validate_settings() -> None:
    for action, value, label in (*BEFORE_PAUSE, *AFTER_PAUSE, *FINISH):
        if not math.isfinite(value) or not ((action == "drive" and value >= 0)
                                            or (action == "turn" and 0 < abs(value) <= 180)):
            raise RuntimeError(f"Invalid route setting for {label}: {action} {value}")
    if not 0 < TURN_STEER <= 1 or not 5 <= PAUSE_S <= 10:
        raise RuntimeError("Set TURN_STEER in (0, 1] and PAUSE_S from 5 to 10 seconds")
    if DRIVE_POWER is not None and not 0 < DRIVE_POWER <= 1000:
        raise RuntimeError("Set DRIVE_POWER between 1 and 1000, or use None for drive.power")
    if not 0 < TURN_FINISH_POWER_FRACTION <= 1:
        raise RuntimeError("Set TURN_FINISH_POWER_FRACTION in (0, 1]")
    if not 0 <= TURN_STOP_LEAD_DEG <= TURN_TOLERANCE_DEG:
        raise RuntimeError("Set TURN_STOP_LEAD_DEG between zero and TURN_TOLERANCE_DEG")
    if not 0 <= TURN_GYRO_LEFT_90_STOP_LEAD_DEG <= 18.0:
        raise RuntimeError("Set TURN_GYRO_LEFT_90_STOP_LEAD_DEG between zero and 18 degrees")
    if not 0 <= TURN_RIGHT_90_STOP_LEAD_DEG <= 18.0:
        raise RuntimeError("Set TURN_RIGHT_90_STOP_LEAD_DEG between zero and 18 degrees")
    left, top, right, bottom = DEPTH_ROI
    if (not 0 <= left < right <= 1 or not 0 <= top < bottom <= 1
            or round(right * 640) - round(left * 640) < 3
            or round(bottom * 360) - round(top * 360) < 3):
        raise RuntimeError("Set DEPTH_ROI to a valid area at least 3 pixels wide and high")
    if not 0 < DEPTH_MIN_M < DEPTH_MAX_M or DEPTH_MIN_PIXELS < 1 or DEPTH_CONFIRM_FRAMES < 1:
        raise RuntimeError("Check the depth range, minimum area and confirmation frame count")
    timing_and_limits = (TURN_SLOWDOWN_DEG, MAX_TURN_DISTANCE_M, TURN_TOLERANCE_DEG, MOTION_TIMEOUT_S,
                         DETECTION_TIMEOUT_S, CAMERA_START_TIMEOUT_S, IMU_CALIBRATION_S)
    if any(not math.isfinite(value) or value <= 0 for value in timing_and_limits):
        raise RuntimeError("Time, angle and distance limits must be positive and finite")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--dry-run", action="store_true", help="print the course without opening hardware")
    mode.add_argument("--camera-only", action="store_true",
                      help="test the detector and stream with the robot stationary")
    mode.add_argument("--turn-test", type=float, metavar="DEG",
                      help="drive a single test arc without video streaming (negative = right)")
    parser.add_argument("--heading", choices=("gyro", "wheels"), default="gyro",
                        help="heading feedback for driving (default: OAK gyro)")
    parser.add_argument("--port", type=int, help="HTTP port (default: camera_stream.port)")
    args = parser.parse_args(argv)

    def work() -> None:
        validate_settings()
        if args.dry_run:
            show_route()
            return
        angle = args.turn_test
        if angle is not None and not 0 < abs(angle) <= 180:
            raise RuntimeError("Set the test angle between -180° and +180° and away from zero")
        cfg = load_config()
        if angle is not None and args.heading == "wheels":
            test_turn(cfg, angle)
            return

        from modules.camera.depth_obstacle import OakObstacleCamera, ObstacleSettings

        stream = cfg["camera_stream"]
        port = int(stream["port"]) if args.port is None else args.port
        settings = ObstacleSettings(DEPTH_MIN_M, DEPTH_MAX_M, DEPTH_ROI,
                                    DEPTH_MIN_PIXELS, DEPTH_CONFIRM_FRAMES)
        use_gyro = args.heading == "gyro" and not args.camera_only
        with OakObstacleCamera(int(stream["fps"]), int(stream["jpeg_quality"]), settings,
                               imu_rate_hz=IMU_RATE_HZ if use_gyro else None) as camera:
            if angle is not None:
                test_turn(cfg, angle, camera.yaw)
                return
            try:
                server = MjpegServer(camera, port).start()
            except OSError as exc:
                raise RuntimeError(f"Could not serve the camera on port {port} ({exc})") from exc
            try:
                require_live_video(camera, timeout=CAMERA_START_TIMEOUT_S)
                print("Open the live stream in your browser")
                for url in server.urls():
                    print(" ", url)
                if args.camera_only:
                    wait_for_enter("Aim the camera at the target, then press Enter to start detection. ")
                    inspect_obstacle(camera, "Press Enter to end the camera test. ")
                else:
                    drive_demo(cfg, camera)
            finally:
                server.stop()

    return run(work)


if __name__ == "__main__":
    sys.exit(main())
