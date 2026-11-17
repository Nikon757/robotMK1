"""Driving the robot: motor power from the Pi, distance and heading from the encoders.

The board takes motor *power* (@M), not speed, so the robot's speed depends on load and
floor. Every motion here therefore ends on what the encoders measured, not on time:
- distance: the mean of the two rear wheels;
- heading: the difference between the two rear wheels divided by the track width (the rear
  wheels don't steer; docs/MOTORS_AND_ENCODERS.md §7).
Both need chassis.counts_per_metre in config/robot.yaml (scripts/00_calibrate_encoders.py).

    from modules.engines import Robot
    with Robot() as robot:
        robot.forward(0.5)
        robot.turn_by(45)          # forward arc, + = left
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass, replace
from typing import Callable, Optional

from .board import NiumoBoard, open_board
from .config import load_config

LOOP_S = 0.05               # control loop; @M is re-sent every pass (board watchdog: 0.5 s)
STEER_SETTLE_S = 0.3        # time for the steering servo to move by a full lock (0 -> 1)
STOP_SETTLE_S = 0.4         # time for the robot to roll to a stop after the motors are cut
HEADING_GAIN = 2.0          # straight driving: steering amount per radian of heading error
MAX_DRIFT_DEG = 20.0        # straight driving: give up beyond this heading error
CHECK_FROM_M = 0.05         # turns: the direction is checked between these two distances,
CHECK_TO_M = 0.12           #   after the wheels have rolled and finished any steering change
TAPER_DEG = 15.0            # turns: the wheels start straightening this far before the target
FINISH_TOL_DEG = 1.0        # turns: done this close to the target heading


class MotionError(RuntimeError):
    """A motion was stopped early: stall, board restart, wrong steering direction, timeout."""


class SteeringDirectionError(MotionError):
    """A turn went the wrong way, or didn't turn at all."""


@dataclass
class Pose:
    x: float = 0.0          # metres, forward of where the Robot object was created
    y: float = 0.0          # metres, to the left of it
    heading: float = 0.0    # radians, counter-clockwise (left) positive
    distance: float = 0.0   # metres driven by the rear-axle centre, signed (backwards < 0)


class Odometry:
    """Pose of the rear-axle centre from the cumulative counts of the two rear wheels."""

    def __init__(self, counts_per_metre: float, track_m: float):
        self.counts_per_metre = counts_per_metre
        self.track_m = track_m
        self.pose = Pose()
        self._last: Optional[tuple[int, int]] = None

    def update(self, left: int, right: int) -> Pose:
        if self._last is not None:
            d_left = (left - self._last[0]) / self.counts_per_metre
            d_right = (right - self._last[1]) / self.counts_per_metre
            ds = (d_left + d_right) / 2
            d_heading = (d_right - d_left) / self.track_m
            mid = self.pose.heading + d_heading / 2
            self.pose.x += ds * math.cos(mid)
            self.pose.y += ds * math.sin(mid)
            self.pose.heading += d_heading
            self.pose.distance += ds
        self._last = (left, right)
        return self.pose

    def restart(self) -> None:
        """Forget the last counts: after a board restart the counts start again from 0."""
        self._last = None


class Robot:
    """Drive motors, steering and encoders. Values come from config/robot.yaml."""

    def __init__(self, board: Optional[NiumoBoard] = None, config: Optional[dict] = None):
        self.cfg = config or load_config()
        chassis, steering, drive = self.cfg["chassis"], self.cfg["steering"], self.cfg["drive"]
        if not chassis.get("counts_per_metre"):
            raise MotionError("chassis.counts_per_metre is not set in config/robot.yaml. "
                              "Run scripts/00_calibrate_encoders.py first.")
        self.wheelbase = float(chassis["wheelbase_m"])
        self.track = float(chassis["rear_track_m"])
        self.steer_center = int(steering["center"])
        self.steer_left = int(steering["left"])
        self.steer_right = int(steering["right"])
        self.full_lock = math.radians(float(steering["full_lock_deg"]))
        self.power = int(drive["power"])
        self.ramp_s = float(drive["ramp_s"])
        self.stall_s = float(drive["stall_s"])

        self.board = board or open_board(self.cfg["board"].get("port"))
        # The heading uses the effective track from the turn calibration when there is one:
        # wide tyres and side-slip make it smaller than the measured tyre-centre distance.
        odometry_track = float(chassis.get("odometry_track_m") or self.track)
        self.odometry = Odometry(float(chassis["counts_per_metre"]), odometry_track)
        self._steer_amount: Optional[float] = None
        self._sync()
        self.steer(0.0)

    # ---- basics -----------------------------------------------------------------------
    @property
    def pose(self) -> Pose:
        return self.odometry.pose

    def steer(self, amount: float, wait: bool = True) -> None:
        """Front wheels: -1 = full right, 0 = straight, +1 = full left."""
        amount = max(-1.0, min(1.0, amount))
        end = self.steer_left if amount > 0 else self.steer_right
        command = round(self.steer_center + abs(amount) * (end - self.steer_center))
        change = 1.0 if self._steer_amount is None else abs(amount - self._steer_amount)
        self.board.steer(command)
        self._steer_amount = amount
        if wait and change > 0.05:          # a full left-to-right swing (2.0) waits twice as long
            time.sleep(STEER_SETTLE_S * max(1.0, change))

    def stop(self) -> None:
        self.board.halt()

    def close(self) -> None:
        """Straighten the steering and stop the motors. Runs at the end of every
        `with Robot()` block, also after an error or Ctrl+C."""
        try:
            self.steer(0.0)
        except OSError:
            pass                            # the board connection is gone: nothing to straighten
        finally:
            self.board.close()              # also stops the motors

    def __enter__(self) -> "Robot":
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    # ---- motions ----------------------------------------------------------------------
    def forward(self, distance_m: float, power: Optional[int] = None, timeout: float = 30.0) -> Pose:
        """Drive straight for distance_m metres (negative = backwards), holding the heading
        the robot had at the start."""
        start = replace(self._sync())
        goal = abs(distance_m)
        level = (1 if distance_m >= 0 else -1) * abs(power or self.power)
        self.steer(0.0)

        def travelled(pose: Pose) -> float:
            return abs(pose.distance - start.distance)

        def done(pose: Pose) -> bool:
            if abs(pose.heading - start.heading) > math.radians(MAX_DRIFT_DEG):
                raise MotionError(f"The robot drifted more than {MAX_DRIFT_DEG:.0f} degrees off its "
                                  "heading. If it curved away further and further, steering.left and "
                                  "steering.right are swapped in config/robot.yaml.")
            return travelled(pose) >= goal

        def slow_down(pose: Pose) -> float:         # gentler in the last 10 cm: less overshoot
            return max(0.5, min(1.0, (goal - travelled(pose)) / 0.10))

        direction = 1 if level >= 0 else -1         # backwards, steering acts the other way

        def steering(pose: Pose) -> float:          # hold the start heading
            error = start.heading - pose.heading
            return max(-0.4, min(0.4, HEADING_GAIN * error * direction))

        end = self._run(level, done, steer_fn=steering, slow_down=slow_down,
                        timeout=timeout, what="Driving straight")
        self.steer(0.0, wait=False)
        return end

    def turn_by(self, angle_deg: float, steer: float = 1.0, power: Optional[int] = None,
                reverse: bool = False, max_distance_m: Optional[float] = None,
                timeout: float = 30.0) -> Pose:
        """Drive an arc until the heading has changed by angle_deg (+ = left).

        steer: how hard to steer, 0..1 of full lock. reverse: drive backwards; the steering
        is then turned the other way, so the robot still turns towards angle_deg.
        max_distance_m: stop after this distance even if the angle isn't reached yet."""
        start = replace(self._sync())
        turn = 1 if angle_deg >= 0 else -1
        backwards = -1 if reverse else 1
        goal = math.radians(abs(angle_deg))
        check_from: Optional[float] = None
        checked = False

        def steering(pose: Pose) -> float:
            """Full steer until TAPER_DEG before the target, then less and less: the wheels
            straighten while the robot still rolls, like a car leaving a bend. Stationary
            tyres grip the floor too hard for the servo to straighten them fully."""
            remaining = goal - (pose.heading - start.heading) * turn
            amount = min(abs(steer), max(0.0, remaining) / math.radians(TAPER_DEG))
            return turn * backwards * amount

        self.steer(steering(start))

        def done(pose: Pose) -> bool:
            nonlocal check_from, checked
            turned = (pose.heading - start.heading) * turn
            travelled = abs(pose.distance - start.distance)
            # Judge the direction only once the wheels have rolled a little: standing tyres
            # resist the steering, so the wheels may finish their swing only while rolling.
            if check_from is None and travelled >= CHECK_FROM_M:
                check_from = turned
            if not checked and check_from is not None and travelled >= CHECK_TO_M:
                change = turned - check_from
                if change < -math.radians(1):
                    raise SteeringDirectionError("The robot turned the wrong way: swap steering.left "
                                                 "and steering.right in config/robot.yaml.")
                if change < math.radians(1):
                    raise SteeringDirectionError("The robot isn't turning: is the steering servo "
                                                 "plugged into PB14 and the battery on?")
                checked = True
            if max_distance_m is not None and travelled >= max_distance_m:
                return True
            return turned >= goal - math.radians(FINISH_TOL_DEG)

        return self._run(backwards * abs(power or self.power), done, steer_fn=steering,
                         timeout=timeout, what="Turning")

    def turn_around(self, angle_deg: float = 180.0, leg_m: float = 0.30,
                    power: Optional[int] = None, max_legs: int = 12,
                    on_leg: Optional[Callable[[int, str, float], None]] = None) -> tuple[Pose, int]:
        """Turn on the spot the way a car does: forward arcs at full lock alternate with
        backward arcs steered the other way, each at most leg_m long, until the heading has
        changed by angle_deg (+ = left). Returns the final pose and the number of legs.
        on_leg(leg number, "forward"/"backward", degrees turned so far) is called after each leg."""
        start = replace(self._sync())
        turn = 1 if angle_deg >= 0 else -1
        goal = math.radians(abs(angle_deg))
        legs = 0
        while True:
            remaining = goal - (self.pose.heading - start.heading) * turn
            if remaining <= math.radians(2):
                break
            if legs >= max_legs:
                raise MotionError(f"The turn wasn't finished after {max_legs} legs.")
            way = "backward" if legs % 2 else "forward"
            try:
                self.turn_by(turn * math.degrees(remaining), power=power,
                             reverse=legs % 2 == 1, max_distance_m=leg_m)
            except SteeringDirectionError:
                if legs == 0:
                    raise
                raise MotionError(
                    f"Leg {legs + 1} ({way}) didn't turn the way leg 1 did, so the steering didn't "
                    "reach its new position. The front tyres resist steering while the robot stands "
                    "still: check that the steering servo has power and isn't straining.") from None
            legs += 1
            if on_leg is not None:
                on_leg(legs, way, math.degrees(self.pose.heading - start.heading))
        self.steer(0.0)
        return self.pose, legs

    # ---- control loop -----------------------------------------------------------------
    def _sync(self) -> Pose:
        enc = self.board.wait_for("encoders")
        return self.odometry.update(enc.left, enc.right)

    def _wheel_powers(self, power: float) -> tuple[float, float]:
        """Share the power between the rear wheels the way their speeds differ in a turn:
        the inner wheel rolls a shorter arc than the outer one."""
        k = self.track * math.tan((self._steer_amount or 0.0) * self.full_lock) / (2 * self.wheelbase)
        return power * (1 - k), power * (1 + k)

    def _run(self, power: float, done: Callable[[Pose], bool], *,
             steer_fn: Optional[Callable[[Pose], float]] = None,
             slow_down: Optional[Callable[[Pose], float]] = None,
             timeout: float = 30.0, what: str = "The motion") -> Pose:
        """Power the motors until done(pose) is true; always stops the motors at the end.
        steer_fn(pose) gives the steering amount on every pass; slow_down(pose) scales the power."""
        board = self.board
        reboots = board.reboots
        pose = self._sync()
        started = last_move = time.monotonic()
        last_counts = (board.encoders.left, board.encoders.right)
        try:
            while not done(pose):
                now = time.monotonic()
                if board.reboots != reboots:
                    self.odometry.restart()
                    raise MotionError("The board restarted during the motion (a power dip?).")
                if now - started > timeout:
                    raise MotionError(f"{what} didn't finish within {timeout:.0f} s.")
                counts = (board.encoders.left, board.encoders.right)
                if counts != last_counts:
                    last_counts, last_move = counts, now
                elif now - last_move > self.stall_s and now - started > self.ramp_s:
                    raise MotionError("The wheels aren't turning although the motors are powered: "
                                      "battery on? Robot blocked? Otherwise raise drive.power in "
                                      "config/robot.yaml.")
                if steer_fn is not None:
                    self.steer(steer_fn(pose), wait=False)
                level = power * (min(1.0, (now - started) / self.ramp_s) if self.ramp_s > 0 else 1.0)
                if slow_down is not None:
                    level *= slow_down(pose)
                board.drive(*self._wheel_powers(level))
                time.sleep(LOOP_S)
                pose = self._sync()
        finally:
            board.halt()
        time.sleep(STOP_SETTLE_S)
        return self._sync()
