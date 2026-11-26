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

from .board import Encoders, NiumoBoard, open_board
from .config import load_config

LOOP_S = 0.05               # control loop; @M is re-sent every pass (board watchdog: 0.5 s)
STEER_SETTLE_S = 0.3        # time for the steering servo to move by a full lock (0 -> 1)
STOP_SETTLE_S = 0.4         # minimum time to observe the wheels after the halt command
STOP_QUIET_MS = 150         # both encoder counts must stay still for this much board time
STOP_TIMEOUT_S = 1.0       # maximum time to wait for a confirmed stop
ENCODER_MAX_AGE_S = 0.15   # firmware sends encoder counts every 20 ms
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

    def update(self, left: int, right: int, heading_delta: Optional[float] = None) -> Pose:
        if self._last is not None:
            d_left = (left - self._last[0]) / self.counts_per_metre
            d_right = (right - self._last[1]) / self.counts_per_metre
            ds = (d_left + d_right) / 2
            d_heading = (d_right - d_left) / self.track_m if heading_delta is None else heading_delta
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

    def __init__(self, board: Optional[NiumoBoard] = None, config: Optional[dict] = None, *,
                 heading_delta: Optional[Callable[[bool], float]] = None):
        """Optional heading callback returns radians since its last read. True resets its baseline."""
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
        self.heading_delta = heading_delta
        self._encoder_epoch = self.board.reboots
        self._steer_amount: Optional[float] = None
        enc = self._encoder_sample()
        self.odometry.update(enc.left, enc.right)
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
        start = replace(self._sync(reset_heading=True))
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

        end, _, _ = self._run(level, done, steer_fn=steering, slow_down=slow_down,
                              timeout=timeout, what="Driving straight")
        self.steer(0.0, wait=False)
        return end

    def turn_by(self, angle_deg: float, steer: float = 1.0, power: Optional[int] = None,
                reverse: bool = False, max_distance_m: Optional[float] = None,
                timeout: float = 30.0, *, slowdown_deg: float = 0.0,
                finish_power_fraction: float = 0.45,
                stop_lead_deg: Optional[float] = None,
                settled_tolerance_deg: Optional[float] = None) -> Pose:
        """Drive an arc until the heading has changed by angle_deg (+ = left).

        steer: how hard to steer, 0..1 of full lock. reverse: drive backwards; the steering
        is then turned the other way, so the robot still turns towards angle_deg.
        max_distance_m: stop after this distance even if the angle isn't reached yet.
        slowdown_deg: reduce power toward finish_power_fraction over the final degrees,
        then straighten at motor cutoff while the wheels can still roll.
        stop_lead_deg: cut power this far before the target to allow for coasting.
        settled_tolerance_deg: raise MotionError if the settled heading misses the target."""
        start = replace(self._sync(reset_heading=True))
        turn = 1 if angle_deg >= 0 else -1
        backwards = -1 if reverse else 1
        goal = math.radians(abs(angle_deg))
        slowdown_rad = math.radians(slowdown_deg)
        cutoff = goal - math.radians(FINISH_TOL_DEG)
        if stop_lead_deg is not None:
            cutoff = max(goal * 0.8, goal - math.radians(stop_lead_deg))
        check_from: Optional[float] = None
        checked = False
        cutoff_deg = 0.0

        def steering(pose: Pose) -> float:
            """Full steer until TAPER_DEG before the target, then less and less: the wheels
            straighten while the robot still rolls, like a car leaving a bend. Stationary
            tyres grip the floor too hard for the servo to straighten them fully."""
            remaining = goal - (pose.heading - start.heading) * turn
            amount = min(abs(steer), max(0.0, remaining) / math.radians(TAPER_DEG))
            return turn * backwards * amount

        self.steer(steering(start))

        def slow_down(pose: Pose) -> float:
            remaining = goal - (pose.heading - start.heading) * turn
            fraction = max(0.0, min(1.0, remaining / slowdown_rad))
            return finish_power_fraction + (1.0 - finish_power_fraction) * fraction

        def done(pose: Pose) -> bool:
            nonlocal check_from, checked, cutoff_deg
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
            distance_limit = max_distance_m is not None and travelled >= max_distance_m
            if distance_limit or turned >= cutoff:
                cutoff_deg = math.degrees(pose.heading - start.heading)
                return True
            return False

        end, at_halt, at_rest = self._run(backwards * abs(power or self.power), done, steer_fn=steering,
                                         slow_down=slow_down if slowdown_rad > 0 else None,
                                         straighten_at_stop=slowdown_rad > 0,
                                         timeout=timeout, what="Turning")
        settled_deg = math.degrees(end.heading - start.heading)
        if settled_tolerance_deg is not None and abs(settled_deg - angle_deg) > settled_tolerance_deg:
            left = at_rest.left - at_halt.left
            right = at_rest.right - at_halt.right
            elapsed_ms = at_rest.ms - at_halt.ms
            source = "Gyro" if self.heading_delta is not None else "Encoder"
            raise MotionError(f"{source} heading settled at {settled_deg:+.1f}° with target {angle_deg:+g}°. "
                              f"Estimated angle when halt was sent: {cutoff_deg:+.1f}°. "
                              f"Encoder change after that sample: L {left:+d}, R {right:+d} "
                              f"counts in {elapsed_ms:d} ms of board time.")
        return end

    def turn_around(self, angle_deg: float = 180.0, leg_m: float = 0.30,
                    power: Optional[int] = None, max_legs: int = 12,
                    on_leg: Optional[Callable[[int, str, float], None]] = None) -> tuple[Pose, int]:
        """Turn on the spot the way a car does: forward arcs at full lock alternate with
        backward arcs steered the other way, each at most leg_m long, until the heading has
        changed by angle_deg (+ = left). Returns the final pose and the number of legs.
        on_leg(leg number, "forward"/"backward", degrees turned so far) is called after each leg."""
        start = replace(self._sync(reset_heading=True))
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
    def _encoder_sample(self) -> Encoders:
        enc = self.board.encoders
        if self.board.reboots != self._encoder_epoch:
            raise MotionError("The board restarted. Restart the script to reset odometry.")
        if enc is None or time.monotonic() - enc.t > ENCODER_MAX_AGE_S:
            raise MotionError("Encoder telemetry went stale. Check the control board's USB connection.")
        return enc

    def _update(self, enc: Encoders, reset_heading: bool = False) -> Pose:
        delta = None if self.heading_delta is None else self.heading_delta(reset_heading)
        return self.odometry.update(enc.left, enc.right, delta)

    def _sync(self, reset_heading: bool = False) -> Pose:
        return self._update(self._encoder_sample(), reset_heading)

    def _wheel_powers(self, power: float) -> tuple[float, float]:
        """Share the power between the rear wheels the way their speeds differ in a turn:
        the inner wheel rolls a shorter arc than the outer one."""
        k = self.track * math.tan((self._steer_amount or 0.0) * self.full_lock) / (2 * self.wheelbase)
        return power * (1 - k), power * (1 + k)

    def _run(self, power: float, done: Callable[[Pose], bool], *,
             steer_fn: Optional[Callable[[Pose], float]] = None,
             slow_down: Optional[Callable[[Pose], float]] = None,
             straighten_at_stop: bool = False,
             timeout: float = 30.0, what: str = "The motion") -> tuple[Pose, Encoders, Encoders]:
        """Power the motors until done(pose) is true; always stops the motors at the end.
        Return the settled pose and the encoder samples at the halt request and at rest."""
        board = self.board
        started = last_move = time.monotonic()
        last_counts = None
        try:
            while True:
                enc = self._encoder_sample()
                counts = (enc.left, enc.right)
                pose = self._update(enc)
                if done(pose):
                    break
                now = time.monotonic()
                if now - started > timeout:
                    raise MotionError(f"{what} didn't finish within {timeout:.0f} s.")
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
        finally:
            board.halt()
        if straighten_at_stop:
            self.steer(0.0, wait=False)
        return self._settle(enc)

    def _settle(self, at_halt: Encoders) -> tuple[Pose, Encoders, Encoders]:
        """Check fresh encoder samples while repeating halt in case a UART packet was lost."""
        started = time.monotonic()
        last_change_ms = at_halt.ms
        last_counts = (at_halt.left, at_halt.right)
        last_heading = self.pose.heading
        while True:
            time.sleep(LOOP_S)
            self.board.halt()
            enc = self._encoder_sample()
            now = time.monotonic()
            counts = (enc.left, enc.right)
            pose = self._update(enc)
            rotating = abs(pose.heading - last_heading) > math.radians(0.75)
            if counts != last_counts or rotating:
                last_counts, last_change_ms = counts, enc.ms
                last_heading = pose.heading
            if now - started >= STOP_SETTLE_S and enc.ms - last_change_ms >= STOP_QUIET_MS:
                return pose, at_halt, enc
            if now - started >= STOP_TIMEOUT_S:
                raise MotionError("The wheels did not report a steady stop after repeated halt commands. "
                                  f"Encoder change: L {enc.left - at_halt.left:+d}, "
                                  f"R {enc.right - at_halt.right:+d} counts since the halt request.")
