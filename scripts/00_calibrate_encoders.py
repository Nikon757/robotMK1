#!/usr/bin/env python3
"""Calibrate the wheel encoders: counts per wheel turn and per metre (run this first).

Hand method (default): lift the rear wheels off the ground, then turn each rear wheel
forward by hand exactly --turns full turns. The script reads the encoder counts and writes
counts_per_wheel_rev and counts_per_metre into config/robot.yaml. Scripts 01-04 need them.

Driving method (--drive): the robot drives straight on the floor for --seconds, then you
type the distance you measured with a tape. This is more accurate, because a loaded tyre
rolls a little smaller than its measured diameter. Run it after the hand method.

Turn method (--track): the robot turns --angle degrees by driving back and forth (like
script 02), then you type how far it really turned. The script works out the effective
track width the heading calculation needs (odometry_track_m): wide tyres and side-slip
make it smaller than the tyre-centre distance. Run it after the other two.
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

# The repository root, for "modules". Appended, not prepended: installed libraries must win
# over project folders with the same name (the project has a depthai/ folder).
sys.path.append(str(Path(__file__).resolve().parents[1]))

from modules.engines import Robot, load_config, open_board, save_value
from modules.script_tools import confirm_start, run


def by_hand(board, cfg: dict, turns: int) -> None:
    diameter = float(cfg["chassis"]["wheel_diameter_m"])
    per_turn = {}
    print("Lift the rear wheels off the ground. Put a chalk mark on each rear tyre.\n")
    for side in ("left", "right"):
        before = board.wait_for("encoders")
        input(f"Turn the {side.upper()} rear wheel FORWARD by hand exactly {turns} full turns, "
              "then press Enter ... ")
        after = board.encoders
        counts = getattr(after, side) - getattr(before, side)
        print(f"  {side}: {counts:+d} counts = {counts / turns:.1f} per turn")
        if counts < 0:
            raise RuntimeError(f"The {side} wheel counted backwards. Turned it backwards? Otherwise "
                               "flip its sign in firmware/User/protocol.h (PROTOCOL_ENC_*_SIGN).")
        if counts < 20 * turns:
            raise RuntimeError(f"Almost no counts from the {side} wheel: is its encoder cable "
                               f"plugged into {'EC1' if side == 'left' else 'EC2'}?")
        per_turn[side] = counts / turns

    left, right = per_turn["left"], per_turn["right"]
    if abs(left - right) / max(left, right) > 0.03:
        print(f"\nWarning: the wheels differ by more than 3 % ({left:.1f} vs {right:.1f}). "
              "Count the turns again carefully and repeat.")
    per_rev = (left + right) / 2
    per_metre = per_rev / (math.pi * diameter)
    save_value("chassis", "counts_per_wheel_rev", round(per_rev, 1))
    save_value("chassis", "counts_per_metre", round(per_metre, 1))
    print(f"\nSaved to config/robot.yaml:\n  counts_per_wheel_rev = {per_rev:.1f}\n"
          f"  counts_per_metre     = {per_metre:.1f}   (wheel diameter {diameter * 1000:.0f} mm)")
    print(f"Gear ratio, if the encoder has 11 pulses per motor turn: about 1:{per_rev / 44:.0f}")


def by_driving(board, cfg: dict, power: int, seconds: float, skip: bool) -> None:
    confirm_start(f"The robot drives straight forward for {seconds:g} s at power {power}. "
                  "Put it on the floor with at least 1.5 m clear ahead, and mark the floor at "
                  "the centre of its rear axle.", skip)
    board.steer(int(cfg["steering"]["center"]))
    before = board.wait_for("encoders")
    board.drive_for(power, power, seconds)          # repeats @M, then stops the motors
    time.sleep(0.5)                                 # let the robot roll to a stop
    after = board.encoders
    counts = ((after.left - before.left) + (after.right - before.right)) / 2
    print(f"Mean count change: {counts:.0f}")
    metres = float(input("Measure from the mark to the rear-axle centre now. Distance in metres: "))
    per_metre = counts / metres
    old = cfg["chassis"].get("counts_per_metre")
    save_value("chassis", "counts_per_metre", round(per_metre, 1))
    print(f"Saved counts_per_metre = {per_metre:.1f}" + (f" (was {old})" if old else ""))


def by_turning(cfg: dict, angle: float, skip: bool) -> None:
    confirm_start(f"The robot turns {angle:g} degrees by driving back and forth (like script 02). "
                  "Before it starts, lay a strip of tape on the floor along its left side; afterwards "
                  "you measure how far it really turned. Clear about 0.5 m all around it.", skip)
    with Robot(config=cfg) as robot:
        track_used = robot.odometry.track_m
        end, _ = robot.turn_around(angle)
    measured = abs(math.degrees(end.heading))
    print(f"\nThe encoders say {measured:.1f} degrees (with a track of {track_used:.4f} m).\n"
          "For a 180-degree turn: if the robot's side is now parallel to the tape, it turned 180;\n"
          "if it went further, add the angle between its side and the tape (e.g. 180 + 30 = 210).")
    real = abs(float(input("How many degrees did the robot really turn? ")))
    if not 0.5 * measured < real < 2 * measured:
        raise RuntimeError(f"{real:g} degrees is too far from the encoders' {measured:.1f}: measure again.")
    track = track_used * measured / real
    save_value("chassis", "odometry_track_m", round(track, 4),
               comment="effective track for the heading; set by scripts/00_calibrate_encoders.py --track")
    print(f"Saved odometry_track_m = {track:.4f} m "
          f"(the measured tyre-centre track is {cfg['chassis']['rear_track_m']} m).")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--turns", type=int, default=5, help="wheel turns by hand (default 5)")
    parser.add_argument("--drive", action="store_true", help="driving method instead of by hand")
    parser.add_argument("--seconds", type=float, default=3.0, help="driving time for --drive")
    parser.add_argument("--power", type=int, help="motor power for --drive (default: drive.power)")
    parser.add_argument("--track", action="store_true", help="turn method: calibrate the effective track")
    parser.add_argument("--angle", type=float, default=180.0, help="turn for --track, degrees (default 180)")
    parser.add_argument("--yes", action="store_true", help="don't ask before driving")
    args = parser.parse_args(argv)
    cfg = load_config()

    def work() -> None:
        if args.track:
            by_turning(cfg, args.angle, args.yes)
            return
        with open_board(cfg["board"].get("port")) as board:
            if args.drive:
                by_driving(board, cfg, args.power or int(cfg["drive"]["power"]), args.seconds, args.yes)
            else:
                by_hand(board, cfg, max(1, args.turns))

    return run(work)


if __name__ == "__main__":
    sys.exit(main())
