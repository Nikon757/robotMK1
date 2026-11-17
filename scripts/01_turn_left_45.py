#!/usr/bin/env python3
"""Drive forward while turning 45 degrees to the left.

The robot drives an arc until the encoders show that its heading has changed by --angle
degrees (+ = left, - = right), then stops. --steer sets how tight the arc is: 1.0 is full
lock, 0.5 half lock. It needs about 0.6 m of free space ahead and to the left.
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

# The repository root, for "modules". Appended, not prepended: installed libraries must win
# over project folders with the same name (the project has a depthai/ folder).
sys.path.append(str(Path(__file__).resolve().parents[1]))

from modules.engines import Robot
from modules.script_tools import confirm_start, run


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--angle", type=float, default=45.0, help="degrees, + = left (default 45)")
    parser.add_argument("--steer", type=float, default=0.7, help="0..1 of full lock (default 0.7)")
    parser.add_argument("--power", type=int, help="motor power, per-mil (default: drive.power)")
    parser.add_argument("--yes", action="store_true", help="start without asking")
    args = parser.parse_args(argv)

    def work() -> None:
        side = "left" if args.angle >= 0 else "right"
        confirm_start(f"The robot drives forward and turns {abs(args.angle):g} degrees to the {side}. "
                      "Clear about 0.6 m ahead of it and to that side.", args.yes)
        with Robot() as robot:
            end = robot.turn_by(args.angle, steer=args.steer, power=args.power)
            robot.steer(0.0)
            print(f"Turned {math.degrees(end.heading):+.1f} degrees (target {args.angle:+g}) "
                  f"over {end.distance:.2f} m; now {end.x:+.2f} m forward, {end.y:+.2f} m left "
                  "of the start.")

    return run(work)


if __name__ == "__main__":
    sys.exit(main())
