#!/usr/bin/env python3
"""Turn around 180 degrees on the spot by driving back and forth (a multi-point turn).

The steering can't turn the robot on the spot, so it turns like a car in a narrow street:
forward with full left lock, backward with full right lock, and so on, each leg at most
--leg metres, until the encoders show the heading has changed by --angle degrees
(+ = left). It needs about 0.5 m of free space all around. The robot can't see behind it,
so check the space behind it yourself.
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))   # repository root, for "modules"

from modules.engines import Robot
from modules.script_tools import confirm_start, run


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--angle", type=float, default=180.0, help="degrees, + = left (default 180)")
    parser.add_argument("--leg", type=float, default=0.30, help="longest leg in metres (default 0.30)")
    parser.add_argument("--power", type=int, help="motor power, per-mil (default: drive.power)")
    parser.add_argument("--yes", action="store_true", help="start without asking")
    args = parser.parse_args(argv)

    def work() -> None:
        confirm_start(f"The robot turns {abs(args.angle):g} degrees by driving back and forth, "
                      f"legs of up to {args.leg:.2f} m. Clear about 0.5 m all around it, "
                      "including behind.", args.yes)
        with Robot() as robot:
            end, legs = robot.turn_around(args.angle, leg_m=args.leg, power=args.power)
            print(f"Turned {math.degrees(end.heading):+.1f} degrees (target {args.angle:+g}) "
                  f"in {legs} legs; ended {math.hypot(end.x, end.y):.2f} m from the start.")

    return run(work)


if __name__ == "__main__":
    sys.exit(main())
