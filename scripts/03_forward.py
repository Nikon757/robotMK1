#!/usr/bin/env python3
"""Drive straight forward a short distance (default 0.5 m).

The robot keeps its starting heading by steering on the encoder difference, slows down for
the last 10 cm and stops when the encoders show --distance metres (negative = backwards).
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
    parser.add_argument("--distance", type=float, default=0.5, help="metres, - = backwards (default 0.5)")
    parser.add_argument("--power", type=int, help="motor power, per-mil (default: drive.power)")
    parser.add_argument("--yes", action="store_true", help="start without asking")
    args = parser.parse_args(argv)

    def work() -> None:
        way = "forward" if args.distance >= 0 else "backward"
        confirm_start(f"The robot drives {abs(args.distance):.2f} m straight {way}. "
                      f"Clear at least {abs(args.distance) + 0.3:.1f} m {'ahead' if way == 'forward' else 'behind'}.",
                      args.yes)
        with Robot() as robot:
            end = robot.forward(args.distance, power=args.power)
            print(f"Drove {end.distance:+.3f} m (target {args.distance:+.2f}); "
                  f"heading change {math.degrees(end.heading):+.1f} degrees, "
                  f"sideways {end.y:+.3f} m.")

    return run(work)


if __name__ == "__main__":
    sys.exit(main())
