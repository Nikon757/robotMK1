#!/usr/bin/env python3
"""Camera pan: turn to the far left, then the far right, then back to the centre.

The limits come from config/robot.yaml (camera_pan, calibrated 30 Oct 2026): about 62 degrees
left and 60 degrees right of straight ahead. The camera stops end_margin_deg (3) degrees inside
them, so the servo never presses against its own end stop. The wheels don't move.
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

# The repository root, for "modules". Appended, not prepended: installed libraries must win
# over project folders with the same name (the project has a depthai/ folder).
sys.path.append(str(Path(__file__).resolve().parents[1]))

from modules.camera import CameraPan
from modules.engines import load_config, open_board
from modules.script_tools import run


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--speed", type=float, help="degrees per second (default: camera_pan.speed_deg_s)")
    parser.add_argument("--pause", type=float, default=1.0, help="seconds to stay at each end (default 1)")
    args = parser.parse_args(argv)

    def work() -> None:
        cfg = load_config()
        with open_board(cfg["board"].get("port")) as board:
            pan = CameraPan(board, cfg)
            for label, move in (("far left", pan.look_left), ("far right", pan.look_right),
                                ("centre", pan.look_center)):
                print(f"Camera to the {label} ...")
                move(args.speed)
                time.sleep(args.pause)
            print("Done: the camera points straight ahead.")

    return run(work)


if __name__ == "__main__":
    sys.exit(main())
