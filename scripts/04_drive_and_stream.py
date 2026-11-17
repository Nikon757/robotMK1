#!/usr/bin/env python3
"""Stream the camera to a browser while driving forward.

The OAK-D Lite stream starts first; open one of the printed addresses in a browser (on the
laptop, same Wi-Fi, or on the robot's own screen). Then the robot drives --distance metres
straight ahead with the camera centred, and the stream keeps running for --keep-streaming
seconds after it stops (Ctrl+C ends it earlier).
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))   # repository root, for "modules"

from modules.camera import CameraPan, MjpegServer, OakMjpegCamera
from modules.engines import Robot, load_config
from modules.script_tools import confirm_start, run


def main(argv: list[str] | None = None) -> int:
    cfg = load_config()
    stream = cfg["camera_stream"]
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--distance", type=float, default=1.0, help="metres forward (default 1.0)")
    parser.add_argument("--power", type=int, help="motor power, per-mil (default: drive.power)")
    parser.add_argument("--port", type=int, default=int(stream["port"]), help="web server port")
    parser.add_argument("--keep-streaming", type=float, default=30.0,
                        help="seconds to keep streaming after the drive (default 30)")
    parser.add_argument("--yes", action="store_true", help="start driving without asking")
    args = parser.parse_args(argv)

    def work() -> None:
        with OakMjpegCamera(int(stream["fps"]), int(stream["jpeg_quality"])) as camera, \
                MjpegServer(camera, args.port) as server:
            _, first = camera.wait_frame(0, timeout=10.0)
            if first is None:
                raise RuntimeError("No picture from the OAK-D Lite within 10 s. "
                                   "Is it on a blue (USB 3) port?")
            print("Camera stream - open in a browser:")
            for url in server.urls():
                print("   ", url)
            confirm_start(f"\nThe robot drives {args.distance:.2f} m straight forward. "
                          f"Clear at least {args.distance + 0.3:.1f} m ahead.", args.yes)
            with Robot() as robot:
                CameraPan(robot.board, cfg)             # centres the camera, taking up the play
                end = robot.forward(args.distance, power=args.power)
                print(f"Drove {end.distance:+.3f} m; heading change {math.degrees(end.heading):+.1f} degrees.")
            if args.keep_streaming > 0:
                print(f"Streaming for {args.keep_streaming:g} s more (Ctrl+C ends) ...")
                time.sleep(args.keep_streaming)

    return run(work)


if __name__ == "__main__":
    sys.exit(main())
