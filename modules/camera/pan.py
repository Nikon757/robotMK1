"""Camera pan servo (@S2): slow movements between the calibrated limits in config/robot.yaml.

Positions are @S2 command angles: 90 = straight ahead, higher = left, lower = right
(docs/CAMERA_PAN_CALIBRATION.md). The firmware also clamps them to the same limits.
"""
from __future__ import annotations

import time
from typing import Optional

from ..engines.board import NiumoBoard
from ..engines.config import load_config


class CameraPan:
    def __init__(self, board: NiumoBoard, config: Optional[dict] = None):
        cfg = (config or load_config())["camera_pan"]
        self.board = board
        self.center = int(cfg["center"])
        self.left = int(cfg["left"])
        self.right = int(cfg["right"])
        self.speed_deg_s = float(cfg["speed_deg_s"])
        self.angle = self.center
        board.cam_pan(self.center)          # start from a known position
        time.sleep(0.5)

    def go_to(self, angle: int, speed_deg_s: Optional[float] = None) -> None:
        """Turn slowly to angle, one degree at a time, within the calibrated limits."""
        low, high = sorted((self.left, self.right))
        target = max(low, min(high, int(angle)))
        step = 1 if target > self.angle else -1
        delay = 1.0 / (speed_deg_s or self.speed_deg_s)
        for a in range(self.angle + step, target + step, step):
            self.board.cam_pan(a)
            time.sleep(delay)
        self.angle = target

    def look_left(self, speed_deg_s: Optional[float] = None) -> None:
        self.go_to(self.left, speed_deg_s)

    def look_right(self, speed_deg_s: Optional[float] = None) -> None:
        self.go_to(self.right, speed_deg_s)

    def look_center(self, speed_deg_s: Optional[float] = None) -> None:
        self.go_to(self.center, speed_deg_s)
