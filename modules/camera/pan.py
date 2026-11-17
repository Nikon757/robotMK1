"""Camera pan servo (@S2): slow movements between the calibrated limits in config/robot.yaml.

Positions are @S2 command angles: 90 = straight ahead, higher = left, lower = right
(docs/CAMERA_PAN_CALIBRATION.md). The firmware also clamps them to the same limits.

The pan mount has 5-10 degrees of play (measured 17 Nov 2026): approached from the left the
camera stops exactly on target, approached from the right it stops short. So every move
ends in the same direction as when the centre was calibrated (approach_from in robot.yaml):
a move that would arrive from the other side first goes approach_overshoot_deg past the
target, then comes back.
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
        self.end_margin = int(cfg.get("end_margin_deg", 3))
        self.approach_from = str(cfg.get("approach_from", "left"))
        self.overshoot = int(cfg.get("approach_overshoot_deg", 12))
        self.low, self.high = sorted((self.left, self.right))

        # The servo's position at start is unknown: jump to the approach side of the centre,
        # then come to the centre the calibrated way.
        self.angle = self._approach_start(self.center)
        board.cam_pan(self.angle)
        time.sleep(0.6)
        self.go_to(self.center)

    def go_to(self, angle: int, speed_deg_s: Optional[float] = None) -> None:
        """Turn slowly to angle, within the calibrated limits, always arriving from the
        approach side so the play in the mount doesn't shift the end position."""
        target = max(self.low, min(self.high, int(angle)))
        start = self._approach_start(target)
        if self._needs_detour(target, start):
            self._sweep(start, speed_deg_s)
        self._sweep(target, speed_deg_s)
        self.board.cam_pan(target)          # once more, in case a step was lost
        time.sleep(0.2)                     # let the servo arrive

    def look_left(self, speed_deg_s: Optional[float] = None) -> None:
        self.go_to(self._inside(self.left), speed_deg_s)

    def look_right(self, speed_deg_s: Optional[float] = None) -> None:
        self.go_to(self._inside(self.right), speed_deg_s)

    def look_center(self, speed_deg_s: Optional[float] = None) -> None:
        self.go_to(self.center, speed_deg_s)

    # ---- helpers ----------------------------------------------------------------------
    def _approach_start(self, target: int) -> int:
        """Where the final approach to target starts: overshoot degrees to the approach side."""
        if self.approach_from == "left":           # higher command = further left
            return min(self.high, target + self.overshoot)
        if self.approach_from == "right":
            return max(self.low, target - self.overshoot)
        return target

    def _needs_detour(self, target: int, start: int) -> bool:
        """True unless the camera is already at or beyond the approach start, so that the
        direct move to target covers the whole approach in the right direction."""
        if self.approach_from == "left":
            return self.angle < start
        if self.approach_from == "right":
            return self.angle > start
        return False

    def _sweep(self, target: int, speed_deg_s: Optional[float]) -> None:
        """One degree at a time, so the camera turns smoothly."""
        step = 1 if target > self.angle else -1
        delay = 1.0 / (speed_deg_s or self.speed_deg_s)
        for a in range(self.angle + step, target + step, step):
            self.board.cam_pan(a)
            time.sleep(delay)
        self.angle = target

    def _inside(self, limit: int) -> int:
        """A limit moved end_margin degrees towards the centre: the servo never presses
        against an end stop, where it draws a large current."""
        return limit - self.end_margin if limit > self.center else limit + self.end_margin
