"""Camera pan servo (@S2): slow movements between the calibrated limits in config/robot.yaml.

Positions are @S2 command angles: 90 = straight ahead, higher = left, lower = right
(docs/CAMERA_PAN_CALIBRATION.md). The firmware also clamps them to the same limits.

The pan mount has 5-10 degrees of play (measured 17 Nov 2026): approached from the left the
camera stops exactly on target, approached from the right it stops short. So every move
ends in the same direction as when the centre was calibrated (approach_from in robot.yaml),
and the same way as in that test: the camera turns slowly to approach_overshoot_deg on the
approach side of the target, stops, then jumps the rest of the way in one command. Sweeping
1 degree at a time all the way in left it 3-5 degrees off: near the target each step is too
small for the loaded servo to follow.
"""
from __future__ import annotations

import time
from typing import Optional

from ..engines.board import NiumoBoard
from ..engines.config import load_config

PAUSE_S = 0.3           # at rest at the approach start before the final jump
ARRIVE_S = 0.4          # the servo covers the final jump and settles


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
        self.overshoot = int(cfg.get("approach_overshoot_deg", 20))
        self.low, self.high = sorted((self.left, self.right))
        self.inner_low, self.inner_high = sorted((self._inside(self.left), self._inside(self.right)))

        # The servo's position at start is unknown: jump to the approach start of the centre,
        # then come to the centre the calibrated way.
        self.angle = self._approach_start(self.center)
        board.cam_pan(self.angle)
        time.sleep(0.6)
        self.go_to(self.center)

    def go_to(self, angle: int, speed_deg_s: Optional[float] = None) -> None:
        """Turn slowly to the approach start of angle, then jump the rest of the way, so the
        camera always arrives from the approach side and the play in the mount doesn't shift
        the end position."""
        target = max(self.low, min(self.high, int(angle)))
        start = self._approach_start(target)
        self._sweep(start, speed_deg_s)
        if start != target:
            time.sleep(PAUSE_S)
            self.board.cam_pan(target)
            self.angle = target
        time.sleep(ARRIVE_S)

    def look_left(self, speed_deg_s: Optional[float] = None) -> None:
        self.go_to(self._inside(self.left), speed_deg_s)

    def look_right(self, speed_deg_s: Optional[float] = None) -> None:
        self.go_to(self._inside(self.right), speed_deg_s)

    def look_center(self, speed_deg_s: Optional[float] = None) -> None:
        self.go_to(self.center, speed_deg_s)

    # ---- helpers ----------------------------------------------------------------------
    def _approach_start(self, target: int) -> int:
        """Where the final jump to target starts: overshoot degrees to the approach side, but
        never past the end margins (so the far end on the approach side has no jump)."""
        if self.approach_from == "left":           # higher command = further left
            return max(target, min(self.inner_high, target + self.overshoot))
        if self.approach_from == "right":
            return min(target, max(self.inner_low, target - self.overshoot))
        return target

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
