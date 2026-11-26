"""Add a depth-based obstacle box to the OAK-D Lite's colour stream."""
from __future__ import annotations

from dataclasses import dataclass
from datetime import timedelta

import cv2
import numpy as np

from .stream import OakMjpegCamera


@dataclass(frozen=True)
class Obstacle:
    x: int
    y: int
    width: int
    height: int
    distance_m: float


@dataclass(frozen=True)
class ObstacleSettings:
    min_m: float
    max_m: float
    roi: tuple[float, float, float, float]
    min_pixels: int
    confirm_frames: int


def find_obstacle(depth: np.ndarray, settings: ObstacleSettings) -> Obstacle | None:
    """Find the largest connected patch of in-range pixels in the inspection area."""
    height, width = depth.shape
    left, top, right, bottom = settings.roi
    x0, x1 = round(left * width), round(right * width)
    y0, y1 = round(top * height), round(bottom * height)
    crop = depth[y0:y1, x0:x1]
    valid = (crop > 0) & (crop >= settings.min_m * 1000) & (crop <= settings.max_m * 1000)
    mask = cv2.morphologyEx(valid.astype(np.uint8), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
    if count <= 1:
        return None
    index = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    x, y, box_w, box_h, area = map(int, stats[index])
    if area < settings.min_pixels:
        return None
    distance_m = float(np.median(crop[labels == index])) / 1000
    return Obstacle(x + x0, y + y0, box_w, box_h, distance_m)


class OakObstacleCamera(OakMjpegCamera):
    """Synchronize RGB and aligned depth on the OAK, then overlay and encode on the Pi."""

    _stream_name = "rgbd"

    def __init__(self, fps: int, quality: int, settings: ObstacleSettings):
        super().__init__(fps, quality)
        self.settings = settings
        self._inspecting = False
        self._detection: Obstacle | None = None
        self._hits = 0

    def inspect(self, enabled: bool) -> None:
        with self._new_frame:
            self._inspecting = enabled
            self._detection = None
            self._hits = 0

    def wait_for_obstacle(self, timeout: float) -> Obstacle | None:
        with self._new_frame:
            ready = self._new_frame.wait_for(
                lambda: self._hits >= self.settings.confirm_frames or self._stop.is_set(), timeout
            )
            return self._detection if ready and not self._stop.is_set() else None

    def _open_device(self, dai):
        device = dai.Device()
        try:
            calibration = device.readCalibration2()
            lens_position = calibration.getLensPosition(dai.CameraBoardSocket.CAM_A)
            device.startPipeline(self._build_pipeline(dai, lens_position))
        except BaseException:
            device.close()
            raise
        return device

    def _build_pipeline(self, dai, lens_position: int | None = None):
        pipeline = dai.Pipeline()
        rgb = pipeline.create(dai.node.Camera)
        rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
        rgb.setSize(1280, 720)
        rgb.setVideoSize(640, 360)
        rgb.setFps(self.fps)
        rgb.setMeshSource(dai.CameraProperties.WarpMeshSource.CALIBRATION)
        if lens_position:
            rgb.initialControl.setManualFocus(lens_position)

        left = pipeline.create(dai.node.MonoCamera)
        left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
        right = pipeline.create(dai.node.MonoCamera)
        right.setBoardSocket(dai.CameraBoardSocket.CAM_C)
        for mono in (left, right):
            mono.setResolution(dai.MonoCameraProperties.SensorResolution.THE_480_P)
            mono.setFps(self.fps)

        stereo = pipeline.create(dai.node.StereoDepth)
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        stereo.setLeftRightCheck(True)
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
        stereo.setOutputSize(640, 360)
        left.out.link(stereo.left)
        right.out.link(stereo.right)

        sync = pipeline.create(dai.node.Sync)
        sync.setSyncThreshold(timedelta(seconds=0.5 / self.fps))
        sync.setSyncAttempts(-1)
        rgb.video.link(sync.inputs["rgb"])
        stereo.depth.link(sync.inputs["depth"])
        out = pipeline.create(dai.node.XLinkOut)
        out.setStreamName(self._stream_name)
        sync.out.link(out.input)
        return pipeline

    def _encode_frame(self, packet) -> bytes:
        image = packet["rgb"].getCvFrame()
        depth = packet["depth"].getFrame()
        if depth.shape != image.shape[:2]:
            raise RuntimeError(f"RGB size {image.shape[:2]} and aligned depth size {depth.shape} differ")

        with self._new_frame:
            if self._inspecting:
                obstacle = find_obstacle(depth, self.settings)
                self._detection = obstacle
                self._hits = self._hits + 1 if obstacle else 0
                if obstacle is not None and self._hits >= self.settings.confirm_frames:
                    x, y, w, h = obstacle.x, obstacle.y, obstacle.width, obstacle.height
                    cv2.rectangle(image, (x, y), (x + w - 1, y + h - 1), (0, 255, 0), 2)
                    self._label(image, f"Obstacle {obstacle.distance_m:.2f} m", x, max(20, y - 6))
                else:
                    self._label(image, "Looking for obstacle", 10, 24)

        ok, jpeg = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
        if not ok:
            raise RuntimeError("Could not encode the camera frame as JPEG")
        return jpeg.tobytes()

    @staticmethod
    def _label(image: np.ndarray, text: str, x: int, y: int) -> None:
        cv2.putText(image, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 4)
        cv2.putText(image, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
