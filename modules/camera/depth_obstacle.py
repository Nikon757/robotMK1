"""Add a depth-based obstacle box to the OAK-D Lite's colour stream."""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from datetime import timedelta

import cv2
import numpy as np

from .stream import OakMjpegCamera
from .yaw import OakYaw


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
    seed_roi: tuple[float, float, float, float] = (0.39, 0.30, 0.61, 0.70)
    depth_tolerance_m: float = 0.045
    close_kernel_px: int = 13


def _roi_bounds(roi: tuple[float, float, float, float], shape: tuple[int, int]) -> tuple[int, int, int, int]:
    height, width = shape
    left, top, right, bottom = roi
    return round(left * width), round(top * height), round(right * width), round(bottom * height)


def _refine_box(image: np.ndarray, observed: np.ndarray, other_depths: np.ndarray,
                box: tuple[int, int, int, int]) -> tuple[int, int, int, int]:
    """Use colour to fill missing depth near the block while protecting measured background."""
    x, y, w, h = box
    height, width = observed.shape
    pad_x = max(w, 2 * h // 3)
    x0, y0 = max(0, x - pad_x), max(0, y - h)
    x1, y1 = min(width, x + w + pad_x), min(height, y + 2 * h)
    region = np.s_[y0:y1, x0:x1]
    seeds = observed[region]
    background = other_depths[region]
    if not background.any() or np.all(seeds | background):
        return box

    mask = np.full(seeds.shape, cv2.GC_PR_FGD, dtype=np.uint8)
    mask[background] = cv2.GC_BGD
    mask[seeds] = cv2.GC_FGD

    # Fit the colour model on a small crop to keep the Pi's stream responsive.
    scale = min(1.0, 160 / max(seeds.shape))
    size = (max(1, round(seeds.shape[1] * scale)), max(1, round(seeds.shape[0] * scale)))
    small = cv2.resize(mask, size, interpolation=cv2.INTER_NEAREST)
    if np.count_nonzero(small == cv2.GC_BGD) < 5 or np.count_nonzero(small == cv2.GC_FGD) < 5:
        return box
    colour = cv2.resize(image[region], size, interpolation=cv2.INTER_AREA)
    cv2.grabCut(colour, small, None, None, None, 1, cv2.GC_INIT_WITH_MASK)
    foreground = ((small == cv2.GC_FGD) | (small == cv2.GC_PR_FGD)).astype(np.uint8)
    refined = cv2.resize(foreground, (seeds.shape[1], seeds.shape[0]), interpolation=cv2.INTER_NEAREST)
    refined[background] = 0
    refined[seeds] = 1
    count, labels, stats, _ = cv2.connectedComponentsWithStats(refined)
    overlap = np.bincount(labels[seeds], minlength=count)
    overlap[0] = 0
    index = int(np.argmax(overlap))
    if not overlap[index]:
        return box
    rx, ry, rw, rh = map(int, stats[index, :4])
    left, top = min(x, x0 + rx), min(y, y0 + ry)
    right, bottom = max(x + w, x0 + rx + rw), max(y + h, y0 + ry + rh)
    return left, top, right - left, bottom - top


def find_obstacle(depth: np.ndarray, settings: ObstacleSettings, image: np.ndarray | None = None) -> Obstacle | None:
    """Find a near surface in the seed region and trace its outline in the inspection area."""
    x0, y0, x1, y1 = _roi_bounds(settings.roi, depth.shape)
    sx0, sy0, sx1, sy1 = _roi_bounds(settings.seed_roi, depth.shape)
    sx0, sy0 = max(sx0, x0), max(sy0, y0)
    sx1, sy1 = min(sx1, x1), min(sy1, y1)
    if sx0 >= sx1 or sy0 >= sy1:
        return None
    seed = np.s_[sy0 - y0:sy1 - y0, sx0 - x0:sx1 - x0]
    crop = depth[y0:y1, x0:x1]
    valid = (crop > 0) & (crop >= settings.min_m * 1000) & (crop <= settings.max_m * 1000)
    seed_depths = np.sort(crop[seed][valid[seed]])
    min_support = max(12, settings.min_pixels // 8)
    tolerance_mm = settings.depth_tolerance_m * 1000
    # Ignore small groups of outliers and choose the nearest band with enough depth support.
    end = np.searchsorted(seed_depths, seed_depths + tolerance_mm, side="right")
    supported = np.flatnonzero(end - np.arange(len(seed_depths)) >= min_support)
    if not supported.size:
        return None
    first = supported[0]
    reference = np.median(seed_depths[first:end[first]])
    near = valid & (np.abs(crop - reference) <= tolerance_mm)
    other_depths = (crop > 0) & ~near
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (settings.close_kernel_px,) * 2)
    mask = cv2.morphologyEx(near.astype(np.uint8), cv2.MORPH_CLOSE, kernel)
    mask[other_depths] = 0
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
    overlap = np.bincount(labels[seed][near[seed]], minlength=count)
    overlap[0] = 0
    index = int(np.argmax(overlap))
    if overlap[index] < min_support:
        return None
    observed = (labels == index) & near
    if np.count_nonzero(observed) < settings.min_pixels:
        return None
    box = tuple(map(int, stats[index, :4]))
    if image is not None:
        box = _refine_box(image[y0:y1, x0:x1], observed, other_depths, box)
    x, y, box_w, box_h = box
    distance_m = float(np.median(crop[observed])) / 1000
    return Obstacle(x + x0, y + y0, box_w, box_h, distance_m)


def _same_obstacle(old: Obstacle, new: Obstacle, tolerance_m: float) -> bool:
    if abs(old.distance_m - new.distance_m) > tolerance_m:
        return False
    width = min(old.x + old.width, new.x + new.width) - max(old.x, new.x)
    height = min(old.y + old.height, new.y + new.height) - max(old.y, new.y)
    shared = max(0, width) * max(0, height)
    union = old.width * old.height + new.width * new.height - shared
    return shared >= 0.30 * union


class OakObstacleCamera(OakMjpegCamera):
    """Synchronize RGB and aligned depth on the OAK, then overlay and encode on the Pi."""

    _stream_name = "rgbd"

    def __init__(self, fps: int, quality: int, settings: ObstacleSettings, *, imu_rate_hz: int | None = None):
        super().__init__(fps, quality)
        self.settings = settings
        self.imu_rate_hz = imu_rate_hz
        self.yaw: OakYaw | None = None
        self._inspecting = False
        self._detection: Obstacle | None = None
        self._recent: deque[Obstacle] = deque(maxlen=settings.confirm_frames)

    def inspect(self, enabled: bool) -> None:
        with self._new_frame:
            self._inspecting = enabled
            self._detection = None
            self._recent.clear()

    def wait_for_obstacle(self, timeout: float) -> Obstacle | None:
        with self._new_frame:
            ready = self._new_frame.wait_for(
                lambda: self._detection is not None or self._stop.is_set(), timeout
            )
            return self._detection if ready and not self._stop.is_set() else None

    def _record_detection(self, obstacle: Obstacle | None) -> Obstacle | None:
        if obstacle is None:
            self._recent.clear()
        else:
            if self._recent and not _same_obstacle(self._recent[-1], obstacle, self.settings.depth_tolerance_m):
                self._recent.clear()
            self._recent.append(obstacle)

        if len(self._recent) < self.settings.confirm_frames:
            self._detection = None
        else:
            edges = [(box.x, box.y, box.x + box.width, box.y + box.height) for box in self._recent]
            x0, y0, x1, y1 = (round(value) for value in np.median(edges, axis=0))
            distance = float(np.median([box.distance_m for box in self._recent]))
            self._detection = Obstacle(x0, y0, x1 - x0, y1 - y0, distance)
        return self._detection

    def _open_device(self, dai):
        device = dai.Device()
        try:
            calibration = device.readCalibration2()
            lens_position = calibration.getLensPosition(dai.CameraBoardSocket.CAM_A)
            pipeline = self._build_pipeline(dai, lens_position)
            if self.imu_rate_hz is not None:
                imu_type = device.getConnectedIMU()
                if not imu_type or imu_type.upper() in ("NONE", "UNKNOWN"):
                    raise RuntimeError("This OAK reports no IMU. Use --heading wheels to select encoder heading.")
                OakYaw.add_to_pipeline(dai, pipeline, self.imu_rate_hz)
            device.startPipeline(pipeline)
            if self.imu_rate_hz is not None:
                self.yaw = OakYaw(dai, device)
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
                obstacle = self._record_detection(find_obstacle(depth, self.settings, image))
                if obstacle is not None:
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
