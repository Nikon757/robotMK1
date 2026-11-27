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
    min_seed_pixels: int = 24


def _roi_bounds(roi: tuple[float, float, float, float], shape: tuple[int, int]) -> tuple[int, int, int, int]:
    height, width = shape
    left, top, right, bottom = roi
    return round(left * width), round(top * height), round(right * width), round(bottom * height)


def _refine_mask(image: np.ndarray, observed: np.ndarray, other_depths: np.ndarray) -> np.ndarray | None:
    """Use colour to fill missing depth next to the measured foreground."""
    x, y, w, h = cv2.boundingRect(observed.astype(np.uint8))
    height, width = observed.shape
    pad_x = max(w, 2 * h // 3, width // 6)
    pad_y = max(h, height // 3)
    x0, y0 = max(0, x - pad_x), max(0, y - pad_y)
    x1, y1 = min(width, x + w + pad_x), min(height, y + h + pad_y)
    region = np.s_[y0:y1, x0:x1]
    seeds = observed[region]
    background = other_depths[region]
    if not background.any() or np.all(seeds | background):
        return None

    # Fit the colour model on a small crop to keep the Pi's stream responsive.
    scale = min(1.0, 160 / max(seeds.shape))
    size = (max(1, round(seeds.shape[1] * scale)), max(1, round(seeds.shape[0] * scale)))
    small = np.full((size[1], size[0]), cv2.GC_PR_FGD, dtype=np.uint8)
    small[cv2.resize(background.astype(np.float32), size, interpolation=cv2.INTER_AREA) >= 0.5] = cv2.GC_BGD
    small[cv2.resize(seeds.astype(np.float32), size, interpolation=cv2.INTER_AREA) > 0] = cv2.GC_FGD
    if np.count_nonzero(small == cv2.GC_BGD) < 5 or np.count_nonzero(small == cv2.GC_FGD) < 5:
        return None
    colour = cv2.resize(image[region], size, interpolation=cv2.INTER_AREA)
    cv2.grabCut(colour, small, None, None, None, 1, cv2.GC_INIT_WITH_MASK)
    foreground = ((small == cv2.GC_FGD) | (small == cv2.GC_PR_FGD)).astype(np.uint8)
    refined = cv2.resize(foreground, (seeds.shape[1], seeds.shape[0]), interpolation=cv2.INTER_NEAREST)
    refined[background] = 0
    refined[seeds] = 1
    count, labels = cv2.connectedComponents(refined)
    overlap = np.bincount(labels[seeds], minlength=count)
    overlap[0] = 0
    index = int(np.argmax(overlap))
    if not overlap[index]:
        return None
    foreground_mask = np.zeros_like(observed)
    foreground_mask[region] = labels == index
    return foreground_mask


def find_obstacle(depth: np.ndarray, settings: ObstacleSettings, image: np.ndarray | None = None) -> Obstacle | None:
    """Find a near surface in the seed region and trace its outline in the inspection area."""
    return _scan_obstacle(depth, settings, image)[0]


def _scan_obstacle(depth: np.ndarray, settings: ObstacleSettings,
                   image: np.ndarray | None) -> tuple[Obstacle | None, str]:
    x0, y0, x1, y1 = _roi_bounds(settings.roi, depth.shape)
    sx0, sy0, sx1, sy1 = _roi_bounds(settings.seed_roi, depth.shape)
    sx0, sy0 = max(sx0, x0), max(sy0, y0)
    sx1, sy1 = min(sx1, x1), min(sy1, y1)
    if sx0 >= sx1 or sy0 >= sy1:
        return None, "Aim region is outside the search area"
    seed = np.s_[sy0 - y0:sy1 - y0, sx0 - x0:sx1 - x0]
    crop = depth[y0:y1, x0:x1]
    valid = (crop > 0) & (crop >= settings.min_m * 1000) & (crop <= settings.max_m * 1000)
    if not np.any(crop[seed]):
        return None, "No stereo depth in the aim region"
    seed_depths = np.sort(crop[seed][valid[seed]])
    if not seed_depths.size:
        return None, f"Centre depth is outside {settings.min_m:g}-{settings.max_m:g} m"
    min_support = settings.min_seed_pixels
    tolerance_mm = settings.depth_tolerance_m * 1000
    # Ignore small groups of outliers and choose the nearest band with enough depth support.
    end = np.searchsorted(seed_depths, seed_depths + tolerance_mm, side="right")
    supported = np.flatnonzero(end - np.arange(len(seed_depths)) >= min_support)
    if not supported.size:
        return None, f"Too few similar depth readings in the aim region (need {min_support})"
    first = supported[0]
    reference = np.median(seed_depths[first:end[first]])
    near = valid & (np.abs(crop - reference) <= tolerance_mm)
    other_depths = (crop > 0) & ~near
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (settings.close_kernel_px,) * 2)
    mask = cv2.morphologyEx(near.astype(np.uint8), cv2.MORPH_CLOSE, kernel)
    mask[other_depths] = 0
    count, labels = cv2.connectedComponents(mask)
    overlap = np.bincount(labels[seed][near[seed]], minlength=count)
    overlap[0] = 0
    index = int(np.argmax(overlap))
    foreground = labels == index if index else np.zeros_like(near)
    observed = foreground & near
    if np.count_nonzero(observed) >= settings.min_pixels and overlap[index] >= min_support:
        if image is not None:
            refined = _refine_mask(image[y0:y1, x0:x1], observed, other_depths)
            if refined is not None:
                foreground |= refined
    else:
        # On dark surfaces several small, separated patches may be the only valid stereo hits.
        # Require real depth on a single connected outline after colour segmentation.
        nearby = cv2.boxFilter(near.astype(np.uint8), cv2.CV_16U, (3, 3), normalize=False)
        observed[seed] |= near[seed] & (nearby[seed] >= 3)
        if image is None or np.count_nonzero(observed[seed]) < min_support:
            return None, "Depth is too fragmented to confirm an outline"
        refined = _refine_mask(image[y0:y1, x0:x1], observed, other_depths)
        if refined is None or np.count_nonzero(refined) < settings.min_pixels:
            return None, "Sparse depth found, but no reliable outline"
        foreground = refined
        observed &= foreground
        if np.count_nonzero(observed[seed]) < min_support:
            return None, "Too few depth readings on a single outline"

    x, y, box_w, box_h = cv2.boundingRect(foreground.astype(np.uint8))
    distance_m = float(np.median(crop[observed])) / 1000
    return Obstacle(x + x0, y + y0, box_w, box_h, distance_m), "Surface found"


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
        self._show_aim = False
        self._inspection_status = "Waiting for stereo depth"
        self._detection: Obstacle | None = None
        self._recent: deque[Obstacle] = deque(maxlen=settings.confirm_frames)

    @property
    def inspection_status(self) -> str:
        with self._new_frame:
            return self._inspection_status

    def inspect(self, enabled: bool, *, show_aim: bool = False) -> None:
        with self._new_frame:
            self._inspecting = enabled
            self._show_aim = show_aim
            self._inspection_status = "Waiting for stereo depth"
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
                candidate, status = _scan_obstacle(depth, self.settings, image)
                obstacle = self._record_detection(candidate)
                self._inspection_status = status
                if obstacle is not None:
                    x, y, w, h = obstacle.x, obstacle.y, obstacle.width, obstacle.height
                    cv2.rectangle(image, (x, y), (x + w - 1, y + h - 1), (0, 255, 0), 2)
                    self._label(image, f"Obstacle {obstacle.distance_m:.2f} m", x, max(20, y - 6))
                else:
                    if candidate is not None:
                        self._inspection_status = f"Confirming obstacle ({len(self._recent)}/{self.settings.confirm_frames})"
                    if self._show_aim:
                        x0, y0, x1, y1 = _roi_bounds(self.settings.seed_roi, depth.shape)
                        cv2.rectangle(image, (x0, y0), (x1 - 1, y1 - 1), (0, 215, 255), 1)
                    self._label(image, self._inspection_status, 10, 24)

        ok, jpeg = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
        if not ok:
            raise RuntimeError("Could not encode the camera frame as JPEG")
        return jpeg.tobytes()

    @staticmethod
    def _label(image: np.ndarray, text: str, x: int, y: int) -> None:
        cv2.putText(image, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 4)
        cv2.putText(image, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
