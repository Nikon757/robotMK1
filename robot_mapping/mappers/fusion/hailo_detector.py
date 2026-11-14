"""Hailo-8 object detection (YOLOv8 with on-chip NMS) used to

* mask *dynamic* objects (people, pets, vehicles) out of the depth image before
  it is integrated into the map, and
* attach a COCO class label to map points that fall inside boxes of static
  objects (chairs, tables, ...).

Requirements (see DEPENDENCIES.md):
* ``sudo apt install hailo-all`` on Raspberry Pi OS -> provides the ``hailo_platform``
  python module (HailoRT) for the *system* python; our venv is created with
  ``--system-site-packages`` so it is visible.
* A ``.hef`` compiled for **Hailo-8** (not 8L) with NMS post-processing built in,
  e.g. ``yolov8s.hef`` from the Hailo Model Zoo.  Output of such models is one
  "nms" stream: per image a list of 80 arrays (one per class), each (N, 5) with
  ``[y_min, x_min, y_max, x_max, score]`` in normalised 0..1 coordinates.

The module degrades gracefully: ``HailoUnavailable`` is raised on any problem
(no chip, no HEF, wrong HailoRT) and the mapper continues without masking.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import List, Sequence

import cv2
import numpy as np

COCO_CLASSES = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light",
    "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
    "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
    "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard",
    "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
    "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
    "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone",
    "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear",
    "hair drier", "toothbrush",
]


class HailoUnavailable(RuntimeError):
    pass


@dataclass
class Detection:
    class_id: int
    name: str
    score: float
    x0: int
    y0: int
    x1: int
    y1: int


class HailoYoloDetector:
    def __init__(self, hef_path, score_threshold: float = 0.4, class_names: Sequence[str] = COCO_CLASSES):
        hef_path = Path(hef_path)
        if not hef_path.exists():
            raise HailoUnavailable(f"HEF not found: {hef_path} (download a Hailo-8 yolov8 .hef, see DEPENDENCIES.md)")
        try:
            from hailo_platform import (HEF, ConfigureParams, FormatType, HailoStreamInterface, InferVStreams,
                                        InputVStreamParams, OutputVStreamParams, VDevice)
        except ImportError as e:
            raise HailoUnavailable(f"hailo_platform not importable ({e}); install hailo-all and use a "
                                   f"--system-site-packages venv") from e
        try:
            self._hef = HEF(str(hef_path))
            self._target = VDevice()
            params = ConfigureParams.create_from_hef(self._hef, interface=HailoStreamInterface.PCIe)
            self._ng = self._target.configure(self._hef, params)[0]
            self._ng_params = self._ng.create_params()
            self._in_params = InputVStreamParams.make(self._ng, format_type=FormatType.UINT8)
            self._out_params = OutputVStreamParams.make(self._ng, format_type=FormatType.FLOAT32)
            self._in_info = self._hef.get_input_vstream_infos()[0]
            self._out_info = self._hef.get_output_vstream_infos()[0]
            shape = self._in_info.shape           # (H, W, C)
            self.in_h, self.in_w = int(shape[0]), int(shape[1])
            self._pipeline = InferVStreams(self._ng, self._in_params, self._out_params)
            self._pipeline.__enter__()
            self._activation = self._ng.activate(self._ng_params)
            self._activation.__enter__()
        except Exception as e:  # noqa: BLE001
            raise HailoUnavailable(f"failed to initialise Hailo device / HEF: {e}") from e
        self.score_threshold = float(score_threshold)
        self.names = list(class_names)
        self.frames = 0

    # ------------------------------------------------------------ inference
    def detect(self, bgr: np.ndarray) -> List[Detection]:
        h, w = bgr.shape[:2]
        # letterbox to the network input, keep aspect ratio
        scale = min(self.in_w / w, self.in_h / h)
        nw, nh = int(round(w * scale)), int(round(h * scale))
        resized = cv2.resize(bgr, (nw, nh), interpolation=cv2.INTER_LINEAR)
        canvas = np.full((self.in_h, self.in_w, 3), 114, dtype=np.uint8)
        ox, oy = (self.in_w - nw) // 2, (self.in_h - nh) // 2
        canvas[oy:oy + nh, ox:ox + nw] = resized
        rgb = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)
        batch = np.expand_dims(rgb, 0)
        results = self._pipeline.infer({self._in_info.name: batch})
        out = results[self._out_info.name]
        per_class = out[0] if isinstance(out, (list, tuple)) else out
        dets: List[Detection] = []
        for cid, arr in enumerate(per_class):
            arr = np.asarray(arr)
            if arr.size == 0:
                continue
            arr = arr.reshape(-1, 5)
            for ymin, xmin, ymax, xmax, score in arr:
                if score < self.score_threshold:
                    continue
                x0 = int((xmin * self.in_w - ox) / scale)
                x1 = int((xmax * self.in_w - ox) / scale)
                y0 = int((ymin * self.in_h - oy) / scale)
                y1 = int((ymax * self.in_h - oy) / scale)
                x0, x1 = max(0, min(w - 1, x0)), max(0, min(w - 1, x1))
                y0, y1 = max(0, min(h - 1, y0)), max(0, min(h - 1, y1))
                if x1 <= x0 or y1 <= y0:
                    continue
                name = self.names[cid] if cid < len(self.names) else str(cid)
                dets.append(Detection(cid, name, float(score), x0, y0, x1, y1))
        self.frames += 1
        return dets

    def close(self):
        for obj in (getattr(self, "_activation", None), getattr(self, "_pipeline", None)):
            try:
                obj.__exit__(None, None, None)
            except Exception:
                pass
        try:
            self._target.release()
        except Exception:
            pass


def make_masks(dets: List[Detection], shape, dynamic: set, label_map_classes: dict, dilate_px: int = 12):
    """-> (drop_mask HxW bool, label_map HxW uint8).  label_map values = label id (class_id + 1)."""
    h, w = shape[:2]
    drop = np.zeros((h, w), dtype=bool)
    labels = np.zeros((h, w), dtype=np.uint8)
    # larger boxes first so small objects on top keep their own label
    for d in sorted(dets, key=lambda d: -(d.x1 - d.x0) * (d.y1 - d.y0)):
        if d.name in dynamic:
            x0, y0 = max(0, d.x0 - dilate_px), max(0, d.y0 - dilate_px)
            x1, y1 = min(w, d.x1 + dilate_px + 1), min(h, d.y1 + dilate_px + 1)
            drop[y0:y1, x0:x1] = True
        elif d.name in label_map_classes:
            labels[d.y0:d.y1 + 1, d.x0:d.x1 + 1] = label_map_classes[d.name]
    return drop, labels


def draw_detections(img: np.ndarray, dets: List[Detection], dynamic: set) -> np.ndarray:
    for d in dets:
        color = (0, 0, 255) if d.name in dynamic else (255, 160, 0)
        cv2.rectangle(img, (d.x0, d.y0), (d.x1, d.y1), color, 2)
        cv2.putText(img, f"{d.name} {d.score:.2f}", (d.x0, max(12, d.y0 - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    color, 1, cv2.LINE_AA)
    return img


class FakeDetector:
    """Stand-in used by --replay tests when no Hailo chip is present (returns no detections)."""
    in_w = in_h = 640
    frames = 0

    def detect(self, bgr):
        self.frames += 1
        return []

    def close(self):
        pass
