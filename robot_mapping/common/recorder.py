"""Raw sensor recording (``--raw``) and replay (``--replay DIR``).

Layout of a raw recording directory::

    meta.json            camera intrinsics, image size, imu type, config snapshot
    frames.csv           seq,t
    rgb/000001.jpg       BGR JPEG (quality 92)
    depth/000001.png     uint16 millimetres
    imu.csv              t,gx,gy,gz,ax,ay,az[,qw,qx,qy,qz]
    scans/000001.npz     t, angles_deg, ranges_m, intensities, speed_dps (raw sensor frame)

``RawReader`` exposes the same duck-typed interface as ``common.sensors.LiveHub``
(``get_frame()``, ``get_imu()``, ``latest_scan()``, ``K`` ...) so the mappers do
not care whether data is live or replayed.
"""
from __future__ import annotations

import csv
import json
from pathlib import Path
from typing import Optional

import numpy as np

from .ldlidar import LidarScan
from .oakd import Frame, ImuSample


class RawRecorder:
    def __init__(self, out_dir, meta: dict, jpeg_quality: int = 92):
        import cv2
        self.cv2 = cv2
        self.dir = Path(out_dir)
        (self.dir / "rgb").mkdir(parents=True, exist_ok=True)
        (self.dir / "depth").mkdir(exist_ok=True)
        (self.dir / "scans").mkdir(exist_ok=True)
        with open(self.dir / "meta.json", "w", encoding="utf-8") as f:
            json.dump(meta, f, indent=2, default=_json_default)
        self._frames = open(self.dir / "frames.csv", "w", newline="", encoding="utf-8")
        self._frames_w = csv.writer(self._frames)
        self._frames_w.writerow(["seq", "t"])
        self._imu = open(self.dir / "imu.csv", "w", newline="", encoding="utf-8")
        self._imu_w = csv.writer(self._imu)
        self._imu_w.writerow(["t", "gx", "gy", "gz", "ax", "ay", "az", "qw", "qx", "qy", "qz"])
        self._jpg = [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_quality]
        self.n_frames = self.n_scans = self.n_imu = 0
        self._last_scan_t = None

    def add_frame(self, f: Frame):
        self.cv2.imwrite(str(self.dir / "rgb" / f"{f.seq:06d}.jpg"), f.rgb, self._jpg)
        self.cv2.imwrite(str(self.dir / "depth" / f"{f.seq:06d}.png"), f.depth.astype(np.uint16))
        self._frames_w.writerow([f.seq, f"{f.t:.6f}"])
        self.n_frames += 1

    def add_imu(self, samples):
        for s in samples:
            q = list(s.quat) if s.quat is not None else ["", "", "", ""]
            self._imu_w.writerow([f"{s.t:.6f}", *[f"{v:.6f}" for v in s.gyro], *[f"{v:.5f}" for v in s.accel], *q])
            self.n_imu += 1

    def add_scan(self, s: Optional[LidarScan]):
        if s is None or s.t == self._last_scan_t:
            return
        self._last_scan_t = s.t
        self.n_scans += 1
        np.savez_compressed(self.dir / "scans" / f"{self.n_scans:06d}.npz", t=s.t, angles_deg=s.angles_deg,
                            ranges_m=s.ranges_m, intensities=s.intensities, speed_dps=s.speed_dps)

    def flush(self):
        self._frames.flush()
        self._imu.flush()

    def close(self):
        self._frames.close()
        self._imu.close()


class RawReader:
    """Replay a raw recording in frame order (optionally paced to real time)."""

    def __init__(self, rec_dir, realtime: bool = False, start: int = 0, stop: Optional[int] = None):
        import cv2
        self.cv2 = cv2
        self.dir = Path(rec_dir)
        if not (self.dir / "meta.json").exists():
            raise FileNotFoundError(f"{self.dir} is not a raw recording (meta.json missing)")
        self.meta = json.loads((self.dir / "meta.json").read_text(encoding="utf-8"))
        self.K = np.array(self.meta["K"], dtype=float)
        self.width = int(self.meta["width"])
        self.height = int(self.meta["height"])
        self.imu_type = self.meta.get("imu", "replay")
        self.has_rotation = bool(self.meta.get("rotation_vector", False))
        self.has_lidar = True
        self.mcu = None
        self._frames = []
        with open(self.dir / "frames.csv", newline="", encoding="utf-8") as f:
            for row in csv.DictReader(f):
                self._frames.append((int(row["seq"]), float(row["t"])))
        self._frames = self._frames[start:stop]
        self._imu = self._load_imu()
        self._scans = sorted(self.dir.glob("scans/*.npz"))
        self._scan_t = np.array([float(np.load(p)["t"]) for p in self._scans]) if self._scans else np.zeros(0)
        self._scan_cache = {}
        self._i = 0
        self._imu_i = 0
        self._t_cur = None
        self.realtime = realtime
        self._wall0 = None
        self._t0 = None

    def _load_imu(self):
        rows = []
        p = self.dir / "imu.csv"
        if not p.exists():
            return rows
        with open(p, newline="", encoding="utf-8") as f:
            for row in csv.DictReader(f):
                q = None
                if row.get("qw") not in (None, ""):
                    q = np.array([float(row["qw"]), float(row["qx"]), float(row["qy"]), float(row["qz"])])
                rows.append(ImuSample(float(row["t"]),
                                      np.array([float(row["gx"]), float(row["gy"]), float(row["gz"])]),
                                      np.array([float(row["ax"]), float(row["ay"]), float(row["az"])]), q))
        return rows

    def __len__(self):
        return len(self._frames)

    @property
    def progress(self) -> float:
        return self._i / max(1, len(self._frames))

    def get_frame(self, timeout: float = 0.0) -> Optional[Frame]:
        if self._i >= len(self._frames):
            return None
        seq, t = self._frames[self._i]
        self._i += 1
        rgb = self.cv2.imread(str(self.dir / "rgb" / f"{seq:06d}.jpg"), self.cv2.IMREAD_COLOR)
        depth = self.cv2.imread(str(self.dir / "depth" / f"{seq:06d}.png"), self.cv2.IMREAD_UNCHANGED)
        if rgb is None or depth is None:
            return self.get_frame(timeout)
        if self.realtime:
            import time
            if self._wall0 is None:
                self._wall0, self._t0 = time.monotonic(), t
            else:
                wait = (t - self._t0) - (time.monotonic() - self._wall0)
                if wait > 0:
                    time.sleep(min(wait, 0.5))
        self._t_cur = t
        return Frame(t, rgb, depth.astype(np.uint16), seq)

    def get_imu(self) -> list:
        """IMU samples up to the current frame time (not yet returned)."""
        if self._t_cur is None:
            return []
        out = []
        while self._imu_i < len(self._imu) and self._imu[self._imu_i].t <= self._t_cur + 1e-6:
            out.append(self._imu[self._imu_i])
            self._imu_i += 1
        return out

    def latest_scan(self) -> Optional[LidarScan]:
        if self._t_cur is None or len(self._scan_t) == 0:
            return None
        j = int(np.searchsorted(self._scan_t, self._t_cur + 1e-6, side="right")) - 1
        if j < 0:
            return None
        if j not in self._scan_cache:
            d = np.load(self._scans[j])
            self._scan_cache = {j: LidarScan(float(d["t"]), d["angles_deg"], d["ranges_m"], d["intensities"],
                                             float(d["speed_dps"]))}
        return self._scan_cache[j]

    def info(self) -> dict:
        return {"replay": str(self.dir), "frames": len(self._frames), "imu_samples": len(self._imu),
                "scans": len(self._scans), **{k: self.meta.get(k) for k in ("imu", "usb_speed", "mxid")}}

    def close(self):
        pass


def _json_default(o):
    if isinstance(o, np.ndarray):
        return o.tolist()
    if isinstance(o, (np.floating, np.integer)):
        return o.item()
    if isinstance(o, Path):
        return str(o)
    return str(o)
