"""OAK-D Lite source: synchronised RGB + depth (aligned to RGB) + IMU, DepthAI v2 API.

    from common.oakd import OakCamera
    with OakCamera(cfg) as cam:
        K = cam.K                        # 3x3 intrinsics for the (width, height) output
        f = cam.get_frame(timeout=1.0)   # Frame(t, rgb BGR uint8, depth uint16 mm, seq)
        imu = cam.get_imu()              # list[ImuSample] since the previous call

Timestamps are seconds on the Linux monotonic clock (DepthAI host-synced
``getTimestamp()``), so they are directly comparable with ``time.monotonic()``
used by the LiDAR and MCU threads.

Notes for the OAK-D Lite
* RGB sensor IMX214 -> 1080p ISP output scaled 1/3 = 640x360.
* Mono cameras OV7251 -> 480p.  Depth is computed at 640x480 and aligned to the RGB
  output (same 640x360 raster), so depth[v, u] belongs to rgb[v, u].
* IMU is BMI270 on most units (raw accel + gyro only, no on-chip fusion); BNO086
  on some.  ``imu_type`` tells which one; rotation vector is used when present.
"""
from __future__ import annotations

import time
from dataclasses import dataclass
from datetime import timedelta
from typing import Optional

import numpy as np

try:
    import depthai as dai
except ImportError:  # offline tools / replay
    dai = None


@dataclass
class Frame:
    t: float                 # seconds, monotonic
    rgb: np.ndarray          # HxWx3 uint8 BGR
    depth: np.ndarray        # HxW uint16, millimetres, 0 = invalid
    seq: int = 0


@dataclass
class ImuSample:
    t: float
    gyro: np.ndarray         # rad/s, IMU frame
    accel: np.ndarray        # m/s^2, IMU frame (specific force: reads +g "up" at rest)
    quat: Optional[np.ndarray] = None   # (w, x, y, z) IMU->world, only with BNO086 rotation vector


class OakCamera:
    def __init__(self, cfg, enable_rotation_vector: bool = True, usb2: bool = False):
        if dai is None:
            raise RuntimeError("depthai is not installed (pip install 'depthai>=2.30,<3')")
        cam = cfg.camera
        self.width, self.height, self.fps = int(cam.width), int(cam.height), float(cam.fps)
        self.imu_rate = int(cam.get("imu_rate_hz", 100))
        self.depth_min_mm = int(float(cam.get("depth_min_m", 0.25)) * 1000)
        self.depth_max_mm = int(float(cam.get("depth_max_m", 6.0)) * 1000)
        self._want_rotation = enable_rotation_vector
        self._usb2 = usb2
        self._seq = 0
        self.device = None
        self.imu_type = "unknown"
        self.has_rotation = False
        self.K = None
        self.T_cam_imu = None      # 4x4 from EEPROM if available (IMU pose in RGB optical frame)
        self._open()

    # ------------------------------------------------------------ pipeline
    def _build_pipeline(self, rotation: bool) -> "dai.Pipeline":
        p = dai.Pipeline()

        rgb = p.create(dai.node.ColorCamera)
        rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
        rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        rgb.setIspScale(1, 3)                       # 1920x1080 -> 640x360
        rgb.setFps(self.fps)
        rgb.setInterleaved(False)
        rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

        left = p.create(dai.node.MonoCamera)
        right = p.create(dai.node.MonoCamera)
        for m, sock in ((left, dai.CameraBoardSocket.CAM_B), (right, dai.CameraBoardSocket.CAM_C)):
            m.setBoardSocket(sock)
            m.setResolution(dai.MonoCameraProperties.SensorResolution.THE_480_P)
            m.setFps(self.fps)

        stereo = p.create(dai.node.StereoDepth)
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        stereo.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
        stereo.setLeftRightCheck(True)
        stereo.setSubpixel(True)
        stereo.setExtendedDisparity(False)
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
        stereo.setOutputSize(self.width, self.height)
        try:  # post-processing filters (available since depthai 2.17)
            c = stereo.initialConfig.get()
            c.postProcessing.speckleFilter.enable = True
            c.postProcessing.speckleFilter.speckleRange = 50
            c.postProcessing.spatialFilter.enable = True
            c.postProcessing.spatialFilter.holeFillingRadius = 2
            c.postProcessing.spatialFilter.numIterations = 1
            c.postProcessing.thresholdFilter.minRange = self.depth_min_mm
            c.postProcessing.thresholdFilter.maxRange = self.depth_max_mm
            c.postProcessing.decimationFilter.decimationFactor = 1
            stereo.initialConfig.set(c)
        except Exception:
            pass
        left.out.link(stereo.left)
        right.out.link(stereo.right)

        sync = p.create(dai.node.Sync)
        sync.setSyncThreshold(timedelta(milliseconds=int(500 / self.fps)))
        rgb.isp.link(sync.inputs["rgb"])
        stereo.depth.link(sync.inputs["depth"])
        xout = p.create(dai.node.XLinkOut)
        xout.setStreamName("sync")
        sync.out.link(xout.input)

        imu = p.create(dai.node.IMU)
        sensors = [dai.IMUSensor.ACCELEROMETER_RAW, dai.IMUSensor.GYROSCOPE_RAW]
        if rotation:
            sensors.append(dai.IMUSensor.ROTATION_VECTOR)
        imu.enableIMUSensor(sensors, self.imu_rate)
        imu.setBatchReportThreshold(1)
        imu.setMaxBatchReports(10)
        xout_imu = p.create(dai.node.XLinkOut)
        xout_imu.setStreamName("imu")
        imu.out.link(xout_imu.input)
        return p

    def _open(self):
        # Rotation vector only exists on BNO086; asking for it on a BMI270 fails at
        # pipeline start, so try with, then without.
        attempts = [True, False] if self._want_rotation else [False]
        last_err = None
        for rot in attempts:
            try:
                pipeline = self._build_pipeline(rot)
                if self._usb2:
                    self.device = dai.Device(pipeline, dai.UsbSpeed.HIGH)
                else:
                    self.device = dai.Device(pipeline)
                self.has_rotation = rot
                break
            except Exception as e:  # noqa: BLE001
                last_err = e
                if self.device is not None:
                    try:
                        self.device.close()
                    except Exception:
                        pass
                    self.device = None
        if self.device is None:
            raise RuntimeError(f"cannot open OAK-D: {last_err}")

        d = self.device
        try:
            self.imu_type = str(d.getConnectedIMU())
        except Exception:
            self.imu_type = "unknown"
        try:
            self.usb_speed = str(d.getUsbSpeed()).split(".")[-1]
        except Exception:
            self.usb_speed = "unknown"
        calib = d.readCalibration()
        self.K = np.array(calib.getCameraIntrinsics(dai.CameraBoardSocket.CAM_A, self.width, self.height), dtype=float)
        try:
            self.dist = np.array(calib.getDistortionCoefficients(dai.CameraBoardSocket.CAM_A), dtype=float)
        except Exception:
            self.dist = None
        try:
            ext = np.array(calib.getImuToCameraExtrinsics(dai.CameraBoardSocket.CAM_A), dtype=float)
            if ext.shape == (4, 4) and not np.allclose(ext, 0):
                ext[:3, 3] /= 100.0  # EEPROM stores cm
                self.T_cam_imu = ext
        except Exception:
            self.T_cam_imu = None
        self.q_sync = d.getOutputQueue("sync", maxSize=4, blocking=False)
        self.q_imu = d.getOutputQueue("imu", maxSize=50, blocking=False)
        self.mxid = d.getMxId()

    # ------------------------------------------------------------ data
    def get_frame(self, timeout: float = 1.0) -> Optional[Frame]:
        t0 = time.monotonic()
        while True:
            msg = self.q_sync.tryGet()
            if msg is not None:
                break
            if time.monotonic() - t0 > timeout:
                return None
            time.sleep(0.002)
        rgb_msg = msg["rgb"]
        depth_msg = msg["depth"]
        rgb = rgb_msg.getCvFrame()
        depth = depth_msg.getFrame()
        if rgb.shape[0] != depth.shape[0] or rgb.shape[1] != depth.shape[1]:
            import cv2
            depth = cv2.resize(depth, (rgb.shape[1], rgb.shape[0]), interpolation=cv2.INTER_NEAREST)
        t = rgb_msg.getTimestamp().total_seconds()
        self._seq += 1
        return Frame(t, rgb, depth, self._seq)

    def get_imu(self) -> list:
        out = []
        while True:
            data = self.q_imu.tryGet()
            if data is None:
                break
            for pk in data.packets:
                a = pk.acceleroMeter
                g = pk.gyroscope
                q = None
                if self.has_rotation:
                    rv = pk.rotationVector
                    q = np.array([rv.real, rv.i, rv.j, rv.k], dtype=float)
                out.append(ImuSample(a.getTimestamp().total_seconds(),
                                     np.array([g.x, g.y, g.z], dtype=float),
                                     np.array([a.x, a.y, a.z], dtype=float), q))
        return out

    def info(self) -> dict:
        return {"mxid": getattr(self, "mxid", "?"), "usb_speed": getattr(self, "usb_speed", "?"),
                "imu": self.imu_type, "rotation_vector": self.has_rotation,
                "width": self.width, "height": self.height, "fps": self.fps,
                "K": self.K.tolist() if self.K is not None else None,
                "T_cam_imu": self.T_cam_imu.tolist() if self.T_cam_imu is not None else None}

    def close(self):
        if self.device is not None:
            try:
                self.device.close()
            except Exception:
                pass
            self.device = None

    def __enter__(self):
        return self

    def __exit__(self, *a):
        self.close()


def list_devices() -> list:
    if dai is None:
        return []
    out = []
    for info in dai.Device.getAllAvailableDevices():
        out.append({"mxid": info.getMxId(), "state": str(info.state).split(".")[-1],
                    "name": getattr(info, "name", "")})
    return out
