"""LiveHub: opens the OAK-D, the LiDAR and (optionally) the STM32 board and exposes the
same interface as ``common.recorder.RawReader`` so the mappers are source-agnostic.

    hub = LiveHub(cfg, use_lidar=True, use_mcu=True)
    hub.K, hub.width, hub.height, hub.imu_type, hub.has_rotation, hub.has_lidar
    f = hub.get_frame(timeout)      # Frame or None
    imu = hub.get_imu()             # new ImuSample list
    scan = hub.latest_scan()        # newest LidarScan or None
    hub.close()
"""
from __future__ import annotations

import sys
from typing import Optional

from .ldlidar import LDLidar, LidarScan
from .oakd import OakCamera


class LiveHub:
    def __init__(self, cfg, use_lidar: bool = True, use_mcu: bool = True, usb2: bool = False):
        self.cfg = cfg
        self.cam = OakCamera(cfg, usb2=usb2)
        self.K = self.cam.K
        self.width, self.height = self.cam.width, self.cam.height
        self.imu_type = self.cam.imu_type
        self.has_rotation = self.cam.has_rotation
        self.lidar = None
        self.mcu = None
        self.has_lidar = False
        if use_lidar:
            try:
                self.lidar = LDLidar(cfg.serial.lidar_port, int(cfg.serial.lidar_baud))
                self.has_lidar = True
            except Exception as e:  # noqa: BLE001
                print(f"[sensors] WARNING: LiDAR unavailable ({e}); continuing with camera only", file=sys.stderr)
        if use_mcu:
            try:
                from .mcu import Mcu
                self.mcu = Mcu(cfg.serial.mcu_port, int(cfg.serial.mcu_baud))
                if self.mcu.alive:
                    self.mcu.center_servos()          # camera must be centred for the fixed extrinsics
                    if not self.mcu.encoders_alive:
                        print("[sensors] note: MCU telemetry OK, encoders report 0 (known firmware limitation)",
                              file=sys.stderr)
                else:
                    print("[sensors] WARNING: MCU port opened but no telemetry; ignoring the board", file=sys.stderr)
            except Exception as e:  # noqa: BLE001
                if bool(cfg.serial.get("mcu_required", False)):
                    raise
                print(f"[sensors] note: MCU board not found ({e}); mapping does not need it", file=sys.stderr)
                self.mcu = None

    def get_frame(self, timeout: float = 1.0):
        return self.cam.get_frame(timeout)

    def get_imu(self):
        return self.cam.get_imu()

    def latest_scan(self) -> Optional[LidarScan]:
        return self.lidar.latest() if self.lidar else None

    def info(self) -> dict:
        d = self.cam.info()
        d["lidar"] = {"port": self.lidar.port, "baud": self.lidar.baud} if self.lidar else None
        d["mcu"] = {"port": self.mcu.port, "encoders_alive": self.mcu.encoders_alive} if self.mcu else None
        return d

    def close(self):
        for dev in (self.lidar, self.mcu, self.cam):
            if dev is not None:
                try:
                    dev.close()
                except Exception:
                    pass
