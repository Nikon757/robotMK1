#!/usr/bin/env python3
"""ROS 2 node: keep only the forward ``fov_deg`` of a LaserScan (rear half is blocked by the
robot body / camera on RobotMK1) and clamp the range limits.

Runs inside the Docker container (rclpy).  Parameters:
    in_topic        (default: scan_raw)
    out_topic       (default: scan)
    forward_yaw_rad the yaw of the static base_link->laser transform (so that a beam at
                    scan angle theta has robot bearing theta + forward_yaw_rad)
    fov_deg         (default: 180)
    range_min / range_max (m)

Masked beams are set to NaN, which RTAB-Map treats as "no return".
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan


class ScanFovFilter(Node):
    def __init__(self):
        super().__init__("scan_fov_filter")
        self.declare_parameter("in_topic", "scan_raw")
        self.declare_parameter("out_topic", "scan")
        self.declare_parameter("forward_yaw_rad", 0.0)
        self.declare_parameter("fov_deg", 180.0)
        self.declare_parameter("range_min", 0.10)
        self.declare_parameter("range_max", 12.0)
        self.fwd = float(self.get_parameter("forward_yaw_rad").value)
        self.half = math.radians(float(self.get_parameter("fov_deg").value)) / 2.0
        self.rmin = float(self.get_parameter("range_min").value)
        self.rmax = float(self.get_parameter("range_max").value)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(LaserScan, self.get_parameter("out_topic").value, 10)
        self.sub = self.create_subscription(LaserScan, self.get_parameter("in_topic").value, self.cb, qos)
        self._mask_key = None
        self._keep = None
        self.n = 0
        self.get_logger().info(f"scan FOV filter: keep |bearing| <= {math.degrees(self.half):.0f} deg, "
                               f"forward yaw {math.degrees(self.fwd):.1f} deg, range {self.rmin}..{self.rmax} m")

    def cb(self, msg: LaserScan):
        n = len(msg.ranges)
        key = (n, msg.angle_min, msg.angle_increment)
        if key != self._mask_key:
            theta = msg.angle_min + msg.angle_increment * np.arange(n)
            bearing = (theta + self.fwd + math.pi) % (2 * math.pi) - math.pi
            self._keep = np.abs(bearing) <= self.half
            self._mask_key = key
        r = np.asarray(msg.ranges, dtype=np.float32)
        bad = ~self._keep | ~np.isfinite(r) | (r < self.rmin) | (r > self.rmax)
        r[bad] = float("nan")
        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = self.rmin
        out.range_max = self.rmax
        out.ranges = r.tolist()
        out.intensities = list(msg.intensities) if len(msg.intensities) == n else []
        self.pub.publish(out)
        self.n += 1
        if self.n == 20:
            self.get_logger().info(f"first scans OK: {n} beams, {int((~bad).sum())} kept in the forward FOV")


def main():
    rclpy.init()
    node = ScanFovFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
