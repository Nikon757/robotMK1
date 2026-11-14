#!/usr/bin/env python3
"""RobotMK1 "fusion" mapper - RGB-D + 2-D LiDAR + IMU SLAM in pure Python (numpy / scipy /
OpenCV), with optional Hailo-8 object detection for dynamic-object removal and semantic labels.

Pipeline per camera frame
    1. (Hailo, every N frames) YOLOv8 -> drop mask for dynamic classes + label map
    2. ORB features on the RGB image, lifted to 3-D with the aligned depth
    3. Visual odometry: PnP RANSAC against the last keyframe, primed with the IMU
       rotation since the previous frame and a constant-velocity translation guess
    4. LiDAR: forward-180deg scan -> 2-D ICP against the local map of recent keyframe
       scans; the planar (x, y, yaw) correction is blended into the pose
    5. Keyframe when moved > keyframe_dist_m or turned > keyframe_rot_deg:
       depth back-projected (minus dynamic objects) into a voxel map, scan added to
       the local map, pose-graph node + odometry edge, loop-closure search (ORB + PnP
       against older keyframes nearby), SE(2) graph optimisation on success
    6. On exit: final optimisation, map rebuilt from keyframes, export:
       map.ply (xyz rgb label) . lidar_map.ply . poses.txt (TUM) . keyframes.json .
       occupancy.pgm/.yaml (Nav2) . preview.png . stats.json

Usage (normally via  python run.py fusion ...):
    python mappers/fusion/fusion_mapper.py --out maps/room1 --record [--hailo] [--viz] [--raw]
    python mappers/fusion/fusion_mapper.py --replay maps/room1/raw --out maps/room1_replay --record

Keys (terminal or --viz window):  p = pause/resume map integration,  q = stop and save.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from common.config import load_config  # noqa: E402
from common.imu_filter import ImuOrientation  # noqa: E402
from common.keys import KeyListener  # noqa: E402
from common.ldlidar import scan_to_points  # noqa: E402
from common.occupancy import OccupancyGrid, render_preview  # noqa: E402
from common.pointcloud import VoxelMap, backproject, voxel_downsample, write_ply  # noqa: E402
from common.transforms import (R_to_quat, T_from_mount, T_from_se2, T_optical_from_mount, inv_T,  # noqa: E402
                               project_to_se2, rotation_angle_deg, se2_from_T, se2_relative,
                               transform_points, wrap_angle)
from mappers.fusion.hailo_detector import COCO_CLASSES, HailoUnavailable, draw_detections, make_masks  # noqa: E402
from mappers.fusion.pose_graph import PoseGraphSE2  # noqa: E402
from mappers.fusion.scan_match import LocalScanMap, icp_2d, icp_pair  # noqa: E402
from mappers.fusion.vo import FeatureTracker, Features  # noqa: E402


@dataclass
class Keyframe:
    idx: int
    t: float
    seq: int
    T_wb: np.ndarray                     # world <- base_link
    feats: Features
    cloud_cam: np.ndarray                # (N,3) float32, camera optical frame, voxel-filtered
    colors: Optional[np.ndarray]         # (N,3) uint8 RGB
    labels: Optional[np.ndarray]         # (N,)  uint8
    scan_base: Optional[np.ndarray]      # (M,2) LiDAR points in base_link (forward FOV only)
    node: int = -1
    integrated: bool = True
    weak: bool = False


class FusionMapper:
    def __init__(self, cfg, K: np.ndarray, width: int, height: int, force_3dof: bool = True,
                 detector=None, log=print):
        self.cfg = cfg
        self.K = np.asarray(K, dtype=float)
        self.w, self.h = int(width), int(height)
        self.force_3dof = bool(force_3dof)
        self.detector = detector
        self.log = log
        m = cfg.mapping
        self.voxel = float(m.voxel_m)
        self.kf_dist = float(m.keyframe_dist_m)
        self.kf_rot = float(m.keyframe_rot_deg)
        self.loop_radius = float(m.get("loop_search_radius_m", 2.0))
        self.loop_gap = int(m.get("loop_min_index_gap", 15))
        self.loop_min_inliers = int(m.get("loop_min_inliers", 40))
        self.min_hits = int(m.get("min_hits", 2))
        self.stride = int(m.get("depth_stride", 2))
        self.depth_min_mm = int(float(cfg.camera.get("depth_min_m", 0.25)) * 1000)
        self.depth_max_mm = int(float(cfg.camera.get("depth_max_m", 6.0)) * 1000)

        self.T_bc = T_optical_from_mount(cfg.camera.mount)     # base <- camera optical
        self.T_cb = inv_T(self.T_bc)
        self.T_bl = T_from_mount(cfg.lidar.mount)              # base <- lidar
        self.lidar_z = float(cfg.lidar.mount.z)

        self.tracker = FeatureTracker(self.K, n_features=1000, min_inliers=15,
                                      min_depth_m=self.depth_min_mm / 1000, max_depth_m=self.depth_max_mm / 1000)
        self.vmap = VoxelMap(self.voxel)
        self.lidar_map = LocalScanMap(voxel=max(0.05, self.voxel), window=40)
        self.graph = PoseGraphSE2()
        self.keyframes: List[Keyframe] = []
        self.ref: Optional[Keyframe] = None
        self.T_wb = np.eye(4)
        self.t_last: Optional[float] = None
        self.v = np.zeros(3)
        self.lost = 0
        self.frames = 0
        self.frames_since_kf = 0
        self.map_dirty = False
        self.last_dets = []
        self._drop_mask = None
        self._label_map = None
        self._scan_cache = (None, None)
        self.stats = {"frames": 0, "vo_ok": 0, "vo_lost": 0, "icp_used": 0, "loops": 0, "optimizations": 0,
                      "keyframes": 0, "hailo_frames": 0, "masked_frames": 0}
        self.status = {}
        hl = cfg.hailo
        self.dynamic = set(hl.dynamic_classes)
        self.label_ids = {name: COCO_CLASSES.index(name) + 1 for name in hl.get("label_classes", []) if name in COCO_CLASSES}
        self.every_n = int(hl.get("every_n_frames", 3))
        self.dilate = int(hl.get("box_dilate_px", 12))
        self.trajectory: List[np.ndarray] = []

    # ------------------------------------------------------------------ helpers
    def scan_to_base(self, scan) -> Optional[np.ndarray]:
        """Cropped LiDAR scan -> (M,2) points in base_link (cached per scan)."""
        if scan is None:
            return None
        if self._scan_cache[0] is scan:
            return self._scan_cache[1]
        L = self.cfg.lidar
        xy, _, _ = scan_to_points(scan, float(L.forward_angle_deg), float(L.fov_deg), bool(L.clockwise),
                                  float(L.min_range_m), float(L.max_range_m), float(L.get("min_intensity", 0)))
        if len(xy):
            R2 = self.T_bl[:2, :2]
            xy = xy @ R2.T + self.T_bl[:2, 3]
        self._scan_cache = (scan, xy)
        return xy

    def _scan_world(self, kf: Keyframe) -> Optional[np.ndarray]:
        if kf.scan_base is None or len(kf.scan_base) == 0:
            return None
        return kf.scan_base @ kf.T_wb[:2, :2].T + kf.T_wb[:2, 3]

    def _integrate_kf(self, kf: Keyframe):
        if len(kf.cloud_cam) == 0:
            return
        pts_w = transform_points(kf.T_wb @ self.T_bc, kf.cloud_cam)
        self.vmap.integrate(pts_w, kf.colors, kf.labels)

    def _rebuild_map(self):
        self.vmap.clear()
        for kf in self.keyframes:
            if kf.integrated:
                self._integrate_kf(kf)
        self.map_dirty = False

    # ------------------------------------------------------------------ main step
    def process(self, frame, imu: Optional[ImuOrientation], scan, integrate: bool = True) -> dict:
        self.frames += 1
        self.stats["frames"] += 1
        gray = cv2.cvtColor(frame.rgb, cv2.COLOR_BGR2GRAY)

        # 1) Hailo detections -> masks
        if self.detector is not None and (self.frames % self.every_n == 1 or self.every_n == 1):
            try:
                self.last_dets = self.detector.detect(frame.rgb)
                self.stats["hailo_frames"] += 1
            except Exception as e:  # noqa: BLE001
                self.log(f"[hailo] detection failed, disabling: {e}")
                self.detector = None
                self.last_dets = []
        if self.last_dets:
            self._drop_mask, self._label_map = make_masks(self.last_dets, frame.rgb.shape, self.dynamic,
                                                          self.label_ids, self.dilate)
            if self._drop_mask.any():
                self.stats["masked_frames"] += 1
        else:
            self._drop_mask, self._label_map = None, None

        # 2) features
        feats = self.tracker.extract(gray, frame.depth, self._drop_mask)

        if self.ref is None:                      # first frame anchors the map
            self.T_wb = np.eye(4)
            self.t_last = frame.t
            self._new_keyframe(frame, feats, self.T_wb, scan, integrate)
            self.status = {"vo": "init", "inliers": 0, "matches": 0}
            return self.status

        # 3) prior from IMU rotation + constant velocity
        dt = max(0.0, frame.t - self.t_last) if self.t_last is not None else 0.0
        R_last = self.T_wb[:3, :3]
        dR = imu.delta_R(self.t_last, frame.t) if (imu is not None and imu.ready) else None
        R_prior = R_last @ dR if dR is not None else R_last
        t_prior = self.T_wb[:3, 3] + self.v * dt * (0.0 if self.lost > 3 else 1.0)
        T_wb_prior = np.eye(4)
        T_wb_prior[:3, :3] = R_prior
        T_wb_prior[:3, 3] = t_prior
        if self.force_3dof:
            T_wb_prior = project_to_se2(T_wb_prior)
        T_wc_prior = T_wb_prior @ self.T_bc
        T_wc_ref = self.ref.T_wb @ self.T_bc
        guess = inv_T(T_wc_prior) @ T_wc_ref

        # 4) visual odometry
        res = self.tracker.estimate(self.ref.feats, feats, guess)
        if res.ok:
            T_wc = T_wc_ref @ inv_T(res.T_cur_ref)
            T_wb = T_wc @ self.T_cb
            if self.force_3dof:
                T_wb = project_to_se2(T_wb)
            self.lost = 0
            self.stats["vo_ok"] += 1
        else:
            T_wb = T_wb_prior
            self.lost += 1
            self.stats["vo_lost"] += 1

        # 5) LiDAR planar correction
        icp_fit = None
        icp_used = False
        xy_base = self.scan_to_base(scan) if self.force_3dof else None
        if xy_base is not None and len(xy_base) >= 30 and self.lidar_map.tree is not None:
            p_vo = se2_from_T(T_wb)
            r = icp_2d(xy_base, self.lidar_map, p_vo, max_corr=0.5, min_fitness=0.5)
            icp_fit = r.fitness
            if r.ok:
                d = math.hypot(r.pose[0] - p_vo[0], r.pose[1] - p_vo[1])
                dyaw = abs(wrap_angle(r.pose[2] - p_vo[2]))
                if d < 0.3 and dyaw < math.radians(15):
                    if res.ok:
                        w = 0.5
                        p = p_vo.copy()
                        p[:2] += w * (r.pose[:2] - p_vo[:2])
                        p[2] = p_vo[2] + w * wrap_angle(r.pose[2] - p_vo[2])
                    else:
                        p = r.pose
                    T_wb = T_from_se2(p)
                    icp_used = True
                    self.stats["icp_used"] += 1

        # 6) bookkeeping
        if dt > 1e-3:
            v = (T_wb[:3, 3] - self.T_wb[:3, 3]) / dt
            self.v = np.clip(0.7 * self.v + 0.3 * v, -2.0, 2.0)
        self.T_wb = T_wb
        self.t_last = frame.t
        self.frames_since_kf += 1
        self.trajectory.append(T_wb[:3, 3].copy())

        # 7) keyframe decision
        dpos = float(np.linalg.norm(T_wb[:3, 3] - self.ref.T_wb[:3, 3]))
        drot = rotation_angle_deg(self.ref.T_wb[:3, :3].T @ T_wb[:3, :3])
        need = dpos > self.kf_dist or drot > self.kf_rot
        weak = False
        if not res.ok:
            if self.lost >= 10 and self.frames_since_kf >= 10:
                need, weak = True, not icp_used          # re-anchor tracking on a fresh keyframe
            elif icp_used and self.frames_since_kf >= 5:
                need = True
        if need:
            self._new_keyframe(frame, feats, T_wb, scan, integrate, weak=weak)

        self.status = {"vo": "ok" if res.ok else "lost", "inliers": res.inliers, "matches": res.matches,
                       "icp": icp_fit, "icp_used": icp_used, "lost": self.lost, "kf": len(self.keyframes),
                       "loops": self.graph.n_loops, "voxels": len(self.vmap), "dets": len(self.last_dets)}
        return self.status

    # ------------------------------------------------------------------ keyframes
    def _new_keyframe(self, frame, feats: Features, T_wb: np.ndarray, scan, integrate: bool, weak: bool = False):
        pts, pix = backproject(frame.depth, self.K, self.stride, self._drop_mask, self.depth_min_mm, self.depth_max_mm)
        colors = frame.rgb[pix[:, 1], pix[:, 0], ::-1] if len(pix) else None
        labels = self._label_map[pix[:, 1], pix[:, 0]] if (self._label_map is not None and len(pix)) else None
        if len(pts):
            pts, colors, labels = voxel_downsample(pts, self.voxel * 0.9, colors, labels)
        xy_base = self.scan_to_base(scan)
        kf = Keyframe(len(self.keyframes), frame.t, frame.seq, T_wb.copy(), feats, pts.astype(np.float32),
                      colors, labels, xy_base.copy() if xy_base is not None else None,
                      integrated=integrate, weak=weak)
        if self.force_3dof:
            kf.node = self.graph.add_node(se2_from_T(T_wb))
            if self.keyframes:
                prev = self.keyframes[-1]
                rel = se2_relative(se2_from_T(prev.T_wb), se2_from_T(T_wb))
                info = (1.0, 1.0, 4.0) if weak else (100.0, 100.0, 400.0)
                self.graph.add_edge(prev.node, kf.node, rel, info, "weak" if weak else "odom")
        self.keyframes.append(kf)
        self.ref = kf
        self.frames_since_kf = 0
        self.stats["keyframes"] = len(self.keyframes)
        if integrate:
            self._integrate_kf(kf)
        sw = self._scan_world(kf)
        if sw is not None and len(sw) >= 20:
            self.lidar_map.add(kf.idx, sw)
        if self.force_3dof:
            self._try_loop_closure(kf)

    def _try_loop_closure(self, kf: Keyframe):
        if len(self.keyframes) < self.loop_gap + 2:
            return
        poses = self.graph.poses()
        p = poses[kf.node]
        cands = []
        for c in self.keyframes[: len(self.keyframes) - self.loop_gap - 1]:
            q = poses[c.node]
            d = math.hypot(q[0] - p[0], q[1] - p[1])
            if d < self.loop_radius:
                cands.append((d, c))
        cands.sort(key=lambda x: x[0])
        found = 0
        for d, c in cands[:3]:
            T_wc_c = c.T_wb @ self.T_bc
            T_wc_k = kf.T_wb @ self.T_bc
            guess = inv_T(T_wc_k) @ T_wc_c
            r = self.tracker.estimate(c.feats, kf.feats, guess, min_inliers=self.loop_min_inliers)
            if not r.ok:
                continue
            T_wb_k_est = T_wc_c @ inv_T(r.T_cur_ref) @ self.T_cb
            rel = se2_relative(se2_from_T(c.T_wb), se2_from_T(T_wb_k_est))
            if math.hypot(rel[0], rel[1]) > 1.5 * self.loop_radius:
                continue
            info = (50.0, 50.0, 200.0)
            if c.scan_base is not None and kf.scan_base is not None and len(c.scan_base) > 50 and len(kf.scan_base) > 50:
                ri = icp_pair(kf.scan_base, c.scan_base, rel, voxel=0.05, max_corr=0.4, min_fitness=0.6)
                if ri.ok and math.hypot(ri.pose[0] - rel[0], ri.pose[1] - rel[1]) < 0.3 and \
                        abs(wrap_angle(ri.pose[2] - rel[2])) < math.radians(10):
                    rel = ri.pose
                    info = (200.0, 200.0, 800.0)
            self.graph.add_edge(c.node, kf.node, rel, info, "loop")
            found += 1
            self.stats["loops"] += 1
        if found:
            self._optimize()

    def _optimize(self, final: bool = False):
        if not self.force_3dof or len(self.graph.nodes) < 2:
            return
        last = self.keyframes[-1]
        T_last_old = last.T_wb.copy()
        before = self.graph.poses()
        st = self.graph.optimize()
        self.stats["optimizations"] += 1
        after = self.graph.poses()
        moved = float(np.max(np.hypot(after[:, 0] - before[:, 0], after[:, 1] - before[:, 1]))) if len(after) else 0.0
        turned = float(np.max(np.abs(wrap_angle(after[:, 2] - before[:, 2])))) if len(after) else 0.0
        for kf in self.keyframes:
            kf.T_wb = T_from_se2(self.graph.nodes[kf.node])
        T_corr = last.T_wb @ inv_T(T_last_old)
        self.T_wb = project_to_se2(T_corr @ self.T_wb)
        if self.trajectory:
            self.trajectory = [(T_corr[:3, :3] @ p + T_corr[:3, 3]) for p in self.trajectory]
        self.lidar_map.replace_all([(kf.idx, self._scan_world(kf)) for kf in self.keyframes
                                    if self._scan_world(kf) is not None])
        significant = moved > 0.5 * self.voxel or turned > math.radians(0.5)
        if significant:
            self.map_dirty = True
        if self.map_dirty and (final or len(self.keyframes) <= 150):
            self._rebuild_map()
        if significant or final:
            self.log(f"[graph] optimised {st['nodes']} nodes, {st['edges']} edges ({st.get('loops', 0)} loops): "
                     f"cost {st['cost0']:.1f} -> {st['cost']:.1f}, max correction {moved * 100:.1f} cm / "
                     f"{math.degrees(turned):.1f} deg")

    # ------------------------------------------------------------------ finish / export
    def finish(self) -> dict:
        if self.force_3dof and self.graph.n_loops > 0:
            self._optimize(final=True)
        if self.map_dirty:
            self._rebuild_map()
        self.stats["voxels"] = len(self.vmap)
        return self.stats

    def lidar_cloud_world(self) -> np.ndarray:
        parts = []
        for kf in self.keyframes:
            sw = self._scan_world(kf)
            if sw is not None and len(sw):
                parts.append(np.column_stack([sw, np.full(len(sw), self.lidar_z)]))
        return np.concatenate(parts).astype(np.float32) if parts else np.zeros((0, 3), np.float32)

    def keyframe_poses(self) -> np.ndarray:
        return np.array([kf.T_wb[:3, 3] for kf in self.keyframes]) if self.keyframes else np.zeros((0, 3))

    def export(self, out_dir, title: str = "") -> List[Path]:
        out = Path(out_dir)
        out.mkdir(parents=True, exist_ok=True)
        files = []
        xyz, rgb, lab, cnt = self.vmap.extract(self.min_hits)
        files.append(write_ply(out / "map.ply", xyz, rgb, lab))
        lidar = self.lidar_cloud_world()
        if len(lidar):
            files.append(write_ply(out / "lidar_map.ply", lidar))
        with open(out / "poses.txt", "w", encoding="utf-8") as f:
            f.write("# TUM format: t x y z qx qy qz qw   (base_link in map frame, keyframes only)\n")
            for kf in self.keyframes:
                q = R_to_quat(kf.T_wb[:3, :3])
                t = kf.T_wb[:3, 3]
                f.write(f"{kf.t:.6f} {t[0]:.4f} {t[1]:.4f} {t[2]:.4f} {q[1]:.6f} {q[2]:.6f} {q[3]:.6f} {q[0]:.6f}\n")
        files.append(out / "poses.txt")
        with open(out / "keyframes.json", "w", encoding="utf-8") as f:
            json.dump([{"idx": kf.idx, "t": kf.t, "seq": kf.seq, "pose": se2_from_T(kf.T_wb).tolist(),
                        "points": int(len(kf.cloud_cam)), "scan_points": int(len(kf.scan_base)) if kf.scan_base is not None else 0,
                        "integrated": kf.integrated, "weak": kf.weak} for kf in self.keyframes], f, indent=1)
        files.append(out / "keyframes.json")

        # occupancy grid: LiDAR ray tracing from every keyframe + camera cloud heights
        m = self.cfg.mapping
        poses = self.keyframe_poses()
        allxy = [xyz[:, :2]] if len(xyz) else []
        if len(lidar):
            allxy.append(lidar[:, :2])
        if len(poses):
            allxy.append(poses[:, :2])
        if allxy:
            grid = OccupancyGrid.from_points(np.concatenate(allxy), float(m.grid_resolution_m))
            for kf in self.keyframes:
                sw = self._scan_world(kf)
                if sw is not None and len(sw):
                    origin = (kf.T_wb @ self.T_bl)[:2, 3]
                    grid.raytrace(origin, sw)
            if len(xyz):
                grid.add_cloud(xyz, float(m.ground_z_m), float(m.obstacle_max_z_m))
            pgm, yml = grid.save(out / "occupancy")
            files += [pgm, yml]
            files.append(render_preview(out / "preview.png", xyz, rgb, grid, poses, title=title))
        stats = dict(self.stats)
        stats.update({"map_points": int(len(xyz)), "lidar_points": int(len(lidar)), "force_3dof": self.force_3dof,
                      "labels_present": int((lab > 0).sum()) if len(lab) else 0})
        with open(out / "stats.json", "w", encoding="utf-8") as f:
            json.dump(stats, f, indent=2)
        files.append(out / "stats.json")
        return files

    # ------------------------------------------------------------------ live view
    def render_topdown(self, size: int = 480, scan_base: Optional[np.ndarray] = None) -> np.ndarray:
        img = np.full((size, size, 3), 30, dtype=np.uint8)
        xyz, rgb, lab, cnt = self.vmap.extract(1)
        pts = [xyz[:, :2]] if len(xyz) else []
        if self.trajectory:
            pts.append(np.array(self.trajectory)[:, :2])
        if not pts:
            return img
        allp = np.concatenate(pts)
        lo, hi = allp.min(axis=0) - 0.5, allp.max(axis=0) + 0.5
        span = float(max(hi - lo) or 1.0)
        scale = (size - 20) / span

        def to_px(p):
            return (10 + (p[:, 0] - lo[0]) * scale).astype(int), (size - 10 - (p[:, 1] - lo[1]) * scale).astype(int)

        if len(xyz):
            z = xyz[:, 2]
            band = (z > float(self.cfg.mapping.ground_z_m)) & (z < float(self.cfg.mapping.obstacle_max_z_m))
            for sel, col in ((~band, (90, 90, 90)), (band, (80, 200, 255))):
                if sel.any():
                    px, py = to_px(xyz[sel])
                    ok = (px >= 0) & (px < size) & (py >= 0) & (py < size)
                    img[py[ok], px[ok]] = col
        if scan_base is not None and len(scan_base):
            sw = scan_base @ self.T_wb[:2, :2].T + self.T_wb[:2, 3]
            px, py = to_px(sw)
            ok = (px >= 0) & (px < size) & (py >= 0) & (py < size)
            img[py[ok], px[ok]] = (0, 0, 255)
        if self.trajectory:
            tr = np.array(self.trajectory)[:, :2]
            px, py = to_px(tr)
            for i in range(1, len(px)):
                cv2.line(img, (int(px[i - 1]), int(py[i - 1])), (int(px[i]), int(py[i])), (0, 255, 0), 1)
        p = self.T_wb[:2, 3].reshape(1, 2)
        px, py = to_px(p)
        yaw = math.atan2(self.T_wb[1, 0], self.T_wb[0, 0])
        cv2.circle(img, (int(px[0]), int(py[0])), 5, (0, 255, 255), -1)
        cv2.line(img, (int(px[0]), int(py[0])), (int(px[0] + 15 * math.cos(yaw)), int(py[0] - 15 * math.sin(yaw))),
                 (0, 255, 255), 2)
        cv2.putText(img, f"{span:.1f} m", (10, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1, cv2.LINE_AA)
        return img


# ---------------------------------------------------------------------- main
def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description="RobotMK1 fusion mapper (RGB-D + LiDAR + IMU, optional Hailo-8)")
    ap.add_argument("--config", default=None, help="robot.yaml (default config/robot.yaml)")
    ap.add_argument("--out", default=None, help="output directory (default maps/<timestamp>)")
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--record", dest="record", action="store_true", default=True, help="save the map on exit (default)")
    g.add_argument("--no-record", dest="record", action="store_false", help="run live but write nothing")
    ap.add_argument("--raw", action="store_true", help="also record raw sensor data to <out>/raw for --replay")
    ap.add_argument("--replay", metavar="DIR", default=None, help="replay a raw recording instead of live sensors")
    ap.add_argument("--realtime", action="store_true", help="pace the replay like the original recording")
    ap.add_argument("--hailo", action="store_true", help="enable Hailo-8 YOLOv8 dynamic-object masking + labels")
    ap.add_argument("--viz", action="store_true", help="show OpenCV windows (needs a display)")
    ap.add_argument("--6dof", dest="six_dof", action="store_true", help="hand-held mode: no planar constraint, no graph")
    ap.add_argument("--no-lidar", action="store_true")
    ap.add_argument("--no-mcu", action="store_true")
    ap.add_argument("--usb2", action="store_true", help="force USB2 mode for the OAK (long cable / hub problems)")
    ap.add_argument("--max-frames", type=int, default=0)
    ap.add_argument("--quiet", action="store_true")
    return ap


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    cfg = load_config(args.config)
    out = Path(args.out) if args.out else ROOT / "maps" / time.strftime("%Y%m%d_%H%M%S")
    log = (lambda *a, **k: None) if args.quiet else (lambda *a, **k: print(*a, **k, flush=True))

    # ---- sensors
    if args.replay:
        from common.recorder import RawReader
        hub = RawReader(args.replay, realtime=args.realtime)
        log(f"[replay] {hub.info()}")
    else:
        from common.sensors import LiveHub
        hub = LiveHub(cfg, use_lidar=not args.no_lidar, use_mcu=not args.no_mcu, usb2=args.usb2)
        log(f"[sensors] {json.dumps({k: v for k, v in hub.info().items() if k != 'K'})}")
        if getattr(hub.cam, "usb_speed", "SUPER") not in ("SUPER", "SUPER_PLUS", "unknown"):
            log("[sensors] WARNING: OAK is on USB2 - use a blue USB3 port and a USB3 cable for full frame rate")

    # ---- hailo
    detector = None
    if args.hailo:
        try:
            from mappers.fusion.hailo_detector import HailoYoloDetector
            hef = Path(cfg.hailo.hef_path)
            if not hef.is_absolute():
                hef = ROOT / hef
            detector = HailoYoloDetector(hef, float(cfg.hailo.score_threshold))
            log(f"[hailo] YOLOv8 loaded from {hef} (input {detector.in_w}x{detector.in_h})")
        except HailoUnavailable as e:
            log(f"[hailo] WARNING: {e}\n[hailo] continuing WITHOUT dynamic-object masking")
            detector = None

    force_3dof = bool(cfg.mapping.force_3dof) and not args.six_dof
    mapper = FusionMapper(cfg, hub.K, hub.width, hub.height, force_3dof=force_3dof, detector=detector, log=log)
    imu = ImuOrientation(tuple(cfg.camera.imu_rpy_deg), bias_samples=int(1.5 * int(cfg.camera.get("imu_rate_hz", 100))))
    log(f"[mapper] mode: {'planar 3-DoF (ground robot)' if force_3dof else '6-DoF hand-held'}, voxel {mapper.voxel} m, "
        f"record={'ON' if args.record else 'OFF'}, out={out if args.record else '-'}")

    recorder = None
    if args.raw and args.record and not args.replay:
        from common.recorder import RawRecorder
        recorder = RawRecorder(out / "raw", {**hub.info(), "config": cfg.to_dict()})
        log(f"[raw] recording raw sensor data to {out / 'raw'}")

    keys = None if args.viz else KeyListener()
    paused = False
    t_start = time.monotonic()
    t_status = 0.0
    t_view = 0.0
    n = 0
    imu_warned = False
    topdown = None
    try:
        while True:
            f = hub.get_frame(1.0)
            if f is None:
                if args.replay:
                    break
                log("[camera] no frame for 1 s ...")
                continue
            for s in hub.get_imu():
                imu.push(s.t, s.gyro, s.accel, s.quat)
            if not imu.ready and not imu_warned and time.monotonic() - t_start > 4.0:
                log(f"[imu] no orientation yet ({imu.samples} samples) - keep the robot still during the first "
                    f"2 s; continuing without IMU prior")
                imu_warned = True
            if imu.ready and imu.samples == imu.bias_samples:
                log(f"[imu] {imu.describe()}")
                if imu.tilt_deg > 8.0:
                    log("[imu] WARNING: configured camera.imu_rpy_deg looks wrong (gravity was "
                        f"{imu.tilt_deg:.0f} deg off vertical). Run:  python run.py calib-imu")
            scan = hub.latest_scan()
            if recorder is not None:
                recorder.add_frame(f)
                recorder.add_scan(scan)
            st = mapper.process(f, imu, scan, integrate=not paused)
            n += 1

            # keys
            pressed = []
            if args.viz:
                k = cv2.waitKey(1) & 0xFF
                if k != 255:
                    pressed.append(chr(k).lower())
            elif keys is not None:
                pressed = keys.pop()
            if "q" in pressed:
                log("[keys] q -> stopping")
                break
            if "p" in pressed:
                paused = not paused
                log(f"[keys] map integration {'PAUSED' if paused else 'RESUMED'}")

            now = time.monotonic()
            if now - t_status >= 1.0:
                t_status = now
                fps = n / max(1e-6, now - t_start)
                p = se2_from_T(mapper.T_wb)
                icp = f"{st['icp']:.2f}" if st.get("icp") is not None else " -- "
                log(f"{fps:5.1f} fps | x {p[0]:+6.2f} y {p[1]:+6.2f} yaw {math.degrees(p[2]):+6.1f} | "
                    f"VO {st.get('vo','?'):4s} {st.get('inliers',0):3d}/{st.get('matches',0):3d} | ICP {icp}"
                    f"{'*' if st.get('icp_used') else ' '} | KF {st.get('kf',0):3d} loops {st.get('loops',0):2d} | "
                    f"voxels {st.get('voxels',0):7d} | dets {st.get('dets',0)} "
                    f"{'| PAUSED' if paused else ''}{'| REC' if args.record else '| no-record'}")
            if args.viz:
                view = f.rgb.copy()
                if mapper.last_dets:
                    draw_detections(view, mapper.last_dets, mapper.dynamic)
                cv2.putText(view, f"VO {st.get('vo','')} {st.get('inliers',0)} inl | KF {st.get('kf',0)} | "
                            f"loops {st.get('loops',0)}{' | PAUSED' if paused else ''}", (8, 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1, cv2.LINE_AA)
                depth_vis = cv2.applyColorMap(np.clip(f.depth / 6000.0 * 255, 0, 255).astype(np.uint8), cv2.COLORMAP_JET)
                depth_vis[f.depth == 0] = 0
                if now - t_view > 0.5 or topdown is None:
                    t_view = now
                    topdown = mapper.render_topdown(f.rgb.shape[0] * 2, mapper.scan_to_base(scan))
                left = np.vstack([view, depth_vis])
                cv2.imshow("RobotMK1 fusion mapper  [p: pause  q: quit]", np.hstack([left, topdown]))
            if args.max_frames and n >= args.max_frames:
                break
    except KeyboardInterrupt:
        log("\n[main] Ctrl+C -> finishing")
    finally:
        if keys is not None:
            keys.stop()
        if recorder is not None:
            recorder.close()
        hub.close()
        if detector is not None:
            detector.close()
        if args.viz:
            cv2.destroyAllWindows()

    stats = mapper.finish()
    dur = time.monotonic() - t_start
    log(f"[done] {stats['frames']} frames in {dur:.0f} s, {stats['keyframes']} keyframes, {stats['loops']} loop closures, "
        f"VO lost {stats['vo_lost']} frames, {len(mapper.vmap)} voxels")
    if args.record:
        files = mapper.export(out, title=out.name)
        log("[export] written:\n  " + "\n  ".join(str(p) for p in files))
    else:
        log("[export] --no-record: nothing written")
    return 0


if __name__ == "__main__":
    sys.exit(main())
