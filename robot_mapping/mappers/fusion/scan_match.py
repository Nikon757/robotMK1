"""2-D scan matching (point-to-point ICP with scipy's cKDTree) against a local map
built from the scans of recent keyframes.

    lm = LocalScanMap(voxel=0.05, window=40)
    lm.add(kf_idx, scan_xy_world)             # after each keyframe
    res = icp_2d(scan_xy_base, lm, init=(x, y, yaw))
    res.pose, res.fitness, res.rmse, res.ok
"""
from __future__ import annotations

import math
from collections import OrderedDict
from dataclasses import dataclass
from typing import Optional

import numpy as np
from scipy.spatial import cKDTree


def _tf(xy: np.ndarray, pose) -> np.ndarray:
    c, s = math.cos(pose[2]), math.sin(pose[2])
    R = np.array([[c, -s], [s, c]])
    return xy @ R.T + np.asarray(pose[:2])


def _voxel(xy: np.ndarray, voxel: float) -> np.ndarray:
    if len(xy) == 0:
        return xy
    ij = np.floor(xy / voxel).astype(np.int64)
    key = (ij[:, 0] + (1 << 20)) * (1 << 21) + (ij[:, 1] + (1 << 20))
    _, first = np.unique(key, return_index=True)
    return xy[first]


class LocalScanMap:
    """World-frame 2-D points of the last ``window`` keyframes (voxel-filtered)."""

    def __init__(self, voxel: float = 0.05, window: int = 40):
        self.voxel = voxel
        self.window = window
        self._scans: "OrderedDict[int, np.ndarray]" = OrderedDict()
        self.xy = np.zeros((0, 2))
        self.tree: Optional[cKDTree] = None

    def add(self, kf_idx: int, xy_world: np.ndarray):
        self._scans[kf_idx] = _voxel(np.asarray(xy_world, dtype=float), self.voxel)
        while len(self._scans) > self.window:
            self._scans.popitem(last=False)
        self.rebuild()

    def replace_all(self, items):
        """items: iterable of (kf_idx, xy_world) after a graph optimisation."""
        self._scans = OrderedDict()
        for k, xy in list(items)[-self.window:]:
            self._scans[k] = _voxel(np.asarray(xy, dtype=float), self.voxel)
        self.rebuild()

    def rebuild(self):
        if self._scans:
            self.xy = _voxel(np.concatenate(list(self._scans.values())), self.voxel)
            self.tree = cKDTree(self.xy) if len(self.xy) >= 10 else None
        else:
            self.xy = np.zeros((0, 2))
            self.tree = None

    def __len__(self):
        return len(self.xy)


@dataclass
class IcpResult:
    ok: bool
    pose: np.ndarray            # (x, y, yaw) of the scan frame in the map frame
    fitness: float              # fraction of source points with a correspondence closer than max_corr
    rmse: float
    iterations: int


def icp_2d(src_xy: np.ndarray, target: LocalScanMap, init, max_corr: float = 0.5, iters: int = 30,
           min_fitness: float = 0.5, tol: float = 1e-4, max_points: int = 1500) -> IcpResult:
    pose = np.array(init, dtype=float)
    if target.tree is None or len(src_xy) < 20:
        return IcpResult(False, pose, 0.0, float("inf"), 0)
    src = np.asarray(src_xy, dtype=float)
    if len(src) > max_points:
        src = src[np.linspace(0, len(src) - 1, max_points).astype(int)]
    corr = max_corr
    it = 0
    fit, rmse = 0.0, float("inf")
    for it in range(1, iters + 1):
        p = _tf(src, pose)
        d, idx = target.tree.query(p, k=1, distance_upper_bound=corr)
        ok = np.isfinite(d)
        n_ok = int(ok.sum())
        fit = n_ok / len(src)
        if n_ok < 10:
            break
        a = src[ok]
        b = target.xy[idx[ok]]
        rmse = float(np.sqrt(np.mean(d[ok] ** 2)))
        # rigid 2-D alignment a -> b (Kabsch)
        ca, cb = a.mean(axis=0), b.mean(axis=0)
        H = (a - ca).T @ (b - cb)
        U, _, Vt = np.linalg.svd(H)
        dsign = np.sign(np.linalg.det(Vt.T @ U.T))
        R = Vt.T @ np.diag([1, dsign]) @ U.T
        t = cb - R @ ca
        yaw = math.atan2(R[1, 0], R[0, 0])
        new_pose = np.array([t[0], t[1], yaw])
        delta = np.abs(new_pose - pose)
        delta[2] = abs((delta[2] + math.pi) % (2 * math.pi) - math.pi)
        pose = new_pose
        # tighten the correspondence radius as we converge
        corr = max(0.08, min(corr, 3.0 * rmse + 0.05))
        if delta[0] < tol and delta[1] < tol and delta[2] < tol:
            break
    return IcpResult(fit >= min_fitness and np.isfinite(rmse), pose, fit, rmse, it)


def icp_pair(src_xy: np.ndarray, dst_xy: np.ndarray, init, voxel: float = 0.05, **kw) -> IcpResult:
    """ICP between two single scans (used to refine loop closures)."""
    lm = LocalScanMap(voxel=voxel, window=1)
    lm.add(0, dst_xy)
    return icp_2d(src_xy, lm, init, **kw)
