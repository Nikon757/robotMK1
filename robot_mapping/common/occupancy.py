"""2-D occupancy grid (Nav2 / map_server format) from a point cloud and/or LiDAR scans.

    grid = OccupancyGrid.from_bounds(xmin, ymin, xmax, ymax, resolution)
    grid.add_cloud(points_world, ground_z, max_z)         # camera or LiDAR points, world frame
    grid.raytrace(origin_xy, endpoints_xy)                 # LiDAR scan from a known pose
    grid.save("maps/room/occupancy")                      # -> occupancy.pgm + occupancy.yaml
    render_preview("maps/room/preview.png", xyz, rgb, grid, poses)

Cell semantics follow nav_msgs/OccupancyGrid: -1 unknown, 0 free, 100 occupied.
PGM pixels: 0 = occupied (black), 254 = free (white), 205 = unknown (grey);
row 0 of the image is the *top* (largest y), origin is the lower-left corner.
"""
from __future__ import annotations

from pathlib import Path
from typing import Optional

import numpy as np


class OccupancyGrid:
    def __init__(self, x0: float, y0: float, width: int, height: int, resolution: float):
        self.x0, self.y0 = float(x0), float(y0)
        self.w, self.h = int(width), int(height)
        self.res = float(resolution)
        self.hits = np.zeros((self.h, self.w), dtype=np.int32)     # obstacle observations
        self.free = np.zeros((self.h, self.w), dtype=np.int32)     # free observations

    @classmethod
    def from_bounds(cls, xmin, ymin, xmax, ymax, resolution, margin: float = 0.5):
        x0, y0 = xmin - margin, ymin - margin
        w = int(np.ceil((xmax + margin - x0) / resolution)) + 1
        h = int(np.ceil((ymax + margin - y0) / resolution)) + 1
        return cls(x0, y0, w, h, resolution)

    @classmethod
    def from_points(cls, xy: np.ndarray, resolution: float, margin: float = 0.5):
        if len(xy) == 0:
            return cls(-1, -1, int(2 / resolution), int(2 / resolution), resolution)
        return cls.from_bounds(xy[:, 0].min(), xy[:, 1].min(), xy[:, 0].max(), xy[:, 1].max(), resolution, margin)

    # ------------------------------------------------------------ indexing
    def to_cell(self, xy: np.ndarray):
        xy = np.asarray(xy, dtype=float).reshape(-1, 2)
        i = np.floor((xy[:, 0] - self.x0) / self.res).astype(np.int64)
        j = np.floor((xy[:, 1] - self.y0) / self.res).astype(np.int64)
        ok = (i >= 0) & (i < self.w) & (j >= 0) & (j < self.h)
        return i, j, ok

    def _acc(self, arr: np.ndarray, i, j, ok):
        if ok.any():
            np.add.at(arr, (j[ok], i[ok]), 1)

    # ------------------------------------------------------------ inputs
    def add_cloud(self, pts: np.ndarray, ground_z: float, max_z: float, free_weight: int = 1):
        """3-D points in the world frame: floor points -> free, in-band points -> occupied."""
        pts = np.asarray(pts)
        if len(pts) == 0:
            return
        z = pts[:, 2]
        obs = (z > ground_z) & (z < max_z)
        flo = z <= ground_z
        i, j, ok = self.to_cell(pts[obs, :2])
        self._acc(self.hits, i, j, ok)
        i, j, ok = self.to_cell(pts[flo, :2])
        for _ in range(free_weight):
            self._acc(self.free, i, j, ok)

    def raytrace(self, origin_xy, end_xy: np.ndarray, hit: Optional[np.ndarray] = None):
        """Mark cells along each ray free and the end cell occupied (if ``hit``)."""
        end_xy = np.asarray(end_xy, dtype=float).reshape(-1, 2)
        if len(end_xy) == 0:
            return
        o = np.asarray(origin_xy, dtype=float).reshape(2)
        d = end_xy - o
        L = np.linalg.norm(d, axis=1)
        n_steps = np.maximum(1, np.ceil(L / (self.res * 0.7)).astype(int))
        maxn = int(n_steps.max())
        s = (np.arange(maxn) + 0.5) / n_steps[:, None]                 # (N, maxn) fractions
        valid = s < 1.0                                                # exclude the end cell
        # stop one cell short of the endpoint so the obstacle cell is not marked free
        s = s * (1 - 0.5 * self.res / np.maximum(L, 1e-6))[:, None]
        px = o[0] + d[:, 0:1] * s
        py = o[1] + d[:, 1:2] * s
        xy = np.stack([px[valid], py[valid]], axis=1)
        i, j, ok = self.to_cell(xy)
        self._acc(self.free, i, j, ok)
        if hit is None:
            hit = np.ones(len(end_xy), dtype=bool)
        i, j, ok = self.to_cell(end_xy[hit])
        self._acc(self.hits, i, j, ok)

    # ------------------------------------------------------------ output
    def cells(self, min_hits: int = 1, occ_ratio: float = 0.2) -> np.ndarray:
        """int8 array (h, w): -1 unknown, 0 free, 100 occupied."""
        out = np.full((self.h, self.w), -1, dtype=np.int8)
        seen = (self.hits + self.free) > 0
        occ = (self.hits >= min_hits) & (self.hits >= occ_ratio * (self.hits + self.free))
        out[seen] = 0
        out[occ] = 100
        return out

    def save(self, stem, min_hits: int = 1, occ_ratio: float = 0.2):
        """Write <stem>.pgm and <stem>.yaml (Nav2 map_server)."""
        stem = Path(stem)
        c = self.cells(min_hits, occ_ratio)
        img = np.full(c.shape, 205, dtype=np.uint8)
        img[c == 0] = 254
        img[c == 100] = 0
        img = img[::-1]                                   # row 0 = top = max y
        with open(stem.with_suffix(".pgm"), "wb") as f:
            f.write(f"P5\n{self.w} {self.h}\n255\n".encode("ascii"))
            f.write(img.tobytes())
        yaml_txt = (f"image: {stem.with_suffix('.pgm').name}\nmode: trinary\nresolution: {self.res}\n"
                    f"origin: [{self.x0:.4f}, {self.y0:.4f}, 0.0]\nnegate: 0\n"
                    f"occupied_thresh: 0.65\nfree_thresh: 0.196\n")
        stem.with_suffix(".yaml").write_text(yaml_txt, encoding="utf-8")
        return stem.with_suffix(".pgm"), stem.with_suffix(".yaml")

    def stats(self) -> dict:
        c = self.cells()
        return {"width": self.w, "height": self.h, "resolution": self.res,
                "free_cells": int((c == 0).sum()), "occupied_cells": int((c == 100).sum()),
                "unknown_cells": int((c == -1).sum())}


def grid_from_cloud(xyz: np.ndarray, resolution: float, ground_z: float, max_z: float,
                    poses_xy: Optional[np.ndarray] = None) -> OccupancyGrid:
    """Convenience: a grid covering the cloud, filled from cloud heights only (no ray tracing)."""
    pts = np.asarray(xyz)
    if poses_xy is not None and len(poses_xy):
        allxy = np.concatenate([pts[:, :2], np.asarray(poses_xy)[:, :2]])
    else:
        allxy = pts[:, :2]
    g = OccupancyGrid.from_points(allxy, resolution)
    g.add_cloud(pts, ground_z, max_z)
    return g


def render_preview(png_path, xyz: np.ndarray, rgb: Optional[np.ndarray], grid: Optional[OccupancyGrid],
                   poses: Optional[np.ndarray] = None, title: str = "", max_points: int = 300000):
    """Two-panel PNG: top-down cloud (left) and occupancy grid with trajectory (right)."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    xyz = np.asarray(xyz)
    if len(xyz) > max_points:
        sel = np.random.default_rng(0).choice(len(xyz), max_points, replace=False)
        xyz = xyz[sel]
        rgb = rgb[sel] if rgb is not None else None
    fig, axes = plt.subplots(1, 2 if grid is not None else 1, figsize=(16, 8), squeeze=False)
    ax = axes[0, 0]
    if len(xyz):
        order = np.argsort(xyz[:, 2])          # draw high points last
        col = (rgb[order] / 255.0) if rgb is not None else xyz[order, 2]
        ax.scatter(xyz[order, 0], xyz[order, 1], c=col, s=0.5, linewidths=0, cmap=None if rgb is not None else "viridis")
    if poses is not None and len(poses):
        poses = np.asarray(poses)
        ax.plot(poses[:, 0], poses[:, 1], "r-", lw=1.2)
        ax.plot(poses[0, 0], poses[0, 1], "go", ms=6)
    ax.set_aspect("equal")
    ax.set_title(f"{title} point cloud top view ({len(xyz)} pts)".strip())
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.grid(True, alpha=0.3)
    if grid is not None:
        ax2 = axes[0, 1]
        c = grid.cells()
        img = np.full(c.shape + (3,), 0.8)
        img[c == 0] = 1.0
        img[c == 100] = 0.0
        extent = [grid.x0, grid.x0 + grid.w * grid.res, grid.y0, grid.y0 + grid.h * grid.res]
        ax2.imshow(img, origin="lower", extent=extent, interpolation="nearest")
        if poses is not None and len(poses):
            ax2.plot(poses[:, 0], poses[:, 1], "r-", lw=1.2)
        ax2.set_title(f"occupancy grid {grid.w}x{grid.h} @ {grid.res} m")
        ax2.set_xlabel("x [m]"); ax2.set_ylabel("y [m]")
    fig.tight_layout()
    fig.savefig(png_path, dpi=110)
    plt.close(fig)
    return png_path
