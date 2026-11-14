"""Point-cloud utilities without Open3D: back-projection, voxel-hash map, PLY / PCD I/O.

The VoxelMap keeps one running-average point per occupied voxel plus an
observation count and a semantic label, so the map stays bounded in memory
(~ 60 bytes / voxel) and noise from single observations can be removed with
``min_hits`` at export time.
"""
from __future__ import annotations

from pathlib import Path
from typing import Optional

import numpy as np

_OFF = 1 << 20          # voxel index offset so negative coordinates hash to positive keys
_BITS = 21


def backproject(depth_mm: np.ndarray, K: np.ndarray, stride: int = 2, mask: Optional[np.ndarray] = None,
                min_mm: int = 200, max_mm: int = 6000):
    """Depth image -> points in the camera optical frame (metres).

    Returns (points (N,3) float32, pixels (N,2) int32 as (u, v)).
    ``mask`` (HxW bool) marks pixels to *drop* (dynamic objects).
    """
    h, w = depth_mm.shape
    d = depth_mm[::stride, ::stride].astype(np.float32)
    vs, us = np.mgrid[0:h:stride, 0:w:stride]
    valid = (d >= min_mm) & (d <= max_mm)
    if mask is not None:
        valid &= ~mask[::stride, ::stride]
    d = d[valid] * 1e-3
    u = us[valid].astype(np.float32)
    v = vs[valid].astype(np.float32)
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    x = (u - cx) / fx * d
    y = (v - cy) / fy * d
    pts = np.stack([x, y, d], axis=1).astype(np.float32)
    pix = np.stack([us[valid], vs[valid]], axis=1).astype(np.int32)
    return pts, pix


def voxel_keys(points: np.ndarray, voxel: float) -> np.ndarray:
    ijk = np.floor(points / voxel).astype(np.int64) + _OFF
    return (ijk[:, 0] << (2 * _BITS)) | (ijk[:, 1] << _BITS) | ijk[:, 2]


def voxel_downsample(points: np.ndarray, voxel: float, colors: Optional[np.ndarray] = None,
                     labels: Optional[np.ndarray] = None):
    """Average points per voxel.  Returns (points, colors, labels) with None passed through."""
    if len(points) == 0:
        return points, colors, labels
    keys = voxel_keys(points, voxel)
    uk, inv, cnt = np.unique(keys, return_inverse=True, return_counts=True)
    n = len(uk)
    out = np.stack([np.bincount(inv, weights=points[:, i], minlength=n) for i in range(3)], axis=1) / cnt[:, None]
    oc = None
    if colors is not None:
        c = colors.astype(np.float64)
        oc = (np.stack([np.bincount(inv, weights=c[:, i], minlength=n) for i in range(3)], axis=1) / cnt[:, None])
        oc = np.clip(oc, 0, 255).astype(np.uint8)
    ol = None
    if labels is not None:
        ol = np.zeros(n, dtype=np.uint8)
        np.maximum.at(ol, inv, labels.astype(np.uint8))
    return out.astype(np.float32), oc, ol


class VoxelMap:
    """Running-average voxel map with hit counts and majority-vote labels."""

    def __init__(self, voxel: float = 0.03):
        self.voxel = float(voxel)
        self.keys = np.zeros(0, dtype=np.int64)
        self.sum_xyz = np.zeros((0, 3), dtype=np.float64)
        self.sum_rgb = np.zeros((0, 3), dtype=np.float32)
        self.count = np.zeros(0, dtype=np.int32)
        self.label = np.zeros(0, dtype=np.uint8)
        self.votes = np.zeros(0, dtype=np.int32)

    def __len__(self) -> int:
        return len(self.keys)

    def clear(self):
        self.__init__(self.voxel)

    def integrate(self, points: np.ndarray, colors: Optional[np.ndarray] = None,
                  labels: Optional[np.ndarray] = None):
        if len(points) == 0:
            return
        points = np.asarray(points, dtype=np.float64)
        keys = voxel_keys(points, self.voxel)
        uk, inv, cnt = np.unique(keys, return_inverse=True, return_counts=True)
        n = len(uk)
        bs_xyz = np.stack([np.bincount(inv, weights=points[:, i], minlength=n) for i in range(3)], axis=1)
        if colors is not None:
            c = np.asarray(colors, dtype=np.float64)
            bs_rgb = np.stack([np.bincount(inv, weights=c[:, i], minlength=n) for i in range(3)], axis=1)
        else:
            bs_rgb = np.full((n, 3), 128.0 * cnt[:, None], dtype=np.float64)
        b_lab = np.zeros(n, dtype=np.uint8)
        if labels is not None:
            np.maximum.at(b_lab, inv, np.asarray(labels, dtype=np.uint8))

        if len(self.keys):
            idx = np.searchsorted(self.keys, uk)
            idx_c = np.minimum(idx, len(self.keys) - 1)
            found = self.keys[idx_c] == uk
        else:
            idx_c = np.zeros(n, dtype=np.int64)
            found = np.zeros(n, dtype=bool)
        if found.any():
            fi = idx_c[found]
            self.sum_xyz[fi] += bs_xyz[found]
            self.sum_rgb[fi] += bs_rgb[found].astype(np.float32)
            self.count[fi] += cnt[found].astype(np.int32)
            # Boyer-Moore style majority vote on labels
            lb = b_lab[found]
            nb = cnt[found].astype(np.int32)
            same = (lb == self.label[fi]) | (lb == 0)
            self.votes[fi[same]] += nb[same]
            diff = ~same
            if diff.any():
                di = fi[diff]
                lose = self.votes[di] <= nb[diff]
                self.votes[di] = np.where(lose, nb[diff] - self.votes[di], self.votes[di] - nb[diff])
                self.label[di] = np.where(lose, lb[diff], self.label[di])
        new = ~found
        if new.any():
            self.keys = np.concatenate([self.keys, uk[new]])
            self.sum_xyz = np.concatenate([self.sum_xyz, bs_xyz[new]])
            self.sum_rgb = np.concatenate([self.sum_rgb, bs_rgb[new].astype(np.float32)])
            self.count = np.concatenate([self.count, cnt[new].astype(np.int32)])
            self.label = np.concatenate([self.label, b_lab[new]])
            self.votes = np.concatenate([self.votes, cnt[new].astype(np.int32)])
            order = np.argsort(self.keys, kind="stable")
            self.keys = self.keys[order]
            self.sum_xyz = self.sum_xyz[order]
            self.sum_rgb = self.sum_rgb[order]
            self.count = self.count[order]
            self.label = self.label[order]
            self.votes = self.votes[order]

    def extract(self, min_hits: int = 1):
        sel = self.count >= min_hits
        c = self.count[sel][:, None].astype(np.float64)
        xyz = (self.sum_xyz[sel] / c).astype(np.float32)
        rgb = np.clip(self.sum_rgb[sel] / c, 0, 255).astype(np.uint8)
        return xyz, rgb, self.label[sel].copy(), self.count[sel].copy()


# ------------------------------------------------------------------ PLY / PCD

def write_ply(path, xyz: np.ndarray, rgb: Optional[np.ndarray] = None, labels: Optional[np.ndarray] = None,
              normals: Optional[np.ndarray] = None, comment: str = "RobotMK1 robot_mapping"):
    path = Path(path)
    n = len(xyz)
    fields = [("x", "f4"), ("y", "f4"), ("z", "f4")]
    if normals is not None:
        fields += [("nx", "f4"), ("ny", "f4"), ("nz", "f4")]
    if rgb is not None:
        fields += [("red", "u1"), ("green", "u1"), ("blue", "u1")]
    if labels is not None:
        fields += [("label", "u1")]
    arr = np.empty(n, dtype=np.dtype([(f, np.dtype(t).newbyteorder("<")) for f, t in fields]))
    arr["x"], arr["y"], arr["z"] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    if normals is not None:
        arr["nx"], arr["ny"], arr["nz"] = normals[:, 0], normals[:, 1], normals[:, 2]
    if rgb is not None:
        arr["red"], arr["green"], arr["blue"] = rgb[:, 0], rgb[:, 1], rgb[:, 2]
    if labels is not None:
        arr["label"] = labels
    ply_t = {"f4": "float", "u1": "uchar"}
    header = ["ply", "format binary_little_endian 1.0", f"comment {comment}", f"element vertex {n}"]
    header += [f"property {ply_t[t]} {f}" for f, t in fields]
    header += ["end_header"]
    with open(path, "wb") as f:
        f.write(("\n".join(header) + "\n").encode("ascii"))
        f.write(arr.tobytes())
    return path


def read_ply(path):
    """Minimal PLY reader (ascii / binary_little_endian, vertex element only).

    Returns dict with 'xyz' (N,3) float32 and optional 'rgb' (N,3) uint8, 'label' (N,) uint8.
    """
    path = Path(path)
    with open(path, "rb") as f:
        header = []
        while True:
            line = f.readline().decode("ascii", errors="ignore").strip()
            header.append(line)
            if line == "end_header":
                break
        fmt = "ascii"
        n = 0
        props = []
        in_vertex = False
        for line in header:
            parts = line.split()
            if not parts:
                continue
            if parts[0] == "format":
                fmt = parts[1]
            elif parts[0] == "element":
                in_vertex = parts[1] == "vertex"
                if in_vertex:
                    n = int(parts[2])
            elif parts[0] == "property" and in_vertex and parts[1] != "list":
                props.append((parts[2], parts[1]))
        tmap = {"float": "f4", "float32": "f4", "double": "f8", "float64": "f8", "uchar": "u1", "uint8": "u1",
                "char": "i1", "int8": "i1", "short": "i2", "ushort": "u2", "int": "i4", "int32": "i4",
                "uint": "u4", "uint32": "u4"}
        if fmt == "ascii":
            data = np.loadtxt(f, max_rows=n, ndmin=2)
            cols = {name: data[:, i] for i, (name, _) in enumerate(props)}
        else:
            endian = "<" if "little" in fmt else ">"
            dt = np.dtype([(name, endian + tmap[t]) for name, t in props])
            arr = np.frombuffer(f.read(n * dt.itemsize), dtype=dt, count=n)
            cols = {name: arr[name] for name, _ in props}
    out = {"xyz": np.stack([cols["x"], cols["y"], cols["z"]], axis=1).astype(np.float32)}
    if all(k in cols for k in ("red", "green", "blue")):
        out["rgb"] = np.stack([cols["red"], cols["green"], cols["blue"]], axis=1).astype(np.uint8)
    if "label" in cols:
        out["label"] = cols["label"].astype(np.uint8)
    return out


def write_pcd(path, xyz: np.ndarray, rgb: Optional[np.ndarray] = None):
    """Binary PCD (x y z [rgb packed float]) readable by PCL / ROS."""
    path = Path(path)
    n = len(xyz)
    if rgb is not None:
        packed = (rgb[:, 0].astype(np.uint32) << 16) | (rgb[:, 1].astype(np.uint32) << 8) | rgb[:, 2].astype(np.uint32)
        arr = np.empty(n, dtype=np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgb", "<f4")]))
        arr["rgb"] = packed.view(np.float32)
        fields, size, typ, count = "x y z rgb", "4 4 4 4", "F F F F", "1 1 1 1"
    else:
        arr = np.empty(n, dtype=np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4")]))
        fields, size, typ, count = "x y z", "4 4 4", "F F F", "1 1 1"
    arr["x"], arr["y"], arr["z"] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    header = (f"# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS {fields}\nSIZE {size}\n"
              f"TYPE {typ}\nCOUNT {count}\nWIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {n}\nDATA binary\n")
    with open(path, "wb") as f:
        f.write(header.encode("ascii"))
        f.write(arr.tobytes())
    return path


def estimate_normals(xyz: np.ndarray, k: int = 12) -> np.ndarray:
    """PCA normals via scipy cKDTree (orientation: away from the centroid... flipped to +z-ish)."""
    from scipy.spatial import cKDTree
    if len(xyz) < k + 1:
        return np.tile(np.array([0, 0, 1.0], dtype=np.float32), (len(xyz), 1))
    tree = cKDTree(xyz)
    _, nn = tree.query(xyz, k=k + 1)
    nb = xyz[nn]                                   # (N, k+1, 3)
    nb = nb - nb.mean(axis=1, keepdims=True)
    cov = np.einsum("nki,nkj->nij", nb, nb)
    w, v = np.linalg.eigh(cov)
    normals = v[:, :, 0]
    flip = normals[:, 2] < 0
    normals[flip] *= -1
    return normals.astype(np.float32)
