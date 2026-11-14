#!/usr/bin/env python3
"""Convert / post-process a map.

    python run.py export --ply maps/room1/map.ply                     # -> occupancy.pgm/.yaml + preview.png
    python run.py export --ply maps/room1/map.ply --pcd --voxel 0.05  # also a PCD, downsampled
    python run.py export --db  maps/room1/rtabmap.db                  # RTAB-Map db -> cloud.ply, scan.ply, grid, preview

Options let you crop the cloud by height (--min-z / --max-z, e.g. remove the ceiling),
choose the grid resolution and the obstacle band used for the occupancy grid.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from common.config import load_config  # noqa: E402
from common.occupancy import grid_from_cloud, render_preview  # noqa: E402
from common.pointcloud import read_ply, voxel_downsample, write_pcd, write_ply  # noqa: E402


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="map export / conversion")
    ap.add_argument("--config", default=None)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--ply", help="input point cloud (map.ply / cloud.ply)")
    src.add_argument("--db", help="RTAB-Map database (needs docker + the robotmk1/rtabmap image)")
    ap.add_argument("--out", default=None, help="output directory (default: next to the input)")
    ap.add_argument("--voxel", type=float, default=0.0, help="voxel-downsample the cloud before export (m)")
    ap.add_argument("--min-z", type=float, default=None)
    ap.add_argument("--max-z", type=float, default=None)
    ap.add_argument("--pcd", action="store_true", help="also write a binary PCD")
    ap.add_argument("--resolution", type=float, default=None, help="occupancy grid cell size (default from config)")
    ap.add_argument("--ground-z", type=float, default=None)
    ap.add_argument("--obstacle-max-z", type=float, default=None)
    ap.add_argument("--poses", default=None, help="poses.txt (TUM) to draw the trajectory")
    a = ap.parse_args(argv)
    cfg = load_config(a.config)
    m = cfg.mapping
    res = a.resolution or float(m.grid_resolution_m)
    gz = float(m.ground_z_m) if a.ground_z is None else a.ground_z
    oz = float(m.obstacle_max_z_m) if a.obstacle_max_z is None else a.obstacle_max_z

    if a.db:
        from mappers.rtabmap.run_rtabmap import export_outputs, IMAGE, docker_ok
        db = Path(a.db).resolve()
        if not docker_ok():
            return 2
        out = Path(a.out).resolve() if a.out else db.parent
        if db.parent != out:
            import shutil
            out.mkdir(parents=True, exist_ok=True)
            shutil.copy2(db, out / "rtabmap.db")
        for f in export_outputs(IMAGE, out, cfg):
            print("  ", f)
        return 0

    ply = Path(a.ply)
    out = Path(a.out) if a.out else ply.parent
    out.mkdir(parents=True, exist_ok=True)
    d = read_ply(ply)
    xyz, rgb, lab = d["xyz"], d.get("rgb"), d.get("label")
    print(f"loaded {ply}: {len(xyz)} points, colors={'yes' if rgb is not None else 'no'}, labels={'yes' if lab is not None else 'no'}")
    keep = np.ones(len(xyz), bool)
    if a.min_z is not None:
        keep &= xyz[:, 2] >= a.min_z
    if a.max_z is not None:
        keep &= xyz[:, 2] <= a.max_z
    if not keep.all():
        xyz = xyz[keep]
        rgb = rgb[keep] if rgb is not None else None
        lab = lab[keep] if lab is not None else None
        print(f"height crop -> {len(xyz)} points")
    if a.voxel > 0:
        xyz, rgb, lab = voxel_downsample(xyz, a.voxel, rgb, lab)
        print(f"voxel {a.voxel} m -> {len(xyz)} points")
    stem = ply.stem
    if a.voxel > 0 or not keep.all():
        p = write_ply(out / f"{stem}_export.ply", xyz, rgb, lab)
        print("wrote", p)
    if a.pcd:
        p = write_pcd(out / f"{stem}.pcd", xyz, rgb)
        print("wrote", p)
    poses = None
    if a.poses:
        arr = np.loadtxt(a.poses, comments="#", ndmin=2)
        poses = arr[:, 1:4]
    grid = grid_from_cloud(xyz, res, gz, oz, poses_xy=poses)
    pgm, yml = grid.save(out / "occupancy")
    print("wrote", pgm, yml, grid.stats())
    png = render_preview(out / "preview.png", xyz, rgb, grid, poses, title=stem)
    print("wrote", png)
    return 0


if __name__ == "__main__":
    sys.exit(main())
