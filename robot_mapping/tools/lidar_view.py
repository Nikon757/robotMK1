#!/usr/bin/env python3
"""Live LiDAR viewer and forward-angle finder.

    python run.py lidar-view                 # live polar plot (matplotlib window)
    python run.py lidar-view --find-forward  # measure lidar.forward_angle_deg with a box in front
    python run.py lidar-view --text          # no display: ASCII summary per scan

--find-forward procedure
    1. Put a box / book 30-60 cm straight in front of the robot (centred on the chassis axis).
       Nothing else should be closer than ~1 m.
    2. Run this tool.  It averages ~20 revolutions, finds the nearest cluster of points and
       prints its raw sensor angle.  That angle is the value for  lidar.forward_angle_deg.
    3. With --apply the value is written to config/robot.yaml.

In the live plot the forward direction (after the configured forward_angle_deg) is at the
top of the polar plot; points outside the configured FOV are drawn in grey.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from common.config import load_config, save_config_values  # noqa: E402
from common.ldlidar import LDLidar, scan_to_points  # noqa: E402
from common.transforms import wrap_deg  # noqa: E402


def find_forward(scans: list, min_r: float = 0.15, max_r: float = 1.0) -> dict:
    """Nearest cluster of points across several scans -> its median raw angle."""
    angs, rngs = [], []
    for s in scans:
        ok = (s.ranges_m >= min_r) & (s.ranges_m <= max_r)
        angs.append(s.angles_deg[ok])
        rngs.append(s.ranges_m[ok])
    a = np.concatenate(angs)
    r = np.concatenate(rngs)
    if len(a) < 20:
        raise RuntimeError("not enough close points - put a box 30-60 cm in front of the LiDAR")
    # cluster around the nearest points: take everything within 6 cm of the 5th percentile range
    r5 = np.percentile(r, 5)
    sel = r <= r5 + 0.06
    a_sel = a[sel]
    # circular median: unwrap around the angle with most support
    hist, edges = np.histogram(a_sel, bins=72, range=(0, 360))
    centre = edges[np.argmax(hist)] + 2.5
    rel = wrap_deg(a_sel - centre)
    keep = np.abs(rel) < 20
    angle = (centre + np.median(rel[keep])) % 360.0
    spread = float(np.std(rel[keep]))
    return {"forward_angle_deg": float(angle), "range_m": float(np.median(r[sel])), "n_points": int(keep.sum()),
            "spread_deg": spread}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="LiDAR live view / forward angle finder")
    ap.add_argument("--config", default=None)
    ap.add_argument("--port", default=None)
    ap.add_argument("--baud", type=int, default=None)
    ap.add_argument("--find-forward", action="store_true")
    ap.add_argument("--apply", action="store_true", help="write the measured forward angle into robot.yaml")
    ap.add_argument("--text", action="store_true", help="no matplotlib window")
    ap.add_argument("--scans", type=int, default=20, help="scans to average for --find-forward")
    a = ap.parse_args(argv)
    cfg = load_config(a.config)
    L = cfg.lidar
    ld = LDLidar(a.port or str(cfg.serial.lidar_port), a.baud or int(cfg.serial.lidar_baud))
    print(f"LiDAR on {ld.port} @ {ld.baud}; configured forward_angle_deg={L.forward_angle_deg} fov={L.fov_deg} "
          f"clockwise={L.clockwise}")

    if a.find_forward:
        print(f"collecting {a.scans} scans - keep the box straight ahead and do not move the robot ...")
        scans = []
        while len(scans) < a.scans:
            s = ld.wait_scan(1.0)
            if s is not None:
                scans.append(s)
        ld.close()
        res = find_forward(scans)
        print(f"\nnearest object: raw angle {res['forward_angle_deg']:.1f} deg, range {res['range_m']:.2f} m, "
              f"{res['n_points']} points, spread {res['spread_deg']:.1f} deg")
        print(f"-> lidar.forward_angle_deg: {res['forward_angle_deg']:.1f}")
        if res["spread_deg"] > 8:
            print("   (large spread: the object may be too wide / too close; use a narrower box farther away)")
        if a.apply:
            p = save_config_values({"lidar.forward_angle_deg": round(res["forward_angle_deg"], 1)}, a.config)
            print(f"   written to {p}")
        else:
            print("   re-run with --apply to write it into config/robot.yaml")
        return 0

    if a.text:
        try:
            while True:
                s = ld.wait_scan(1.0)
                if s is None:
                    print("timeout")
                    continue
                xy, b, r = scan_to_points(s, L.forward_angle_deg, L.fov_deg, L.clockwise, L.min_range_m, L.max_range_m)
                sectors = ""
                for lo in range(-90, 90, 15):
                    m = (np.degrees(b) >= lo) & (np.degrees(b) < lo + 15)
                    sectors += f" {np.median(r[m]):4.2f}" if m.any() else "   -- "
                print(f"scan {ld.scans:5d} {len(s):4d} pts, FOV {len(xy):4d} | left..right (m):{sectors}")
        except KeyboardInterrupt:
            pass
        finally:
            ld.close()
        return 0

    import matplotlib.pyplot as plt
    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, projection="polar")
    ax.set_theta_zero_location("N")       # forward = up
    ax.set_theta_direction(1)             # CCW positive (REP-103)
    sc_in = ax.scatter([], [], s=3, c="tab:blue", label="inside FOV")
    sc_out = ax.scatter([], [], s=2, c="lightgrey", label="masked")
    ax.set_rmax(float(L.max_range_m))
    ax.legend(loc="lower right")
    title = ax.set_title("")
    plt.ion()
    plt.show()
    try:
        while plt.fignum_exists(fig.number):
            s = ld.wait_scan(1.0)
            if s is None:
                plt.pause(0.05)
                continue
            raw = s.angles_deg
            bearing = np.radians(wrap_deg((L.forward_angle_deg - raw) if L.clockwise else (raw - L.forward_angle_deg)))
            r = s.ranges_m
            valid = (r >= L.min_range_m) & (r <= L.max_range_m)
            inside = valid & (np.abs(np.degrees(bearing)) <= L.fov_deg / 2)
            sc_in.set_offsets(np.column_stack([bearing[inside], r[inside]]))
            sc_out.set_offsets(np.column_stack([bearing[valid & ~inside], r[valid & ~inside]]))
            title.set_text(f"scan {ld.scans}  {len(s)} pts  {int(inside.sum())} in FOV  {s.speed_dps:.0f} deg/s")
            fig.canvas.draw_idle()
            plt.pause(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        ld.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
