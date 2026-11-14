"""Load and validate config/robot.yaml.

Usage:
    from common.config import load_config
    cfg = load_config()                 # default location
    cfg = load_config("/path/robot.yaml")
    cfg.lidar.fov_deg, cfg.camera.mount.x, ...

The returned object is a read-only attribute view over the YAML dict, so
typos in field names raise AttributeError instead of silently returning None.
"""
from __future__ import annotations

import os
from pathlib import Path
from typing import Any

import yaml

ROOT = Path(__file__).resolve().parent.parent
DEFAULT_CONFIG = ROOT / "config" / "robot.yaml"


class Section:
    """Attribute access wrapper for a nested dict."""

    def __init__(self, d: dict, path: str = ""):
        self._d = d
        self._path = path

    def __getattr__(self, name: str) -> Any:
        if name.startswith("_"):
            raise AttributeError(name)
        try:
            v = self._d[name]
        except KeyError:
            raise AttributeError(f"config field '{self._path}{name}' is missing in robot.yaml") from None
        if isinstance(v, dict):
            return Section(v, f"{self._path}{name}.")
        return v

    def get(self, name: str, default: Any = None) -> Any:
        v = self._d.get(name, default)
        return Section(v, f"{self._path}{name}.") if isinstance(v, dict) else v

    def to_dict(self) -> dict:
        return self._d

    def __repr__(self) -> str:
        return f"Section({self._path or '/'}: {list(self._d)})"


_REQUIRED = {
    "serial": ["lidar_port", "lidar_baud", "mcu_port", "mcu_baud"],
    "camera": ["width", "height", "fps", "mount", "imu_rpy_deg"],
    "lidar": ["mount", "forward_angle_deg", "fov_deg", "clockwise", "min_range_m", "max_range_m"],
    "mapping": ["voxel_m", "keyframe_dist_m", "keyframe_rot_deg", "force_3dof",
                "ground_z_m", "obstacle_max_z_m", "grid_resolution_m"],
    "hailo": ["hef_path", "score_threshold", "dynamic_classes"],
}


def load_config(path: str | os.PathLike | None = None) -> Section:
    p = Path(path) if path else Path(os.environ.get("ROBOT_CONFIG", DEFAULT_CONFIG))
    with open(p, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    for sec, keys in _REQUIRED.items():
        if sec not in data:
            raise ValueError(f"{p}: missing section '{sec}'")
        for k in keys:
            if k not in data[sec]:
                raise ValueError(f"{p}: missing field '{sec}.{k}'")
    for sensor in ("camera", "lidar"):
        m = data[sensor]["mount"]
        for k in ("x", "y", "z", "roll", "pitch", "yaw"):
            m.setdefault(k, 0.0)
    if not 0 < data["lidar"]["fov_deg"] <= 360:
        raise ValueError("lidar.fov_deg must be in (0, 360]")
    cfg = Section(data)
    cfg._d["_path"] = str(p)
    return cfg


def save_config_values(updates: dict, path: str | os.PathLike | None = None) -> Path:
    """Write a few dotted keys ('lidar.forward_angle_deg', 'lidar.mount.x') back into robot.yaml.

    Keeps the file's comments by doing textual replacement of the matching
    'key:' line inside the section; falls back to a full YAML dump for keys
    that do not exist yet.
    """
    p = Path(path) if path else Path(os.environ.get("ROBOT_CONFIG", DEFAULT_CONFIG))
    lines = p.read_text(encoding="utf-8").splitlines()
    missing = {}
    for dotted, value in updates.items():
        parts = dotted.split(".")
        sec, key = parts[0], parts[1]
        sub = parts[2] if len(parts) > 2 else None
        done = False
        in_sec = False
        for i, line in enumerate(lines):
            if line.strip() and not line.startswith(" "):
                in_sec = line.split(":")[0].strip() == sec
                continue
            if not in_sec:
                continue
            stripped = line.strip()
            if not stripped.startswith(key + ":"):
                continue
            indent = line[: len(line) - len(line.lstrip())]
            comment = ""
            if sub is None:
                if "#" in line:
                    comment = "  #" + line.split("#", 1)[1]
                lines[i] = f"{indent}{key}: {_fmt(value)}{comment}"
                done = True
                break
            if "{" in stripped and "}" in stripped:
                body = stripped[stripped.index("{") + 1: stripped.rindex("}")]
                items = {}
                for kv in body.split(","):
                    if ":" in kv:
                        k, v = kv.split(":", 1)
                        items[k.strip()] = v.strip()
                items[sub] = _fmt(value)
                tail = line[line.rindex("}") + 1:]
                if "#" in tail:
                    comment = "  #" + tail.split("#", 1)[1]
                lines[i] = f"{indent}{key}: {{" + ", ".join(f"{k}: {v}" for k, v in items.items()) + "}" + comment
                done = True
                break
        if not done:
            missing[dotted] = value
    p.write_text("\n".join(lines) + "\n", encoding="utf-8")
    if missing:
        data = yaml.safe_load(p.read_text(encoding="utf-8"))
        for dotted, value in missing.items():
            d = data
            parts = dotted.split(".")
            for part in parts[:-1]:
                d = d.setdefault(part, {})
            d[parts[-1]] = value
        p.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
    return p


def _fmt(v: Any) -> str:
    if isinstance(v, bool):
        return "true" if v else "false"
    if isinstance(v, float):
        s = f"{v:.4f}".rstrip("0").rstrip(".")
        return s if s not in ("", "-0") else "0"
    if isinstance(v, (list, tuple)):
        return "[" + ", ".join(_fmt(x) for x in v) + "]"
    return str(v)
