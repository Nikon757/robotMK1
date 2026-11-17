"""Read and update config/robot.yaml.

The path can be overridden with the ROBOTMK1_CONFIG environment variable (useful for tests
or a second robot).
"""
from __future__ import annotations

import os
import re
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = REPO_ROOT / "config" / "robot.yaml"


def config_path() -> Path:
    return Path(os.environ.get("ROBOTMK1_CONFIG", DEFAULT_CONFIG))


def load_config() -> dict:
    with open(config_path(), encoding="utf-8") as f:
        return yaml.safe_load(f)


def save_value(section: str, key: str, value) -> None:
    """Change one value in robot.yaml and keep everything else, comments included."""
    path = config_path()
    lines = path.read_text(encoding="utf-8").splitlines(keepends=True)
    pattern = re.compile(rf"^(\s+{re.escape(key)}:)[ \t]*([^#\r\n]*?)([ \t]*#[^\r\n]*)?(\r?\n)?$")
    in_section = False
    for i, line in enumerate(lines):
        if line.strip() and not line[0].isspace():          # a top-level "section:" line
            in_section = line.split(":", 1)[0].strip() == section
            continue
        match = pattern.match(line)
        if not (in_section and match):
            continue
        new = f"{value:.1f}" if isinstance(value, float) else str(value)
        ending = match.group(4) or ""
        comment = (match.group(3) or "").lstrip(" \t")
        if comment:                                         # keep the comment in its column
            width = len(match.group(2)) + len(match.group(3)) - len(comment)
            lines[i] = f"{match.group(1)} {new}{' ' * max(1, width - len(new))}{comment}{ending}"
        else:
            lines[i] = f"{match.group(1)} {new}{ending}"
        path.write_text("".join(lines), encoding="utf-8")
        return
    raise KeyError(f"{section}.{key} not found in {path}")
