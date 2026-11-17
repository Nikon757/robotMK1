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


def save_value(section: str, key: str, value, comment: str = "") -> None:
    """Change one value in robot.yaml and keep everything else, comments included. A key
    that is missing from the section (an older robot.yaml) is added at its end."""
    path = config_path()
    lines = path.read_text(encoding="utf-8").splitlines(keepends=True)
    pattern = re.compile(rf"^(\s+{re.escape(key)}:)[ \t]*([^#\r\n]*?)([ \t]*#[^\r\n]*)?(\r?\n)?$")
    new = repr(value) if isinstance(value, float) else str(value)
    in_section = False
    section_end = None                                      # index after the section's last line
    for i, line in enumerate(lines):
        if line.strip() and not line[0].isspace():          # a top-level "section:" line
            in_section = line.split(":", 1)[0].strip() == section
            continue
        if in_section and line.strip():
            section_end = i + 1
        match = pattern.match(line)
        if not (in_section and match):
            continue
        ending = match.group(4) or ""
        old_comment = (match.group(3) or "").lstrip(" \t")
        if old_comment:                                     # keep the comment in its column
            width = len(match.group(2)) + len(match.group(3)) - len(old_comment)
            lines[i] = f"{match.group(1)} {new}{' ' * max(1, width - len(new))}{old_comment}{ending}"
        else:
            lines[i] = f"{match.group(1)} {new}{ending}"
        path.write_text("".join(lines), encoding="utf-8")
        return
    if section_end is None:
        raise KeyError(f"section {section!r} not found in {path}")
    entry = f"  {key}: {new}"
    if comment:
        entry = f"{entry.ljust(30)}# {comment}"
    newline = "\r\n" if lines[section_end - 1].endswith("\r\n") else "\n"
    if not lines[section_end - 1].endswith(("\n", "\r\n")):
        lines[section_end - 1] += newline
    lines.insert(section_end, entry + newline)
    path.write_text("".join(lines), encoding="utf-8")
