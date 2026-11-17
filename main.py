#!/usr/bin/env python3
"""RobotMK1 console menu: choose a script from scripts/ and run it.

    python main.py

Type a script's number to run it, optionally followed by its options ("3 --distance 1.0");
"h 3" shows the options of script 3; Enter or q quits. A new script shows up by itself when
its file name starts with a number (06_something.py); the menu text is the first line of
its docstring.
"""
from __future__ import annotations

import ast
import shlex
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
SCRIPTS = ROOT / "scripts"


def find_scripts() -> dict[int, Path]:
    found = {}
    for path in sorted(SCRIPTS.glob("*.py")):
        number = path.name.split("_", 1)[0]
        if number.isdigit():
            found[int(number)] = path
    return found


def describe(path: Path) -> str:
    try:
        doc = ast.get_docstring(ast.parse(path.read_text(encoding="utf-8")))
    except (OSError, SyntaxError, UnicodeDecodeError):
        doc = None
    return doc.strip().splitlines()[0] if doc else path.name


def python() -> str:
    """The project's virtual environment, if there is one: the scripts then find its packages
    (DepthAI) even when it wasn't activated."""
    for candidate in (ROOT / ".venv" / "bin" / "python", ROOT / ".venv" / "Scripts" / "python.exe"):
        if candidate.exists():
            return str(candidate)
    return sys.executable


def run_script(path: Path, args: list[str]) -> int:
    process = subprocess.Popen([python(), str(path), *args], cwd=ROOT)
    while True:
        try:
            return process.wait()
        except KeyboardInterrupt:
            continue                    # the script got Ctrl+C too; let it stop the motors


def main() -> int:
    while True:
        scripts = find_scripts()
        print("\nRobotMK1 scripts")
        for number, path in scripts.items():
            print(f"  {number:2d}  {describe(path)}")
        print("\n  <number> [options]   run a script, e.g. 3 --distance 1.0"
              "\n  h <number>           show a script's options"
              "\n  q or Enter           quit")
        try:
            line = input("> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            return 0
        if line.lower() in ("", "q", "quit", "exit"):
            return 0
        words = shlex.split(line)
        show_help = words[0].lower() in ("h", "help", "?")
        if show_help:
            words = words[1:]
        if not words or not words[0].isdigit() or int(words[0]) not in scripts:
            print("Type the number of a script from the list.")
            continue
        path = scripts[int(words[0])]
        args = ["--help"] if show_help else words[1:]
        print(f"\n--- {path.name} {' '.join(args)}".rstrip() + "\n")
        code = run_script(path, args)
        print(f"\n--- {path.name} finished" + ("" if code == 0 else f" (exit code {code})"))


if __name__ == "__main__":
    sys.exit(main())
