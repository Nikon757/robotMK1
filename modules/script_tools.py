"""Small helpers shared by the scripts in scripts/."""
from __future__ import annotations

from typing import Callable

from .engines import MotionError


def confirm_start(message: str, skip: bool = False) -> None:
    """Say what is about to happen and wait for Enter (skipped with --yes)."""
    print(message)
    if not skip:
        input("Press Enter to start (Ctrl+C stops the robot at any time) ... ")


def run(main: Callable[[], int | None]) -> int:
    """Run a script's main(): Ctrl+C and expected errors end it with a message, not a traceback.
    The motors are always stopped first (Robot and the board client stop them on exit)."""
    try:
        return main() or 0
    except KeyboardInterrupt:
        print("\nStopped with Ctrl+C. The motors are off.")
        return 130
    except (MotionError, RuntimeError) as exc:
        print(f"\nStopped: {exc}")
        return 1
