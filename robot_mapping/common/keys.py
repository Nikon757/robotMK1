"""Non-blocking single-key reader for the terminal (p = pause, q = quit hot-keys)."""
from __future__ import annotations

import sys
import threading
from typing import List


class KeyListener:
    """Single-key reader on POSIX terminals (falls back to line input elsewhere)."""

    def __init__(self):
        self.keys: List[str] = []
        self._stop = False
        self._th = threading.Thread(target=self._run, daemon=True)
        self._th.start()

    def _run(self):
        try:
            import select
            import termios
            import tty
            fd = sys.stdin.fileno()
            old = termios.tcgetattr(fd)
            try:
                tty.setcbreak(fd)
                while not self._stop:
                    r, _, _ = select.select([sys.stdin], [], [], 0.2)
                    if r:
                        ch = sys.stdin.read(1)
                        if ch:
                            self.keys.append(ch.lower())
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old)
        except Exception:
            while not self._stop:
                try:
                    line = sys.stdin.readline()
                except Exception:
                    return
                if not line:
                    return
                self.keys.append(line.strip()[:1].lower())

    def pop(self) -> List[str]:
        k, self.keys = self.keys, []
        return k

    def stop(self):
        self._stop = True
