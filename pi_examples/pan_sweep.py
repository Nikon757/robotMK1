#!/usr/bin/env python3
"""Find how far the camera pan servo can safely turn on your mount.

Copy this file next to niumo_board.py and hw_test.py on the Pi, then run:

    python3 pan_sweep.py                   # /dev/robot_mcu, else finds the board by USB ID
    python3 pan_sweep.py --step 2          # finer steps
    python3 pan_sweep.py --trim-only       # only find the straight-ahead centre

Needs the firmware where @S2 maps 0..180 to 500..2500 us with the pan open to
that full range. With older firmware the camera stops after about 18 degrees
on one side. The microsecond values it prints assume the firmware's pan centre
is --center-us (default 1500, i.e. no trim set yet in Hardware/Servo.c).

1. The camera goes to the middle (90); trim it until it points straight ahead
   (+ / - nudge by 1 degree, +10 / -20 etc. by that many).
2. It turns slowly, step by step, to one side. Watch the camera at each step,
   then press Enter to go further, or say that this is as far as it should go,
   or that it went too far: it touched something, the USB cable got tight, the
   servo buzzes, or the camera stopped moving although the angle changed (the
   servo is at its own end stop). Keys pressed while it moves are ignored.
3. The same for the other side.
At the end it prints the safe range and the line for Hardware/Servo.c.
Ctrl+C stops; the camera stays where it is.
"""
from __future__ import annotations

import argparse
import sys
import time

import serial

from hw_test import diagnose_silence, pick_port
from niumo_board import NiumoBoard

DEG_PER_S = 30           # slow enough to stop before a collision
SETTLE_S = 0.3           # let the servo arrive before asking
center_us = 1500         # the firmware's pan centre (--center-us)


def angle_to_us(angle: int) -> int:
    """The firmware's mapping (Servo_SetAngle in Hardware/Servo.c): 90 is the
    centre, 2000/180 us per degree, rounded half away from zero."""
    offset = (angle - 90) * 2000
    steps = (abs(offset) + 90) // 180
    return center_us + (steps if offset >= 0 else -steps)


def ask_after_move(prompt: str) -> str:
    """input() that ignores keys pressed while the camera was moving, so every
    step needs its own Enter after you have looked at the camera."""
    try:
        import termios
        if sys.stdin.isatty():
            termios.tcflush(sys.stdin, termios.TCIFLUSH)
    except (ImportError, OSError):
        pass                            # not a Linux terminal: nothing to flush
    return input(prompt)


def move_slowly(board: NiumoBoard, start: int, end: int) -> None:
    step = 1 if end > start else -1
    for angle in range(start + step, end + step, step):
        board.cam_pan(angle)
        time.sleep(1 / DEG_PER_S)
    time.sleep(SETTLE_S)


def trim_center(board: NiumoBoard) -> int:
    angle = 90
    board.cam_pan(angle)
    time.sleep(1.0)
    print("\nThe camera is at 90, the middle.")
    while True:
        answer = input(f"  Straight ahead? Enter = yes; + / - nudge by 1, +10 / -20 by that many (now {angle}): ").strip()
        if answer == "":
            return angle
        if answer in ("+", "-"):
            answer += "1"
        try:
            nudge = int(answer)
        except ValueError:
            continue
        target = max(0, min(180, angle + nudge if answer[0] in "+-" else nudge))   # "70" = go to 70
        move_slowly(board, angle, target)
        angle = target


def find_limit(board: NiumoBoard, center: int, direction: int, step: int) -> tuple[int, bool]:
    """Turn from the centre until the user stops it. Returns (limit, found_by_user)."""
    end = 0 if direction < 0 else 180
    angle = center
    print("  At each step: Enter = further, s = this is as far as it should go,\n"
          "  b = too far (touches, cable tight, servo buzzes, or the camera stopped moving).")
    while angle != end:
        nxt = max(0, min(180, angle + direction * step))
        reboots = board.reboots
        move_slowly(board, angle, nxt)
        if board.reboots != reboots:
            print("  The board restarted: the servo is probably pushing against something.\n"
                  f"  Taking {angle} as the limit on this side.")
            return angle, True
        answer = ask_after_move(f"  {nxt:3d} ({angle_to_us(nxt)} us)  [Enter / s / b]: ").strip().lower()[:1]
        if answer == "s":
            return nxt, True
        if answer == "b":
            move_slowly(board, nxt, angle)
            return angle, True
        angle = nxt
    print(f"  Reached {end}, the end of the servo's own range.")
    return end, False


def main() -> int:
    parser = argparse.ArgumentParser(description="Find the safe range of the camera pan servo.")
    parser.add_argument("port", nargs="?", default=None)
    parser.add_argument("--step", type=int, default=5, help="degrees per step (default 5)")
    parser.add_argument("--margin", type=int, default=3,
                        help="degrees to stay away from a limit you found (default 3)")
    parser.add_argument("--trim-only", action="store_true",
                        help="only find the straight-ahead centre, no sweep")
    parser.add_argument("--center-us", type=int, default=1500,
                        help="pan centre currently set in Hardware/Servo.c (default 1500)")
    args = parser.parse_args()
    step = max(1, min(30, args.step))
    global center_us
    center_us = args.center_us

    port = pick_port(args.port)
    if port is None:
        print("Can't find the board: no /dev/robot_mcu and no CH340 (USB 1a86:7523).")
        return 2
    try:
        board = NiumoBoard(port)
    except serial.SerialException as exc:
        print(f"Can't open {port}: {exc}")
        return 2

    with board:
        try:
            board.wait_for("encoders", timeout=2.0)
        except TimeoutError:
            pass
        else:
            if args.trim_only:
                return trim_only(board)
            return sweep(board, step, args.margin)
    diagnose_silence(port)               # the port is closed again here
    return 2


def trim_only(board: NiumoBoard) -> int:
    center = trim_center(board)
    print("\n========== result ==========")
    print(f"Straight ahead is angle {center} = {angle_to_us(center)} us.")
    print("Put it in Hardware/Servo.c as the middle number of the SERVO_PAN line "
          "(servo_limit[]), or send it to Claude:")
    print(f"\tcenter_us = {angle_to_us(center)}")
    return 0


def sweep(board: NiumoBoard, step: int, margin: int) -> int:
    center = trim_center(board)

    print(f"\nSIDE 1: from {center} down towards 0.")
    low, low_found = find_limit(board, center, -1, step)
    turned = input("  Which way did the camera turn on this side? (l = left, r = right): ").strip().lower()[:1]
    move_slowly(board, low, center)

    print(f"\nSIDE 2: from {center} up towards 180.")
    high, high_found = find_limit(board, center, +1, step)
    move_slowly(board, high, center)

    safe_low = min(low + margin, center) if low_found else low
    safe_high = max(high - margin, center) if high_found else high
    print("\n========== result ==========")
    print(f"Measured range: {low}..{high}   centre: {center}")
    if turned in ("l", "r"):
        below, above = ("left", "right") if turned == "l" else ("right", "left")
        print(f"Angles below the centre turn the camera {below}, above it {above}.")
    print(f"Safe range (with a {margin}-degree margin where you stopped it): "
          f"{safe_low}..{safe_high}  =  {angle_to_us(safe_low)}..{angle_to_us(safe_high)} us")
    print("\nLine for Hardware/Servo.c (servo_limit[], SERVO_PAN); or send these numbers to Claude:")
    print(f"\t{{{angle_to_us(safe_low):4d}, {angle_to_us(center)}, {angle_to_us(safe_high)}}},\t\t// SERVO_PAN,   PB13")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\nStopped.")
        sys.exit(130)
