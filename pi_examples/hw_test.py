#!/usr/bin/env python3
"""Check that the servos, motors and encoders work. No IMU needed.

Copy this file next to niumo_board.py on the Pi, then run:

    python3 hw_test.py                     # /dev/robot_mcu, else finds the board by USB ID
    python3 hw_test.py /dev/ttyUSB0        # or name the port
    python3 hw_test.py --power 450         # if the motors don't start at 350
    python3 hw_test.py --tilt              # also test a camera-tilt servo on PB15

It moves one thing at a time and asks what you saw; the encoder counts are
checked automatically. For the motor part the rear wheels must be OFF THE
GROUND. Ctrl+C stops everything (the board also stops the motors by itself
0.5 s after the last command).
"""
from __future__ import annotations

import argparse
import os
import sys
import time

import serial
from serial.tools import list_ports

from niumo_board import NiumoBoard

MIN_COUNTS = 20          # fewer counts during a motor run: the encoder isn't counting
SETTLE_S = 0.3           # after a run: let the wheel stop and the last #T arrive
BOARD_USB_ID = (0x1A86, 0x7523)     # the board's CH340; the LiDAR's CP2102 is 10c4:ea60
DEFAULT_PORT = "/dev/robot_mcu"     # made by the udev rule in PI_GUIDE.md


def pick_port(requested: str | None) -> str | None:
    """The board's serial port. ttyUSB numbers swap between the board and the
    LiDAR when they reconnect, so a port that is some other USB adapter is
    replaced by the board's CH340, found by its USB ID."""
    ports = {p.device: p for p in list_ports.comports()}
    board = next((dev for dev, p in ports.items() if (p.vid, p.pid) == BOARD_USB_ID), None)
    if requested is None:
        return DEFAULT_PORT if os.path.exists(DEFAULT_PORT) else board
    other = ports.get(os.path.realpath(requested))
    if other is not None and (other.vid, other.pid) != BOARD_USB_ID:
        usb = f"USB {other.vid:04x}:{other.pid:04x}" if other.vid is not None else "not USB"
        print(f"{requested} is not the board: it is {other.description} ({usb}), "
              "probably the LiDAR.")
        if board:
            print(f"The board's CH340 is on {board}; using that.\n")
        return board
    return requested


class Results:
    def __init__(self) -> None:
        self.rows: list[tuple[str, bool, str]] = []

    def add(self, name: str, ok: bool, note: str = "") -> None:
        self.rows.append((name, ok, note))
        print(f"  -> {'OK' if ok else 'FAIL'}: {name}" + (f" - {note}" if note else ""))

    def summary(self) -> bool:
        print("\n========== summary ==========")
        for name, ok, note in self.rows:
            print(f"{'OK  ' if ok else 'FAIL'}  {name:<30} {note}")
        failed = sum(not ok for _, ok, _ in self.rows)
        print(f"\n{len(self.rows) - failed} passed, {failed} failed")
        return failed == 0


def ask(question: str, choices: dict[str, str]) -> str:
    """Ask until the answer is one of the keys of `choices`."""
    hint = ", ".join(f"{key}={meaning}" for key, meaning in choices.items())
    while True:
        answer = input(f"{question} ({hint}): ").strip().lower()[:1]
        if answer in choices:
            return answer


def diagnose_silence(port: str, seconds: float = 3.0) -> None:
    """Listen to the raw port and say why no encoder data arrived."""
    print(f"No encoder data from the board. Listening to {port} for {seconds:g} s ...")
    with serial.Serial(port, 115200, timeout=0.1) as ser:
        raw = b""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            raw += ser.read(256)
    text = raw.decode("ascii", "backslashreplace")       # garbage shows as \xNN, prints anywhere
    lines = [l.strip() for l in text.splitlines() if l.strip()]
    boots = [l for l in lines if l.startswith("#BOOT")]
    t_lines = [l for l in lines if l.startswith("#T ")]

    if not raw:
        print("The board sends nothing at all. Check, in this order:\n"
              f"  1. Another program is reading the port (an old miniterm or 'grep ... &'):\n"
              f"       fuser -v {port}      then  kill <PID>\n"
              "  2. The board is switched off or hung: switch the battery on, press RST.")
    elif len(boots) >= 2:
        print(f"The board restarted {len(boots)} times in {seconds:g} s ({boots[-1]}).\n"
              "It browns out when something draws current: battery switched on and charged?\n"
              "A servo pushing against an end stop draws a lot; centre the steering by hand.")
    elif t_lines:
        print(f"Data arrives now ({len(t_lines)} #T lines): the board was probably restarting.\n"
              "Run the test again; if it happens again, check the battery.")
    else:
        print(f"Received {len(raw)} bytes but no #T lines. First lines: {[l[:60] for l in lines[:3]]}\n"
              "Garbled text means a wrong baud rate or a different firmware on the board.")


def board_problem(board: NiumoBoard, reboots_before: int) -> str:
    """Why a step can't be trusted, or "" if the board behaved."""
    if board.reboots != reboots_before:
        return ("the board restarted during this step: the supply sags under load "
                "(battery charged and switched on? common ground between the power boards?)")
    if board.errors:
        return "the board replied " + "; ".join(board.errors)
    return ""


# ---- servos ---------------------------------------------------------------
def check_servo(board: NiumoBoard, results: Results, name: str, move, header: str,
                low: int = 60, high: int = 120, ask_direction: bool = False) -> None:
    print(f"\n--- {name} (header {header}) ---")
    while True:
        board.errors.clear()
        reboots = board.reboots
        for angle in (90, low, high, 90):
            print(f"  angle {angle}")
            move(angle)
            time.sleep(1.0)
        answer = ask(f"Did the {name} move to both sides and back to the middle?",
                     {"y": "yes", "n": "no", "r": "repeat"})
        if answer != "r":
            break

    problem = board_problem(board, reboots)
    if problem:
        results.add(name, False, problem)
    elif answer == "n":
        results.add(name, False, f"servo plugged into {header}, signal wire facing inward? battery on?")
    elif ask_direction:
        move(low)
        side = ask(f"Steering is now at {low}. Which way do the front wheels point?",
                   {"l": "left", "r": "right"})
        move(90)
        results.add(name, True, f"angle {low} = {'left' if side == 'l' else 'right'}, "
                                f"{high} = {'right' if side == 'l' else 'left'}")
    else:
        results.add(name, True)


# ---- motors and encoders ----------------------------------------------------
def run_motor(board: NiumoBoard, results: Results, side: str, sign: int,
              power: int, seconds: float) -> None:
    other = "right" if side == "left" else "left"
    motor_header = "M1" if side == "left" else "M2"
    enc_header = "EC1" if side == "left" else "EC2"
    direction = "forward" if sign > 0 else "backward"
    name = f"{side} motor {direction}"

    input(f"\nPress Enter to run the {side.upper()} motor {direction} for {seconds:g} s ...")
    board.errors.clear()
    reboots = board.reboots
    start = board.encoders
    if side == "left":
        board.drive_for(sign * power, 0, seconds)
    else:
        board.drive_for(0, sign * power, seconds)
    time.sleep(SETTLE_S)
    end = board.encoders

    problem = board_problem(board, reboots)
    if problem:
        results.add(name, False, problem)
        return

    seen = ask(f"What did the {side} rear wheel do?",
               {"f": "turned forward", "b": "turned backward",
                "o": f"the {other} wheel turned instead", "n": "nothing turned"})
    own = getattr(end, side) - getattr(start, side)
    opp = getattr(end, other) - getattr(start, other)
    print(f"  encoder counts during the run: {side} {own:+d}, {other} {opp:+d}")

    if seen == "n":
        results.add(name, False, f"didn't turn: motor cable in {motor_header}? battery on? "
                                 f"try --power {min(power + 150, 1000)}")
        return
    if seen == "o":
        results.add(name, False, "the other wheel turned: the M1 (left) and M2 (right) cables are swapped")
        return
    if seen != ("f" if sign > 0 else "b"):
        results.add(name, False, "turns the wrong way: swap this motor's two wires")
    else:
        results.add(name, True)

    # The encoder is judged against what the wheel really did.
    turned = 1 if seen == "f" else -1
    enc_name = f"{side} encoder, wheel {'forward' if turned > 0 else 'backward'}"
    if abs(own) < MIN_COUNTS and abs(opp) >= MIN_COUNTS:
        results.add(enc_name, False, f"the counts show up on the {other} side: "
                                     "the EC1 (left) and EC2 (right) cables are swapped")
    elif abs(own) < MIN_COUNTS:
        results.add(enc_name, False, f"no counts ({own:+d}): encoder cable in {enc_header}?")
    elif own * turned < 0:
        results.add(enc_name, False, f"counts the wrong way ({own:+d}): flip "
                                     f"PROTOCOL_ENC_{side.upper()}_SIGN in User/protocol.h")
    else:
        results.add(enc_name, True, f"{own:+d} counts")


def run_both(board: NiumoBoard, results: Results, power: int, seconds: float) -> None:
    name = "both motors forward"
    input(f"\nPress Enter to run BOTH motors forward for {seconds:g} s ...")
    board.errors.clear()
    reboots = board.reboots
    start = board.encoders
    board.drive_for(power, power, seconds)
    time.sleep(SETTLE_S)
    end = board.encoders

    problem = board_problem(board, reboots)
    if problem:
        results.add(name, False, problem)
        return
    left, right = end.left - start.left, end.right - start.right
    print(f"  encoder counts during the run: left {left:+d}, right {right:+d}")
    if ask("Did both rear wheels turn forward?", {"y": "yes", "n": "no"}) == "n":
        results.add(name, False, "see the single-motor results above")
    else:
        results.add(name, True, f"left {left:+d}, right {right:+d} counts")


def main() -> int:
    parser = argparse.ArgumentParser(description="Servo, motor and encoder check for the Niumo board.")
    parser.add_argument("port", nargs="?", default=None,
                        help=f"default: {DEFAULT_PORT}, else the board's CH340 found by USB ID")
    parser.add_argument("--power", type=int, default=350, help="motor power 60..1000 (default 350)")
    parser.add_argument("--seconds", type=float, default=1.5, help="length of each motor run (default 1.5)")
    parser.add_argument("--tilt", action="store_true", help="also test a camera-tilt servo on PB15")
    args = parser.parse_args()
    power = max(60, min(1000, args.power))

    args.port = pick_port(args.port)
    if args.port is None:
        print("Can't find the board: no /dev/robot_mcu and no CH340 (USB 1a86:7523).\n"
              "Is its USB cable plugged in? `lsusb` should list 1a86:7523.")
        return 2

    try:
        board = NiumoBoard(args.port)
    except serial.SerialException as exc:
        print(f"Can't open {args.port}: {exc}\nFind the board's port with:  ls -l /dev/serial/by-id/")
        return 2

    with board:
        try:
            enc = board.wait_for("encoders", timeout=2.0)
        except TimeoutError:
            enc = None
        if enc is not None:
            return run_all(board, args, power, enc)
    diagnose_silence(args.port)          # the port is closed again here
    return 2


def run_all(board: NiumoBoard, args: argparse.Namespace, power: int, enc) -> int:
    print(f"Connected to {args.port}; the board has been running for {enc.ms / 1000:.0f} s.")
    results = Results()

    print("\nSERVOS: each one goes to the middle, one side, the other side, and back.")
    check_servo(board, results, "steering servo", board.steer, "PB14", ask_direction=True)
    # +-15 degrees: inside what the old firmware already reached on every mount
    check_servo(board, results, "camera pan servo", board.cam_pan, "PB13", low=75, high=105)
    if args.tilt:
        check_servo(board, results, "camera tilt servo", board.cam_tilt, "PB15")
    board.center_servos()

    print("\nMOTORS: lift the rear wheels off the ground (put the robot on a box).")
    input("Press Enter when the rear wheels can spin freely ...")
    for side in ("left", "right"):
        for sign in (1, -1):
            run_motor(board, results, side, sign, power, args.seconds)
    run_both(board, results, power, args.seconds)
    board.center_servos()

    return 0 if results.summary() else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\nStopped; motors halted.")   # leaving `with board:` sent @H
        sys.exit(130)
