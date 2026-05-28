#!/usr/bin/env python
"""
Setup script for Feetech motors in an Automatic Tool Changer (ATC).

Motor ID assignment:
  1 = ATC lock mechanism
  2 = Tool motor (first / only motor in a tool)
  3 = Tool motor (second motor, only for tools with 2 motors)

Usage:
  # Configure the ATC lock motor
  python atc_setup.py --port /dev/tty.usbmodem... --target atc

  # Configure a tool with 1 motor
  python atc_setup.py --port /dev/tty.usbmodem... --target tool

  # Configure a tool with 2 motors
  python atc_setup.py --port /dev/tty.usbmodem... --target tool --motors 2

  # Configure ATC + tool (1 or 2 motors) in one go
  python atc_setup.py --port /dev/tty.usbmodem... --target all
  python atc_setup.py --port /dev/tty.usbmodem... --target all --motors 2
"""

import argparse

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus

MOTOR_MODEL = "sts3215"


def setup_motor(port: str, motor_id: int, label: str) -> None:
    print(f"\n{'=' * 60}")
    print(f"Setting up: {label} (ID {motor_id})")
    print(f"{'=' * 60}")
    input(f"Connect ONLY this motor to {port}, then press ENTER...")

    name = f"motor_{motor_id}"
    bus = FeetechMotorsBus(
        port=port,
        motors={name: Motor(motor_id, MOTOR_MODEL, MotorNormMode.RANGE_M100_100)},
    )
    try:
        print("Scanning for motor (this may take a moment)...")
        bus.setup_motor(name)
        print(f"✓ Done: ID={motor_id}, model={MOTOR_MODEL}")
    finally:
        if bus.is_connected:
            bus.disconnect(disable_torque=False)


def main() -> None:
    parser = argparse.ArgumentParser(description="Configure ATC Feetech motor IDs")
    parser.add_argument("--port", required=True, help="Serial port, e.g. /dev/tty.usbmodem...")
    parser.add_argument(
        "--target",
        required=True,
        choices=["atc", "tool", "all"],
        help="atc = lock motor only; tool = tool motor(s); all = ATC + tool",
    )
    parser.add_argument(
        "--motors",
        type=int,
        choices=[1, 2],
        default=1,
        help="Number of motors in the tool (default: 1). Only relevant for --target tool or all.",
    )
    args = parser.parse_args()

    steps = []
    if args.target in ("atc", "all"):
        steps.append((1, "ATC lock mechanism"))
    if args.target in ("tool", "all"):
        steps.append((2, "Tool motor 1"))
        if args.motors == 2:
            steps.append((3, "Tool motor 2"))

    print("ATC Motor Setup")
    print(f"Port  : {args.port}")
    print(f"Steps : {len(steps)}")

    for motor_id, label in steps:
        setup_motor(args.port, motor_id, label)

    print(f"\n{'=' * 60}")
    print("Setup complete:")
    for motor_id, label in steps:
        print(f"  {label:25s} -> ID {motor_id}")


if __name__ == "__main__":
    main()
