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

  # Use a different motor model (default: sts3215)
  python atc_setup.py --port /dev/tty.usbmodem... --target atc --model scs0009
"""

import argparse
import re

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus

# SCS series uses protocol 1; STS/SMS series uses protocol 0
SCS_MODELS = {"scs0009"}


def protocol_for(model: str) -> int:
    return 1 if model in SCS_MODELS else 0


def setup_motor(port: str, motor_id: int, label: str, model: str) -> None:
    print(f"\n{'=' * 60}")
    print(f"Setting up: {label} (target ID {motor_id}, model {model})")
    print(f"{'=' * 60}")
    input(f"Connect ONLY this motor to {port}, then press ENTER...")

    name = f"motor_{motor_id}"
    bus = FeetechMotorsBus(
        port=port,
        motors={name: Motor(motor_id, model, MotorNormMode.RANGE_M100_100)},
        protocol_version=protocol_for(model),
    )
    try:
        print("Scanning for motor (this may take a moment)...")
        try:
            bus.setup_motor(name)
        except RuntimeError as e:
            # lerobot's scanner found the motor but rejected it due to a model number
            # mismatch (e.g. motor firmware reports a different number than the table).
            # Parse the actual baudrate and ID from the error, then retry bypassing the check.
            m_baud = re.search(r"baudrate=(\d+)", str(e))
            m_id = re.search(r"with id=(\d+)", str(e))
            if not (m_baud and m_id):
                raise
            found_baudrate = int(m_baud.group(1))
            found_id = int(m_id.group(1))
            print(f"  Motor found (ID={found_id}, baudrate={found_baudrate}), bypassing model check.")
            bus.setup_motor(name, initial_baudrate=found_baudrate, initial_id=found_id)
        print(f"✓ Done: ID={motor_id}, model={model}")
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
    parser.add_argument(
        "--model",
        default="sts3215",
        help="Feetech motor model (default: sts3215). Use scs0009 for SCS series.",
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
    print(f"Model : {args.model}")
    print(f"Steps : {len(steps)}")

    for motor_id, label in steps:
        setup_motor(args.port, motor_id, label, args.model)

    print(f"\n{'=' * 60}")
    print("Setup complete:")
    for motor_id, label in steps:
        print(f"  {label:25s} -> ID {motor_id}")


if __name__ == "__main__":
    main()
