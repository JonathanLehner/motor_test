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

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus

# SCS series uses protocol 1; STS/SMS series uses protocol 0
SCS_MODELS = {"scs0009"}
SCAN_BAUDRATES = [1_000_000, 500_000, 250_000, 128_000, 115_200, 57_600, 38_400, 19_200]


def scan_for_motor(port: str, protocol_version: int) -> tuple[int, int]:
    """Return (baudrate, motor_id) of the single connected motor."""
    probe = FeetechMotorsBus(
        port=port,
        motors={"probe": Motor(1, "sts3215", MotorNormMode.RANGE_M100_100)},
        protocol_version=protocol_version,
    )
    probe.connect(handshake=False)
    try:
        for baudrate in SCAN_BAUDRATES:
            probe.set_baudrate(baudrate)
            result = probe.broadcast_ping()
            if result:
                if len(result) > 1:
                    raise RuntimeError(
                        f"Found {len(result)} motors: {list(result.keys())}. "
                        "Connect ONLY the motor you want to configure."
                    )
                motor_id = next(iter(result))
                print(f"  Found motor: ID={motor_id} at baudrate={baudrate}")
                return baudrate, motor_id
    finally:
        probe.disconnect(disable_torque=False)
    raise RuntimeError("No motor found. Check the connection and power.")


def setup_motor(port: str, motor_id: int, label: str, model: str) -> None:
    print(f"\n{'=' * 60}")
    print(f"Setting up: {label} (target ID {motor_id}, model {model})")
    print(f"{'=' * 60}")
    input(f"Connect ONLY this motor to {port}, then press ENTER...")

    protocol_version = 1 if model in SCS_MODELS else 0

    print("Scanning for motor (this may take a moment)...")
    initial_baudrate, initial_id = scan_for_motor(port, protocol_version)

    name = f"motor_{motor_id}"
    bus = FeetechMotorsBus(
        port=port,
        motors={name: Motor(motor_id, model, MotorNormMode.RANGE_M100_100)},
        protocol_version=protocol_version,
    )
    try:
        bus.setup_motor(name, initial_baudrate=initial_baudrate, initial_id=initial_id)
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
