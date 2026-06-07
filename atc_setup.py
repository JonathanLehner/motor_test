#!/usr/bin/env python
"""
Setup script for Feetech motors in an Automatic Tool Changer (ATC).

Motor ID assignment:
  1 = ATC lock mechanism  (STS-series motor, e.g. sts3215)
  2 = Tool motor 1        (SCS-series motor, e.g. scs0009, or sts3215)
  3 = Tool motor 2        (optional)

ID and baudrate assignment is done with LeRobot's FeetechMotorsBus.setup_motor(),
which scans every baudrate/ID for the single connected motor, then writes the
target ID and the bus default baudrate (1 Mbps). LeRobot handles the EEPROM
unlock/lock internally, so the change persists across power cycles.

Usage:
  python atc_setup.py --port /dev/ttyACM1 --target atc
  python atc_setup.py --port /dev/ttyACM1 --target tool --model scs0009
  python atc_setup.py --port /dev/ttyACM1 --target tool --model scs0009 --motors 2
  python atc_setup.py --port /dev/ttyACM1 --target all --atc-model sts3215 --tool-model scs0009
"""

import argparse

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus

# SC-series motors speak protocol 1; ST/SMS-series (sts3215, ...) speak protocol 0.
SCS_MODELS = {"scs0009"}


def protocol_for(model):
    return 1 if model in SCS_MODELS else 0


def setup_motor(port, motor_id, label, model):
    print(f"\n{'=' * 60}")
    print(f"Setting up: {label} (target ID {motor_id}, model {model})")
    print(f"{'=' * 60}")
    input(f"Connect ONLY this motor to {port}, then press ENTER...")

    # The motor key is arbitrary; its configured id is the TARGET id that
    # setup_motor() will write to whatever motor it discovers on the bus.
    bus = FeetechMotorsBus(
        port=port,
        motors={"motor": Motor(id=motor_id, model=model, norm_mode=MotorNormMode.RANGE_M100_100)},
        protocol_version=protocol_for(model),
    )
    bus.connect(handshake=False)
    try:
        print("Scanning for motor...")
        bus.setup_motor("motor")
        print(f"  Done: ID={motor_id} set, baudrate programmed to bus default (1 Mbps)")
    finally:
        bus.disconnect()


def main():
    parser = argparse.ArgumentParser(description="Configure ATC Feetech motor IDs")
    parser.add_argument("--port", required=True, help="Serial port, e.g. /dev/ttyACM1")
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
        help="Number of tool motors (default: 1). Only relevant for --target tool or all.",
    )
    parser.add_argument("--model", default="sts3215",
                        help="Motor model for --target atc or tool (default: sts3215)")
    parser.add_argument("--atc-model", default="sts3215",
                        help="ATC motor model for --target all (default: sts3215)")
    parser.add_argument("--tool-model", default="scs0009",
                        help="Tool motor model for --target all (default: scs0009)")
    args = parser.parse_args()

    steps = []
    if args.target == "atc":
        steps.append((1, "ATC lock mechanism", args.model))
    elif args.target == "tool":
        steps.append((2, "Tool motor 1", args.model))
        if args.motors == 2:
            steps.append((3, "Tool motor 2", args.model))
    elif args.target == "all":
        steps.append((1, "ATC lock mechanism", args.atc_model))
        steps.append((2, "Tool motor 1", args.tool_model))
        if args.motors == 2:
            steps.append((3, "Tool motor 2", args.tool_model))

    print("ATC Motor Setup")
    print(f"Port  : {args.port}")
    print(f"Steps : {len(steps)}")

    for motor_id, label, model in steps:
        setup_motor(args.port, motor_id, label, model)

    print(f"\n{'=' * 60}")
    print("Setup complete:")
    for motor_id, label, model in steps:
        print(f"  {label:25s} -> ID {motor_id}  ({model})")


if __name__ == "__main__":
    main()
