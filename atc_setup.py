#!/usr/bin/env python
"""
Setup script for Feetech motors in an Automatic Tool Changer (ATC).

Motor ID assignment:
  1 = ATC lock mechanism  (STS-series motor, e.g. sts3215)
  2 = Tool motor 1        (SCS-series motor, e.g. scs0009)
  3 = Tool motor 2        (SCS-series motor, optional)

All motors are expected to be at their factory default ID and baudrate.
The script scans, finds the motor, then writes the target ID and 1 Mbps baudrate.

Usage:
  python atc_setup.py --port /dev/ttyACM1 --target atc
  python atc_setup.py --port /dev/ttyACM1 --target tool --model scs0009
  python atc_setup.py --port /dev/ttyACM1 --target tool --model scs0009 --motors 2
  python atc_setup.py --port /dev/ttyACM1 --target all --atc-model sts3215 --tool-model scs0009
"""

import argparse

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus

SCS_MODELS = {"scs0009"}
SCAN_BAUDRATES = [1_000_000, 500_000, 250_000, 115_200, 57_600, 19_200]


def protocol_for(model: str) -> int:
    return 1 if model in SCS_MODELS else 0


def find_motor(port: str, model: str) -> tuple[int, int]:
    """Locate the single connected motor. Returns (baudrate, motor_id)."""
    protocol = protocol_for(model)
    bus = FeetechMotorsBus(
        port=port,
        motors={"probe": Motor(1, model, MotorNormMode.RANGE_M100_100)},
        protocol_version=protocol,
    )
    bus.connect(handshake=False)
    try:
        for baudrate in SCAN_BAUDRATES:
            bus.set_baudrate(baudrate)
            if protocol == 0:
                result = bus.broadcast_ping()
                if result:
                    if len(result) > 1:
                        raise RuntimeError(
                            f"Found {len(result)} motors: {list(result.keys())}. "
                            "Connect ONLY the motor you want to configure."
                        )
                    motor_id = next(iter(result))
                    print(f"  Found motor: ID={motor_id} at baudrate={baudrate}")
                    return baudrate, motor_id
            else:
                # Protocol 1 (SCS) does not support broadcast ping.
                for try_id in range(0, 20):
                    if bus.ping(try_id) is not None:
                        print(f"  Found motor: ID={try_id} at baudrate={baudrate}")
                        return baudrate, try_id
    finally:
        bus.disconnect(disable_torque=False)
    raise RuntimeError(
        f"No motor found for model '{model}' (protocol {protocol}). "
        "Check the connection, power, and that only one motor is connected."
    )


def setup_motor(port: str, motor_id: int, label: str, model: str) -> None:
    print(f"\n{'=' * 60}")
    print(f"Setting up: {label} (target ID {motor_id}, model {model})")
    print(f"{'=' * 60}")
    input(f"Connect ONLY this motor to {port}, then press ENTER...")

    print("Scanning for motor (this may take a moment)...")
    initial_baudrate, initial_id = find_motor(port, model)

    name = f"motor_{motor_id}"
    bus = FeetechMotorsBus(
        port=port,
        motors={name: Motor(motor_id, model, MotorNormMode.RANGE_M100_100)},
        protocol_version=protocol_for(model),
    )
    try:
        bus.setup_motor(name, initial_baudrate=initial_baudrate, initial_id=initial_id)
        print(f"✓ Done: ID={motor_id}, model={model}")
    finally:
        if bus.is_connected:
            bus.disconnect(disable_torque=False)


def main() -> None:
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

    # Build steps: (motor_id, label, model)
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
