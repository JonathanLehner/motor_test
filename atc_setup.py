#!/usr/bin/env python
"""
Setup script for Feetech motors in an Automatic Tool Changer (ATC).

Motor ID assignment:
  1 = ATC lock mechanism  (STS-series motor, e.g. sts3215)
  2 = Tool motor 1        (SCS-series motor, e.g. scs0009)
  3 = Tool motor 2        (SCS-series motor, optional)

Usage:
  python atc_setup.py --port /dev/ttyACM1 --target atc
  python atc_setup.py --port /dev/ttyACM1 --target tool --model scs0009
  python atc_setup.py --port /dev/ttyACM1 --target tool --model scs0009 --motors 2
  python atc_setup.py --port /dev/ttyACM1 --target all --atc-model sts3215 --tool-model scs0009
"""

import argparse
import time

from scservo_sdk.port_handler import PortHandler
from scservo_sdk.sms_sts import sms_sts


class EchoFreePortHandler(PortHandler):
    def openPort(self):
        result = super().openPort()
        if result:
            self.ser.timeout = 0.01
        return result

    def writePort(self, packet):
        result = super().writePort(packet)
        self.ser.read(len(packet))
        return result
from scservo_sdk.scscl import scscl
from scservo_sdk.scservo_def import COMM_SUCCESS

# All Feetech-supported baud codes (0..7)
SCAN_BAUDRATES = [1_000_000, 500_000, 250_000, 128_000, 115_200, 76_800, 57_600, 38_400]
TARGET_BAUDRATE = 1_000_000
BAUD_CODE = {1_000_000: 0, 500_000: 1, 250_000: 2, 115_200: 4, 57_600: 6, 19_200: 7}

ID_ADDR = 5
BAUD_ADDR = 6

SCS_MODELS = {"scs0009"}


def make_handler(port, model):
    ph = EchoFreePortHandler(port)
    handler = scscl(ph) if model in SCS_MODELS else sms_sts(ph)
    return ph, handler


def find_motor(port, model):
    """Scan all baudrates/IDs and return (baudrate, motor_id)."""
    ph, handler = make_handler(port, model)
    if not ph.openPort():
        raise RuntimeError(f"Cannot open port {port}")
    try:
        for baudrate in SCAN_BAUDRATES:
            ph.setBaudRate(baudrate)
            for try_id in range(1, 21):
                ph.clearPort()
                _, comm, _ = handler.ping(try_id)
                if comm == COMM_SUCCESS:
                    ph.clearPort()
                    actual_id, comm2, _ = handler.read1ByteTxRx(try_id, ID_ADDR)
                    if comm2 == COMM_SUCCESS and actual_id == try_id:
                        print(f"  Found motor: ID={try_id} at baudrate={baudrate}")
                        return baudrate, try_id
    finally:
        ph.closePort()
    raise RuntimeError(
        "No motor found. Check connection, power, and that only one motor is connected."
    )


def configure_motor(port, model, initial_baudrate, initial_id, target_id):
    """Write target ID and 1 Mbps baudrate to a motor."""
    ph, handler = make_handler(port, model)
    if not ph.openPort():
        raise RuntimeError(f"Cannot open port {port}")
    ph.setBaudRate(initial_baudrate)
    try:
        handler.unLockEprom(initial_id)
        handler.write1ByteTxRx(initial_id, ID_ADDR, target_id)
        handler.write1ByteTxRx(target_id, BAUD_ADDR, BAUD_CODE[TARGET_BAUDRATE])
        handler.LockEprom(target_id)
    finally:
        ph.closePort()


def setup_motor(port, motor_id, label, model):
    print(f"\n{'=' * 60}")
    print(f"Setting up: {label} (target ID {motor_id}, model {model})")
    print(f"{'=' * 60}")
    input(f"Connect ONLY this motor to {port}, then press ENTER...")

    print("Scanning for motor...")
    initial_baudrate, initial_id = find_motor(port, model)

    if initial_id == motor_id and initial_baudrate == TARGET_BAUDRATE:
        print("  Already configured correctly.")
        return

    configure_motor(port, model, initial_baudrate, initial_id, motor_id)
    print(f"  Done: ID={motor_id}, baudrate={TARGET_BAUDRATE}")


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
