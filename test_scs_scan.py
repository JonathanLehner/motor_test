#!/usr/bin/env python
"""
Scan for any Feetech motor (STS or SCS) using the high-level scservo_sdk classes.
Reports model number and error byte so you can see if a motor responds but has
an error condition (overload, voltage, etc.).

Usage:
  python test_scs_scan.py --port /dev/ttyACM1
"""

import argparse

from scservo_sdk.port_handler import PortHandler
from scservo_sdk.sms_sts import sms_sts
from scservo_sdk.scscl import scscl
from scservo_sdk.scservo_def import COMM_SUCCESS

BAUDRATES = [1_000_000, 500_000, 250_000, 115_200]
IDS = range(1, 11)
CONFIGS = [
    ("STS/SMS", sms_sts),
    ("SCS",     scscl),
]


def scan(port):
    print(f"Scanning {port} ...")
    print(f"{'Baudrate':>10}  {'Protocol':>9}  {'ID':>4}  {'Model':>7}  {'Error':>6}")
    print("-" * 50)

    found_any = False
    ph = PortHandler(port)
    if not ph.openPort():
        raise RuntimeError(f"Cannot open port {port}")

    try:
        for label, handler_cls in CONFIGS:
            handler = handler_cls(ph)
            for baudrate in BAUDRATES:
                ph.setBaudRate(baudrate)
                for motor_id in IDS:
                    ph.clearPort()
                    model_number, comm, error = handler.ping(motor_id)
                    if comm == COMM_SUCCESS:
                        ph.clearPort()
                        actual_id, comm2, _ = handler.read1ByteTxRx(motor_id, 5)
                        if comm2 == COMM_SUCCESS and actual_id == motor_id:
                            print(f"{baudrate:>10}  {label:>9}  {motor_id:>4}  {model_number:>7}  {error:#04x}")
                            found_any = True
    finally:
        ph.closePort()

    if not found_any:
        print("No motors found.")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True)
    scan(parser.parse_args().port)


if __name__ == "__main__":
    main()
