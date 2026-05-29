#!/usr/bin/env python
"""
Scan for any Feetech motor (STS or SCS) using lerobot's bus.
Reports comm status and error byte separately so you can see if a motor
responds but has an error condition (overload, voltage, etc.).

Usage:
  python test_scs_scan.py --port /dev/ttyACM1
"""

import argparse

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus

BAUDRATES = [1_000_000, 500_000, 250_000, 115_200]
IDS = range(1, 11)
CONFIGS = [
    ("STS/SMS", "sts3215", 0),
    ("SCS",     "scs0009", 1),
]


def scan(port: str) -> None:
    print(f"Scanning {port} ...")
    print(f"{'Baudrate':>10}  {'Protocol':>9}  {'ID':>4}  {'Model':>7}  {'Error':>6}")
    print("-" * 50)

    found_any = False

    for label, model, protocol_version in CONFIGS:
        bus = FeetechMotorsBus(
            port=port,
            motors={"probe": Motor(1, model, MotorNormMode.RANGE_M100_100)},
            protocol_version=protocol_version,
        )
        bus.connect(handshake=False)
        try:
            for baudrate in BAUDRATES:
                bus.set_baudrate(baudrate)
                if protocol_version == 0:
                    result = bus.broadcast_ping()
                    for motor_id, model_number in result.items():
                        print(f"{baudrate:>10}  {label:>9}  {motor_id:>4}  {model_number:>7}  (broadcast)")
                        found_any = True
                else:
                    for try_id in IDS:
                        model_number, comm, error = bus.packet_handler.ping(bus.port_handler, try_id)
                        if bus._is_comm_success(comm):
                            print(f"{baudrate:>10}  {label:>9}  {try_id:>4}  {model_number:>7}  {error:#04x}")
                            found_any = True
        finally:
            bus.disconnect(disable_torque=False)

    if not found_any:
        print("No motors found.")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True)
    scan(parser.parse_args().port)


if __name__ == "__main__":
    main()
