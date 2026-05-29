#!/usr/bin/env python
"""
Raw scan for SCS0009 (or any Feetech motor) using the scservo_sdk directly.
Tries all baudrates, both protocol_end values, and IDs 1-20.
Run this with the motor connected to diagnose whether it responds at all.

Usage:
  python test_scs_scan.py --port /dev/ttyACM1
"""

import argparse

import scservo_sdk as scs

BAUDRATES = [1_000_000, 500_000, 250_000, 115_200, 57_600, 38_400, 19_200]
IDS = range(1, 21)


def scan(port: str) -> None:
    port_handler = scs.PortHandler(port)

    if not port_handler.openPort():
        raise RuntimeError(f"Failed to open port {port}")

    print(f"Scanning {port} ...")
    print(f"{'Baudrate':>12}  {'Protocol':>9}  {'ID':>4}  {'Model':>7}  Result")
    print("-" * 55)

    found_any = False

    for baudrate in BAUDRATES:
        port_handler.setBaudRate(baudrate)

        for protocol_end in [0, 1]:
            scs.PacketHandler(protocol_end)  # sets the global SCS_END
            ph = scs.protocol_packet_handler()

            for try_id in IDS:
                model_number, comm, error = ph.ping(port_handler, try_id)
                if comm == scs.COMM_SUCCESS:
                    label = "STS/SMS" if protocol_end == 0 else "SCS"
                    print(f"{baudrate:>12}  {label:>9}  {try_id:>4}  {model_number:>7}  OK")
                    found_any = True

    port_handler.closePort()

    if not found_any:
        print("No motors found. Check wiring, power, and that the motor is connected.")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True)
    scan(parser.parse_args().port)


if __name__ == "__main__":
    main()
