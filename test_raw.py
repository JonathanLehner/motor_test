#!/usr/bin/env python
"""
Raw byte-level diagnostic. Sends a READ Present_Position request to a motor
and dumps every byte received, with no SDK parsing. This tells us definitively
whether the bus echoes TX, and what the real response looks like.

Usage:
  python test_raw.py --port /dev/ttyACM0 --id 2 --baud 1000000
"""

import argparse
import time

import serial


def checksum(body):
    return (~sum(body)) & 0xFF


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--id", type=int, default=2)
    ap.add_argument("--baud", type=int, default=1_000_000)
    ap.add_argument("--addr", type=int, default=56, help="register addr (56=Present_Position)")
    ap.add_argument("--len", type=int, default=2, dest="length")
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.05)

    # READ instruction packet: FF FF ID LEN(=4) INST(=2) ADDR LEN CHK
    body = [args.id, 0x04, 0x02, args.addr, args.length]
    packet = bytes([0xFF, 0xFF] + body + [checksum(body)])

    print(f"Port {args.port} @ {args.baud}, reading addr {args.addr} len {args.length} from ID {args.id}")
    print(f"TX ({len(packet)} bytes): {packet.hex(' ')}")

    ser.reset_input_buffer()
    ser.write(packet)
    time.sleep(0.02)

    rx = ser.read(64)
    print(f"RX ({len(rx)} bytes): {rx.hex(' ')}")

    if len(rx) >= len(packet) and rx[:len(packet)] == packet:
        print("  -> bus ECHOES TX. Response after echo:")
        resp = rx[len(packet):]
        print(f"     {resp.hex(' ')}")
    else:
        print("  -> no TX echo (or partial). Bytes above are the raw response.")

    ser.close()


if __name__ == "__main__":
    main()
