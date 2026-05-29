#!/usr/bin/env python
"""
Raw byte-level scanner for an echoing half-duplex Feetech bus.

The bus echoes every transmitted byte back on RX. This scanner sends a PING to
each ID, consumes exactly the echoed request bytes, and only reports an ID if
there are REAL response bytes after the echo. No SDK parsing.

Usage:
  python test_raw.py --port /dev/ttyACM0
  python test_raw.py --port /dev/ttyACM0 --baud 1000000
"""

import argparse
import time

import serial

# All Feetech-supported baud codes (0..7)
BAUDRATES = [1_000_000, 500_000, 250_000, 128_000, 115_200, 76_800, 57_600, 38_400]


def checksum(body):
    return (~sum(body)) & 0xFF


def ping_packet(motor_id):
    # PING: FF FF ID LEN(=2) INST(=1) CHK
    body = [motor_id, 0x02, 0x01]
    return bytes([0xFF, 0xFF] + body + [checksum(body)])


def scan_baud(port, baud, verbose=False):
    """Open the port fresh at this baud so the rate is guaranteed applied."""
    found = []
    with serial.Serial(port, baud, timeout=0.02) as ser:
        print(f"      (pyserial reports baudrate={ser.baudrate})")
        for motor_id in range(1, 21):
            pkt = ping_packet(motor_id)
            ser.reset_input_buffer()
            ser.write(pkt)
            time.sleep(0.01)
            rx = ser.read(64)
            if verbose and motor_id <= 3:
                print(f"      ID={motor_id} TX={pkt.hex(' ')}  RX={rx.hex(' ') or '<nothing>'}")
            # Strip the echoed request, look for a real response after it.
            resp = rx[len(pkt):] if rx[:len(pkt)] == pkt else rx
            if len(resp) >= 6 and resp[0] == 0xFF and resp[1] == 0xFF and resp[2] == motor_id:
                found.append((motor_id, resp[4], resp.hex(" ")))
    return found


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, help="single baudrate to scan (default: all)")
    ap.add_argument("--verbose", action="store_true", help="dump raw RX bytes for IDs 1-3")
    args = ap.parse_args()

    bauds = [args.baud] if args.baud else BAUDRATES

    print(f"Scanning {args.port} (echo-aware)...")
    any_found = False
    for baud in bauds:
        print(f"  trying baud {baud} ...")
        for motor_id, err, raw in scan_baud(args.port, baud, args.verbose):
            print(f"    FOUND  baud={baud}  ID={motor_id}  err={err:#04x}  resp={raw}")
            any_found = True

    if not any_found:
        print("No real motor responses found (only echo). Check power/wiring/ID.")


if __name__ == "__main__":
    main()
