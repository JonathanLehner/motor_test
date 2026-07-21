#!/usr/bin/env python3
"""Record hand-moved DaMiao position minimum and maximum, read-only.

The motor must already be disabled.  This script only sends the 0xCC status
refresh request and refuses to continue if the motor reports ENABLED.
"""

from __future__ import annotations

import argparse
from datetime import datetime
import json
import math
from pathlib import Path
import sys
import time

try:
    import serial
except ImportError:
    print("Missing pyserial. Install it with: pip install pyserial", file=sys.stderr)
    raise SystemExit(1)

from damiao_status_monitor import (
    MOTOR_LIMITS,
    decode_status,
    extract_packets,
    parse_int,
    send_status_request,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=921600)
    parser.add_argument("--can-id", type=parse_int, default=0x01)
    parser.add_argument("--motor-type", choices=sorted(MOTOR_LIMITS), default="DM4310")
    parser.add_argument("--period", type=float, default=0.05, help="sample interval in seconds")
    parser.add_argument(
        "--duration",
        type=float,
        default=15.0,
        help="recording time in seconds; use 0 to run until Ctrl-C",
    )
    parser.add_argument("--output", type=Path, help="optionally save the result as JSON")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.period <= 0:
        raise SystemExit("--period must be greater than zero")
    if args.duration < 0:
        raise SystemExit("--duration must be zero or greater")

    limits = MOTOR_LIMITS[args.motor_type]
    minimum: float | None = None
    maximum: float | None = None
    samples = 0
    buffer = b""
    started_at = datetime.now().astimezone()
    deadline = None if args.duration == 0 else time.monotonic() + args.duration
    next_poll = time.monotonic()
    last_reply = next_poll

    print("DaMiao read-only position calibration")
    print(f"  adapter   : {args.port} @ {args.baud}")
    print(f"  motor     : {args.motor_type}, CAN ID 0x{args.can_id:X}")
    print("  safety    : motor must remain DISABLED")
    print("  operation : status refresh only (no enable/disable/control)")
    if deadline is None:
        print("Move the gripper fully open and fully closed, then press Ctrl-C.\n")
    else:
        print(f"Move the gripper fully open and fully closed within {args.duration:g} seconds.\n")

    try:
        with serial.Serial(args.port, args.baud, timeout=min(0.05, args.period)) as port:
            while deadline is None or time.monotonic() < deadline:
                now = time.monotonic()
                if now >= next_poll:
                    send_status_request(port, args.can_id)
                    next_poll = now + args.period

                chunk = port.read_all()
                if not chunk:
                    time.sleep(0.002)
                else:
                    buffer += chunk
                    packets, buffer = extract_packets(buffer)
                    for packet in packets:
                        if packet[1] != 0x11:
                            continue
                        outer_can_id = int.from_bytes(packet[3:7], "little")
                        status = decode_status(packet[7:15], outer_can_id, limits)
                        if outer_can_id not in (0, args.can_id) and status.motor_id != (args.can_id & 0x0F):
                            continue

                        if status.state_code != 0x0:
                            print("\nCalibration stopped: motor is not DISABLED.", file=sys.stderr)
                            print(f"Reported state: {status.state}", file=sys.stderr)
                            print("Disable it separately before moving the gripper by hand.", file=sys.stderr)
                            return 2

                        position = status.position_rad
                        minimum = position if minimum is None else min(minimum, position)
                        maximum = position if maximum is None else max(maximum, position)
                        samples += 1
                        last_reply = time.monotonic()
                        print(
                            f"\rpos={position:+9.4f} rad ({math.degrees(position):+8.2f} deg)"
                            f" | min={minimum:+9.4f} | max={maximum:+9.4f} | samples={samples:5d}",
                            end="",
                            flush=True,
                        )

                if time.monotonic() - last_reply > max(2.0, args.period * 10):
                    print("\nNo reply; check motor power, CAN wiring, bitrate, and CAN ID.", file=sys.stderr)
                    last_reply = time.monotonic()
    except KeyboardInterrupt:
        print("\nStopped by user.")
    except serial.SerialException as exc:
        print(f"\nSerial error: {exc}", file=sys.stderr)
        return 1

    print()
    if minimum is None or maximum is None:
        print("No valid position samples were received.", file=sys.stderr)
        return 1

    span = maximum - minimum
    result = {
        "timestamp": started_at.isoformat(timespec="seconds"),
        "port": args.port,
        "baud": args.baud,
        "can_id": args.can_id,
        "motor_type": args.motor_type,
        "samples": samples,
        "position_min_rad": minimum,
        "position_max_rad": maximum,
        "position_span_rad": span,
        "position_min_deg": math.degrees(minimum),
        "position_max_deg": math.degrees(maximum),
        "position_span_deg": math.degrees(span),
    }

    print("Calibration result")
    print(f"  min  : {minimum:+.6f} rad  ({math.degrees(minimum):+.3f} deg)")
    print(f"  max  : {maximum:+.6f} rad  ({math.degrees(maximum):+.3f} deg)")
    print(f"  span : {span:.6f} rad  ({math.degrees(span):.3f} deg)")
    print(f"  samples: {samples}")

    if args.output:
        args.output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
        print(f"  saved: {args.output}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
