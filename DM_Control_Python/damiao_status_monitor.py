#!/usr/bin/env python3
"""Human-readable, read-only DaMiao motor status monitor.

This talks to a DaMiao USB2CAN serial adapter.  It only sends the 0xCC status
refresh request; it never enables, disables, zeros, or commands the motor.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from datetime import datetime
import math
import sys
import time

try:
    import serial
except ImportError:
    print("Missing pyserial. Install it with: pip install pyserial", file=sys.stderr)
    raise SystemExit(1)


DEFAULT_PORT = "/dev/ttyACM0"
DEFAULT_BAUD = 921600

# Position, velocity, and torque limits used by DM_CAN.py for feedback decoding.
MOTOR_LIMITS = {
    "DM4310": (12.5, 30.0, 10.0),
    "DM4310_48V": (12.5, 50.0, 10.0),
    "DM4340": (12.5, 8.0, 28.0),
    "DM4340_48V": (12.5, 10.0, 28.0),
    "DM6006": (12.5, 45.0, 20.0),
    "DM8006": (12.5, 45.0, 40.0),
    "DM8009": (12.5, 45.0, 54.0),
    "DM10010L": (12.5, 25.0, 200.0),
    "DM10010": (12.5, 20.0, 200.0),
    "DMH3510": (12.5, 280.0, 1.0),
    "DMH6215": (12.5, 45.0, 10.0),
    "DMG6220": (12.5, 45.0, 10.0),
}

STATUS = {
    0x0: "DISABLED",
    0x1: "ENABLED / OK",
    0x8: "OVERVOLTAGE",
    0x9: "UNDERVOLTAGE",
    0xA: "OVERCURRENT",
    0xB: "MOS OVER-TEMPERATURE",
    0xC: "ROTOR OVER-TEMPERATURE",
    0xD: "COMMUNICATION LOST",
    0xE: "OVERLOAD",
}

SEND_FRAME_TEMPLATE = bytearray(
    [
        0x55, 0xAA, 0x1E, 0x03, 0x01, 0x00, 0x00, 0x00,
        0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    ]
)


@dataclass(frozen=True)
class MotorStatus:
    outer_can_id: int
    motor_id: int
    state_code: int
    position_rad: float
    velocity_rad_s: float
    torque_nm: float
    mos_temp_c: int
    rotor_temp_c: int
    raw: bytes

    @property
    def state(self) -> str:
        return STATUS.get(self.state_code, f"UNKNOWN 0x{self.state_code:X}")


def parse_int(value: str) -> int:
    return int(value, 0)


def uint_to_float(value: int, limit: float, bits: int) -> float:
    return value / ((1 << bits) - 1) * (2.0 * limit) - limit


def decode_status(data: bytes, outer_can_id: int, limits: tuple[float, float, float]) -> MotorStatus:
    if len(data) != 8:
        raise ValueError(f"expected 8 status bytes, got {len(data)}")

    q_max, dq_max, tau_max = limits
    position_raw = (data[1] << 8) | data[2]
    velocity_raw = (data[3] << 4) | (data[4] >> 4)
    torque_raw = ((data[4] & 0x0F) << 8) | data[5]

    return MotorStatus(
        outer_can_id=outer_can_id,
        motor_id=data[0] & 0x0F,
        state_code=data[0] >> 4,
        position_rad=uint_to_float(position_raw, q_max, 16),
        velocity_rad_s=uint_to_float(velocity_raw, dq_max, 12),
        torque_nm=uint_to_float(torque_raw, tau_max, 12),
        mos_temp_c=data[6],
        rotor_temp_c=data[7],
        raw=data,
    )


def send_status_request(port: serial.Serial, motor_id: int) -> None:
    frame = bytearray(SEND_FRAME_TEMPLATE)
    frame[13] = 0xFF
    frame[14] = 0x07
    frame[21:29] = bytes([motor_id & 0xFF, (motor_id >> 8) & 0xFF, 0xCC, 0, 0, 0, 0, 0])
    port.write(frame)


def extract_packets(buffer: bytes) -> tuple[list[bytes], bytes]:
    packets: list[bytes] = []
    frame_length = 16
    index = 0
    consumed = 0

    while index <= len(buffer) - frame_length:
        if buffer[index] == 0xAA and buffer[index + frame_length - 1] == 0x55:
            packets.append(buffer[index:index + frame_length])
            index += frame_length
            consumed = index
        else:
            index += 1

    # Do not retain arbitrary noise forever, but preserve a possible partial frame.
    remainder = buffer[consumed:]
    if not packets and len(remainder) > frame_length - 1:
        remainder = remainder[-(frame_length - 1):]
    return packets, remainder


def print_status(status: MotorStatus, show_raw: bool) -> None:
    timestamp = datetime.now().astimezone().isoformat(timespec="milliseconds")
    degrees = math.degrees(status.position_rad)
    fault_marker = "" if status.state_code in (0x0, 0x1) else "  !!! FAULT !!!"
    print(f"[{timestamp}] motor 0x{status.motor_id:X}  {status.state}{fault_marker}")
    print(f"  position : {status.position_rad:9.4f} rad  ({degrees:8.2f} deg)")
    print(f"  velocity : {status.velocity_rad_s:9.4f} rad/s")
    print(f"  torque   : {status.torque_nm:9.4f} N*m")
    print(f"  temp     : MOS {status.mos_temp_c:3d} C | rotor {status.rotor_temp_c:3d} C")
    print(f"  CAN      : feedback ID 0x{status.outer_can_id:X}")
    if show_raw:
        print(f"  raw      : {status.raw.hex(' ').upper()}")
    print(flush=True)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--can-id", type=parse_int, default=0x01, help="motor receive/CAN ID")
    parser.add_argument("--motor-type", choices=sorted(MOTOR_LIMITS), default="DM4310")
    parser.add_argument("--period", type=float, default=0.2, help="poll interval in seconds")
    parser.add_argument("--count", type=int, default=0, help="number of replies; 0 means run until Ctrl-C")
    parser.add_argument("--raw", action="store_true", help="also print the eight raw status bytes")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.period <= 0:
        raise SystemExit("--period must be greater than zero")
    if args.count < 0:
        raise SystemExit("--count must be zero or greater")

    limits = MOTOR_LIMITS[args.motor_type]
    print("DaMiao read-only status monitor")
    print(f"  adapter   : {args.port} @ {args.baud}")
    print(f"  motor     : {args.motor_type}, CAN ID 0x{args.can_id:X}")
    print("  operation : status refresh only (no enable/disable/control)")
    print("Press Ctrl-C to stop.\n")

    received = 0
    buffer = b""
    next_poll = time.monotonic()
    last_reply = next_poll

    try:
        with serial.Serial(args.port, args.baud, timeout=min(0.1, args.period)) as port:
            while args.count == 0 or received < args.count:
                now = time.monotonic()
                if now >= next_poll:
                    send_status_request(port, args.can_id)
                    next_poll = now + args.period

                chunk = port.read_all()
                if chunk:
                    buffer += chunk
                    packets, buffer = extract_packets(buffer)
                    for packet in packets:
                        if packet[1] != 0x11:
                            continue
                        outer_can_id = int.from_bytes(packet[3:7], "little")
                        status = decode_status(packet[7:15], outer_can_id, limits)
                        # Some firmware replies on CAN ID 0 and puts the motor ID
                        # in byte 0; accept either representation.
                        if outer_can_id not in (0, args.can_id) and status.motor_id != (args.can_id & 0x0F):
                            continue
                        print_status(status, args.raw)
                        received += 1
                        last_reply = time.monotonic()
                        if args.count and received >= args.count:
                            break
                else:
                    time.sleep(0.005)

                if time.monotonic() - last_reply > max(2.0, args.period * 5):
                    print("No motor reply yet; check motor power, CAN wiring, bitrate, and CAN ID.", file=sys.stderr)
                    last_reply = time.monotonic()
    except KeyboardInterrupt:
        print("\nStopped.")
    except serial.SerialException as exc:
        print(f"Serial error: {exc}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
