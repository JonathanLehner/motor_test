#!/usr/bin/env python3

import argparse
import errno
import sys
import time

try:
    import serial
except ImportError:
    print("Missing pyserial. Install with:")
    print("  pip install pyserial numpy")
    sys.exit(1)


DEFAULT_PORT = "/dev/cu.usbmodem00000000050C1"
DEFAULT_BAUD = 921600

SEND_DATA_FRAME = bytearray(
    [
        0x55, 0xAA, 0x1E, 0x03, 0x01, 0x00, 0x00, 0x00,
        0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    ]
)


def parse_int(value):
    return int(value, 0)


def send_damiao_frame(serial_device, can_id, data):
    frame = bytearray(SEND_DATA_FRAME)
    padded = data + bytes(8 - len(data))
    frame[13] = can_id & 0xFF
    frame[14] = (can_id >> 8) & 0xFF
    frame[18] = 0x08
    frame[21:29] = padded
    serial_device.write(frame)


def status_request_data(can_id):
    return bytes([can_id & 0xFF, (can_id >> 8) & 0xFF, 0xCC, 0, 0, 0, 0, 0])


def extract_packets(buffer):
    packets = []
    header = 0xAA
    tail = 0x55
    frame_length = 16
    i = 0
    remainder_pos = 0

    while i <= len(buffer) - frame_length:
        if buffer[i] == header and buffer[i + frame_length - 1] == tail:
            packets.append(buffer[i : i + frame_length])
            i += frame_length
            remainder_pos = i
        else:
            i += 1

    return packets, buffer[remainder_pos:]


def format_packet(packet):
    cmd = packet[1]
    can_id = (packet[6] << 24) | (packet[5] << 16) | (packet[4] << 8) | packet[3]
    data = packet[7:15]
    return f"CMD=0x{cmd:02X} ID=0x{can_id:X} DLC=8 DATA={data.hex().upper()}"


def parse_args():
    parser = argparse.ArgumentParser(
        description="Log raw frames from the DaMiao USB2CAN serial adapter."
    )
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument(
        "--raw",
        action="store_true",
        help="Print raw bytes in addition to parsed adapter packets.",
    )
    parser.add_argument(
        "--poll-status-id",
        type=parse_int,
        default=None,
        help="Actively poll motor status for this CAN ID while logging.",
    )
    parser.add_argument("--poll-period", type=float, default=0.2)
    return parser.parse_args()


def main():
    args = parse_args()

    print("Opening DaMiao USB2CAN serial adapter:")
    print(f"  port = {args.port}")
    print(f"  baud = {args.baud}")
    if args.poll_status_id is not None:
        print(f"  polling status for CAN ID 0x{args.poll_status_id:X}")
    print("Listening. Press Ctrl-C to stop.")

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
    except Exception as e:
        print(f"Could not open serial port: {e}")
        sys.exit(1)

    buffer = b""
    next_poll = time.monotonic()
    try:
        while True:
            if args.poll_status_id is not None and time.monotonic() >= next_poll:
                try:
                    send_damiao_frame(ser, 0x7FF, status_request_data(args.poll_status_id))
                except OSError as e:
                    if e.errno == errno.ENXIO:
                        print("Serial device disconnected or reset while polling.")
                        break
                    raise
                next_poll = time.monotonic() + args.poll_period

            try:
                chunk = ser.read_all()
            except OSError as e:
                if e.errno == errno.ENXIO:
                    print("Serial device disconnected or reset: [Errno 6] Device not configured")
                    print("Unplug/replug the adapter, then check CAN wiring, termination, motor power, and bitrate.")
                    break
                raise
            if not chunk:
                time.sleep(0.01)
                continue

            if args.raw:
                print("RAW:", chunk.hex(" ").upper())

            buffer += chunk
            packets, buffer = extract_packets(buffer)
            for packet in packets:
                print(f"{time.time():.6f} {format_packet(packet)}")
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()


if __name__ == "__main__":
    main()
