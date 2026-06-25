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


def parse_cansend_frame(frame):
    if "#" not in frame:
        raise ValueError("Frame must use cansend format, e.g. 001#FFFFFFFFFFFFFFFC")

    can_id_text, data_text = frame.split("#", 1)
    if not can_id_text:
        raise ValueError("Missing CAN ID before #")
    if len(data_text) % 2 != 0:
        raise ValueError("Data hex must contain an even number of characters")
    if len(data_text) > 16:
        raise ValueError("DaMiao classic frame data may contain at most 8 bytes")

    can_id = int(can_id_text, 16)
    data = bytes.fromhex(data_text)
    return can_id, data


def send_damiao_frame(serial_device, can_id, data):
    if len(data) > 8:
        raise ValueError("CAN data may contain at most 8 bytes")

    frame = bytearray(SEND_DATA_FRAME)
    padded = data + bytes(8 - len(data))
    frame[13] = can_id & 0xFF
    frame[14] = (can_id >> 8) & 0xFF
    frame[18] = 0x08
    frame[21:29] = padded
    serial_device.write(frame)


def extract_packets(buffer):
    packets = []
    header = 0xAA
    tail = 0x55
    frame_length = 16
    i = 0

    while i <= len(buffer) - frame_length:
        if buffer[i] == header and buffer[i + frame_length - 1] == tail:
            packets.append(buffer[i : i + frame_length])
            i += frame_length
        else:
            i += 1

    return packets


def format_packet(packet):
    cmd = packet[1]
    can_id = (packet[6] << 24) | (packet[5] << 16) | (packet[4] << 8) | packet[3]
    data = packet[7:15]
    return f"CMD=0x{cmd:02X} ID=0x{can_id:X} DLC=8 DATA={data.hex().upper()}"


def parse_args():
    parser = argparse.ArgumentParser(
        description="Send one raw CAN frame through the DaMiao USB2CAN serial protocol."
    )
    parser.add_argument("frame", help="cansend format, e.g. 001#FFFFFFFFFFFFFFFC")
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--repeat", type=int, default=1)
    parser.add_argument("--delay", type=float, default=0.02)
    parser.add_argument("--read-after", type=float, default=0.5)
    return parser.parse_args()


def main():
    args = parse_args()

    try:
        can_id, data = parse_cansend_frame(args.frame)
    except Exception as e:
        print(f"Invalid frame: {e}")
        sys.exit(1)

    print("Opening DaMiao USB2CAN serial adapter:")
    print(f"  port   = {args.port}")
    print(f"  baud   = {args.baud}")
    print(f"  CAN ID = 0x{can_id:X}")
    print(f"  data   = {data.hex().upper()}")
    if data == bytes.fromhex("FFFFFFFFFFFFFFFC"):
        print("  command = DaMiao enable")
    elif data == bytes.fromhex("FFFFFFFFFFFFFFFD"):
        print("  command = DaMiao disable")
    elif data == bytes.fromhex("FFFFFFFFFFFFFFFE"):
        print("  command = DaMiao set zero")

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.5)
    except Exception as e:
        print(f"Could not open serial port: {e}")
        sys.exit(1)

    try:
        for i in range(args.repeat):
            try:
                send_damiao_frame(ser, can_id, data)
            except OSError as e:
                if e.errno == errno.ENXIO:
                    print("Serial device disconnected or reset: [Errno 6] Device not configured")
                    print("Unplug/replug the adapter, then check CAN wiring, termination, motor power, and bitrate.")
                    sys.exit(1)
                raise
            print(f"Sent {i + 1}/{args.repeat}")
            if i + 1 < args.repeat:
                time.sleep(args.delay)

        if args.read_after > 0:
            time.sleep(args.read_after)
            try:
                reply = ser.read_all()
            except OSError as e:
                if e.errno == errno.ENXIO:
                    print("Serial device disconnected or reset while reading reply.")
                    sys.exit(1)
                raise
            if reply:
                print("Raw adapter reply:")
                print(reply.hex(" ").upper())
                for packet in extract_packets(reply):
                    print("Parsed:", format_packet(packet))
            else:
                print("No raw reply bytes.")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
