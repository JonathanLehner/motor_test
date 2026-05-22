#!/usr/bin/env python
"""Test script to verify basic communication with Waveshare board."""

import serial
import time

PORT = "/dev/tty.usbmodem5AB01814711"
BAUDRATE = 1_000_000

print("Testing Waveshare board communication...")
print(f"Port: {PORT}")
print(f"Baudrate: {BAUDRATE}")
print("=" * 60)

try:
    # Open serial port
    ser = serial.Serial(PORT, BAUDRATE, timeout=0.5)
    print(f"✓ Serial port opened successfully")
    print(f"  Port: {ser.name}")
    print(f"  Baudrate: {ser.baudrate}")
    print(f"  Timeout: {ser.timeout}s")

    # Try to send a ping command (Feetech protocol)
    # Ping command format: [0xFF, 0xFF, ID, Length, Instruction, Checksum]
    # Broadcast ID = 0xFE
    ping_cmd = bytes([0xFF, 0xFF, 0xFE, 0x02, 0x01, 0x00])  # Broadcast ping

    print(f"\nSending broadcast ping command: {ping_cmd.hex()}")
    ser.write(ping_cmd)

    # Wait for response
    time.sleep(0.1)

    # Check if any data received
    if ser.in_waiting > 0:
        response = ser.read(ser.in_waiting)
        print(f"✓ RECEIVED RESPONSE: {response.hex()}")
        print(f"  Length: {len(response)} bytes")
        print("  This suggests the board and motor are communicating!")
    else:
        print("✗ No response received")
        print("\nPossible issues:")
        print("  1. Motor not powered (check external power supply)")
        print("  2. Motor not connected to data line")
        print("  3. Wrong baudrate (motor configured differently)")
        print("  4. Board TX/RX not connected properly")

    ser.close()
    print("\n✓ Test complete")

except serial.SerialException as e:
    print(f"✗ Serial port error: {e}")
except Exception as e:
    print(f"✗ Error: {e}")

print("=" * 60)
