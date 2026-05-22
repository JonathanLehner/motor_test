#!/usr/bin/env python
"""Simple script to scan for Feetech motors on all baudrates and IDs."""

import sys
from lerobot.motors.feetech import FeetechMotorsBus

PORT = "/dev/cu.usbmodem5AB01831481"
BAUDRATES = [1_000_000, 500_000, 250_000, 128_000, 115_200, 57_600, 38_400, 19_200]

print(f"Scanning for Feetech motors on {PORT}...")
print("=" * 60)

for baudrate in BAUDRATES:
    print(f"\nTrying baudrate: {baudrate}...")
    try:
        # Create a minimal bus with a dummy motor
        from lerobot.motors import Motor, MotorNormMode
        bus = FeetechMotorsBus(
            port=PORT,
            motors={"test": Motor(1, "sts3215", MotorNormMode.RANGE_0_100)},
        )
        bus.connect(handshake=False)
        bus.set_baudrate(baudrate)

        # Try broadcast ping
        result = bus.broadcast_ping()
        if result:
            print(f"✓ FOUND MOTOR(S) at {baudrate}!")
            for motor_id, model_number in result.items():
                print(f"  ID: {motor_id}, Model Number: {model_number}")
        else:
            print(f"  No response")

        bus.disconnect()
    except Exception as e:
        print(f"  Error: {e}")

print("\n" + "=" * 60)
print("Scan complete.")
