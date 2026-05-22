#!/usr/bin/env python
"""Test script to control a single motor."""

import time
from lerobot.motors.feetech import FeetechMotorsBus
from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.motors_bus import get_address

PORT = "/dev/cu.usbmodem5AB01831481"
MOTOR_ID = 1  # Gripper motor
MOTOR_MODEL = "sts3215"
MOTOR_NAME = "gripper"

# Empirical gripper range in normalized units (0-100 scale from calibration).
GRIPPER_OPEN_N = 50.0
GRIPPER_CLOSED_N = 90.0

print(f"Testing motor ID {MOTOR_ID} ({MOTOR_MODEL}) - {MOTOR_NAME}")
print("=" * 60)

# Create motor bus with the specified motor
# Use RANGE_0_100 for gripper, DEGREES or RANGE_M100_100 for other motors
from lerobot.motors import MotorCalibration
norm_mode = MotorNormMode.RANGE_0_100 if MOTOR_NAME == "gripper" else MotorNormMode.RANGE_M100_100

# Simple calibration (using default values)
# STS3215 has 4096 positions (0-4095), center is 2048
calibration = {
    MOTOR_NAME: MotorCalibration(
        id=MOTOR_ID,
        drive_mode=0,
        homing_offset=0,
        range_min=0,      # Minimum position
        range_max=4095,   # Maximum position (STS3215 has 4096 steps)
    )
}

bus = FeetechMotorsBus(
    port=PORT,
    motors={
        MOTOR_NAME: Motor(MOTOR_ID, MOTOR_MODEL, norm_mode),
    },
    calibration=calibration,
)

try:
    # Connect to the motor
    print("Connecting to motor...")
    bus.connect(handshake=False)
    print("✓ Connected!")

    def ensure_status_packets() -> bool:
        try:
            _ = bus.read("Present_Voltage", MOTOR_NAME, normalize=False)
            return True
        except RuntimeError as e:
            if "Overload error" in str(e):
                print("⚠ Motor reports overload. Continuing, but movement may be blocked.")
                return True
            raise
        except ConnectionError as e:
            if "There is no status packet" not in str(e):
                raise
            print("⚠ No status packets. Trying to enable Response_Status_Level=2 via sync_write...")
            try:
                bus.sync_write("Lock", {MOTOR_NAME: 0}, normalize=False)
                bus.sync_write("Response_Status_Level", {MOTOR_NAME: 2}, normalize=False)
                time.sleep(0.05)
            except Exception as sync_err:
                print(f"✗ Failed to set Response_Status_Level: {sync_err}")
                return False
            return True

    ensure_status_packets()

    def write_with_fallback(data_name: str, value):
        try:
            bus.write(data_name, MOTOR_NAME, value)
        except RuntimeError as e:
            if "Overload error" in str(e):
                print(f"⚠ Overload error on write '{data_name}'. Skipping command.")
                return
            raise
        except ConnectionError as e:
            if "There is no status packet" not in str(e):
                raise
            print(f"⚠ No status packet on write '{data_name}'. Falling back to sync_write.")
            bus.sync_write(data_name, {MOTOR_NAME: value}, normalize=True)

    def write_with_reverse_on_overload(data_name: str, value, reverse_value):
        try:
            bus.write(data_name, MOTOR_NAME, value)
            return
        except RuntimeError as e:
            if "Overload error" in str(e):
                print(f"⚠ Overload on '{data_name}' -> trying reverse to {reverse_value}.")
                write_with_fallback(data_name, reverse_value)
                return
            raise
        except ConnectionError as e:
            if "There is no status packet" not in str(e):
                raise
            print(f"⚠ No status packet on write '{data_name}'. Falling back to sync_write.")
            bus.sync_write(data_name, {MOTOR_NAME: value}, normalize=True)

    # Apply conservative settings to reduce overload risk
    try:
        print("Setting conservative motion limits...")
        write_with_fallback("Acceleration", 10)
        write_with_fallback("Torque_Limit", 300)
        write_with_fallback("Operating_Mode", 0)  # position mode
    except Exception as e:
        print(f"⚠ Could not set conservative limits: {e}")

    # Read current position (fall back to raw read if the motor reports an error)
    # Also read present voltage to diagnose "Input voltage error".
    try:
        voltage = bus.read("Present_Voltage", MOTOR_NAME, normalize=False)
        print(f"Present voltage (raw): {voltage}")
    except (RuntimeError, ConnectionError) as e:
        print(f"⚠ Voltage read failed: {e}")
        motor_id = bus.motors[MOTOR_NAME].id
        model = bus.motors[MOTOR_NAME].model
        addr, length = get_address(bus.model_ctrl_table, model, "Present_Voltage")
        value, comm, error = bus._read(addr, length, motor_id, raise_on_error=False, err_msg="")
        print(
            "Raw voltage read -> "
            f"value={value} comm={bus.packet_handler.getTxRxResult(comm)} "
            f"error={bus.packet_handler.getRxPacketError(error)}"
        )

    pos = None
    try:
        pos = bus.read("Present_Position", MOTOR_NAME)
        print(f"Current position (normalized): {pos:.2f}")
        raw_pos = bus.read("Present_Position", MOTOR_NAME, normalize=False)
        print(f"Current position (raw): {raw_pos}")
    except (RuntimeError, ConnectionError) as e:
        if "Overload error" in str(e):
            print("⚠ Overload on position read; continuing without position.")
        else:
            print(f"⚠ Read failed: {e}")
        motor_id = bus.motors[MOTOR_NAME].id
        model = bus.motors[MOTOR_NAME].model
        addr, length = get_address(bus.model_ctrl_table, model, "Present_Position")
        value, comm, error = bus._read(addr, length, motor_id, raise_on_error=False, err_msg="")
        print(f"Raw read -> value={value} comm={bus.packet_handler.getTxRxResult(comm)} error={bus.packet_handler.getRxPacketError(error)}")

    # Enable torque
    print("\nEnabling torque...")
    write_with_fallback("Torque_Enable", 1)
    print("✓ Torque enabled")

    # Move to different positions
    print("\nTesting movement:")
    if MOTOR_NAME == "gripper":  # Gripper uses 0-100
        if pos is not None and pos <= GRIPPER_OPEN_N + 1.0:
            print(f"Gripper appears already open (<= {GRIPPER_OPEN_N + 1.0:.1f}).")
        else:
            print("Opening gripper (0%)...")
            write_with_reverse_on_overload(
                "Goal_Position",
                GRIPPER_OPEN_N,
                min(GRIPPER_OPEN_N + 5.0, GRIPPER_CLOSED_N),
            )
            time.sleep(2)

        # Use a gentle close to avoid overload when already near-open
        print("Gentle close (20%)...")
        gentle = GRIPPER_OPEN_N + 0.2 * (GRIPPER_CLOSED_N - GRIPPER_OPEN_N)
        write_with_reverse_on_overload("Goal_Position", gentle, GRIPPER_OPEN_N)
        time.sleep(2)

        print("Opening gripper (0%)...")
        write_with_reverse_on_overload("Goal_Position", GRIPPER_OPEN_N, gentle)
        time.sleep(2)

        # Raw small-step test around current position
        try:
            raw_pos = bus.read("Present_Position", MOTOR_NAME, normalize=False)
            step = 150
            raw_min = max(0, raw_pos - step)
            raw_max = min(4095, raw_pos + step)
            print(f"Raw nudge to {raw_min} then {raw_max} (from {raw_pos})...")
            bus.write("Goal_Position", MOTOR_NAME, raw_min, normalize=False)
            time.sleep(1.5)
            bus.write("Goal_Position", MOTOR_NAME, raw_max, normalize=False)
            time.sleep(1.5)
        except Exception as e:
            print(f"⚠ Raw nudge failed: {e}")
    else:  # Other motors use -100 to 100
        print("Moving to -50...")
        write_with_fallback("Goal_Position", -50.0)
        time.sleep(2)

        print("Moving to +50...")
        write_with_fallback("Goal_Position", 50.0)
        time.sleep(2)

        print("Moving to 0...")
        write_with_fallback("Goal_Position", 0.0)
        time.sleep(2)

    # Read final position
    pos = bus.read("Present_Position", MOTOR_NAME)
    print(f"\nFinal position: {pos:.2f}")

    # Interactive mode
    print("\n" + "=" * 60)
    if MOTOR_NAME == "gripper":
        print("Interactive mode - Enter position (0-100) or 'q' to quit:")
        range_str = "0-100"
    else:
        print("Interactive mode - Enter position (-100 to 100) or 'q' to quit:")
        range_str = "-100 to 100"
    print("=" * 60)

    while True:
        try:
            user_input = input(f"Position ({range_str}): ").strip()
            if user_input.lower() == 'q':
                break

            position = float(user_input)
            min_pos = 0 if MOTOR_NAME == "gripper" else -100
            max_pos = 100
            if min_pos <= position <= max_pos:
                if MOTOR_NAME == "gripper":
                    mapped = GRIPPER_OPEN_N + (position / 100.0) * (GRIPPER_CLOSED_N - GRIPPER_OPEN_N)
                    write_with_fallback("Goal_Position", mapped)
                    print(f"✓ Moving to {position} (mapped {mapped:.2f})")
                else:
                    write_with_fallback("Goal_Position", position)
                    print(f"✓ Moving to {position}")
                # Telemetry after command (helps diagnose current/overload limits)
                try:
                    cur = bus.read("Present_Current", MOTOR_NAME, normalize=False)
                    load = bus.read("Present_Load", MOTOR_NAME, normalize=False)
                    temp = bus.read("Present_Temperature", MOTOR_NAME, normalize=False)
                    stat = bus.read("Status", MOTOR_NAME, normalize=False)
                    mv = bus.read("Moving", MOTOR_NAME, normalize=False)
                    print(
                        "Telemetry -> "
                        f"current={cur} load={load} temp={temp} status={stat} moving={mv}"
                    )
                except Exception as e:
                    print(f"⚠ Telemetry read failed: {e}")
            else:
                print(f"⚠ Position must be between {min_pos} and {max_pos}")
        except ValueError:
            print("⚠ Invalid input. Enter a number or 'q' to quit")
        except KeyboardInterrupt:
            print("\n\nStopping...")
            break

    # Disable torque before exit
    print("\nDisabling torque...")
    write_with_fallback("Torque_Enable", 0)

except Exception as e:
    print(f"✗ Error: {e}")
    import traceback
    traceback.print_exc()

finally:
    # Disconnect
    if bus.is_connected:
        try:
            bus.disconnect(disable_torque=False)
            print("✓ Disconnected")
        except Exception as e:
            print(f"⚠ Disconnect error: {e}")

print("=" * 60)
print("Test complete")
