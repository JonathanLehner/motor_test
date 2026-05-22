#!/usr/bin/env python
"""Open/close a single motor (gripper default)."""

import time
from lerobot.motors.feetech import FeetechMotorsBus
from lerobot.motors import Motor, MotorNormMode, MotorCalibration
from lerobot.motors.motors_bus import get_address

PORT = "/dev/cu.usbmodem5AB01831481"
MOTOR_ID = 1
MOTOR_MODEL = "sts3215"
MOTOR_NAME = "gripper"

# Empirical gripper range in normalized units (0-100 scale from calibration).
GRIPPER_OPEN_N = 50.0
GRIPPER_CLOSED_N = 90.0
OPEN_CLOSE_DELAY_S = 0.6
ACCELERATION = 80
TORQUE_LIMIT = 600
STEPS = 12
STEP_DELAY_S = 0.05

print(f"Open/close motor ID {MOTOR_ID} ({MOTOR_MODEL}) - {MOTOR_NAME}")
print("=" * 60)

norm_mode = MotorNormMode.RANGE_0_100 if MOTOR_NAME == "gripper" else MotorNormMode.RANGE_M100_100

calibration = {
    MOTOR_NAME: MotorCalibration(
        id=MOTOR_ID,
        drive_mode=0,
        homing_offset=0,
        range_min=0,
        range_max=4095,
    )
}

bus = FeetechMotorsBus(
    port=PORT,
    motors={MOTOR_NAME: Motor(MOTOR_ID, MOTOR_MODEL, norm_mode)},
    calibration=calibration,
)

try:
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

    def write_with_fallback(data_name: str, value):
        try:
            bus.write(data_name, MOTOR_NAME, value)
        except ConnectionError as e:
            if "There is no status packet" not in str(e):
                raise
            print(f"⚠ No status packet on write '{data_name}'. Falling back to sync_write.")
            bus.sync_write(data_name, {MOTOR_NAME: value}, normalize=True)

    ensure_status_packets()

    print("\nEnabling torque...")
    write_with_fallback("Torque_Enable", 1)
    print("✓ Torque enabled")
    try:
        write_with_fallback("Acceleration", ACCELERATION)
        write_with_fallback("Torque_Limit", TORQUE_LIMIT)
    except Exception as e:
        print(f"⚠ Could not set acceleration/torque limits: {e}")

    print("\nOpen/close loop (Ctrl+C to stop):")
    while True:
        if MOTOR_NAME == "gripper":
            def ramp(a, b):
                for i in range(1, STEPS + 1):
                    v = a + (b - a) * (i / STEPS)
                    write_with_fallback("Goal_Position", v)
                    time.sleep(STEP_DELAY_S)

            print(f"Opening gripper ({GRIPPER_OPEN_N})...")
            ramp(GRIPPER_CLOSED_N, GRIPPER_OPEN_N)
            time.sleep(OPEN_CLOSE_DELAY_S)

            print(f"Closing gripper ({GRIPPER_CLOSED_N})...")
            ramp(GRIPPER_OPEN_N, GRIPPER_CLOSED_N)
            time.sleep(OPEN_CLOSE_DELAY_S)
        else:
            print("Moving to -50...")
            write_with_fallback("Goal_Position", -50.0)
            time.sleep(OPEN_CLOSE_DELAY_S)

            print("Moving to +50...")
            write_with_fallback("Goal_Position", 50.0)
            time.sleep(OPEN_CLOSE_DELAY_S)

    try:
        pos = bus.read("Present_Position", MOTOR_NAME)
        print(f"\nFinal position: {pos:.2f}")
    except (RuntimeError, ConnectionError) as e:
        print(f"⚠ Final position read failed: {e}")
        motor_id = bus.motors[MOTOR_NAME].id
        model = bus.motors[MOTOR_NAME].model
        addr, length = get_address(bus.model_ctrl_table, model, "Present_Position")
        value, comm, error = bus._read(addr, length, motor_id, raise_on_error=False, err_msg="")
        print(
            "Raw read -> "
            f"value={value} comm={bus.packet_handler.getTxRxResult(comm)} "
            f"error={bus.packet_handler.getRxPacketError(error)}"
        )

    print("\nDisabling torque...")
    write_with_fallback("Torque_Enable", 0)

except Exception as e:
    print(f"✗ Error: {e}")
    import traceback
    traceback.print_exc()

finally:
    if bus.is_connected:
        try:
            bus.disconnect(disable_torque=False)
            print("✓ Disconnected")
        except Exception as e:
            print(f"⚠ Disconnect error: {e}")

print("=" * 60)
print("Done")
