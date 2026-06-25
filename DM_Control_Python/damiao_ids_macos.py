#!/usr/bin/env python3
"""
Damiao motor ID read/write helper for macOS using the Damiao USB2CAN/USB2CANFD serial adapter.

Default port for your Mac:
    /dev/cu.usbmodem00000000050C1

Usage:

1) Scan/read IDs:
    python damiao_ids_macos.py

2) Read one known motor ID:
    python damiao_ids_macos.py --current-id 1

3) Change motor CAN_ID from 1 to 2 and Master ID to 0x12:
    python damiao_ids_macos.py --current-id 1 --set-can-id 2 --set-master-id 0x12 --write

Notes:
- ESC_ID / CAN_ID is register 8.
- MST_ID / Master ID is register 7.
- Recommended pair: CAN_ID=0x01 -> Master_ID=0x11, CAN_ID=0x02 -> Master_ID=0x12, etc.
"""

import argparse
import sys
import time

try:
    import serial
except ImportError:
    print("Missing dependency: pyserial")
    print("Install with: pip install pyserial numpy")
    sys.exit(1)

try:
    from DM_CAN import *
except ImportError:
    print("Could not import DM_CAN.py.")
    print("Put this script in the same folder as DM_CAN.py from DM_Control_Python.")
    sys.exit(1)


DEFAULT_PORT = "/dev/cu.usbmodem00000000050C1"
DEFAULT_BAUD = 921600


def parse_int(value: str) -> int:
    """Accept decimal or hex, e.g. 2 or 0x02."""
    return int(value, 0)


def get_motor_type(name: str):
    """
    Map CLI string to DM_Motor_Type enum.
    Default is DM4310.
    """
    name = name.upper().replace("-", "").replace("_", "")

    aliases = {
        "4310": "DM4310",
        "DM4310": "DM4310",
        "431048": "DM4310_48V",
        "431048V": "DM4310_48V",
        "DM431048": "DM4310_48V",
        "DM431048V": "DM4310_48V",
        "4340": "DM4340",
        "DM4340": "DM4340",
        "434048": "DM4340_48V",
        "434048V": "DM4340_48V",
        "DM434048": "DM4340_48V",
        "DM434048V": "DM4340_48V",
        "6006": "DM6006",
        "DM6006": "DM6006",
        "8006": "DM8006",
        "DM8006": "DM8006",
        "8009": "DM8009",
        "DM8009": "DM8009",
        "10010L": "DM10010L",
        "DM10010L": "DM10010L",
        "10010": "DM10010",
        "DM10010": "DM10010",
        "H3510": "DMH3510",
        "DMH3510": "DMH3510",
        "H6215": "DMH6215",
        "DMH6215": "DMH6215",
        "G6220": "DMG6220",
        "DMG6220": "DMG6220",
    }

    enum_name = aliases.get(name, name)

    if not hasattr(DM_Motor_Type, enum_name):
        print(f"Unknown motor type: {name}")
        print("Try: 4310, 4310-48V, 4340, 4340-48V, 6006, 8006, 8009, 10010L, 10010, H3510, H6215, G6220.")
        print("Available DM_Motor_Type values:")
        for x in DM_Motor_Type:
            print(f"  {x.name}")
        sys.exit(1)

    return getattr(DM_Motor_Type, enum_name)


def make_motor(motor_type, can_id: int, master_id: int):
    return Motor(motor_type, can_id, master_id)


def read_ids(ctrl, motor):
    """
    Read ESC_ID and MST_ID.
    Returns (esc_id, mst_id). Either can be None if no reply.
    """
    esc = ctrl.read_motor_param(motor, DM_variable.ESC_ID)
    time.sleep(0.05)
    mst = ctrl.read_motor_param(motor, DM_variable.MST_ID)
    time.sleep(0.05)
    return esc, mst


def scan(ctrl, motor_type, start_id: int, end_id: int):
    print(f"\nScanning CAN_ID 0x{start_id:02X} to 0x{end_id:02X} ...\n")

    found = []

    for can_id in range(start_id, end_id + 1):
        assumed_master = can_id + 0x10

        motor = make_motor(motor_type, can_id, assumed_master)
        ctrl.addMotor(motor)

        try:
            esc, mst = read_ids(ctrl, motor)
        except Exception as e:
            print(f"0x{can_id:02X}: error: {e}")
            continue

        if esc is not None or mst is not None:
            print(
                f"FOUND command ID 0x{can_id:02X}: "
                f"ESC_ID/CAN_ID={fmt_id(esc)}, MST_ID/Master_ID={fmt_id(mst)}"
            )
            found.append((can_id, esc, mst))
        else:
            print(f"0x{can_id:02X}: no parameter reply")

    print("\nScan done.")
    if not found:
        print("No motors replied to parameter read.")
        print("But if the motor moves, check that no other program has the serial port open.")
        print("Also verify DM_CAN.py is the cmjang DM_Control_Python version.")
    return found


def fmt_id(value):
    if value is None:
        return "None"
    try:
        return f"0x{int(value):02X} ({int(value)})"
    except Exception:
        return str(value)


def write_ids(ctrl, motor_type, current_id: int, current_master: int, new_can_id, new_master_id):
    print("\nWARNING: Writing IDs changes motor flash parameters.")
    print(f"Current assumed CAN_ID:    {fmt_id(current_id)}")
    print(f"Current assumed Master ID: {fmt_id(current_master)}")
    print(f"New CAN_ID:                {fmt_id(new_can_id) if new_can_id is not None else 'unchanged'}")
    print(f"New Master ID:             {fmt_id(new_master_id) if new_master_id is not None else 'unchanged'}")

    motor = make_motor(motor_type, current_id, current_master)
    ctrl.addMotor(motor)

    print("\nReading before write...")
    esc_before, mst_before = read_ids(ctrl, motor)
    print(f"Before: ESC_ID/CAN_ID={fmt_id(esc_before)}, MST_ID/Master_ID={fmt_id(mst_before)}")

    if esc_before is None and mst_before is None:
        print("\nNo parameter reply from the current CAN_ID/Master_ID.")
        print("Not writing. Verify the current IDs first, or scan the bus.")
        print(f"Try: python damiao_ids_macos.py --scan-start 0x01 --scan-end 0x10")
        return

    active_can_id = current_id
    active_master_id = current_master
    writes_ok = True

    if new_master_id is not None:
        if new_master_id == 0:
            print("Refusing to set Master ID to 0x00.")
            sys.exit(1)

        print(f"\nWriting MST_ID / Master ID -> {fmt_id(new_master_id)}")
        ok = ctrl.change_motor_param(motor, DM_variable.MST_ID, int(new_master_id))
        print("MST_ID write:", "OK" if ok else "FAILED/NO ACK")
        writes_ok = writes_ok and ok

        if ok:
            active_master_id = int(new_master_id)
            motor = make_motor(motor_type, active_can_id, active_master_id)
            ctrl.addMotor(motor)
            time.sleep(0.2)

    if new_can_id is not None:
        if new_can_id == 0:
            print("Refusing to set CAN_ID to 0x00.")
            sys.exit(1)

        print(f"\nWriting ESC_ID / CAN_ID -> {fmt_id(new_can_id)}")
        ok = ctrl.change_motor_param(motor, DM_variable.ESC_ID, int(new_can_id))
        print("ESC_ID write:", "OK" if ok else "FAILED/NO ACK")
        writes_ok = writes_ok and ok

        if ok:
            active_can_id = int(new_can_id)
            motor = make_motor(motor_type, active_can_id, active_master_id)
            ctrl.addMotor(motor)
            time.sleep(0.2)

    if not writes_ok:
        print("\nOne or more writes failed. Not saving parameters to flash.")
        print("The motor IDs should be treated as unchanged.")
        return

    print("\nSaving parameters to flash...")
    try:
        ctrl.save_motor_param(motor)
        print("Save command sent.")
    except Exception as e:
        print(f"Save command error: {e}")

    time.sleep(0.5)

    print("\nReading after write...")
    esc_after, mst_after = read_ids(ctrl, motor)
    print(f"After: ESC_ID/CAN_ID={fmt_id(esc_after)}, MST_ID/Master_ID={fmt_id(mst_after)}")

    print("\nSuggested next command:")
    print(
        f"python damiao_ids_macos.py --current-id 0x{active_can_id:02X} "
        f"--current-master 0x{active_master_id:02X}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--motor-type", default="4310")

    parser.add_argument("--scan-start", type=parse_int, default=0x01)
    parser.add_argument("--scan-end", type=parse_int, default=0x10)

    parser.add_argument("--current-id", type=parse_int, default=None)
    parser.add_argument("--current-master", type=parse_int, default=None)

    parser.add_argument("--set-can-id", type=parse_int, default=None)
    parser.add_argument("--set-master-id", type=parse_int, default=None)
    parser.add_argument("--write", action="store_true")

    args = parser.parse_args()

    motor_type = get_motor_type(args.motor_type)

    print("Opening serial adapter:")
    print(f"  port: {args.port}")
    print(f"  baud: {args.baud}")
    print(f"  motor type: {args.motor_type}")

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.5)
    except Exception as e:
        print(f"Could not open serial port: {e}")
        print("\nCheck with:")
        print("  ls /dev/cu.*")
        sys.exit(1)

    try:
        ctrl = MotorControl(ser)

        if args.write:
            if args.current_id is None:
                print("For writing, provide --current-id.")
                sys.exit(1)

            current_master = args.current_master
            if current_master is None:
                current_master = args.current_id + 0x10

            write_ids(
                ctrl=ctrl,
                motor_type=motor_type,
                current_id=args.current_id,
                current_master=current_master,
                new_can_id=args.set_can_id,
                new_master_id=args.set_master_id,
            )

        elif args.current_id is not None:
            current_master = args.current_master
            if current_master is None:
                current_master = args.current_id + 0x10

            motor = make_motor(motor_type, args.current_id, current_master)
            ctrl.addMotor(motor)

            print("\nReading one motor...")
            print(f"Assumed CAN_ID:    {fmt_id(args.current_id)}")
            print(f"Assumed Master ID: {fmt_id(current_master)}")

            esc, mst = read_ids(ctrl, motor)
            print(f"ESC_ID / CAN_ID:        {fmt_id(esc)}")
            print(f"MST_ID / Master ID:     {fmt_id(mst)}")

        else:
            scan(ctrl, motor_type, args.scan_start, args.scan_end)

    finally:
        try:
            ser.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
