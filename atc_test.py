#!/usr/bin/env python
"""
Test and calibration script for ATC Feetech motors using LeRobot.

Motor IDs:
  1 = ATC lock mechanism  (sts3215 by default)
  2 = Tool motor 1        (scs0009 or sts3215)
  3 = Tool motor 2        (optional)
"""

import argparse
import json
import threading
import time
from contextlib import contextmanager
from pathlib import Path

from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus


SCS_MODELS = {"scs0009"}
CALIBRATION_FILE = Path("atc_calibration.json")
ATC_ID = 1
TOOL_IDS = [2, 3]
SCRIPT_VERSION = "2026-06-07-lerobot"
<<<<<<< HEAD
=======
MOVE_SETTLE_S = 0.8
>>>>>>> 0abb1f152c5f028ff2f1f01934f3e8df65d75d1d


def protocol_for(model):
    return 1 if model in SCS_MODELS else 0


def default_calibration(motor_id):
    return MotorCalibration(
        id=motor_id,
        drive_mode=0,
        homing_offset=0,
        range_min=0,
        range_max=4095,
    )


@contextmanager
def motor_bus(port, specs):
    """Open a LeRobot bus for motors that share the same Feetech protocol.

    specs is a mapping {name: (motor_id, model)}. Mixed protocol families cannot
    share one LeRobot bus, so callers should open one bus per family/action.
    """
    protocols = {protocol_for(model) for _, model in specs.values()}
    if len(protocols) != 1:
        raise ValueError("motor_bus specs must use one protocol family")

    bus = FeetechMotorsBus(
        port=port,
        motors={
            name: Motor(motor_id, model, MotorNormMode.RANGE_M100_100)
            for name, (motor_id, model) in specs.items()
        },
        calibration={
            name: default_calibration(motor_id)
            for name, (motor_id, _) in specs.items()
        },
        protocol_version=protocols.pop(),
    )
    bus.connect(handshake=False)
    try:
        yield bus
    finally:
        if bus.is_connected:
            bus.disconnect(disable_torque=False)


def read_pos(bus, name):
    return int(bus.read("Present_Position", name, normalize=False))


def write(bus, data_name, name, value, normalize=False):
    try:
        bus.write(data_name, name, value, normalize=normalize)
    except ConnectionError as exc:
        if "There is no status packet" not in str(exc):
            raise
        bus.sync_write(data_name, {name: value}, normalize=normalize)


def torque(bus, name, enable):
    try:
        write(bus, "Torque_Enable", name, 1 if enable else 0, normalize=False)
    except Exception as exc:
        state = "enable" if enable else "disable"
        print(f"  Warning: torque {state} failed for {name}: {exc}")


def move(bus, name, pos):
    write(bus, "Goal_Position", name, int(pos), normalize=False)


<<<<<<< HEAD
=======
def move_and_report(bus, name, pos):
    before = read_pos(bus, name)
    move(bus, name, pos)
    time.sleep(MOVE_SETTLE_S)
    after = read_pos(bus, name)
    print(f"  {name}: target={pos}  before={before}  after={after}")
    return before, after


>>>>>>> 0abb1f152c5f028ff2f1f01934f3e8df65d75d1d
def record_range(bus, name, motor_id):
    state = {"min": None, "max": None, "running": True, "reads": 0, "last_err": None}

    def poll():
        while state["running"]:
            try:
                pos = read_pos(bus, name)
                state["reads"] += 1
                state["min"] = pos if state["min"] is None else min(state["min"], pos)
                state["max"] = pos if state["max"] is None else max(state["max"], pos)
                print(f"\r    pos={pos}  min={state['min']}  max={state['max']}    ", end="", flush=True)
            except Exception as exc:
                state["last_err"] = exc
                print(f"\r    read failed: {exc}    ", end="", flush=True)
            time.sleep(0.1)
        print()

    thread = threading.Thread(target=poll, daemon=True)
    thread.start()
    input("    Move through the full range of motion, then press ENTER to stop...")
    state["running"] = False
    thread.join(timeout=1.0)

    if state["reads"] == 0:
        raise RuntimeError(
            f"No position read from ID {motor_id} (last error: {state['last_err']}). "
            f"Check the motor is powered, on the bus, and set to ID {motor_id}."
        )
    return state["min"], state["max"]


def load_calibration():
    if not CALIBRATION_FILE.exists():
        return {}
    return json.loads(CALIBRATION_FILE.read_text())


def save_calibration(data):
    CALIBRATION_FILE.write_text(json.dumps(data, indent=2))
    print(f"Calibration saved to {CALIBRATION_FILE}")


def calibrate_atc(port, atc_model):
    print("\n--- ATC Lock Calibration ---")
    with motor_bus(port, {"atc": (ATC_ID, atc_model)}) as bus:
        torque(bus, "atc", False)
        input("  Move to the LOCKED position, then press ENTER...")
        locked = read_pos(bus, "atc")
        print(f"  Locked position: {locked}")

        input("  Move to the UNLOCKED position, then press ENTER...")
        unlocked = read_pos(bus, "atc")
        print(f"  Unlocked position: {unlocked}")

    print("  ATC calibration done.")
    return {"locked": locked, "unlocked": unlocked}


def calibrate_tool(port, tool_name, tool_model, num_motors):
    print(f"\n--- Tool Motor Calibration: '{tool_name}' ({tool_model}) ---")
    specs = {f"tool_{i + 1}": (TOOL_IDS[i], tool_model) for i in range(num_motors)}
    ranges = {}
    with motor_bus(port, specs) as bus:
        for name, (motor_id, _) in specs.items():
            torque(bus, name, False)
            print(f"\n  Calibrating {name} (ID {motor_id})...")
            mn, mx = record_range(bus, name, motor_id)
            print(f"  Range: min={mn}, max={mx}")
            ranges[name] = {"min": mn, "max": mx}

    print("  Tool calibration done.")
    return {"model": tool_model, "ranges": ranges}


def run_motor_action(port, specs, action):
    with motor_bus(port, specs) as bus:
        return action(bus)


def interactive(port, atc_model, tool_name):
    cal = load_calibration()
    if not cal:
        print("No calibration file found. Run with --calibrate first.")
        return

    atc_cal = cal.get("atc", {})
    tool_cfg = cal.get("tools", {}).get(tool_name)
    if tool_cfg:
        tool_model = tool_cfg["model"]
        tool_cal = tool_cfg["ranges"]
        num_motors = len(tool_cal)
    else:
        print(f"Tool '{tool_name}' is not calibrated. ATC-only control available.")
        print(f"Calibrate it with: --tool {tool_name} --tool-model MODEL --calibrate tool")
        tool_model = None
        tool_cal = {}
        num_motors = 0

    tool_specs = {
        f"tool_{i + 1}": (TOOL_IDS[i], tool_model)
        for i in range(num_motors)
    } if tool_model else {}

    print(f"\nActive tool: '{tool_name}'" + (f" ({tool_model}, {num_motors} motor(s))" if tool_model else " (none)"))
    print("\nCommands:")
    print("  l  =  Lock ATC")
    print("  u  =  Unlock ATC")
    print("  a  =  Activate tool  (move to range max)")
    print("  h  =  Home tool      (move to range min)")
<<<<<<< HEAD
=======
    print("  p  =  Print positions")
>>>>>>> 0abb1f152c5f028ff2f1f01934f3e8df65d75d1d
    print("  q  =  Quit")

    while True:
        try:
            cmd = input("\n> ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            break

        if cmd == "q":
            break
        if cmd == "l":
            if "locked" not in atc_cal:
                print("ATC not calibrated.")
                continue
            def lock_atc(bus):
                torque(bus, "atc", True)
<<<<<<< HEAD
                move(bus, "atc", atc_cal["locked"])
=======
                move_and_report(bus, "atc", atc_cal["locked"])
>>>>>>> 0abb1f152c5f028ff2f1f01934f3e8df65d75d1d

            run_motor_action(
                port,
                {"atc": (ATC_ID, atc_model)},
                lock_atc,
            )
<<<<<<< HEAD
            print(f"  Locking ATC -> {atc_cal['locked']}")
=======
>>>>>>> 0abb1f152c5f028ff2f1f01934f3e8df65d75d1d
        elif cmd == "u":
            if "unlocked" not in atc_cal:
                print("ATC not calibrated.")
                continue
            def unlock_atc(bus):
                torque(bus, "atc", True)
<<<<<<< HEAD
                move(bus, "atc", atc_cal["unlocked"])
=======
                move_and_report(bus, "atc", atc_cal["unlocked"])
>>>>>>> 0abb1f152c5f028ff2f1f01934f3e8df65d75d1d

            run_motor_action(
                port,
                {"atc": (ATC_ID, atc_model)},
                unlock_atc,
            )
<<<<<<< HEAD
            print(f"  Unlocking ATC -> {atc_cal['unlocked']}")
=======
>>>>>>> 0abb1f152c5f028ff2f1f01934f3e8df65d75d1d
        elif cmd in ("a", "h"):
            if not tool_specs:
                print("  No tool motors configured.")
                continue

            def action(bus):
                for name in tool_specs:
                    if name not in tool_cal:
                        print(f"  {name} not calibrated.")
                        continue
                    torque(bus, name, True)
                    pos = tool_cal[name]["max"] if cmd == "a" else tool_cal[name]["min"]
<<<<<<< HEAD
                    move(bus, name, pos)
                    print(f"  {name} -> {pos}")

            run_motor_action(port, tool_specs, action)
=======
                    move_and_report(bus, name, pos)

            run_motor_action(port, tool_specs, action)
        elif cmd == "p":
            if atc_cal:
                run_motor_action(
                    port,
                    {"atc": (ATC_ID, atc_model)},
                    lambda bus: print(f"  atc position: {read_pos(bus, 'atc')}"),
                )
            if tool_specs:
                def print_tools(bus):
                    for name in tool_specs:
                        print(f"  {name} position: {read_pos(bus, name)}")

                run_motor_action(port, tool_specs, print_tools)
>>>>>>> 0abb1f152c5f028ff2f1f01934f3e8df65d75d1d
        else:
            print("  Unknown command.")


def main():
    parser = argparse.ArgumentParser(description="ATC motor test and calibration")
    parser.add_argument("--port", required=True, help="Serial port, e.g. /dev/ttyACM1")
    parser.add_argument("--atc-model", default="sts3215", help="ATC motor model (default: sts3215)")
    parser.add_argument("--tool", default="default",
                        help="Tool configuration name (default: 'default'). Each tool "
                             "stores its own model, motor count and ranges.")
    parser.add_argument("--tool-model", default="scs0009",
                        help="Tool motor model when calibrating: scs0009 or sts3215 "
                             "(default: scs0009). Ignored in interactive mode.")
    parser.add_argument(
        "--motors",
        type=int,
        choices=[1, 2],
        default=1,
        help="Number of tool motors when calibrating (default: 1)",
    )
    parser.add_argument(
        "--calibrate",
        choices=["atc", "tool", "all"],
        help="Calibrate: atc = lock positions only, tool = tool range only, all = both",
    )
    args = parser.parse_args()

    print("ATC Motor Test")
    print(f"Script: {SCRIPT_VERSION}")

    if args.calibrate:
        cal = load_calibration()
        if args.calibrate in ("atc", "all"):
            cal["atc"] = calibrate_atc(args.port, args.atc_model)
        if args.calibrate in ("tool", "all"):
            cal.setdefault("tools", {})[args.tool] = calibrate_tool(
                args.port, args.tool, args.tool_model, args.motors)
        save_calibration(cal)
    else:
        interactive(args.port, args.atc_model, args.tool)


if __name__ == "__main__":
    main()
