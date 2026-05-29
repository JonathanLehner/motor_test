#!/usr/bin/env python
"""
Test and calibration script for ATC Feetech motors.

Motor IDs:
  1 = ATC lock mechanism
  2 = Tool motor 1
  3 = Tool motor 2 (optional)

Calibration is saved to atc_calibration.json in the current directory.

Usage:
  # Calibrate ATC lock and tool motors
  python atc_test.py --port /dev/ttyACM1 --calibrate

  # Interactive: lock/unlock ATC, activate/home tool
  python atc_test.py --port /dev/ttyACM1

  # With 2 tool motors and scs0009 model
  python atc_test.py --port /dev/ttyACM1 --model scs0009 --motors 2 --calibrate
"""

import argparse
import json
import threading
import time
from pathlib import Path

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus

SCS_MODELS = {"scs0009"}
CALIBRATION_FILE = Path("atc_calibration.json")
ATC_MOTOR_ID = 1
TOOL_MOTOR_IDS = [2, 3]


def protocol_for(model: str) -> int:
    return 1 if model in SCS_MODELS else 0


def make_bus(port: str, motors: dict, model: str) -> FeetechMotorsBus:
    return FeetechMotorsBus(
        port=port,
        motors=motors,
        protocol_version=protocol_for(model),
    )


def load_calibration() -> dict:
    if not CALIBRATION_FILE.exists():
        return {}
    return json.loads(CALIBRATION_FILE.read_text())


def save_calibration(data: dict) -> None:
    CALIBRATION_FILE.write_text(json.dumps(data, indent=2))
    print(f"Calibration saved to {CALIBRATION_FILE}")


def read_pos(bus: FeetechMotorsBus, name: str) -> int:
    return int(bus.read("Present_Position", name, normalize=False))


def record_range(bus: FeetechMotorsBus, name: str) -> tuple[int, int]:
    """Poll motor until ENTER is pressed, return (min, max) raw positions."""
    state = {"min": float("inf"), "max": float("-inf"), "running": True}

    def poll():
        while state["running"]:
            try:
                pos = read_pos(bus, name)
                if pos < state["min"]:
                    state["min"] = pos
                if pos > state["max"]:
                    state["max"] = pos
            except Exception:
                pass
            time.sleep(0.05)

    t = threading.Thread(target=poll, daemon=True)
    t.start()
    input("    Move through the full range of motion, then press ENTER to stop...")
    state["running"] = False
    t.join(timeout=1.0)
    return int(state["min"]), int(state["max"])


# ---------------------------------------------------------------------------
# Calibration
# ---------------------------------------------------------------------------

def calibrate_atc(port: str, model: str) -> dict:
    print("\n--- ATC Lock Calibration ---")
    name = "atc_lock"
    bus = make_bus(port, {name: Motor(ATC_MOTOR_ID, model, MotorNormMode.RANGE_M100_100)}, model)
    bus.connect(handshake=False)
    try:
        bus.disable_torque()
        input("  Move to the LOCKED position, then press ENTER...")
        locked = read_pos(bus, name)
        print(f"  Locked position: {locked}")

        input("  Move to the UNLOCKED position, then press ENTER...")
        unlocked = read_pos(bus, name)
        print(f"  Unlocked position: {unlocked}")
    finally:
        bus.disconnect(disable_torque=False)

    print("  ATC calibration done.")
    return {"locked": locked, "unlocked": unlocked}


def calibrate_tool(port: str, model: str, num_motors: int) -> dict:
    print("\n--- Tool Motor Calibration ---")
    motor_map = {
        f"tool_{i + 1}": Motor(TOOL_MOTOR_IDS[i], model, MotorNormMode.RANGE_M100_100)
        for i in range(num_motors)
    }
    bus = make_bus(port, motor_map, model)
    bus.connect(handshake=False)
    result = {}
    try:
        bus.disable_torque()
        for name in motor_map:
            print(f"\n  Calibrating {name}...")
            mn, mx = record_range(bus, name)
            print(f"  Range: min={mn}, max={mx}")
            result[name] = {"min": mn, "max": mx}
    finally:
        bus.disconnect(disable_torque=False)

    print("  Tool calibration done.")
    return result


# ---------------------------------------------------------------------------
# Interactive mode
# ---------------------------------------------------------------------------

def interactive(port: str, model: str, num_motors: int) -> None:
    cal = load_calibration()
    if not cal:
        print("No calibration file found. Run with --calibrate first.")
        return

    atc_cal = cal.get("atc", {})
    tool_cal = cal.get("tool", {})

    atc_name = "atc_lock"
    tool_names = [f"tool_{i + 1}" for i in range(num_motors)]

    all_motors = {atc_name: Motor(ATC_MOTOR_ID, model, MotorNormMode.RANGE_M100_100)}
    for i, name in enumerate(tool_names):
        all_motors[name] = Motor(TOOL_MOTOR_IDS[i], model, MotorNormMode.RANGE_M100_100)

    bus = make_bus(port, all_motors, model)
    bus.connect(handshake=False)
    try:
        bus.enable_torque()

        print("\nCommands:")
        print("  l  =  Lock ATC")
        print("  u  =  Unlock ATC")
        print("  a  =  Activate tool  (move to range max)")
        print("  h  =  Home tool      (move to range min)")
        print("  q  =  Quit")

        while True:
            try:
                cmd = input("\n> ").strip().lower()
            except (EOFError, KeyboardInterrupt):
                break

            if cmd == "q":
                break
            elif cmd == "l":
                if "locked" not in atc_cal:
                    print("ATC not calibrated — run with --calibrate first.")
                    continue
                bus.write("Goal_Position", atc_name, atc_cal["locked"], normalize=False)
                print(f"  Locking ATC -> position {atc_cal['locked']}")
            elif cmd == "u":
                if "unlocked" not in atc_cal:
                    print("ATC not calibrated — run with --calibrate first.")
                    continue
                bus.write("Goal_Position", atc_name, atc_cal["unlocked"], normalize=False)
                print(f"  Unlocking ATC -> position {atc_cal['unlocked']}")
            elif cmd in ("a", "h"):
                for name in tool_names:
                    if name not in tool_cal:
                        print(f"  {name} not calibrated — run with --calibrate first.")
                        continue
                    pos = tool_cal[name]["max"] if cmd == "a" else tool_cal[name]["min"]
                    bus.write("Goal_Position", name, pos, normalize=False)
                    print(f"  {name} -> position {pos}")
            else:
                print("  Unknown command.")
    finally:
        bus.disable_torque()
        bus.disconnect(disable_torque=False)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(description="ATC motor test and calibration")
    parser.add_argument("--port", required=True, help="Serial port, e.g. /dev/ttyACM1")
    parser.add_argument("--model", default="sts3215", help="Motor model (default: sts3215)")
    parser.add_argument(
        "--motors",
        type=int,
        choices=[1, 2],
        default=1,
        help="Number of tool motors (default: 1)",
    )
    parser.add_argument(
        "--calibrate",
        action="store_true",
        help="Run calibration: record ATC lock positions and tool range of motion",
    )
    args = parser.parse_args()

    if args.calibrate:
        cal = load_calibration()
        cal["atc"] = calibrate_atc(args.port, args.model)
        cal["tool"] = calibrate_tool(args.port, args.model, args.motors)
        save_calibration(cal)
    else:
        interactive(args.port, args.model, args.motors)


if __name__ == "__main__":
    main()
