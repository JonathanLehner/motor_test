#!/usr/bin/env python
"""
Test and calibration script for ATC Feetech motors.

Motor IDs:
  1 = ATC lock mechanism  (sts3215 by default, protocol 0)
  2 = Tool motor 1        (model depends on the tool: scs0009 or sts3215)
  3 = Tool motor 2 (optional)

The tool motor model can differ per tool version, so it is selectable with
--tool-model. SC-series models (scs0009) use protocol 1; ST/SMS-series models
(sts3215) use protocol 0. ATC and tool motors share the same physical bus.

Calibration is saved to atc_calibration.json in the current directory.

Usage:
  # Calibrate ATC lock and tool motors
  python atc_test.py --port /dev/ttyACM1 --calibrate all

  # Interactive: lock/unlock ATC, activate/home tool
  python atc_test.py --port /dev/ttyACM1

  # Tool with an sts3215 motor instead of scs0009
  python atc_test.py --port /dev/ttyACM1 --tool-model sts3215 --calibrate all

  # With 2 tool motors
  python atc_test.py --port /dev/ttyACM1 --motors 2 --calibrate all
"""

import argparse
import json
import threading
import time
from pathlib import Path

from scservo_sdk.port_handler import PortHandler
from scservo_sdk.sms_sts import sms_sts
from scservo_sdk.scscl import scscl
from scservo_sdk.scservo_def import COMM_SUCCESS


class EchoFreePortHandler(PortHandler):
    """Discards TX echo on the half-duplex single-wire bus before reading the response."""
    def openPort(self):
        result = super().openPort()
        if result:
            self.ser.timeout = 0.01
        return result

    def writePort(self, packet):
        result = super().writePort(packet)
        self.ser.read(len(packet))
        return result


SCS_MODELS = {"scs0009"}
CALIBRATION_FILE = Path("atc_calibration.json")
ATC_ID = 1
TOOL_IDS = [2, 3]
TORQUE_ADDR = 40


def open_port(port):
    ph = EchoFreePortHandler(port)
    if not ph.openPort():
        raise RuntimeError(f"Cannot open port {port}")
    ph.setBaudRate(1_000_000)
    return ph


def make_handler(ph, model):
    """Pick the protocol handler for a model. SC-series -> scscl, else sms_sts."""
    return scscl(ph) if model in SCS_MODELS else sms_sts(ph)


def read_pos(handler, motor_id):
    handler.portHandler.clearPort()
    pos, comm, _ = handler.ReadPos(motor_id)
    if comm != COMM_SUCCESS:
        raise RuntimeError(f"Failed to read position from ID {motor_id}")
    return int(pos)


def move(handler, model, motor_id, pos):
    if model in SCS_MODELS:
        handler.WritePos(motor_id, pos, 0, 500)    # scscl: position, time, speed
    else:
        handler.WritePosEx(motor_id, pos, 500, 50)  # sms_sts: position, speed, acc


def torque(handler, motor_id, enable):
    result, error = handler.write1ByteTxRx(motor_id, TORQUE_ADDR, 1 if enable else 0)
    if result != COMM_SUCCESS:
        state = "enable" if enable else "disable"
        print(f"  Warning: torque {state} failed for ID {motor_id} (result={result}, error={error})")


def record_range(handler, motor_id):
    """Poll motor until ENTER is pressed, return (min, max) raw positions."""
    state = {"min": float("inf"), "max": float("-inf"), "running": True}

    def poll():
        while state["running"]:
            try:
                pos = read_pos(handler, motor_id)
                state["min"] = min(state["min"], pos)
                state["max"] = max(state["max"], pos)
                print(f"\r    pos={pos}  min={int(state['min'])}  max={int(state['max'])}    ", end="", flush=True)
            except Exception:
                pass
            time.sleep(0.1)
        print()

    t = threading.Thread(target=poll, daemon=True)
    t.start()
    input("    Move through the full range of motion, then press ENTER to stop...")
    state["running"] = False
    t.join(timeout=1.0)
    return int(state["min"]), int(state["max"])


def load_calibration():
    if not CALIBRATION_FILE.exists():
        return {}
    return json.loads(CALIBRATION_FILE.read_text())


def save_calibration(data):
    CALIBRATION_FILE.write_text(json.dumps(data, indent=2))
    print(f"Calibration saved to {CALIBRATION_FILE}")


# ---------------------------------------------------------------------------
# Calibration
# ---------------------------------------------------------------------------

def calibrate_atc(port, atc_model):
    print("\n--- ATC Lock Calibration ---")
    ph = open_port(port)
    handler = make_handler(ph, atc_model)
    try:
        torque(handler, ATC_ID, False)
        input("  Move to the LOCKED position, then press ENTER...")
        locked = read_pos(handler, ATC_ID)
        print(f"  Locked position: {locked}")

        input("  Move to the UNLOCKED position, then press ENTER...")
        unlocked = read_pos(handler, ATC_ID)
        print(f"  Unlocked position: {unlocked}")
    finally:
        ph.closePort()

    print("  ATC calibration done.")
    return {"locked": locked, "unlocked": unlocked}


def calibrate_tool(port, tool_model, num_motors):
    print("\n--- Tool Motor Calibration ---")
    ph = open_port(port)
    handler = make_handler(ph, tool_model)
    result = {}
    try:
        for i in range(num_motors):
            motor_id = TOOL_IDS[i]
            name = f"tool_{i + 1}"
            torque(handler, motor_id, False)
            print(f"\n  Calibrating {name} (ID {motor_id})...")
            mn, mx = record_range(handler, motor_id)
            print(f"  Range: min={mn}, max={mx}")
            result[name] = {"min": mn, "max": mx}
    finally:
        ph.closePort()

    print("  Tool calibration done.")
    return result


# ---------------------------------------------------------------------------
# Interactive mode
# ---------------------------------------------------------------------------

def interactive(port, atc_model, tool_model, num_motors):
    cal = load_calibration()
    if not cal:
        print("No calibration file found. Run with --calibrate first.")
        return

    atc_cal = cal.get("atc", {})
    tool_cal = cal.get("tool", {})
    tool_ids = [TOOL_IDS[i] for i in range(num_motors)]

    # ATC and tool motors are on the same physical bus -> one shared port.
    ph = open_port(port)
    atc = make_handler(ph, atc_model)
    tool = make_handler(ph, tool_model) if num_motors else None

    try:
        torque(atc, ATC_ID, True)
        if tool:
            for tid in tool_ids:
                torque(tool, tid, True)

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
                    print("ATC not calibrated.")
                    continue
                move(atc, atc_model, ATC_ID, atc_cal["locked"])
                print(f"  Locking ATC -> {atc_cal['locked']}")
            elif cmd == "u":
                if "unlocked" not in atc_cal:
                    print("ATC not calibrated.")
                    continue
                move(atc, atc_model, ATC_ID, atc_cal["unlocked"])
                print(f"  Unlocking ATC -> {atc_cal['unlocked']}")
            elif cmd in ("a", "h"):
                if not tool:
                    print("  No tool motors configured.")
                    continue
                for i, tid in enumerate(tool_ids):
                    name = f"tool_{i + 1}"
                    if name not in tool_cal:
                        print(f"  {name} not calibrated.")
                        continue
                    pos = tool_cal[name]["max"] if cmd == "a" else tool_cal[name]["min"]
                    move(tool, tool_model, tid, pos)
                    print(f"  {name} -> {pos}")
            else:
                print("  Unknown command.")
    finally:
        torque(atc, ATC_ID, False)
        if tool:
            for tid in tool_ids:
                torque(tool, tid, False)
        ph.closePort()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="ATC motor test and calibration")
    parser.add_argument("--port", required=True, help="Serial port, e.g. /dev/ttyACM1")
    parser.add_argument("--atc-model", default="sts3215", help="ATC motor model (default: sts3215)")
    parser.add_argument("--tool-model", default="scs0009",
                        help="Tool motor model: scs0009 or sts3215 (default: scs0009)")
    parser.add_argument(
        "--motors",
        type=int,
        choices=[1, 2],
        default=1,
        help="Number of tool motors (default: 1)",
    )
    parser.add_argument(
        "--calibrate",
        choices=["atc", "tool", "all"],
        help="Calibrate: atc = lock positions only, tool = tool range only, all = both",
    )
    args = parser.parse_args()

    if args.calibrate:
        cal = load_calibration()
        if args.calibrate in ("atc", "all"):
            cal["atc"] = calibrate_atc(args.port, args.atc_model)
        if args.calibrate in ("tool", "all"):
            cal["tool"] = calibrate_tool(args.port, args.tool_model, args.motors)
        save_calibration(cal)
    else:
        interactive(args.port, args.atc_model, args.tool_model, args.motors)


if __name__ == "__main__":
    main()
