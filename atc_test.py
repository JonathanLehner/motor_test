#!/usr/bin/env python
"""
Test and calibration script for ATC Feetech motors.

Motor IDs:
  1 = ATC lock mechanism  (sts3215, protocol 0)
  2 = Tool motor 1        (scs0009, protocol 1)
  3 = Tool motor 2 (optional)

Calibration is saved to atc_calibration.json in the current directory.

Usage:
  # Calibrate ATC lock and tool motors
  python atc_test.py --port /dev/ttyACM1 --calibrate

  # Interactive: lock/unlock ATC, activate/home tool
  python atc_test.py --port /dev/ttyACM1

  # With 2 tool motors
  python atc_test.py --port /dev/ttyACM1 --motors 2 --calibrate
"""

import argparse
import json
import threading
import time
from pathlib import Path

import time

from scservo_sdk.port_handler import PortHandler
from scservo_sdk.sms_sts import sms_sts
from scservo_sdk.scscl import scscl
from scservo_sdk.scservo_def import COMM_SUCCESS


class EchoFreePortHandler(PortHandler):
    """Discards TX echo on half-duplex RS485 bus before reading the response."""
    def openPort(self):
        result = super().openPort()
        if result:
            self.ser.timeout = 0.01
        return result

    def writePort(self, packet):
        result = super().writePort(packet)
        self.ser.read(len(packet))
        return result

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


PRESENT_POSITION_ADDR = 56  # same address for both sms_sts and scscl

def read_pos(handler, motor_id):
    handler.portHandler.clearPort()
    data, comm, _ = handler.readTxRx(motor_id, PRESENT_POSITION_ADDR, 2)
    if comm != COMM_SUCCESS:
        raise RuntimeError(f"Failed to read position from ID {motor_id}")
    return data[0] | (data[1] << 8)  # little-endian, explicit


def torque(handler, motor_id, enable):
    result, error = handler.write1ByteTxRx(motor_id, TORQUE_ADDR, 1 if enable else 0)
    state = "enabled" if enable else "disabled"
    if result != COMM_SUCCESS:
        print(f"  Warning: torque {state} failed for ID {motor_id} (result={result}, error={error})")
        return
    handler.portHandler.clearPort()
    readback, comm2, _ = handler.read1ByteTxRx(motor_id, TORQUE_ADDR)
    print(f"  Torque register: {readback} (0=off, 1=on)")


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

def calibrate_atc(port):
    print("\n--- ATC Lock Calibration ---")
    ph = open_port(port)
    handler = sms_sts(ph)
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


def calibrate_tool(port, num_motors):
    print("\n--- Tool Motor Calibration ---")
    ph = open_port(port)
    handler = scscl(ph)
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

def interactive(port, num_motors):
    cal = load_calibration()
    if not cal:
        print("No calibration file found. Run with --calibrate first.")
        return

    atc_cal = cal.get("atc", {})
    tool_cal = cal.get("tool", {})
    tool_ids = [TOOL_IDS[i] for i in range(num_motors)]

    atc_ph = open_port(port)
    atc = sms_sts(atc_ph)

    tool_ph = open_port(port) if num_motors > 0 else None
    tool = scscl(tool_ph) if tool_ph else None

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
                atc.WritePosEx(ATC_ID, atc_cal["locked"], 200, 50)
                print(f"  Locking ATC -> {atc_cal['locked']}")
            elif cmd == "u":
                if "unlocked" not in atc_cal:
                    print("ATC not calibrated.")
                    continue
                atc.WritePosEx(ATC_ID, atc_cal["unlocked"], 200, 50)
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
                    tool.WritePos(tid, pos, 500, 200)
                    print(f"  {name} -> {pos}")
            else:
                print("  Unknown command.")
    finally:
        torque(atc, ATC_ID, False)
        atc_ph.closePort()
        if tool_ph:
            for tid in tool_ids:
                torque(tool, tid, False)
            tool_ph.closePort()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="ATC motor test and calibration")
    parser.add_argument("--port", required=True, help="Serial port, e.g. /dev/ttyACM1")
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
            cal["atc"] = calibrate_atc(args.port)
        if args.calibrate in ("tool", "all"):
            cal["tool"] = calibrate_tool(args.port, args.motors)
        save_calibration(cal)
    else:
        interactive(args.port, args.motors)


if __name__ == "__main__":
    main()
