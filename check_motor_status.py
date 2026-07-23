#!/usr/bin/env python
"""Check Damiao (OpenArm follower) motor status over CAN.

Sends the MIT "enable" frame to each motor id and decodes the status nibble
from the feedback frame. Useful for diagnosing why motors won't move
(undervoltage, faults, unpowered, etc.).

Run with a venv that has python-can, e.g.:
    /home/jonathan/.local/share/virtualenvs/teleop-wC-ijVq5/bin/python check_motor_status.py

Examples:
    check_motor_status.py                 # probe can0 (right) and can1 (left)
    check_motor_status.py --channels can0
    check_motor_status.py --no-enable     # passive: don't send enable, just refresh
"""

import argparse
import sys
import time

import can

# Damiao status nibble (high nibble of feedback data[0])
STATUS = {
    0x0: "disabled",
    0x1: "OK/enabled",
    0x8: "OVERVOLTAGE",
    0x9: "UNDERVOLTAGE",
    0xA: "OVERCURRENT",
    0xB: "MOS-OVERTEMP",
    0xC: "ROTOR-OVERTEMP",
    0xD: "COMMS-LOSS",
    0xE: "OVERLOAD",
}
OK_STATES = {0x0, 0x1}

# OpenArm follower motor id -> joint name (send id; recv id = send id + 0x10)
JOINTS = {
    1: "joint_1", 2: "joint_2", 3: "joint_3", 4: "joint_4",
    5: "joint_5", 6: "joint_6", 7: "joint_7", 8: "gripper",
}

CMD_ENABLE = 0xFC
CMD_REFRESH = 0xCC
PARAM_ID = 0x7FF


def _send(bus, sid, byte, use_param_id=False):
    if use_param_id:  # passive refresh frame (id 0x7FF, motor id in payload)
        data = [sid & 0xFF, (sid >> 8) & 0xFF, byte, 0, 0, 0, 0, 0]
        arb = PARAM_ID
    else:
        data = [0xFF] * 7 + [byte]
        arb = sid
    bus.send(can.Message(arbitration_id=arb, data=data, is_extended_id=False, is_fd=True))


def _recv(bus, rid, window=0.1):
    t0 = time.monotonic()
    while time.monotonic() - t0 < window:
        m = bus.recv(timeout=window)
        if m and m.arbitration_id == rid:
            return m
    return None


def probe_channel(channel, enable=True):
    print(f"\n=== {channel} ===")
    try:
        bus = can.interface.Bus(channel=channel, interface="socketcan", fd=True)
    except Exception as e:
        print(f"  could not open {channel}: {e}")
        return False

    all_ok = True
    try:
        for sid, name in JOINTS.items():
            rid = sid + 0x10
            if enable:
                _send(bus, sid, CMD_ENABLE)
            else:
                _send(bus, sid, CMD_REFRESH, use_param_id=True)
            reply = _recv(bus, rid)
            if reply is None:
                print(f"  {name:8} id 0x{sid:02X}: NO REPLY (unpowered / off bus)")
                all_ok = False
                continue
            st = reply.data[0] >> 4
            label = STATUS.get(st, f"unknown 0x{st:X}")
            flag = "" if st in OK_STATES else "  <-- FAULT"
            print(f"  {name:8} id 0x{sid:02X}: status 0x{st:X} ({label}){flag}")
            if st not in OK_STATES:
                all_ok = False
    finally:
        bus.shutdown()
    return all_ok


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--channels", nargs="+", default=["can0", "can1"],
                    help="CAN interfaces to probe (default: can0 can1)")
    ap.add_argument("--no-enable", action="store_true",
                    help="passive refresh instead of sending the enable frame")
    args = ap.parse_args()

    print("Probing OpenArm follower motors (status 0x1=OK, 0x9=UNDERVOLTAGE → check 24V supply)")
    healthy = True
    for ch in args.channels:
        if not probe_channel(ch, enable=not args.no_enable):
            healthy = False

    print("\n" + ("All motors OK." if healthy else "Some motors are faulted or unreachable (see above)."))
    sys.exit(0 if healthy else 1)


if __name__ == "__main__":
    main()
