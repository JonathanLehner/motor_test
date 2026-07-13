#!/usr/bin/env python
"""Clear Damiao (OpenArm follower) motor faults over CAN.

For each motor it sends the Damiao recovery sequence:
    disable (0xFD) -> clear-fault (0xFB) -> enable (0xFC)
then re-reads the status nibble to confirm.

IMPORTANT: clearing only works once the *fault condition is gone*.
An UNDERVOLTAGE (0x9) / OVERVOLTAGE / OVERTEMP fault will NOT clear while the
condition persists -- fix the 24V supply / let motors cool first, then run this.

Run with a venv that has python-can, e.g.:
    /home/jonathan/.local/share/virtualenvs/teleop-wC-ijVq5/bin/python clear_motor_faults.py

Examples:
    clear_motor_faults.py                  # clear faults on can0 (right) + can1 (left)
    clear_motor_faults.py --channels can0
    clear_motor_faults.py --motors 8       # only the gripper
"""

import argparse
import sys
import time

import can

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

JOINTS = {
    1: "joint_1", 2: "joint_2", 3: "joint_3", 4: "joint_4",
    5: "joint_5", 6: "joint_6", 7: "joint_7", 8: "gripper",
}

CMD_DISABLE = 0xFD
CMD_CLEAR = 0xFB   # Damiao clear-fault
CMD_ENABLE = 0xFC


def _send_recv(bus, sid, byte, window=0.1):
    """Send a simple command frame and return the matching feedback frame."""
    rid = sid + 0x10
    bus.send(can.Message(arbitration_id=sid, data=[0xFF] * 7 + [byte],
                         is_extended_id=False, is_fd=True))
    t0 = time.monotonic()
    while time.monotonic() - t0 < window:
        m = bus.recv(timeout=window)
        if m and m.arbitration_id == rid:
            return m
    return None


def _status(msg):
    return None if msg is None else msg.data[0] >> 4


def recover_motor(bus, sid, name):
    before = _status(_send_recv(bus, sid, CMD_ENABLE))  # enable also returns status
    if before is None:
        print(f"  {name:8} id 0x{sid:02X}: NO REPLY (unpowered / off bus) -- skipped")
        return False
    if before in OK_STATES:
        print(f"  {name:8} id 0x{sid:02X}: already {STATUS[before]} -- nothing to clear")
        return True

    # recovery sequence
    _send_recv(bus, sid, CMD_DISABLE)
    time.sleep(0.02)
    _send_recv(bus, sid, CMD_CLEAR)
    time.sleep(0.02)
    after = _status(_send_recv(bus, sid, CMD_ENABLE))

    b = STATUS.get(before, f"0x{before:X}")
    a = STATUS.get(after, f"0x{after:X}") if after is not None else "no reply"
    ok = after in OK_STATES
    print(f"  {name:8} id 0x{sid:02X}: {b} -> {a}  {'[cleared]' if ok else '[STILL FAULTED]'}")
    return ok


def process_channel(channel, motor_ids):
    print(f"\n=== {channel} ===")
    try:
        bus = can.interface.Bus(channel=channel, interface="socketcan", fd=True)
    except Exception as e:
        print(f"  could not open {channel}: {e}")
        return False
    all_ok = True
    try:
        for sid in motor_ids:
            if not recover_motor(bus, sid, JOINTS.get(sid, f"id{sid}")):
                all_ok = False
    finally:
        bus.shutdown()
    return all_ok


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--channels", nargs="+", default=["can0", "can1"],
                    help="CAN interfaces (default: can0 can1)")
    ap.add_argument("--motors", nargs="+", type=int, default=list(JOINTS),
                    help="motor send-ids to clear (default: 1-8)")
    args = ap.parse_args()

    print("Clearing OpenArm follower motor faults (disable -> clear -> enable)")
    healthy = True
    for ch in args.channels:
        if not process_channel(ch, args.motors):
            healthy = False

    if healthy:
        print("\nAll targeted motors are now OK.")
    else:
        print("\nSome motors are STILL faulted.")
        print("Undervoltage/overtemp will not clear until the condition is fixed:")
        print("  - power-cycle the 24V supply, release the e-stop, let motors cool,")
        print("    then run this again.")
    sys.exit(0 if healthy else 1)


if __name__ == "__main__":
    main()
