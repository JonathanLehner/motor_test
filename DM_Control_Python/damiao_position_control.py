#!/usr/bin/env python3
"""Move a DaMiao gripper to a calibrated absolute position using MIT control."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
import time


def parse_int(value: str) -> int:
    return int(value, 0)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--calibration", type=Path, default=Path("gripper_calibration.json"))
    target = parser.add_mutually_exclusive_group(required=True)
    target.add_argument("--position-rad", type=float, help="absolute motor position in radians")
    target.add_argument("--position-deg", type=float, help="absolute motor position in degrees")
    target.add_argument(
        "--normalized",
        type=float,
        help="position from 0=min calibration bound to 1=max calibration bound",
    )
    parser.add_argument("--port", help="override the serial port stored in calibration")
    parser.add_argument("--baud", type=int, help="override the adapter baud rate")
    parser.add_argument("--can-id", type=parse_int, help="override the motor CAN ID")
    parser.add_argument("--master-id", type=parse_int, help="feedback/master ID; default CAN ID + 0x10")
    parser.add_argument("--motor-type", help="override the motor type stored in calibration")
    parser.add_argument("--max-velocity", type=float, default=0.30, help="ramp limit in rad/s")
    parser.add_argument("--rate", type=float, default=50.0, help="MIT command rate in Hz")
    parser.add_argument("--kp", type=float, default=1.0, help="MIT position gain")
    parser.add_argument("--kd", type=float, default=1.5, help="MIT damping gain")
    parser.add_argument(
        "--hold-seconds",
        type=float,
        default=0.5,
        help="hold the target before disabling; 0 disables immediately",
    )
    parser.add_argument(
        "--hold-until-ctrl-c",
        action="store_true",
        help="hold the target until Ctrl-C, then disable",
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="actually move; without this flag the command is a dry run",
    )
    return parser.parse_args()


def load_calibration(path: Path) -> dict:
    try:
        calibration = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError:
        raise SystemExit(f"Calibration file not found: {path}")
    except (OSError, json.JSONDecodeError) as exc:
        raise SystemExit(f"Could not read calibration file {path}: {exc}")

    required = {"position_min_rad", "position_max_rad", "port", "baud", "can_id", "motor_type"}
    missing = sorted(required - calibration.keys())
    if missing:
        raise SystemExit(f"Calibration file is missing: {', '.join(missing)}")
    return calibration


def requested_target(args: argparse.Namespace, minimum: float, maximum: float) -> tuple[float, str]:
    if args.position_rad is not None:
        return args.position_rad, f"{args.position_rad:.6f} rad"
    if args.position_deg is not None:
        return math.radians(args.position_deg), f"{args.position_deg:.3f} deg"
    if not 0.0 <= args.normalized <= 1.0:
        raise SystemExit("--normalized must be between 0 and 1")
    target = minimum + args.normalized * (maximum - minimum)
    return target, f"normalized {args.normalized:.3f}"


def read_position(ctrl, motor, wait: float = 0.03) -> tuple[float, float, float]:
    # DM_CAN.refresh_motor_status() parses immediately.  A second recv after a
    # short wait is needed for USB serial replies that arrive asynchronously.
    ctrl.refresh_motor_status(motor)
    time.sleep(wait)
    ctrl.recv()
    return (
        float(motor.getPosition()),
        float(motor.getVelocity()),
        float(motor.getTorque()),
    )


def main() -> int:
    args = parse_args()
    calibration = load_calibration(args.calibration)
    minimum = float(calibration["position_min_rad"])
    maximum = float(calibration["position_max_rad"])
    if minimum >= maximum:
        raise SystemExit("Invalid calibration: position_min_rad must be less than position_max_rad")

    target, target_source = requested_target(args, minimum, maximum)
    tolerance = 1e-9
    if target < minimum - tolerance or target > maximum + tolerance:
        raise SystemExit(
            f"Refusing target {target:.6f} rad ({math.degrees(target):.3f} deg): "
            f"outside calibrated range [{minimum:.6f}, {maximum:.6f}] rad "
            f"([{math.degrees(minimum):.3f}, {math.degrees(maximum):.3f}] deg)"
        )

    if args.max_velocity <= 0 or args.rate <= 0:
        raise SystemExit("--max-velocity and --rate must be greater than zero")
    if args.hold_seconds < 0:
        raise SystemExit("--hold-seconds must be zero or greater")

    port_name = args.port or str(calibration["port"])
    baud = args.baud or int(calibration["baud"])
    can_id = args.can_id if args.can_id is not None else int(calibration["can_id"])
    master_id = args.master_id if args.master_id is not None else can_id + 0x10
    motor_type_name = args.motor_type or str(calibration["motor_type"])

    print("DaMiao calibrated position control")
    print(f"  calibration : {args.calibration}")
    print(f"  safe range  : {minimum:.6f} .. {maximum:.6f} rad")
    print(f"                {math.degrees(minimum):.3f} .. {math.degrees(maximum):.3f} deg")
    print(f"  target      : {target:.6f} rad ({math.degrees(target):.3f} deg), from {target_source}")
    print(f"  adapter     : {port_name} @ {baud}")
    print(f"  IDs         : CAN 0x{can_id:X}, master 0x{master_id:X}")
    print(f"  gains       : kp={args.kp:g}, kd={args.kd:g}, max velocity={args.max_velocity:g} rad/s")

    if not args.execute:
        print("\nDRY RUN: no command was sent. Add --execute to move the gripper.")
        return 0

    try:
        import serial
        from DM_CAN import Control_Type, DM_Motor_Type, DM_variable, Motor, MotorControl
    except ImportError as exc:
        print(f"Missing runtime dependency: {exc}", file=sys.stderr)
        print("Activate the conda environment containing numpy and pyserial.", file=sys.stderr)
        return 1

    try:
        motor_type = getattr(DM_Motor_Type, motor_type_name)
    except AttributeError:
        print(f"Unknown DM motor type: {motor_type_name}", file=sys.stderr)
        return 1

    serial_port = None
    ctrl = None
    motor = None
    enabled = False
    try:
        serial_port = serial.Serial(port_name, baud, timeout=0.5)
        ctrl = MotorControl(serial_port)
        motor = Motor(motor_type, can_id, master_id)
        ctrl.addMotor(motor)

        current, velocity, torque = read_position(ctrl, motor)
        print(f"\nCurrent: {current:.6f} rad ({math.degrees(current):.3f} deg), "
              f"vel={velocity:+.4f} rad/s, torque={torque:+.4f} N*m")
        if current < minimum - 0.05 or current > maximum + 0.05:
            print("Refusing move: current position is unexpectedly outside calibration.", file=sys.stderr)
            return 2

        mode = ctrl.read_motor_param(motor, DM_variable.CTRL_MODE)
        if mode is not None and int(mode) != int(Control_Type.MIT):
            print(f"Refusing move: motor control mode is {int(mode)}, expected MIT mode 1.", file=sys.stderr)
            print("This script does not modify persistent motor configuration.", file=sys.stderr)
            return 2

        if mode is None:
            print("Warning: control mode register did not reply; proceeding with MIT commands.")

        distance = abs(target - current)
        duration = max(0.20, distance / args.max_velocity)
        steps = max(1, math.ceil(duration * args.rate))
        period = 1.0 / args.rate
        print(f"Moving {distance:.4f} rad over {duration:.2f} s ({steps} steps)...")

        ctrl.enable(motor)
        enabled = True
        time.sleep(0.1)

        for index in range(steps):
            fraction = (index + 1) / steps
            smooth = fraction * fraction * (3.0 - 2.0 * fraction)
            command = current + smooth * (target - current)
            ctrl.controlMIT(motor, args.kp, args.kd, command, 0.0, 0.0)
            if index % max(1, round(args.rate / 5.0)) == 0 or index == steps - 1:
                actual = float(motor.getPosition())
                print(f"  command={command:.4f} rad  feedback={actual:.4f} rad")
            time.sleep(period)

        hold_deadline = time.monotonic() + args.hold_seconds
        if args.hold_until_ctrl_c:
            print("Holding target; press Ctrl-C to disable.")
        elif args.hold_seconds:
            print(f"Holding target for {args.hold_seconds:g} s...")

        while args.hold_until_ctrl_c or time.monotonic() < hold_deadline:
            ctrl.controlMIT(motor, args.kp, args.kd, target, 0.0, 0.0)
            time.sleep(period)

        final, velocity, torque = read_position(ctrl, motor)
        print(f"Final: {final:.6f} rad ({math.degrees(final):.3f} deg), "
              f"error={target - final:+.5f} rad, vel={velocity:+.4f}, torque={torque:+.4f}")
        return 0
    except KeyboardInterrupt:
        print("\nInterrupted; disabling motor.")
        return 130
    except (OSError, serial.SerialException) as exc:
        print(f"Hardware communication error: {exc}", file=sys.stderr)
        return 1
    finally:
        if enabled and ctrl is not None and motor is not None:
            try:
                ctrl.disable(motor)
                print("Motor disabled.")
            except Exception as exc:
                print(f"WARNING: failed to disable motor: {exc}", file=sys.stderr)
        if serial_port is not None:
            try:
                serial_port.close()
            except Exception:
                pass


if __name__ == "__main__":
    raise SystemExit(main())
