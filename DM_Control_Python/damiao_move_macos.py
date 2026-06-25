#!/usr/bin/env python3

import argparse
import sys
import time

try:
    import serial
except ImportError:
    print("Missing pyserial. Install with:")
    print("  pip install pyserial numpy")
    sys.exit(1)

try:
    from DM_CAN import *
except ImportError:
    print("Could not import DM_CAN.py.")
    print("Put this script in the same folder as DM_CAN.py.")
    sys.exit(1)


DEFAULT_PORT = "/dev/cu.usbmodem00000000050C1"
DEFAULT_BAUD = 921600


def parse_int(x):
    return int(x, 0)


def motor_type_from_string(name):
    name = name.upper().replace("-", "").replace("_", "")

    mapping = {
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

    enum_name = mapping.get(name, name)

    if not hasattr(DM_Motor_Type, enum_name):
        print(f"Unknown motor type: {name}")
        print("Available motor types:")
        for item in DM_Motor_Type:
            print(" ", item.name)
        sys.exit(1)

    return getattr(DM_Motor_Type, enum_name)


def safe_disable(ctrl, motor):
    try:
        print("Disabling motor...")
        ctrl.disable(motor)
    except Exception as e:
        print(f"Disable failed: {e}")


def print_status(ctrl, motor):
    try:
        ctrl.refresh_motor_status(motor)
        time.sleep(0.05)
        print(
            "STATUS:",
            "pos =", motor.getPosition(),
            "vel =", motor.getVelocity(),
            "tau =", motor.getTorque(),
        )
    except Exception as e:
        print(f"Status read failed: {e}")


def switch_mode_if_requested(ctrl, motor, mode):
    if mode is None:
        return

    mode_map = {
        "mit": Control_Type.MIT,
        "posvel": Control_Type.POS_VEL,
        "vel": Control_Type.VEL,
    }

    if mode not in mode_map:
        print(f"Unsupported switch mode: {mode}")
        return

    print(f"Switching control mode temporarily to {mode}...")
    ok = ctrl.switchControlMode(motor, mode_map[mode])
    print("Switch mode:", "OK" if ok else "FAILED")


def mit_position_limit(ctrl, motor):
    return float(ctrl.Limit_Param[motor.MotorType][0])


def wrap_position_to_mit_range(position, q_limit):
    span = 2.0 * q_limit
    return ((position + q_limit) % span) - q_limit


def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


def unwrap_position(previous_unwrapped, current_wrapped, q_limit):
    previous_wrapped = wrap_position_to_mit_range(previous_unwrapped, q_limit)
    delta = wrap_position_to_mit_range(current_wrapped - previous_wrapped, q_limit)
    return previous_unwrapped + delta


def crosses_mit_wrap(start_position, target_position, q_limit):
    low = min(start_position, target_position)
    high = max(start_position, target_position)
    return low < -q_limit or high > q_limit


def continuous_mit_position_move(ctrl, motor, args, start_pos, target_pos, q_limit, period, steps):
    dq_limit = float(ctrl.Limit_Param[motor.MotorType][1])
    move_delta = target_pos - start_pos
    feedforward_velocity = move_delta / max(args.duration, period)
    velocity_limit = min(dq_limit, max(abs(args.velocity), 0.05))
    unwrapped_pos = start_pos

    print(
        "Using unwrapped feedback with MIT velocity commands "
        f"(velocity limit {velocity_limit:.3f} rad/s)."
    )

    for i in range(steps):
        alpha = (i + 1) / steps
        desired_pos = start_pos + alpha * move_delta

        ctrl.refresh_motor_status(motor)
        unwrapped_pos = unwrap_position(unwrapped_pos, float(motor.getPosition()), q_limit)

        error = desired_pos - unwrapped_pos
        velocity_cmd = feedforward_velocity + args.kp * error
        velocity_cmd = clamp(velocity_cmd, -velocity_limit, velocity_limit)
        ctrl.controlMIT(motor, 0.0, args.kd, 0.0, velocity_cmd, 0.0)
        time.sleep(period)

    print("Stopping continuous move...")
    for _ in range(20):
        ctrl.controlMIT(motor, 0.0, args.kd, 0.0, 0.0, 0.0)
        time.sleep(period)


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--motor-type", default="4310")

    parser.add_argument("--can-id", type=parse_int, default=0x01)
    parser.add_argument("--master-id", type=parse_int, default=None)

    parser.add_argument(
        "--mode",
        choices=["mit-vel", "mit-pos", "mit-torque", "vel", "posvel", "status"],
        default="mit-vel",
        help="Default mit-vel is safest if motor is in MIT mode.",
    )

    parser.add_argument(
        "--switch-mode",
        choices=["mit", "posvel", "vel"],
        default=None,
        help="Temporarily switch motor control mode before moving.",
    )

    parser.add_argument("--duration", type=float, default=1.5)
    parser.add_argument("--rate", type=float, default=50.0)

    parser.add_argument("--velocity", type=float, default=0.5)
    parser.add_argument("--position-delta", type=float, default=0.2)
    parser.add_argument("--target-position", type=float, default=None)

    parser.add_argument("--kp", type=float, default=5.0)
    parser.add_argument("--kd", type=float, default=0.5)
    parser.add_argument("--torque", type=float, default=0.05)
    parser.add_argument(
        "--continuous-wrap",
        action="store_true",
        help="Experimental: use software-unwrapped MIT velocity control when mit-pos crosses the command wrap.",
    )

    parser.add_argument("--old-enable", action="store_true")
    parser.add_argument("--no-disable", action="store_true")

    args = parser.parse_args()

    motor_type = motor_type_from_string(args.motor_type)

    can_id = args.can_id
    master_id = args.master_id if args.master_id is not None else can_id + 0x10

    print("Opening:")
    print("  port      =", args.port)
    print("  baud      =", args.baud)
    print("  motor     =", args.motor_type)
    print(f"  CAN_ID    = 0x{can_id:02X}")
    print(f"  Master_ID = 0x{master_id:02X}")

    ser = serial.Serial(args.port, args.baud, timeout=0.5)
    ctrl = MotorControl(ser)

    motor = Motor(motor_type, can_id, master_id)
    ctrl.addMotor(motor)

    try:
        print_status(ctrl, motor)

        if args.mode == "status":
            return

        switch_mode_if_requested(ctrl, motor, args.switch_mode)

        print("Enabling motor...")
        if args.old_enable:
            if args.mode.startswith("mit"):
                ctrl.enable_old(motor, Control_Type.MIT)
            elif args.mode == "vel":
                ctrl.enable_old(motor, Control_Type.VEL)
            elif args.mode == "posvel":
                ctrl.enable_old(motor, Control_Type.POS_VEL)
        else:
            ctrl.enable(motor)

        time.sleep(0.2)

        period = 1.0 / args.rate
        steps = max(1, int(args.duration * args.rate))

        print(f"Moving in mode: {args.mode}")

        if args.mode == "mit-vel":
            # MIT velocity command:
            # kp=0, kd>0, q=0, dq=desired velocity, tau=0
            for _ in range(steps):
                ctrl.controlMIT(motor, 0.0, args.kd, 0.0, args.velocity, 0.0)
                time.sleep(period)

            print("Stopping MIT velocity...")
            for _ in range(20):
                ctrl.controlMIT(motor, 0.0, args.kd, 0.0, 0.0, 0.0)
                time.sleep(period)

        elif args.mode == "mit-pos":
            ctrl.refresh_motor_status(motor)
            time.sleep(0.05)
            start_pos = float(motor.getPosition())

            if args.target_position is None:
                target_pos = start_pos + args.position_delta
            else:
                target_pos = args.target_position

            q_limit = mit_position_limit(ctrl, motor)
            print(f"Start position:  {start_pos}")
            print(f"Target position: {target_pos}")
            if crosses_mit_wrap(start_pos, target_pos, q_limit):
                if not args.continuous_wrap:
                    print(
                        f"Refusing MIT position move across wrap range [-{q_limit}, {q_limit}]. "
                        "Use a smaller in-range move or pass --continuous-wrap for experimental velocity-based crossing."
                    )
                    return
                continuous_mit_position_move(ctrl, motor, args, start_pos, target_pos, q_limit, period, steps)
                print_status(ctrl, motor)
                return

            # Smooth ramp in unwrapped position space, then wrap each command into
            # the MIT protocol range before packing it into the CAN frame.
            for i in range(steps):
                alpha = (i + 1) / steps
                q = start_pos + alpha * (target_pos - start_pos)
                ctrl.controlMIT(motor, args.kp, args.kd, wrap_position_to_mit_range(q, q_limit), 0.0, 0.0)
                time.sleep(period)

            print("Holding briefly...")
            hold_q = wrap_position_to_mit_range(target_pos, q_limit)
            for _ in range(20):
                ctrl.controlMIT(motor, args.kp, args.kd, hold_q, 0.0, 0.0)
                time.sleep(period)

        elif args.mode == "mit-torque":
            print("Applying small torque. Be careful: this can rotate continuously.")
            for _ in range(steps):
                ctrl.controlMIT(motor, 0.0, 0.0, 0.0, 0.0, args.torque)
                time.sleep(period)

            print("Stopping torque...")
            for _ in range(20):
                ctrl.controlMIT(motor, 0.0, args.kd, 0.0, 0.0, 0.0)
                time.sleep(period)

        elif args.mode == "vel":
            for _ in range(steps):
                ctrl.control_Vel(motor, args.velocity)
                time.sleep(period)

            print("Stopping velocity...")
            for _ in range(20):
                ctrl.control_Vel(motor, 0.0)
                time.sleep(period)

        elif args.mode == "posvel":
            ctrl.refresh_motor_status(motor)
            time.sleep(0.05)
            start_pos = float(motor.getPosition())

            if args.target_position is None:
                target_pos = start_pos + args.position_delta
            else:
                target_pos = args.target_position

            print(f"Start position:  {start_pos}")
            print(f"Target position: {target_pos}")

            for _ in range(steps):
                ctrl.control_Pos_Vel(motor, target_pos, abs(args.velocity))
                time.sleep(period)

        print_status(ctrl, motor)

    finally:
        if not args.no_disable:
            safe_disable(ctrl, motor)
        ser.close()


if __name__ == "__main__":
    main()
