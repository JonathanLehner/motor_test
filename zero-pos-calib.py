#!/usr/bin/env python3
# ----------------------------------------------------------------------------
# Patched zero-position calibration for OpenArm (7x arm joints + gripper).
#
# ORIGIN:
#   Derived from the original `openarm-can-zero-position-calibration` console
#   command installed in the user's venv. That entry point is NOT part of the
#   `openarm_can` Python binding package (github.com/enactic/openarm_can) --
#   openarm_can defines no console scripts. The exact source package was not
#   identified; to find it on your machine, run:
#       cat $(which openarm-can-zero-position-calibration)
#   and read the module it imports `main` from.
#
#   Built against the openarm_can 1.2.9 binding API (oa.OpenArm, MITParam,
#   set_zero_all, etc.). API is marked UNSTABLE upstream and may change.
#
# ORIGINAL COMMAND (the one that hung):
#       openarm-can-zero-position-calibration --canport can1 --arm-side left_arm
#
# NEW COMMAND (this patched script):
#       python zero_position_calibration_fixed.py --canport can1 --arm-side left_arm
# ----------------------------------------------------------------------------
#
# Fixes vs original:
#   1. bump_to_limit now has a hard max-travel guard -> can never infinite-loop.
#   2. Gripper is no longer bumped to a limit (its result was discarded anyway
#      via "ideal[GRIPPER] skipped"), which is what caused the hang when the
#      gripper encoder started saturated at the +P_MAX rail (0xFFF4).
#   3. JointID.GRIPPER no longer aliases J1 (was both = 0). The ideal-delta
#      loop now skips the gripper slot by position, so J1 is computed correctly.

import openarm_can as oa
import argparse
import time
import numpy as np
from enum import IntEnum

# ---------- IDs / Limits / Signs ----------


class JointID(IntEnum):
    J1 = 0
    J2 = 1
    J3 = 2
    J4 = 3
    J5 = 4
    J6 = 5
    J7 = 6
    GRIPPER = 7          # was 0 -> aliased J1. Distinct value now.


# Index of the gripper motor *within the gripper component* (single motor).
GRIPPER_MOTOR_IDX = 0

mech_lim = {
    JointID.J1: [np.deg2rad(-80),  np.deg2rad(200)],
    JointID.J2: [np.deg2rad(-100), np.deg2rad(100)],
    JointID.J3: [np.deg2rad(-90),  np.deg2rad(90)],
    JointID.J4: [np.deg2rad(0),    np.deg2rad(140)],
    JointID.J5: [np.deg2rad(-90),  np.deg2rad(90)],
    JointID.J6: [np.deg2rad(-45),  np.deg2rad(45)],
    JointID.J7: [np.deg2rad(-90),  np.deg2rad(90)],
    JointID.GRIPPER: [np.deg2rad(-60), np.deg2rad(0)],
}

JOINT_SIGN = {
    JointID.J1: -1.0, JointID.J2: -1.0, JointID.J3: +1.0, JointID.J4: +1.0,
    JointID.J5: +1.0, JointID.J6: +1.0, JointID.J7: +1.0, JointID.GRIPPER: +1.0,
}

# Joints whose bump aborted without finding a mechanical stop. Populated by
# bump_to_limit(); if non-empty at the end, we must NOT write zero (the final
# pose is unreliable).
BUMP_FAILURES = []

# ---------- Core motion primitives ----------


def interpolate(openarm, comp, joint_id: JointID, delta_rad,
                kp=52.0, kd=1.5, torque_assist=0.0, interp_time=2.0):
    """Linear interp: current -> current+delta with MIT control."""
    idx = int(joint_id)
    m = comp.get_motors()[idx]
    q0 = m.get_position()
    q1 = q0 + float(delta_rad)
    n_steps, dt = 500, (interp_time / 500.0)
    tau = np.copysign(1.0, q1 - q0) * abs(torque_assist)
    for i in range(n_steps + 1):
        alpha = i / n_steps
        q = q0 + (q1 - q0) * alpha
        comp.mit_control_one(idx, oa.MITParam(kp, kd, q, 0.0, tau))
        openarm.refresh_all()   # poll motors so get_position/velocity/torque update
        openarm.recv_all()
        time.sleep(dt)


def _hit_thresholds(comp, idx):
    """Return (dq_th, tau_th) based on joint/gripper."""
    is_gripper = (len(comp.get_motors()) == 1)
    if is_gripper:
        return 0.3, 0.3
    if idx == 0:
        return 0.0125, 5.0
    return 0.1, 2.0


def bump_to_limit(openarm, comp, joint_id, motor_idx=None,
                  step_deg=0.2, kp=45.0, kd=1.2, torque_bias=0.0,
                  max_travel_deg=360.0):
    """Step until mechanical stop. Return traveled delta [rad].

    Hardened: aborts after max_travel_deg of commanded travel so a missing
    or undetectable stop can never hang the process.
    """
    idx = int(joint_id) if motor_idx is None else motor_idx
    step_rad = np.deg2rad(step_deg)
    tau_cmd = np.copysign(1.0, step_deg) * abs(torque_bias)

    motors = comp.get_motors()
    q_start = motors[idx].get_position()
    q_target = q_start

    dq_th, tau_th = _hit_thresholds(comp, idx)
    max_travel_rad = np.deg2rad(abs(max_travel_deg))
    peak_tau = 0.0            # largest |torque| seen this bump
    min_vel = np.inf          # smallest |velocity| seen this bump

    while True:
        q_target += step_rad
        comp.mit_control_one(idx, oa.MITParam(kp, kd, q_target, 0.0, tau_cmd))
        openarm.refresh_all()   # poll motors so get_position/velocity/torque update
        openarm.recv_all()
        time.sleep(0.005)

        m = comp.get_motors()[idx]
        vel = np.abs(m.get_velocity())
        tau = np.abs(m.get_torque())
        peak_tau = max(peak_tau, tau)
        min_vel = min(min_vel, vel)

        # SAFETY: fail fast if the joint isn't actually moving. If we've commanded
        # several degrees but the measured position barely changed AND torque
        # telemetry reads ~0, then either feedback is dead or the joint is jammed.
        # Either way the stop-detector is blind and the motor is heading into a
        # stall - abort now (within ~5 deg) instead of driving it for 360 deg.
        commanded = np.abs(q_target - q_start)
        measured = np.abs(m.get_position() - q_start)
        if commanded > np.deg2rad(5.0) and measured < np.deg2rad(0.5) and peak_tau < 1e-3:
            print(f"[ABORT] {joint_id.name}: commanded {np.rad2deg(commanded):.1f} deg but "
                  f"joint moved only {np.rad2deg(measured):.2f} deg with torque telemetry "
                  f"reading ~0.")
            print("        Motor feedback is not live (or the joint is jammed). Refusing to "
                  "keep driving - CUT 24V POWER and fix feedback before rerunning.")
            BUMP_FAILURES.append(joint_id.name)
            return float(m.get_position() - q_start)

        # Safety: never loop forever.
        if np.abs(q_target - q_start) > max_travel_rad:
            print(f"[WARN] no mechanical stop detected for {joint_id.name} "
                  f"after {max_travel_deg:.0f} deg of travel - aborting bump.")
            print(f"       diagnostics: peak |torque|={peak_tau:.2f} Nm "
                  f"(needed >{tau_th:.2f}); min |velocity|={min_vel:.3f} rad/s "
                  f"(needed <{dq_th:.3f}).")
            print(f"       -> if peak torque nearly reached the threshold, the joint "
                  f"stalled but the threshold is too high (lower tau_th or raise kp).")
            print(f"       -> if peak torque stayed near 0, the joint spun freely "
                  f"(decoupled, wrong start pose, or no hard stop that way).")
            BUMP_FAILURES.append(joint_id.name)
            return float(m.get_position() - q_start)

        if vel < dq_th and tau > tau_th:
            delta_rad = m.get_position() - q_start
            delta_deg = np.rad2deg(delta_rad)
            print(f"[INFO] mechanical stop (Joint {joint_id.name}): "
                  f"{delta_rad:.4f} rad / {delta_deg:.2f}deg")
            return float(delta_rad)


def calc_delta_to_zero_pos_joint(initial_rad, ideal_limit_rad,
                                 delta_to_stop_rad, joint_id):
    q_hit = initial_rad + delta_to_stop_rad
    delta_to_ideal = ideal_limit_rad - q_hit
    return float(JOINT_SIGN.get(joint_id, 1.0) * delta_to_ideal)


def move_to_precise_home(openarm, arm_goal_abs):
    pass

# ---------- Per-side sequences ----------
# NOTE: gripper is no longer bumped (its result is discarded later).
# d_grip is kept as 0.0 so the deltas list shape is unchanged.


def _run_right_sequence(openarm, arm, grip):
    d_grip = 0.0  # gripper bump removed - was the hang, result was discarded
    d_j4 = bump_to_limit(openarm, arm,  JointID.J4, step_deg=-0.2)
    time.sleep(0.5)

    interpolate(openarm, arm, JointID.J2, np.deg2rad(5),  interp_time=0.4)
    time.sleep(0.5)
    d_j3 = bump_to_limit(openarm, arm,  JointID.J3)
    time.sleep(0.5)

    interpolate(openarm, arm, JointID.J3, -mech_lim[JointID.J3][1], interp_time=1.0)
    time.sleep(0.5)
    interpolate(openarm, arm, JointID.J2, -np.deg2rad(5), interp_time=0.4)
    time.sleep(0.5)
    interpolate(openarm, arm, JointID.J4,  np.pi/2.0)
    time.sleep(0.5)

    d_j5 = bump_to_limit(openarm, arm,  JointID.J5)
    time.sleep(0.5)
    interpolate(openarm, arm, JointID.J5, -mech_lim[JointID.J5][1])
    time.sleep(0.5)

    d_j6 = bump_to_limit(openarm, arm,  JointID.J6)
    time.sleep(0.5)
    interpolate(openarm, arm, JointID.J6, -mech_lim[JointID.J6][1])
    time.sleep(0.5)

    d_j7 = bump_to_limit(openarm, arm,  JointID.J7)
    time.sleep(0.5)
    interpolate(openarm, arm, JointID.J7, -mech_lim[JointID.J7][1])
    time.sleep(0.5)

    d_j2 = bump_to_limit(openarm, arm,  JointID.J2, step_deg=-0.2)
    time.sleep(0.5)
    interpolate(openarm, arm, JointID.J2, np.deg2rad(10), interp_time=0.9, kp=180, kd=2.0)
    time.sleep(0.5)

    d_j1 = bump_to_limit(openarm, arm,  JointID.J1, step_deg=-0.2, kp=180, kd=2.1)
    time.sleep(0.5)
    interpolate(openarm, arm, JointID.J1, np.deg2rad(80))
    time.sleep(0.5)

    interpolate(openarm, arm, JointID.J4, -np.pi/2.0)
    time.sleep(2.5)

    ideal = {
        JointID.J7: np.pi/2.0, JointID.J6: np.pi/4.0, JointID.J5: np.pi/2.0,
        JointID.J4: 0.0, JointID.J3: np.pi/2.0,
        JointID.J2: np.deg2rad(-10), JointID.J1: np.deg2rad(-80),
    }
    deltas = [d_j1, d_j2, d_j3, d_j4, d_j5, d_j6, d_j7, d_grip]
    return ideal, deltas


def _run_left_sequence(openarm, arm, grip):
    d_grip = 0.0  # gripper bump removed - was the hang, result was discarded
    d_j4 = bump_to_limit(openarm, arm,  JointID.J4, step_deg=-0.2)
    time.sleep(0.5)

    interpolate(openarm, arm, JointID.J2, -np.deg2rad(5), interp_time=0.4)
    time.sleep(0.5)
    d_j3 = bump_to_limit(openarm, arm,  JointID.J3)
    time.sleep(0.5)

    interpolate(openarm, arm, JointID.J3, -mech_lim[JointID.J3][1], interp_time=1.0)
    time.sleep(0.5)
    interpolate(openarm, arm, JointID.J2,  np.deg2rad(5),  interp_time=0.4)
    time.sleep(0.5)
    interpolate(openarm, arm, JointID.J4,  np.pi/2.0)
    time.sleep(0.5)

    d_j5 = bump_to_limit(openarm, arm,  JointID.J5)
    time.sleep(0.5)
    interpolate(openarm, arm, JointID.J5, -mech_lim[JointID.J5][1])
    time.sleep(0.5)

    d_j6 = bump_to_limit(openarm, arm,  JointID.J6)
    time.sleep(0.5)
    interpolate(openarm, arm, JointID.J6, -mech_lim[JointID.J6][1])
    time.sleep(0.5)

    d_j7 = bump_to_limit(openarm, arm,  JointID.J7)
    time.sleep(0.5)
    interpolate(openarm, arm, JointID.J7, -mech_lim[JointID.J7][1])
    time.sleep(0.5)

    d_j2 = bump_to_limit(openarm, arm,  JointID.J2)
    time.sleep(0.5)
    interpolate(openarm, arm, JointID.J2, -np.deg2rad(10), interp_time=0.9, kp=180, kd=2.0)
    time.sleep(0.5)

    d_j1 = bump_to_limit(openarm, arm,  JointID.J1, kp=180, kd=2.1)
    time.sleep(0.5)
    interpolate(openarm, arm, JointID.J1, -np.deg2rad(80))
    time.sleep(0.5)

    interpolate(openarm, arm, JointID.J4, -np.pi/2.0)
    time.sleep(2.5)

    ideal = {
        JointID.J7: np.pi/2.0, JointID.J6: np.pi/4.0, JointID.J5: np.pi/2.0,
        JointID.J4: 0.0, JointID.J3: np.pi/2.0,
        JointID.J2: np.deg2rad(10), JointID.J1: np.deg2rad(80),
    }
    deltas = [d_j1, d_j2, d_j3, d_j4, d_j5, d_j6, d_j7, d_grip]
    return ideal, deltas

# ---------- Main ----------


def main():
    parser = argparse.ArgumentParser(description='Zero-pos calibration (patched)')
    parser.add_argument('--canport', type=str, default='can0')
    parser.add_argument('--arm-side', type=str, default='right_arm',
                        choices=['right_arm', 'left_arm'])
    args = parser.parse_args()
    print(f"parser arg : {args}")

    openarm = oa.OpenArm(args.canport, True)
    openarm.init_arm_motors(
        [oa.MotorType.DM8009, oa.MotorType.DM8009, oa.MotorType.DM4340, oa.MotorType.DM4340,
         oa.MotorType.DM4310, oa.MotorType.DM4310, oa.MotorType.DM4310],
        [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07],
        [0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17]
    )
    openarm.init_gripper_motor(oa.MotorType.DM4310, 0x08, 0x18)
    openarm.set_callback_mode_all(oa.CallbackMode.STATE)

    # Motors are still limp here (not enabled yet), so the operator can move the
    # arm by hand. The bump sequence assumes a rough starting pose so each bump
    # travels *into* the correct hard stop and the arm doesn't collide with itself.
    print("\n" + "=" * 70)
    print(f"[ACTION REQUIRED] Arm side: {args.arm_side}")
    print("Physically move the arm into its approximate HOME pose while the")
    print("motors are still OFF/limp:")
    print("  - upper arm hanging straight down")
    print("  - elbow (J4) slightly bent")
    print("  - wrist (J5/J6/J7) roughly centered")
    print("Keep the workspace clear - joints will drive into their hard stops.")
    print("=" * 70)
    input("Press ENTER when the arm is posed and clear, to enable motors and begin...")

    print("Enabling...")
    openarm.enable_all()
    time.sleep(0.1)
    print("Enabled...")

    openarm.refresh_all()   # poll so initial_arm_q reflects real positions, not zeros
    openarm.recv_all()
    arm = openarm.get_arm()
    grip = openarm.get_gripper()
    am = arm.get_motors()
    gm = grip.get_motors()

    initial_arm_q = [m.get_position() for m in am]
    initial_grip_q = [m.get_position() for m in gm]

    arm_params = [oa.MITParam(kp, kd, q, 0.0, 0.0)
                  for kp, kd, q in zip([300, 300, 150, 150, 40, 40, 30],
                                       [2.5, 2.5, 2.5, 2.5, 0.8, 0.8, 0.8],
                                       initial_arm_q)]
    grip_params = [oa.MITParam(10.0, 0.9, initial_grip_q[0], 0.0, 0.0)]
    arm.mit_control_all(arm_params)
    grip.mit_control_all(grip_params)
    openarm.recv_all()

    try:
        ideal, deltas = (_run_right_sequence if args.arm_side == 'right_arm'
                         else _run_left_sequence)(openarm, arm, grip)

        joint_order = [JointID.J1, JointID.J2, JointID.J3, JointID.J4,
                       JointID.J5, JointID.J6, JointID.J7, JointID.GRIPPER]
        ideal_delta_map = {}
        for k, (j, d) in enumerate(zip(joint_order, deltas)):
            # gripper is the last slot; skip by position, not by value
            if k == len(joint_order) - 1:
                print("ideal[GRIPPER] skipped")
                continue
            val = calc_delta_to_zero_pos_joint(
                initial_rad=initial_arm_q[int(j)],
                ideal_limit_rad=ideal[j],
                delta_to_stop_rad=d,
                joint_id=j
            )
            ideal_delta_map[j] = val

        arm_goal_abs = [initial_arm_q[i] +
                        ideal_delta_map.get(JointID(i), 0.0) for i in range(7)]
        # TODO: move_to_precise_home(openarm, arm_goal_abs)

        # Do NOT write zero if any bump failed: set_zero_all() zeros the motors
        # at the CURRENT pose, which is only correct if every joint reached its
        # hard stop. A failed bump leaves the arm in a garbage pose.
        if BUMP_FAILURES:
            print(f"\n[ABORT] Not writing zero. {len(BUMP_FAILURES)} joint(s) never "
                  f"found a mechanical stop: {BUMP_FAILURES}.")
            print("        The final pose is unreliable; writing zero now would set a "
                  "wrong offset. Fix the bump (threshold/kp or mechanics) and rerun.")
        else:
            confirm = input("\nAll bumps found their stops. Type 'y' + ENTER to WRITE "
                            "ZERO at the current pose (overwrites motor zero offsets): ")
            if confirm.strip().lower() == "y":
                openarm.disable_all()
                openarm.recv_all()
                openarm.set_zero_all()
                openarm.recv_all()
                print("wrote zero position to arm")
            else:
                print("[INFO] Zero NOT written (user declined).")

    except KeyboardInterrupt:
        print("\n[INFO] Ctrl+C pressed -> stopping safely")
    finally:
        openarm.disable_all()
        print("[INFO] Motors disabled, exiting safely.")


if __name__ == "__main__":
    main()
