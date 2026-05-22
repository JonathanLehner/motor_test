#!/usr/bin/env python3
"""Teleop trigger -> follower gripper mirror.

Reads the trigger button's position on the leader device and mirrors it to
the follower gripper.  The two hardware-specific pieces are isolated behind
the TriggerReader and GripperController abstractions -- swap in the
concrete implementation that matches your setup.

Usage:
    # 1. Calibrate raw min/max (full release / full squeeze)
    python teleop_trigger.py --reader feetech --calibrate

    # 2. Run the mirror loop
    python teleop_trigger.py --reader feetech --gripper dm4310 \
        --can-port /dev/tty.usbmodemXXXX \
        --raw-min 1676 --raw-max 2236
"""

from __future__ import annotations

import argparse
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass

import numpy as np


# ----------------------------------------------------------------------
# Abstractions
# ----------------------------------------------------------------------

class TriggerReader(ABC):
    """Reads the raw position of the trigger button.

    `read_raw` returns a scalar in whatever units the underlying sensor
    produces (servo ticks, ADC counts, radians, ...).  Calibration maps
    those raw units to a normalized [0, 1] command downstream.
    """

    @abstractmethod
    def read_raw(self) -> float: ...

    def close(self) -> None:
        pass


class GripperController(ABC):
    """Commands the follower gripper in normalized [0, 1] space.

    0.0 = fully closed, 1.0 = fully open.  The implementation maps this
    to the actuator's native units (ticks, millimetres, percent, ...).
    """

    @abstractmethod
    def set_normalized(self, pos: float) -> None: ...

    def close(self) -> None:
        pass


# ----------------------------------------------------------------------
# Concrete trigger readers
# ----------------------------------------------------------------------

class FeetechTriggerReader(TriggerReader):
    """Trigger position from a Feetech servo on the LeRobot leader bus.

    Use this if the trigger is mechanically coupled to a Feetech motor
    whose encoder serves as the position sensor (typical SO-ARM-style
    leader: torque off, read Present_Position).
    """

    def __init__(self, port: str, motor_id: int, model: str = "sts3215"):
        # Lazy import so the file is importable without lerobot installed.
        from lerobot.motors import Motor, MotorCalibration, MotorNormMode
        from lerobot.motors.feetech import FeetechMotorsBus

        self.bus = FeetechMotorsBus(
            port=port,
            motors={
                "trigger": Motor(motor_id, model, MotorNormMode.RANGE_0_100),
            },
            calibration={
                "trigger": MotorCalibration(
                    id=motor_id,
                    drive_mode=0,
                    homing_offset=0,
                    range_min=0,
                    range_max=4095,
                ),
            },
        )
        self.bus.connect(handshake=False)
        # Free the trigger so the operator can move it.
        self.bus.disable_torque("trigger")

    def read_raw(self) -> float:
        return float(self.bus.read("Present_Position", "trigger", normalize=False))

    def close(self) -> None:
        try:
            self.bus.disconnect()
        except Exception:
            pass


class SerialTriggerReader(TriggerReader):
    """Trigger position over a USB-serial link to a microcontroller.

    Use this if the trigger has a potentiometer / Hall sensor wired to
    an Arduino / RP2040 / ESP32 that prints one raw ADC value per line.
    """

    def __init__(self, port: str = "/dev/ttyACM0", baudrate: int = 115200):
        import serial

        self.ser = serial.Serial(port, baudrate, timeout=0.1)
        # Discard boot noise.
        time.sleep(0.5)
        self.ser.reset_input_buffer()

    def read_raw(self) -> float:
        # Drain to the latest sample so we don't lag behind buffered data.
        last = None
        while self.ser.in_waiting:
            last = self.ser.readline()
        if last is None:
            last = self.ser.readline()
        try:
            return float(last.decode(errors="ignore").strip())
        except ValueError:
            # Malformed line -- skip by returning the previous value if you
            # extend this class to remember it.  For now, re-raise upward.
            raise

    def close(self) -> None:
        try:
            self.ser.close()
        except Exception:
            pass


# ----------------------------------------------------------------------
# Concrete gripper controllers
# ----------------------------------------------------------------------

class FeetechGripperController(GripperController):
    """Drive a Feetech gripper motor on the follower bus directly.

    Use this if the follower gripper is the last motor in the SO-100M /
    Nova5 joint chain.  `closed_ticks` and `open_ticks` come from manual
    calibration of the gripper jaw extremes.
    """

    def __init__(self, port: str, motor_id: int, model: str = "sts3215",
                 closed_ticks: int = 1900, open_ticks: int = 2800):
        from lerobot.motors import Motor, MotorCalibration, MotorNormMode
        from lerobot.motors.feetech import FeetechMotorsBus

        self.bus = FeetechMotorsBus(
            port=port,
            motors={
                "grip": Motor(motor_id, model, MotorNormMode.RANGE_0_100),
            },
            calibration={
                "grip": MotorCalibration(
                    id=motor_id,
                    drive_mode=0,
                    homing_offset=0,
                    range_min=0,
                    range_max=4095,
                ),
            },
        )
        self.bus.connect(handshake=False)
        self.bus.enable_torque("grip")
        self.closed_ticks = closed_ticks
        self.open_ticks = open_ticks

    def set_normalized(self, pos: float) -> None:
        pos = float(np.clip(pos, 0.0, 1.0))
        ticks = int(self.closed_ticks + pos * (self.open_ticks - self.closed_ticks))
        self.bus.write("Goal_Position", "grip", ticks, normalize=False)

    def close(self) -> None:
        try:
            self.bus.disconnect()
        except Exception:
            pass


class DM4310GripperController(GripperController):
    """Drive a Damiao DM4310 gripper motor over CAN (via USB-CAN adapter).

    pos=0.0 -> closed_pos (fully closed), pos=1.0 -> open_pos (fully open).
    Uses MIT position control with the kp/kd tuned for gripper use.
    """

    def __init__(self, port: str, baud: int = 921600, can_id: int = 0x01,
                 master_id: int = 0x11,
                 open_pos: float = 1.047, closed_pos: float = 0.0,
                 kp: float = 10.0, kd: float = 0.5,
                 max_vel: float = 1.5, rate_hz: float = 50.0):
        import sys
        import os
        import serial

        dm_dir = os.path.join(os.path.dirname(__file__), "DM_Control_Python")
        if dm_dir not in sys.path:
            sys.path.insert(0, dm_dir)
        from DM_CAN import Motor, MotorControl, DM_Motor_Type

        self.open_pos = open_pos
        self.closed_pos = closed_pos
        self.kp = kp
        self.kd = kd
        self._rate_hz = rate_hz
        self.max_step = max_vel / rate_hz  # max rad per control tick

        ser = serial.Serial(port, baud, timeout=0.5)
        self.ctrl = MotorControl(ser)
        self.motor = Motor(DM_Motor_Type.DM4310, can_id, master_id)
        self.ctrl.addMotor(self.motor)
        self.ctrl.enable(self.motor)

        import time
        time.sleep(0.1)
        self.ctrl.refresh_motor_status(self.motor)
        self._cmd_pos: float = float(self.motor.getPosition())
        print(f"[dm4310] motor pos at startup: {self._cmd_pos:.4f} rad")
        print(f"[dm4310] open={open_pos:.4f} rad  closed={closed_pos:.4f} rad")
        low = min(open_pos, closed_pos)
        high = max(open_pos, closed_pos)
        if not low <= self._cmd_pos <= high:
            print(
                "[dm4310] WARNING: startup position is outside the configured "
                "open/closed range; both trigger extremes may initially move "
                "the motor in the same direction. Recalibrate --gripper-open "
                "and --gripper-closed from actual reported positions."
            )

    def enable_torque(self) -> None:
        self.ctrl.enable(self.motor)

    def disable_torque(self) -> None:
        self.ctrl.disable(self.motor)

    def set_range(self, open_pos: float, closed_pos: float) -> None:
        self.open_pos = open_pos
        self.closed_pos = closed_pos
        print(f"[dm4310] calibrated open={open_pos:.4f} rad  closed={closed_pos:.4f} rad")

    def _refresh_reported_state(self) -> tuple[float, float, float] | None:
        try:
            self.ctrl.refresh_motor_status(self.motor)
            return (
                float(self.motor.getPosition()),
                float(self.motor.getVelocity()),
                float(self.motor.getTorque()),
            )
        except Exception as exc:
            print(f"[dm4310] status refresh failed: {exc}")
            return None

    def set_normalized(self, pos: float, verbose: bool = False) -> bool:
        """Send a position command. Returns True if still ramping toward target."""
        pos = float(np.clip(pos, 0.0, 1.0))
        target = self.closed_pos + pos * (self.open_pos - self.closed_pos)
        return self.move_toward(target, pos=pos, verbose=verbose)

    def move_toward(self, target: float, pos: float | None = None,
                    verbose: bool = False) -> bool:
        """Send a position command toward an absolute target in motor radians."""
        delta = target - self._cmd_pos
        step = float(np.clip(delta, -self.max_step, self.max_step))
        self._cmd_pos += step
        vel_ff = step * self._rate_hz  # feedforward: D term reinforces motion instead of fighting it
        self.ctrl.controlMIT(self.motor, self.kp, self.kd, self._cmd_pos, vel_ff, 0.0)
        still_ramping = abs(delta) > self.max_step * 0.5
        if verbose:
            reported_state = self._refresh_reported_state()
            prefix = f"[dm4310] trigger_norm={pos:.3f}" if pos is not None else "[dm4310]"
            reported = ""
            if reported_state is not None:
                reported_pos, reported_vel, reported_torque = reported_state
                reported = (
                    f"  actual={reported_pos:.4f} rad"
                    f"  vel={reported_vel:+.4f} rad/s"
                    f"  torque={reported_torque:+.4f} Nm"
                )
            print(
                prefix
                + f"  target={target:.4f} rad"
                + f"  delta={delta:+.4f}"
                + f"  step={step:+.4f}"
                + f"  cmd={self._cmd_pos:.4f} rad"
                + f"{reported}"
                + f"  {'RAMPING' if still_ramping else 'settled'}"
            )
        return still_ramping

    def reported_position(self) -> float | None:
        state = self._refresh_reported_state()
        return None if state is None else state[0]

    def close(self) -> None:
        try:
            self.ctrl.disable(self.motor)
        except Exception:
            pass


# ----------------------------------------------------------------------
# Mirror loop
# ----------------------------------------------------------------------

@dataclass
class MirrorConfig:
    raw_min: float           # raw value when trigger fully released
    raw_max: float           # raw value when trigger fully squeezed
    invert: bool = False     # True if squeezing should OPEN the gripper
    rate_hz: float = 50.0    # control rate
    deadband: float = 0.01   # min normalized change before sending a command
    ema_alpha: float = 0.3   # 0.0 = no smoothing, 1.0 = no memory


def normalize(raw: float, cfg: MirrorConfig) -> float:
    if cfg.raw_max == cfg.raw_min:
        return 0.0
    x = (raw - cfg.raw_min) / (cfg.raw_max - cfg.raw_min)
    x = float(np.clip(x, 0.0, 1.0))
    return 1.0 - x if cfg.invert else x


def run_mirror(reader: TriggerReader, gripper: GripperController,
               cfg: MirrorConfig, verbose: bool = False) -> None:
    period = 1.0 / cfg.rate_hz
    last_sent: float | None = None
    smoothed: float | None = None
    gripper_ramping = True  # assume unsettled until first command confirms otherwise
    tick = 0
    print(f"[teleop] mirror loop @ {cfg.rate_hz:.0f} Hz  "
          f"raw_min={cfg.raw_min}  raw_max={cfg.raw_max}  "
          f"invert={cfg.invert}  deadband={cfg.deadband}")
    print("[teleop] Ctrl-C to stop.")
    try:
        while True:
            t0 = time.perf_counter()
            tick += 1

            raw = reader.read_raw()
            pos = normalize(raw, cfg)
            prev_smoothed = smoothed
            smoothed = pos if smoothed is None else (
                cfg.ema_alpha * pos + (1.0 - cfg.ema_alpha) * smoothed
            )

            trigger_moved = last_sent is None or abs(pos - last_sent) > cfg.deadband
            if trigger_moved:
                last_sent = pos

            if trigger_moved or gripper_ramping:
                gripper_ramping = gripper.set_normalized(smoothed, verbose=verbose)
                reason = "trigger" if trigger_moved else "ramping"
            else:
                gripper_ramping = False
                reason = "deadband"

            if verbose:
                print(
                    f"[teleop] tick={tick:05d}"
                    f"  raw={raw:.1f}"
                    f"  pos={pos:.3f}"
                    f"  smoothed={smoothed:.3f}"
                    f"  sent={reason}"
                )

            dt = time.perf_counter() - t0
            if dt < period:
                time.sleep(period - dt)
    except KeyboardInterrupt:
        print("\n[teleop] stopped.")


# ----------------------------------------------------------------------
# Calibration
# ----------------------------------------------------------------------

def calibrate(reader: TriggerReader, n_samples: int = 100) -> tuple[float, float]:
    """Walk the operator through full-release / full-squeeze and report bounds."""

    def sample(prompt: str) -> float:
        input(prompt + " -- hold steady, then press Enter.")
        samples = [reader.read_raw() for _ in range(n_samples)]
        return float(np.median(samples))

    a = sample("Release the trigger fully (gripper OPEN side)")
    b = sample("Squeeze the trigger fully (gripper CLOSED side)")
    raw_min, raw_max = sorted((a, b))
    print(f"[calibrate] raw_min={raw_min:.2f}  raw_max={raw_max:.2f}")
    if a > b:
        print("[calibrate] note: released > squeezed, so pass --invert "
              "when running the mirror loop.")
    return raw_min, raw_max


def dm4310_nudge(gripper: DM4310GripperController, delta: float,
                 hold_s: float, verbose: bool = False) -> None:
    """Move the DM4310 by a small relative amount for direction testing."""
    start = gripper.reported_position()
    if start is None:
        raise RuntimeError("could not read DM4310 position")
    target = start + delta
    print(f"[dm4310-nudge] start={start:.4f} rad  target={target:.4f} rad  delta={delta:+.4f} rad")
    deadline = time.perf_counter() + hold_s
    ramping = True
    while time.perf_counter() < deadline and ramping:
        ramping = gripper.move_toward(target, verbose=verbose)
        time.sleep(1.0 / 50.0)
    end = gripper.reported_position()
    if end is not None:
        print(f"[dm4310-nudge] end={end:.4f} rad")


def calibrate_dm4310_gripper(
    gripper: DM4310GripperController,
    seconds: float,
    close_direction: str,
    sample_hz: float = 20.0,
) -> tuple[float, float]:
    """Record Damiao min/max while torque is disabled and assign open/closed."""
    print("[dm4310-calibrate] disabling torque; move the gripper through full open and full closed by hand.")
    print(f"[dm4310-calibrate] sampling for {seconds:.1f}s. Press Ctrl-C to finish early.")
    gripper.disable_torque()
    positions: list[float] = []
    period = 1.0 / sample_hz
    deadline = time.perf_counter() + seconds
    try:
        while time.perf_counter() < deadline:
            pos = gripper.reported_position()
            if pos is not None:
                positions.append(pos)
                print(
                    f"[dm4310-calibrate] pos={pos:.4f} rad"
                    f"  min={min(positions):.4f}"
                    f"  max={max(positions):.4f}"
                )
            time.sleep(period)
    except KeyboardInterrupt:
        print("\n[dm4310-calibrate] stopped early.")

    if not positions:
        raise RuntimeError("no Damiao positions were read during calibration")

    min_pos = min(positions)
    max_pos = max(positions)
    if close_direction == "positive":
        open_pos, closed_pos = min_pos, max_pos
    else:
        open_pos, closed_pos = max_pos, min_pos

    print(
        f"[dm4310-calibrate] recorded min={min_pos:.4f} rad  max={max_pos:.4f} rad"
    )
    print(
        f"[dm4310-calibrate] using open={open_pos:.4f} rad  closed={closed_pos:.4f} rad"
    )
    print(
        "[dm4310-calibrate] reuse with:"
        f" --gripper-open {open_pos:.4f} --gripper-closed {closed_pos:.4f}"
    )

    gripper.enable_torque()
    gripper._refresh_reported_state()
    current = float(gripper.motor.getPosition())
    gripper._cmd_pos = current
    gripper.set_range(open_pos, closed_pos)
    return open_pos, closed_pos


# ----------------------------------------------------------------------
# CLI plumbing
# ----------------------------------------------------------------------

def build_reader(args) -> TriggerReader:
    if args.reader == "feetech":
        return FeetechTriggerReader(port=args.leader_port,
                                     motor_id=args.trigger_id)
    if args.reader == "serial":
        return SerialTriggerReader(port=args.leader_port)
    raise ValueError(f"unknown reader: {args.reader}")


def build_gripper(args) -> GripperController:
    if args.gripper == "feetech":
        return FeetechGripperController(port=args.follower_port,
                                         motor_id=args.gripper_id)
    if args.gripper == "dm4310":
        return DM4310GripperController(
            port=args.can_port,
            baud=args.can_baud,
            can_id=args.can_id,
            master_id=args.master_id,
            open_pos=args.gripper_open,
            closed_pos=args.gripper_closed,
            max_vel=args.gripper_max_vel,
            rate_hz=args.rate,
        )
    raise ValueError(f"unknown gripper: {args.gripper}")


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--reader", choices=["feetech", "serial"], default="feetech")
    p.add_argument("--gripper", choices=["feetech", "dm4310"], default="dm4310")
    p.add_argument("--leader-port", default="/dev/ttyACM0")
    p.add_argument("--follower-port", default="/dev/ttyACM1")
    p.add_argument("--trigger-id", type=int, default=6)
    p.add_argument("--gripper-id", type=int, default=6)
    # DM4310 CAN options
    p.add_argument("--can-port", default="/dev/ttyACM2")
    p.add_argument("--can-baud", type=int, default=921600)
    p.add_argument("--can-id", type=lambda x: int(x, 0), default=0x01)
    p.add_argument("--master-id", type=lambda x: int(x, 0), default=0x11)
    p.add_argument("--gripper-open", type=float, default=1.047,
                   help="Motor position (rad) when gripper is fully open (default: 60deg from zero)")
    p.add_argument("--gripper-closed", type=float, default=0.0,
                   help="Motor position (rad) when gripper is fully closed (default: motor zero)")
    p.add_argument("--gripper-max-vel", type=float, default=1.5,
                   help="Max gripper speed in rad/s (limits jump on start, default: 1.5)")
    p.add_argument("--calibrate", action="store_true")
    p.add_argument("--calibrate-gripper", action="store_true",
                   help="DM4310 only: disable torque, record hand-moved min/max, then use them")
    p.add_argument("--calibrate-gripper-seconds", type=float, default=10.0)
    p.add_argument("--gripper-close-direction", choices=["positive", "negative"], default="positive",
                   help="Which Damiao angle direction closes the gripper during calibration")
    p.add_argument("--raw-min", type=float)
    p.add_argument("--raw-max", type=float)
    p.add_argument("--invert", action="store_true")
    p.add_argument("--rate", type=float, default=50.0)
    p.add_argument("--deadband", type=float, default=0.01)
    p.add_argument("--ema", type=float, default=0.3)
    p.add_argument("--dm-nudge", type=float,
                   help="For DM4310 only: move by this relative radian amount, then exit")
    p.add_argument("--dm-nudge-hold", type=float, default=3.0,
                   help="Seconds to keep commanding --dm-nudge target (default: 3)")
    p.add_argument("--verbose", action="store_true")
    args = p.parse_args()

    if (
        not args.calibrate
        and not args.calibrate_gripper
        and args.dm_nudge is None
        and (args.raw_min is None or args.raw_max is None)
    ):
        raise SystemExit(
            "Need --raw-min and --raw-max. Run --calibrate first."
        )

    if args.calibrate:
        reader = build_reader(args)
        try:
            calibrate(reader)
            return
        finally:
            reader.close()

    if args.calibrate_gripper or args.dm_nudge is not None:
        gripper = build_gripper(args)
        try:
            if args.calibrate_gripper:
                if not isinstance(gripper, DM4310GripperController):
                    raise SystemExit("--calibrate-gripper is only supported with --gripper dm4310")
                calibrate_dm4310_gripper(
                    gripper,
                    seconds=args.calibrate_gripper_seconds,
                    close_direction=args.gripper_close_direction,
                )
                if args.raw_min is None or args.raw_max is None:
                    return

            if args.dm_nudge is not None:
                if not isinstance(gripper, DM4310GripperController):
                    raise SystemExit("--dm-nudge is only supported with --gripper dm4310")
                dm4310_nudge(gripper, args.dm_nudge, args.dm_nudge_hold, verbose=args.verbose)
                return

            reader = build_reader(args)
            try:
                cfg = MirrorConfig(
                    raw_min=args.raw_min,
                    raw_max=args.raw_max,
                    invert=args.invert,
                    rate_hz=args.rate,
                    deadband=args.deadband,
                    ema_alpha=args.ema,
                )
                run_mirror(reader, gripper, cfg, verbose=args.verbose)
            finally:
                reader.close()
        finally:
            gripper.close()
        return

    reader = build_reader(args)
    try:
        gripper = build_gripper(args)
        try:
            cfg = MirrorConfig(
                raw_min=args.raw_min,
                raw_max=args.raw_max,
                invert=args.invert,
                rate_hz=args.rate,
                deadband=args.deadband,
                ema_alpha=args.ema,
            )
            run_mirror(reader, gripper, cfg, verbose=args.verbose)
        finally:
            gripper.close()
    finally:
        reader.close()


if __name__ == "__main__":
    main()
