# macOS DaMiao USB2CAN Helpers

This repository includes two small macOS-focused helper scripts:

- `damiao_ids_macos.py` reads and writes DaMiao motor IDs.
- `damiao_move_macos.py` performs simple one-motor status and motion tests.

They are intended for the DaMiao USB2CAN/USB2CANFD serial adapter on macOS and use the local `DM_CAN.py` library.

## Setup

Create and activate a Python environment, then install dependencies:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install pyserial numpy
```

Find the adapter device:

```bash
ls /dev/cu.*
```

The scripts default to:

```text
/dev/cu.usbmodem00000000050C1
```

If your adapter has a different device path, pass it with `--port`.

## Motor IDs

DaMiao uses two IDs:

- `CAN_ID`, also called `ESC_ID`, is the motor command ID.
- `Master_ID`, also called `MST_ID`, is the host reply ID.

A practical convention is:

```text
CAN_ID 0x01 -> Master_ID 0x11
CAN_ID 0x02 -> Master_ID 0x12
CAN_ID 0x03 -> Master_ID 0x13
```

Do not set `Master_ID` to `0x00`.

## Read or Change IDs

Scan IDs `0x01` through `0x10`:

```bash
python damiao_ids_macos.py
```

Scan a specific range:

```bash
python damiao_ids_macos.py --scan-start 0x01 --scan-end 0x05
```

Read one known motor:

```bash
python damiao_ids_macos.py --current-id 0x03 --current-master 0x13
```

Change a motor from `CAN_ID=0x01` to `CAN_ID=0x03`, and set `Master_ID=0x13`:

```bash
python damiao_ids_macos.py \
  --current-id 0x01 \
  --current-master 0x11 \
  --set-can-id 0x03 \
  --set-master-id 0x13 \
  --write
```

Writing IDs saves parameters to flash. Use this only when one motor is connected, or when you are certain which motor is responding.

## Status Check

Before moving anything, verify communication:

```bash
python damiao_move_macos.py \
  --can-id 0x03 \
  --master-id 0x13 \
  --mode status
```

Expected output includes:

```text
STATUS: pos = ... vel = ... tau = ...
```

If status works, the serial port, adapter, CAN wiring, `CAN_ID`, and `Master_ID` are at least good enough for replies. The motor type affects decoded motion units, but ID parameter reads are not expected to depend on position/velocity/torque limits.

## Motion Tests

The script can optionally switch the control mode before moving:

```bash
--switch-mode mit
--switch-mode posvel
--switch-mode vel
```

The mode switch is temporary and resets after power cycling unless saved through lower-level parameter writes.

### MIT Position

Move relative to the current position:

```bash
python damiao_move_macos.py \
  --can-id 0x03 \
  --master-id 0x13 \
  --switch-mode mit \
  --mode mit-pos \
  --position-delta -0.3 \
  --kp 8 \
  --kd 0.5 \
  --duration 2
```

For a DM4310, the MIT position field is encoded in this protocol range:

```text
-12.5 rad to +12.5 rad
```

That is not the same thing as a mechanical travel limit. The motor can rotate through the wrap point. For moves that stay inside the encoded range, the script uses MIT position commands directly. For moves that cross `+12.5` or `-12.5`, the script unwraps feedback in software and uses MIT velocity commands to avoid a discontinuous position setpoint.

Example crossing the positive wrap point:

```bash
python damiao_move_macos.py \
  --can-id 0x03 \
  --master-id 0x13 \
  --switch-mode mit \
  --mode mit-pos \
  --position-delta -0.5 \
  --kp 8 \
  --kd 0.5 \
  --duration 2
```

You can also command an absolute target inside the range:

```bash
python damiao_move_macos.py \
  --can-id 0x03 \
  --master-id 0x13 \
  --switch-mode mit \
  --mode mit-pos \
  --target-position 11.8 \
  --kp 8 \
  --kd 0.5 \
  --duration 2
```

### MIT Velocity

MIT velocity uses `kp=0`, `kd>0`, desired velocity, and zero feedforward torque:

```bash
python damiao_move_macos.py \
  --can-id 0x03 \
  --master-id 0x13 \
  --switch-mode mit \
  --mode mit-vel \
  --velocity 0.3 \
  --kd 0.5 \
  --duration 1
```

### MIT Torque

Torque mode can rotate continuously. Start very small, keep the motor unloaded, and be ready to cut power:

```bash
python damiao_move_macos.py \
  --can-id 0x03 \
  --master-id 0x13 \
  --switch-mode mit \
  --mode mit-torque \
  --torque 0.03 \
  --duration 0.5
```

### Velocity Mode

This uses DaMiao velocity control mode rather than MIT control:

```bash
python damiao_move_macos.py \
  --can-id 0x03 \
  --master-id 0x13 \
  --switch-mode vel \
  --mode vel \
  --velocity 0.3 \
  --duration 1
```

### Position-Velocity Mode

This uses DaMiao position-velocity control mode:

```bash
python damiao_move_macos.py \
  --can-id 0x03 \
  --master-id 0x13 \
  --switch-mode posvel \
  --mode posvel \
  --position-delta -0.3 \
  --velocity 0.5 \
  --duration 2
```

## Useful Options

```text
--port              Serial adapter device. Default: /dev/cu.usbmodem00000000050C1
--baud              Serial baud rate. Default: 921600
--motor-type        Motor type. Default: 4310
--can-id            Motor CAN_ID / ESC_ID. Accepts decimal or hex.
--master-id         Motor Master_ID / MST_ID. Defaults to CAN_ID + 0x10.
--switch-mode       Temporarily switch mode: mit, posvel, or vel.
--mode              status, mit-pos, mit-vel, mit-torque, vel, or posvel.
--duration          Motion command duration in seconds.
--rate              Command rate in Hz.
--position-delta    Relative position target for position tests.
--target-position   Absolute position target for position tests.
--velocity          Velocity command.
--kp                MIT position stiffness.
--kd                MIT damping.
--torque            MIT feedforward torque.
--old-enable        Use legacy firmware enable command.
--no-disable        Leave the motor enabled after the script exits.
```

## Troubleshooting

### Status Works but the Motor Does Not Move

If you see output like this:

```text
Start position:  12.3569
Target position: 12.8569
Crossing MIT position wrap; using unwrapped feedback with MIT velocity commands
```

the script is crossing the MIT command wrap point and is using continuous software control. If the motor does not move, try the same size move in the other direction to check whether the issue is directional, mechanical, or control-mode related:

```bash
python damiao_move_macos.py --can-id 0x03 --master-id 0x13 --switch-mode mit --mode mit-pos --position-delta -0.5 --kp 8 --kd 0.5 --duration 2
```

If negative position moves still do nothing, try these checks:

- Increase `kp` slightly for MIT position, for example `--kp 10 --kd 0.5`.
- Test `mit-torque` with a very small torque to confirm the motor can produce motion.
- Test `vel` mode with `--switch-mode vel --mode vel`.
- Confirm the motor is not against a hard stop or mechanically constrained.
- Confirm the motor is not disabled by an external fault or low supply voltage.
- Try `--old-enable` if the firmware is old.
- Power-cycle the motor after mode or ID changes.

### Mode Switch Says FAILED

Check that `CAN_ID` and `Master_ID` are correct:

```bash
python damiao_ids_macos.py --current-id 0x03 --current-master 0x13
```

Also verify that only one process has the serial adapter open.

### Serial Port Cannot Open

List macOS serial devices:

```bash
ls /dev/cu.*
```

Then pass the actual port:

```bash
python damiao_move_macos.py --port /dev/cu.usbmodemXXXX --mode status
```

### No Parameter Replies During ID Scan

If movement/status works but ID scanning does not, the motor may reply on a different `Master_ID`, another process may have the serial port open, or the motor firmware may not respond to parameter reads in the expected way. The `--motor-type` value is not the first suspect for ID scanning. Read the known ID directly first:

```bash
python damiao_ids_macos.py --current-id 0x03 --current-master 0x13
```

## Safety Notes

- Keep the motor unloaded for first tests.
- Start with small velocity, torque, and position deltas.
- Keep a physical power cutoff within reach.
- Do not use `--no-disable` until the control behavior is understood.
- Do not write IDs while multiple motors are connected unless the bus setup is intentional.

## Raw DaMiao CAN Helpers

`damiao_raw_send_macos.py` sends one raw CAN frame through the DaMiao USB2CAN serial protocol used by `DM_CAN.py`.

`damiao_raw_log_macos.py` logs raw adapter reply frames using the same 16-byte packet format parsed by `DM_CAN.py`.

Log adapter frames:

```bash
python damiao_raw_log_macos.py
```

Log adapter frames and raw serial bytes:

```bash
python damiao_raw_log_macos.py --raw
```

Actively poll status for motor `0x01` while logging:

```bash
python damiao_raw_log_macos.py --poll-status-id 0x01 --raw
```

Send the equivalent payload of `cansend can0 001#FFFFFFFFFFFFFFFC` through the DaMiao adapter:

```bash
python damiao_raw_send_macos.py 001#FFFFFFFFFFFFFFFC
```

Use a specific serial port:

```bash
python damiao_raw_send_macos.py --port /dev/cu.usbmodem00000000050C1 001#FFFFFFFFFFFFFFFC
```

Repeat a frame:

```bash
python damiao_raw_send_macos.py 001#FFFFFFFFFFFFFFFC --repeat 5 --delay 0.02
```

This helper is for low-level testing. Prefer `damiao_move_macos.py` and `damiao_ids_macos.py` for normal motor operations because they decode replies and handle the DaMiao control modes.
