# motor_test

This directory contains a small set of Python scripts for testing Feetech motor communication, scanning for motors, driving a single motor, repeatedly opening and closing a gripper, and configuring/calibrating the motors of an Automatic Tool Changer (ATC).

`atc_setup.py` and the LeRobot test scripts use LeRobot's Feetech motor bus and require `feetech-servo-sdk` (imported as `scservo_sdk`). `atc_test.py` and `test_scs_scan.py` use the standalone Feetech SDK, `ftservo-python-sdk`, which also imports as `scservo_sdk`. These two SDK packages conflict with each other, so use one environment per SDK.

For the Chinese version of this guide, see [README.zh.md](/Users/jonathanlehner/wundercode/robotics/motor_test/README.zh.md).

## Quick start with the bash script

The first and simplest way to prepare the environment is:

```bash
./setup_env.sh
```

That command will:

- create `.venv` if it does not exist
- activate the virtual environment inside the script
- try China-friendly package mirrors first
- upgrade `pip`
- install everything from `requirements.txt`

If you want the virtual environment to stay active in your current shell, use:

```bash
source ./setup_env.sh
```

After that, you can run the scripts with `python ...`.

By default the script tries these package indexes in order:

- Tsinghua Tuna mirror
- Aliyun PyPI mirror
- the default PyPI index as a final fallback

If you want to force a specific mirror, you can override it:

```bash
PIP_INDEX_URL=https://pypi.tuna.tsinghua.edu.cn/simple \
PIP_TRUSTED_HOST=pypi.tuna.tsinghua.edu.cn \
./setup_env.sh
```

## Files

- `setup_env.sh`: creates or reuses `.venv` and installs dependencies
- `atc_setup.py`: configures Feetech motor IDs for an Automatic Tool Changer
- `atc_test.py`: calibrates and tests ATC lock and tool motors interactively
- `test_raw.py`: raw byte-level scanner (pure pyserial, no SDK) — the ground-truth tool for "is the servo even talking?"
- `test_scs_scan.py`: scans for ST/SMS and SC servos using the Feetech SDK high-level classes
- `test_waveshare_communication.py`: checks basic serial communication with a Waveshare controller board
- `test_motor_scan.py`: scans Feetech motors across several baud rates
- `test_single_motor.py`: controls one motor and provides an interactive position prompt
- `test_open_close.py`: repeatedly opens and closes a gripper
- `lerobot_setup_motors.py`: runs LeRobot motor setup for supported devices

## Requirements

- Python 3.10 or newer
- A connected serial device
- A powered Feetech motor / Waveshare controller board

### SDK note: feetech-servo-sdk vs ftservo-python-sdk (conflict)

Two different Feetech SDK packages install a Python module named `scservo_sdk`, but they expose different APIs. Whichever package is installed last wins that module name and breaks scripts that expect the other API.

For `atc_setup.py`, LeRobot scripts, and the default `requirements.txt`, use **`feetech-servo-sdk`**:

```bash
pip uninstall -y ftservo-python-sdk
pip install feetech-servo-sdk==1.0.0
```

For `atc_test.py` and `test_scs_scan.py`, use **`ftservo-python-sdk`** in a separate environment:

```bash
pip uninstall -y feetech-servo-sdk
pip install ftservo-python-sdk
```

| Package | Provides | Used by |
|---|---|---|
| `feetech-servo-sdk` | `PacketHandler` | `atc_setup.py`, LeRobot scripts, `test_motor_scan.py`, `test_single_motor.py`, `test_open_close.py` |
| `ftservo-python-sdk` | `sms_sts`, `scscl` | `atc_test.py`, `test_scs_scan.py` |

Keeping both sets working at once requires separate virtualenvs, or vendoring one SDK under a private module name.

To verify that the current environment is ready for `atc_setup.py` and LeRobot:

```bash
python -c "import scservo_sdk; print(scservo_sdk.__file__); print(hasattr(scservo_sdk, 'PacketHandler'))"
```

The final line should print `True`.

## Manual installation

If you do not want to use the bash script:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -i https://pypi.tuna.tsinghua.edu.cn/simple \
  --trusted-host pypi.tuna.tsinghua.edu.cn \
  -r requirements.txt
```

If one mirror is slow or unavailable, replace the Tsinghua URL above with the Aliyun mirror:

```bash
https://mirrors.aliyun.com/pypi/simple/
```

## Finding the serial port

To find the port name of a connected Waveshare/Feetech controller board:

```bash
pipenv run lerobot-find-port
```

Plug in the board, run the command, and it will print the port (e.g. `/dev/tty.usbmodemXXXX` on macOS or `/dev/ttyUSB0` on Linux).

## Hardware setup (Waveshare Bus Servo Adapter A)

The ATC uses the **Waveshare Bus Servo Adapter (A)** — a transparent USB↔single-wire-TTL adapter for Feetech ST/SC bus servos. Two things must be right or the servo will never respond, even though everything looks connected:

1. **Control-mode jumper → position B (USB).** The adapter has a jumper that selects who drives the servo bus:
   - **A** = UART header (Raspberry Pi / ESP32 / Arduino)
   - **B** = USB (a PC over the USB-C port) ← use this for these scripts

   If the jumper is in **A**, the USB chip still echoes your transmitted bytes back to you (so a scan "sees" traffic), but your packets never reach the servo and it never replies. This looks exactly like a dead motor. See the [wiring example](https://docs.waveshare.com/Bus_Servo_Adapter_A/Product-Wiring-Example).

2. **External power: 6–7.4 V.** USB does **not** power the servos — the adapter needs a separate supply on its DC jack / terminal block, and the voltage must suit the servos:
   - `sts3215` (ATC lock): 6–12.6 V
   - `scs0009` (tool): 4.0–7.4 V

   The overlap that powers **both** motor types is **6–7.4 V**. 5 V is too low for the sts3215 (it won't boot) and below the scs0009's 6 V nominal, so use ~6–7 V.

### How the bus works (so the scripts make sense)

- All servos share **one half-duplex single-wire bus**. TX and RX share the same line, so the adapter **echoes** every byte you send. The scripts account for this: `EchoFreePortHandler` consumes the echo before reading the real reply, and `test_raw.py` strips the echoed request and only reports a servo as "found" if there are genuine response bytes after the echo.
- Every servo on the bus must have a **unique ID** and run at the **same baud rate** (the scripts use 1 Mbps).
- Two servo families are supported and can coexist on the same bus:
  - **ST/SMS** (e.g. `sts3215`) — protocol 0, little-endian → SDK class `sms_sts`
  - **SC** (e.g. `scs0009`) — protocol 1, big-endian → SDK class `scscl`

  The scripts pick the right class from the model name, so a tool motor can be either family.

### First, prove the servo talks

Before configuring or calibrating anything, confirm the servo actually responds:

```bash
python test_raw.py --port /dev/ttyACM0 --verbose
```

- A line like `FOUND baud=1000000 ID=1 ...` means comms are good.
- "No real motor responses found (only echo)" means the servo isn't replying — check the jumper (B) and the 6–7.4 V supply first.

## What to change before running

These scripts currently hardcode the serial port and motor settings near the top of each file. Before running them, update values such as:

- `PORT`
- `MOTOR_ID`
- `MOTOR_MODEL`
- `MOTOR_NAME`
- `BAUDRATE`

Typical serial port names:

macOS:

```bash
/dev/cu.usbmodemXXXX
```

Linux:

```bash
/dev/ttyUSB0
```

## Usage

### 1. Test Waveshare communication

```bash
python test_waveshare_communication.py
```

This script opens the configured serial port, sends a broadcast ping, and reports whether any response data is received.

### 2. Scan motor IDs and baud rates

```bash
python test_motor_scan.py
```

This script cycles through common baud rates and tries to discover connected Feetech motors.

### 3. Control a single motor

```bash
python test_single_motor.py
```

This script:

- connects to one motor
- reads voltage and position when possible
- sends movement commands
- enters an interactive prompt for manual position input

If `MOTOR_NAME` is `gripper`, the expected input range is `0-100`. For other joints it is usually `-100` to `100`.

### 4. Run repeated open/close motion

```bash
python test_open_close.py
```

This is useful for simple repeatability or stress checks. Stop it with `Ctrl+C`.

### 5. Configure ATC motors

`atc_setup.py` assigns the correct IDs to Feetech motors used in an Automatic Tool Changer. Connect one motor at a time when prompted. The script scans all common baud rates automatically — no need to know the motor's current settings.

ID assignment:

| Motor | ID |
|---|---|
| ATC lock mechanism | 1 |
| Tool motor (first / only) | 2 |
| Tool motor (second, if present) | 3 |

IDs 2 and 3 are the same for every tool — they describe the motors *within* a tool, not which tool number it is.

```bash
# Configure the ATC lock motor only
pipenv run python atc_setup.py --port /dev/tty.usbmodemXXXX --target atc

# Configure a tool with 1 motor (ID 2)
pipenv run python atc_setup.py --port /dev/tty.usbmodemXXXX --target tool

# Configure a tool with 2 motors (IDs 2 and 3)
pipenv run python atc_setup.py --port /dev/tty.usbmodemXXXX --target tool --motors 2

# Configure ATC + tool (1 motor) in one go
pipenv run python atc_setup.py --port /dev/tty.usbmodemXXXX --target all

# Configure ATC + tool (2 motors) in one go
pipenv run python atc_setup.py --port /dev/tty.usbmodemXXXX --target all --motors 2
```

**How it works:** for each motor the script calls LeRobot's `FeetechMotorsBus.setup_motor()`, which scans every Feetech baud rate and ID for the single connected motor, then writes the target ID and sets the baud rate to 1 Mbps.

**Motor models.** The default models are `sts3215` for the ATC lock and `scs0009` for the tool, but the tool motor can be either family depending on the tool version. Set it with `--model` (for `--target tool`) or `--tool-model` (for `--target all`):

```bash
# ATC lock (defaults to sts3215)
python atc_setup.py --port /dev/ttyACM0 --target atc

# A tool whose motor is an scs0009 (the default)
python atc_setup.py --port /dev/ttyACM0 --target tool

# A tool whose motor is an sts3215 instead
python atc_setup.py --port /dev/ttyACM0 --target tool --model sts3215

# ATC + tool in one go, tool is sts3215
python atc_setup.py --port /dev/ttyACM0 --target all --tool-model sts3215
```

### 6. Test and calibrate ATC motors

`atc_test.py` has two modes. As with setup, the tool model is selectable (`--tool-model scs0009` default, or `--tool-model sts3215`); the ATC model defaults to `sts3215` (`--atc-model`).

**Named tool configurations.** A single ATC can carry different physical tools, each with its own motor model, motor count and range of motion. Each tool is stored under a name with `--tool NAME` (default: `default`). Calibrate each tool once; afterwards interactive mode loads that tool's model and motor count from the saved config, so you only pass `--tool NAME`. The ATC lock calibration is shared across all tools.

`atc_calibration.json` has this shape:

```json
{
  "atc":   {"locked": 1024, "unlocked": 2048},
  "tools": {
    "gripper": {"model": "scs0009", "ranges": {"tool_1": {"min": 100, "max": 900}}},
    "welder":  {"model": "sts3215", "ranges": {"tool_1": {"min": 0, "max": 4095}}}
  }
}
```

**Calibration** (`--calibrate atc|tool|all`): records the ATC lock/unlock positions and the tool motor range of motion. Results are saved to `atc_calibration.json`. The motors' torque is disabled during calibration so you can move them by hand. When calibrating a tool, `--tool-model` and `--motors` describe that tool and are saved into its config.

```bash
# Calibrate the ATC lock only (shared by every tool)
python atc_test.py --port /dev/ttyACM0 --calibrate atc

# Calibrate a tool named "gripper" (scs0009 default)
python atc_test.py --port /dev/ttyACM0 --tool gripper --calibrate tool

# Calibrate a tool named "welder" with an sts3215 motor
python atc_test.py --port /dev/ttyACM0 --tool welder --tool-model sts3215 --calibrate tool

# Tool with 2 motors
python atc_test.py --port /dev/ttyACM0 --tool gripper --motors 2 --calibrate tool

# Calibrate ATC + a named tool in one go
python atc_test.py --port /dev/ttyACM0 --tool welder --tool-model sts3215 --calibrate all
```

During calibration you are prompted to:
1. Move the ATC to the locked position → press ENTER
2. Move the ATC to the unlocked position → press ENTER
3. Move each tool motor through its full range → the live position is printed as you move it; press ENTER to stop. The min/max seen become the tool's range. If no position is ever read (motor unpowered, wrong ID, or off the bus), calibration aborts with a clear error instead of crashing.

**Interactive mode** (no `--calibrate`): loads the saved calibration and accepts commands. Select which tool to drive with `--tool NAME`; its model and motor count come from the saved config (no need to repass `--tool-model`/`--motors`). The ATC and tool motors share the one bus, so they're driven over a single connection. If the named tool isn't calibrated, ATC-only control is still available.

```bash
# Drive the "gripper" tool
python atc_test.py --port /dev/ttyACM0 --tool gripper
```

| Command | Action |
|---|---|
| `l` | Lock ATC |
| `u` | Unlock ATC |
| `a` | Activate tool (move to range max) |
| `h` | Home tool (move to range min) |
| `q` | Quit |

### 7. Run LeRobot motor setup

Example:

```bash
python lerobot_setup_motors.py \
  --teleop.type=so100_leader \
  --teleop.port=/dev/tty.usbmodemXXXX
```

You can also use a `robot` configuration instead of `teleop`. Supported device types in the script are:

- `koch_follower`
- `koch_leader`
- `so100_follower`
- `so100_leader`
- `so101_follower`
- `so101_leader`
- `lekiwi`

## Troubleshooting

### Scan finds nothing / motor never responds ("only echo")

This is the most common ATC problem and it's almost always the adapter, not the motor or the code. In order:

1. **Jumper in position B (USB).** In position A the USB port is disconnected from the servo bus; you get the adapter's echo but no servo reply. This produces "No real motor responses found (only echo)" in `test_raw.py`.
2. **Supply is 6–7.4 V** (not USB-only, not 5 V). See [Hardware setup](#hardware-setup-waveshare-bus-servo-adapter-a).
3. **3-pin cable seated and not reversed.**
4. Run `python test_raw.py --port <PORT> --verbose` — if `RX=` shows only the echo of your `TX=` and nothing after it, the servo isn't getting the command (revisit 1–3). If you see extra bytes after the echo, the servo is talking and the higher-level scripts will work.

### No response received

Check:

- the motor has external power (6–7.4 V on the adapter, not USB alone)
- the control-mode jumper is in position B (USB)
- the serial port path is correct
- the baud rate is correct
- the motor ID is correct

### "There is no status packet"

This usually means the device is not returning status packets. Common causes:

- wrong serial port
- wrong baud rate
- bus wiring issue
- motor not powered

### Overload / voltage errors

These errors are commonly related to:

- insufficient supply voltage
- too much mechanical resistance in the gripper
- unsuitable torque limits
- commanded positions outside the usable movement range

## Notes

These scripts are closer to debugging utilities than a reusable Python package. If this directory is going to be used regularly, the next practical improvement is to replace the hardcoded configuration values with command-line arguments.
