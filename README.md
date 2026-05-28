# motor_test

This directory contains a small set of Python scripts for testing LeRobot/Feetech motor communication, scanning for motors, driving a single motor, and repeatedly opening and closing a gripper.

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
- `test_waveshare_communication.py`: checks basic serial communication with a Waveshare controller board
- `test_motor_scan.py`: scans Feetech motors across several baud rates
- `test_single_motor.py`: controls one motor and provides an interactive position prompt
- `test_open_close.py`: repeatedly opens and closes a gripper
- `lerobot_setup_motors.py`: runs LeRobot motor setup for supported devices

## Requirements

- Python 3.10 or newer
- A connected serial device
- A powered Feetech motor / Waveshare controller board

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

### 6. Run LeRobot motor setup

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

### No response received

Check:

- the motor has external power
- TX/RX wiring is correct
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
