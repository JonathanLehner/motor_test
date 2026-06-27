# motor_test

This directory contains a small set of Python scripts for testing Feetech motor communication, scanning for motors, driving a single motor, repeatedly opening and closing a gripper, and configuring/calibrating the motors of an Automatic Tool Changer (ATC).

`atc_setup.py`, `atc_test.py`, and the LeRobot test scripts use LeRobot's Feetech motor bus and require `feetech-servo-sdk` (imported as `scservo_sdk`). `test_scs_scan.py` uses the standalone Feetech SDK, `ftservo-python-sdk`, which also imports as `scservo_sdk`. These two SDK packages conflict with each other, so use one environment per SDK.

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
- `apriltag_cube_pose.py`: estimates the 3D position of an AprilTag-marked cube from the external stereo camera of a LeRobot v3.0 dataset, with two selectable detector backends (`pupil`/`aruco`)
- `episode_picker.py`: web viewer/clip-splitter for a LeRobot v3.0 dataset; also overlays the detected AprilTag and estimated cube position on the external camera
- `camera_test.py`: opens the camera(s) for a live preview and reports the resolution they actually deliver (`--list` enumerates indices)
- `teleop_trigger_record.py`: records the two side-by-side stereo cameras + gripper trigger to a LeRobot v3.0 dataset
- `ik_track_cube.py`: generates a Nova5 arm trajectory (mink IK) that tracks the cube estimate and bakes it into the dataset (`observation.arm_qpos_ik`, `observation.ee_pose_target`)
- `ik_replay.py`: replays a generated IK trajectory in the interactive MuJoCo viewer
- `render_sim_video.py`: renders the IK trajectory to video and adds it to the dataset as a new camera feature (`observation.images.sim`)

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

For `test_scs_scan.py`, use **`ftservo-python-sdk`** in a separate environment:

```bash
pip uninstall -y feetech-servo-sdk
pip install ftservo-python-sdk
```

| Package | Provides | Used by |
|---|---|---|
| `feetech-servo-sdk` | `PacketHandler` | `atc_setup.py`, `atc_test.py`, LeRobot scripts, `test_motor_scan.py`, `test_single_motor.py`, `test_open_close.py` |
| `ftservo-python-sdk` | `sms_sts`, `scscl` | `test_scs_scan.py` |

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

`atc_test.py` uses LeRobot and has two modes. As with setup, the tool model is selectable (`--tool-model scs0009` default, or `--tool-model sts3215`); the ATC model defaults to `sts3215` (`--atc-model`).

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

**Interactive mode** (no `--calibrate`): loads the saved calibration and accepts commands. Select which tool to drive with `--tool NAME`; its model and motor count come from the saved config (no need to repass `--tool-model`/`--motors`). If the ATC and tool use different Feetech protocol families, the script opens the port for the active motor family per command. If the named tool isn't calibrated, ATC-only control is still available.

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

### 8. Estimate AprilTag cube pose from a dataset

`apriltag_cube_pose.py` detects a `tag36h11` AprilTag (the cube marker) in the
external **stereo** camera of a LeRobot v3.0 dataset and estimates the cube's 3D
position two ways: stereo triangulation (left/right disparity) and per-eye
`solvePnP`. Each frame is a side-by-side stereo image (left eye = left half,
right eye = right half). Results are written to a sidecar parquet, and can
optionally be baked back into the dataset as an `observation.cube_position`
feature.

This script needs `opencv-python`, `numpy`, `pandas`, and — for the default
detector — `pupil-apriltags`:

```bash
pip install opencv-python numpy pandas pupil-apriltags
```

```bash
# Default run: pupil backend, auto-pick a tag seen in both eyes
python apriltag_cube_pose.py --dataset lerobot_dataset_clean

# Override marker assumptions (50 mm tag, id 0, 60 mm baseline, 70° per-eye FOV)
python apriltag_cube_pose.py --dataset lerobot_dataset_strawberry \
  --tag-size 0.05 --tag-id 0 --baseline 0.06 --hfov 70

# Bake the result back into the dataset (in place) and register the feature
python apriltag_cube_pose.py --dataset lerobot_dataset_clean --write-dataset
```

#### Detector backends (`--backend`)

Two interchangeable AprilTag detectors are available; both feed the identical
pose/triangulation pipeline:

| `--backend` | Library | Notes |
|---|---|---|
| `pupil` (default) | `pupil-apriltags` (AprilRobotics C library) | Finds noticeably more tags on the small, downscaled stereo markers here |
| `aruco` | OpenCV `cv2.aruco` | No extra dependency beyond OpenCV |

```bash
# Use the OpenCV aruco detector instead of the default pupil backend
python apriltag_cube_pose.py --dataset lerobot_dataset_clean --backend aruco
```

Because the tags are tiny in the downscaled 320×240 stereo eyes, each eye is
upscaled (cubic) before detection; tune with `--upscale` (default `3.0`).

#### Comparing the two backends (`--compare`)

`--compare` runs **both** backends over the same frames and prints how many tags
each detects, then continues the normal pose pipeline using the `--backend`
selection:

```bash
python apriltag_cube_pose.py --dataset lerobot_dataset_clean --compare --episodes 0
```

Example output (episode 0 of `lerobot_dataset_clean`, 5981 frames / 11962 eye
images, `--upscale 3`):

```
[compare] 5981 frames (11962 eye images) over 1 episode(s)
          backend      tags  eyes_with_tag  eye_hit_rate
          aruco        7026           6310         52.8%
          pupil        9799           7941         66.4%
```

Here `pupil` detected ~39% more tags than `aruco`, which is why it is the
default. Run `python apriltag_cube_pose.py --help` for the full option list
(camera key, tag family, calibration JSON, episode selection, etc.).

#### Calibration

No camera calibration exists for this rig yet, so intrinsics are **estimated**
from an assumed per-eye horizontal FOV (`--hfov`) and the baseline is a guess
(`--baseline`); all metric output is therefore approximate. When you have a real
calibration, pass `--calib calib.json` with any of `{fx, fy, cx, cy, baseline}`
to override the estimate — nothing else changes.

## Dataset → IK → simulation pipeline

This pipeline turns a recorded teleop dataset into training data augmented with a
simulated Nova5 arm that tracks the AprilTag cube. End to end:

```
record → detect cube → IK trajectory → render sim video → view
teleop_trigger_record → apriltag_cube_pose → ik_track_cube → render_sim_video → episode_picker / ik_replay
```

Each step writes back into the LeRobot v3.0 dataset (1:1 by frame index) and/or a
sidecar parquet next to it. The cube/IK/sim features added are:

| Feature | Shape | Written by | Meaning |
|---|---|---|---|
| `observation.cube_position` | `[13]` | `apriltag_cube_pose.py` | stereo XYZ, pnp XYZ, pixel centres, visibility flags (left-eye camera frame) |
| `observation.cube_orientation` | `[6]` | `apriltag_cube_pose.py` | in-plane image yaw + full tag quaternion (camera frame) + visibility flag |
| `observation.arm_qpos_ik` | `[6]` | `ik_track_cube.py` | Nova5 joint angles `joint1..6` (rad), incl. wrist yaw aligned to the marker |
| `observation.ee_pose_target` | `[3]` | `ik_track_cube.py` | the IK target in the robot base frame (m) |
| `observation.images.sim` | video | `render_sim_video.py` | rendered top-down view of the arm tracking the cube |

### Cube orientation (position vs. angle)

`apriltag_cube_pose.py` computes the **full** tag pose: the per-eye `solvePnP`
rotation (stored as a quaternion in the camera frame) **and** the in-plane image
yaw (the tag's rotation in the top-down image plane, robustly read from the tag
corners). Both go into `observation.cube_orientation`.

The IK uses **only the in-plane yaw**: the gripper is held pointing straight down
(top-down approach) and its wrist is rotated about the vertical to match the
marker's yaw, so the fingers align with the cube. Full 6-DOF orientation is
*calculated and stored* but not driven into the arm — it's noisy on small tags and
can't be transformed into the robot base frame without calibration. The yaw
mapping has a sign and offset (`--yaw-sign`, `--yaw-offset`): the default
`--yaw-sign -1` is calibrated so the rendered gripper rotates *with* the marker
(the top-down render rotates a world-Z yaw the opposite way in the image, so the
wrist must negate the tag's image yaw — verified to ~0.6°). The 90° offset choice
(fingers parallel vs. perpendicular to the tag edge) and any base-frame alignment
remain arbitrary until real extrinsics exist; use `--yaw-offset` to set it.

### Two environments

The detection/viewer scripts and the IK/simulation scripts need different stacks,
so there are two virtualenvs:

| venv | Python | Used for | Key packages |
|---|---|---|---|
| `.venv` | 3.14 | `apriltag_cube_pose.py`, `episode_picker.py`, `teleop_trigger_record.py`, `camera_test.py` | `opencv-python`, `pandas`, `pyarrow` |
| `.venv-ik` | 3.12 | `ik_track_cube.py`, `ik_replay.py`, `render_sim_video.py` | `mink`, `mujoco`, `imageio[-ffmpeg]`, `pandas` |

`mink`/`mujoco` do not install on the 3.14 `.venv`, hence the separate `.venv-ik`:

```bash
uv venv --python 3.12 .venv-ik
uv pip install --python .venv-ik/bin/python mink mujoco imageio imageio-ffmpeg numpy pandas pyarrow
```

The Nova5 model is **vendored in this repo** under `nova5_sim/` (scene +
meshes/textures, ~5.6 MB), so there is no external-repo dependency.
`ik_track_cube.py` / `ik_replay.py` / `render_sim_video.py` default to
`nova5_sim/scene_single.xml` — a single-arm scene (so the viewer shows only the
driven arm), with the table, floor, and a vendored gripper.

### Step 1 — Detect the cube

See [section 8](#8-estimate-apriltag-cube-pose-from-a-dataset). Run with
`--write-dataset` so `observation.cube_position` is baked in:

```bash
.venv/bin/python apriltag_cube_pose.py --dataset lerobot_dataset_grasp --tag-id 0 --write-dataset
```

### Step 2 — View the cube detections (`episode_picker.py`)

`episode_picker.py` serves a web UI that plays each episode's camera videos. When
an `apriltag_cube_pose.parquet` sidecar is present it overlays, on the external
camera (`observation.images.cam_1`), the detected tag as a green quad in each eye
plus the estimated 3D position. It also shows every video feature side by side,
so once Step 4 has run the `observation.images.sim` view appears to the right of
`cam_0`/`cam_1` automatically.

```bash
.venv/bin/python episode_picker.py --dataset lerobot_dataset_grasp
# open the printed URL
```

### Step 3 — Generate the IK trajectory (`ik_track_cube.py`)

Reads the cube sidecar, **interpolates** frames with no detection (positions
linearly, the yaw circularly), **auto-fits** the cube positions into the Nova5's
reachable workspace, and solves mink IK per frame, warm-started for a smooth
trajectory. The gripper points straight down with its **wrist yaw aligned to the
marker's in-plane yaw** (see [Cube orientation](#cube-orientation-position-vs-angle)).
With `--write-dataset` it bakes `observation.arm_qpos_ik` and
`observation.ee_pose_target` into the dataset and writes an `ik_track_cube.parquet`
sidecar.

```bash
.venv-ik/bin/python ik_track_cube.py --dataset lerobot_dataset_grasp --write-dataset
```

Key options: `--orientation-cost` (default `0.5`; keeps the gripper pointing down,
`0` = free), `--yaw-sign` / `--yaw-offset` (map the marker's image yaw to the wrist;
flip the sign if the wrist rotates the wrong way), `--scene`.

> **Auto-fit is a placeholder, not physically grounded.** There is no
> camera→robot-base calibration yet, so cube positions are range-mapped into the
> workspace via `AXIS_MAP`/`WORKSPACE` in `ik_track_cube.py`. The arm tracks the
> cube's *relative* motion, not its true position. `AXIS_MAP` assumes a **top-down**
> external camera with the operator at the top: camera depth → table height; the
> cube's vertical motion (camera y) → the arm's reach axis; cube horizontal
> (camera x) → sideways. This is paired with the render azimuth (180) so the sim
> matches cam_1's layout. If an axis looks mirrored, flip its `+1.0`/`-1.0` sign in
> `AXIS_MAP`. Once the stereo camera is calibrated and its pose relative to the arm
> base is known, replace the whole auto-fit with that transform.

### Step 4 — Render the sim video (`render_sim_video.py`)

Renders the arm following the IK trajectory (with the red target marker) to one
mp4 per episode, from a **top-down** viewpoint matching the external camera, and
registers `observation.images.sim` in `meta/info.json` + `meta/episodes`. Must run
after Step 3 (it reads `ik_track_cube.parquet`).

```bash
.venv-ik/bin/python render_sim_video.py --dataset lerobot_dataset_grasp
```

Viewpoint is tunable with `--azimuth` / `--elevation` / `--distance` (defaults
`180 / -70 / 1.4` — azimuth 180 puts the arm base at the top of the frame so the
arm reaches downward like the human operator in cam_1).

### Step 5 — Replay in the interactive viewer (`ik_replay.py`)

Plays an episode's IK trajectory in the live MuJoCo viewer (orbit with the mouse).
macOS needs `mjpython`:

```bash
.venv-ik/bin/mjpython ik_replay.py --dataset lerobot_dataset_grasp --episode 27 --loop
```

### Redo all renders

After changing the IK mapping or orientation, regenerate the IK + sim video for
every dataset:

```bash
for ds in lerobot_dataset_grasp lerobot_dataset_clean; do
  .venv-ik/bin/python ik_track_cube.py    --dataset $ds --write-dataset &&
  .venv-ik/bin/python render_sim_video.py --dataset $ds
done
```

The cube detection (Step 1) is upstream and unaffected by IK/viewpoint changes, so
it only needs rerunning if the detector or marker assumptions change.

## Stereo camera recording (`teleop_trigger_record.py`)

The rig uses two **side-by-side stereo** USB cameras (one third-person/external,
one egocentric on the gripper). Each stores as a single frame whose left half is
the left eye and right half is the right eye.

> **Set the resolution to a real side-by-side mode.** Because both eyes share the
> frame width, valid modes are *double-width* (e.g. `2560x720` → 1280×720 per eye,
> `1280x480` → 640×480 per eye) — **not** `640x480`. The recorder sets MJPG (which
> unlocks the high-res modes on these cameras) and then verifies and uses the
> resolution the camera actually delivers, warning if it fell back. List a
> camera's real modes with `v4l2-ctl --list-formats-ext` (Linux), or probe with
> `camera_test.py`.

```bash
# List camera indices and the resolution each delivers
.venv/bin/python camera_test.py --list
.venv/bin/python camera_test.py --cam-ids 0 2 --no-display

# Record (pick a real side-by-side mode for your camera)
.venv/bin/python teleop_trigger_record.py --cam-ids 0 2 --width 2560 --height 720
```

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
