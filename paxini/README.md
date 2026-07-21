# PaXini S1813-Elite Python SDK

Python SDK, command-line reader, and tactile-array dashboard for the **PX-6AX GEN3 DP-S1813-Elite (PXSR-STDDP03F)**. This implementation communicates through the single-channel serial converter / USB UART at `921600-8-N-1`.

## Installation

```bash
cd ~/code/motor_test/paxini
python3 -m pip install -e .
```

The stable serial-port name of the currently tested hardware is:

```text
/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B90053902-if00
```

The SDK can also discover the USB port and probe device addresses automatically. The tested S1813-Elite uses UART device address `3`.

## SDK query API

```python
from paxini_sdk import S1813Elite

PORT = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B90053902-if00"

with S1813Elite(port=PORT, device_address=3) as sensor:
    resultant = sensor.query_resultant()  # Force(fx, fy, fz), in N
    points = sensor.query_points()        # 31 PointForce objects
    frame = sensor.query_frame()          # resultant + 31 points + timestamp
    snapshot = sensor.query_snapshot()    # JSON-serializable dictionary

print(frame.resultant.fz)
print(frame.points[0].force.fx)
print(frame.points[0].coordinate)         # point 1 x/y/z position, in mm
```

Public interfaces:

| Interface | Return value | Description |
|---|---|---|
| `S1813Elite.list_ports()` | `tuple[PortInfo, ...]` | List USB serial ports |
| `connect()` / `close()` | — | Open or close the sensor; context-manager use is supported |
| `query_resultant()` | `Force` | Query resultant Fx/Fy/Fz |
| `query_points()` | `tuple[PointForce, ...]` | Query all 31 distributed forces and coordinates |
| `query_frame()` | `SensorFrame` | Query resultant and array data together |
| `query_snapshot()` | `dict` | Return a JSON-friendly snapshot |

All `Fx/Fy/Fz` values are converted to newtons. Point ordering matches point IDs 1 through 31 in the vendor coordinate file.

## Command-line reader

Read one frame and print all 31 points:

```bash
paxini-read \
  --port /dev/serial/by-id/usb-1a86_USB_Single_Serial_5B90053902-if00 \
  --address 3 --once --show-points
```

The script can also be run without installing the command entry point:

```bash
python3 read_s1813_elite.py --port /dev/ttyACM0 --address 3 --once --show-points
```

Continuously read and save CSV data:

```bash
paxini-read --address 3 --interval 0.1 --csv s1813_force.csv
```

## Tactile-array dashboard

Start the dashboard:

```bash
paxini-ui \
  --port /dev/serial/by-id/usb-1a86_USB_Single_Serial_5B90053902-if00 \
  --address 3
```

Or run the script directly:

```bash
python3 s1813_ui.py --port /dev/ttyACM0 --address 3
```

Open the following address in a browser:

```text
http://localhost:8765
```

Point color and size represent normal force `Fz`; white arrows represent tangential force `Fx/Fy`. Hover over a point to inspect its ID, force components, and physical coordinates.

Run the dashboard with generated data when no sensor is connected:

```bash
paxini-ui --demo
```

Press `Ctrl+C` to stop the dashboard.

## About the 83.3 Hz rating

The datasheet's **83.3 Hz** value is the sensor's output/data-update rate. It is not the USB baud rate or a guarantee that a browser will draw 83.3 visible frames per second.

The dashboard backend now targets the sensor rate by default:

```text
polling interval = 1 / 83.3 s ≈ 0.0120 s
```

Each complete SDK frame performs two protocol transactions: one for the resultant force and one for the 31-point array. On the connected hardware, the UART link can complete approximately 330 such combined queries per second, so it has enough transport capacity for the 83.3 Hz sensor rate. Querying faster than 83.3 Hz may only return duplicate sensor samples.

Browser rendering normally follows the display refresh rate, commonly 60 Hz. The page therefore fetches the latest sensor sample once per browser frame (about every 16 ms), while the backend continues sampling at approximately 83.3 Hz. The displayed **Sensor polling** value reports the backend sampling rate, not the monitor's rendering rate.

To request a different backend rate, specify an interval explicitly. For example, 50 Hz is:

```bash
paxini-ui --interval 0.02
```
