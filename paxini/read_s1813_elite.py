#!/usr/bin/env python3
"""Command-line reader built on top of paxini_sdk."""

from __future__ import annotations

import argparse
import csv
import sys
import time
from pathlib import Path
from typing import Sequence, TextIO

import serial

from paxini_sdk import PaxiniError, S1813Elite, SensorFrame


def print_frame(frame: SensorFrame, show_points: bool) -> None:
    force = frame.resultant
    print(
        f"{frame.timestamp:.6f}  total: "
        f"Fx={force.fx:+5.1f} N  Fy={force.fy:+5.1f} N  Fz={force.fz:5.1f} N"
    )
    if show_points:
        for point in frame.points:
            force = point.force
            print(
                f"  P{point.index:02d}: Fx={force.fx:+5.1f} N  "
                f"Fy={force.fy:+5.1f} N  Fz={force.fz:5.1f} N"
            )


def write_csv_header(writer: csv.writer) -> None:
    fields = ["timestamp", "device_address", "total_fx_N", "total_fy_N", "total_fz_N"]
    for index in range(1, 32):
        fields.extend((f"p{index:02d}_fx_N", f"p{index:02d}_fy_N", f"p{index:02d}_fz_N"))
    writer.writerow(fields)


def write_csv_frame(writer: csv.writer, frame: SensorFrame) -> None:
    row: list[float | int] = [
        frame.timestamp,
        frame.device_address,
        frame.resultant.fx,
        frame.resultant.fy,
        frame.resultant.fz,
    ]
    for point in frame.points:
        row.extend((point.force.fx, point.force.fy, point.force.fz))
    writer.writerow(row)


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Read a PaXini DP-S1813-Elite")
    parser.add_argument("--port", help="serial device; omitted means auto-discover")
    parser.add_argument("--address", type=int, help="UART address; omitted means probe 1..6")
    parser.add_argument("--interval", type=float, default=0.1, help="sample interval in seconds")
    parser.add_argument("--once", action="store_true", help="read one sample and exit")
    parser.add_argument("--show-points", action="store_true", help="print all 31 points")
    parser.add_argument("--csv", type=Path, help="append samples to this CSV file")
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    if args.interval < 0:
        raise SystemExit("--interval must not be negative")

    csv_file: TextIO | None = None
    try:
        with S1813Elite(args.port, args.address) as sensor:
            print(
                f"Connected to {sensor.port_name} at 921600 baud, "
                f"device address {sensor.device_address}",
                file=sys.stderr,
            )
            writer = None
            if args.csv:
                existed = args.csv.exists() and args.csv.stat().st_size > 0
                csv_file = args.csv.open("a", newline="", encoding="utf-8")
                writer = csv.writer(csv_file)
                if not existed:
                    write_csv_header(writer)

            while True:
                started = time.monotonic()
                frame = sensor.query_frame()
                print_frame(frame, args.show_points)
                if writer is not None and csv_file is not None:
                    write_csv_frame(writer, frame)
                    csv_file.flush()
                if args.once:
                    break
                time.sleep(max(0.0, args.interval - (time.monotonic() - started)))
    except KeyboardInterrupt:
        print("Stopped.", file=sys.stderr)
    except (OSError, PaxiniError, TimeoutError, serial.SerialException) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1
    finally:
        if csv_file is not None:
            csv_file.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
