#!/usr/bin/env python3
"""Local web dashboard for the PaXini S1813-Elite SDK."""

from __future__ import annotations

import argparse
import json
import math
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from importlib.resources import files
from typing import Any, Sequence
from urllib.parse import urlparse

from paxini_sdk import (
    Force,
    PaxiniError,
    PointForce,
    SENSOR_OUTPUT_HZ,
    S1813Elite,
    S1813_ELITE_COORDINATES,
    SensorFrame,
)


ASSET_DIRECTORY = files("paxini_ui")


class DashboardState:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._frame: dict[str, Any] | None = None
        self._error: str | None = None
        self._port: str | None = None
        self._address: int | None = None
        self._samples = 0
        self._sample_times: deque[float] = deque(maxlen=120)

    def set_connected(self, port: str, address: int) -> None:
        with self._lock:
            self._port = port
            self._address = address
            self._error = None

    def update(self, frame: SensorFrame) -> None:
        with self._lock:
            self._frame = frame.to_dict()
            self._samples += 1
            self._sample_times.append(time.monotonic())
            self._error = None

    def set_error(self, error: BaseException | str) -> None:
        with self._lock:
            self._error = str(error)

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            if len(self._sample_times) >= 2:
                elapsed = self._sample_times[-1] - self._sample_times[0]
                measured_hz = (len(self._sample_times) - 1) / max(elapsed, 0.001)
            else:
                measured_hz = 0.0
            return {
                "ok": self._frame is not None and self._error is None,
                "error": self._error,
                "port": self._port,
                "device_address": self._address,
                "sample_count": self._samples,
                "average_hz": measured_hz,
                "rated_hz": SENSOR_OUTPUT_HZ,
                "frame": self._frame,
            }


def sensor_worker(
    state: DashboardState,
    stop_event: threading.Event,
    port: str | None,
    address: int | None,
    interval: float,
) -> None:
    try:
        with S1813Elite(port, address) as sensor:
            state.set_connected(sensor.port_name or "unknown", sensor.device_address or 0)
            while not stop_event.is_set():
                started = time.monotonic()
                state.update(sensor.query_frame())
                stop_event.wait(max(0.0, interval - (time.monotonic() - started)))
    except (OSError, PaxiniError, TimeoutError) as exc:
        state.set_error(exc)


def demo_worker(
    state: DashboardState, stop_event: threading.Event, interval: float
) -> None:
    state.set_connected("demo", 3)
    started = time.monotonic()
    while not stop_event.is_set():
        phase = time.monotonic() - started
        center_x = math.sin(phase * 0.7) * 3.8
        center_y = 7.5 + math.cos(phase * 0.45) * 6.0
        points: list[PointForce] = []
        for coordinate in S1813_ELITE_COORDINATES:
            distance2 = (coordinate.x - center_x) ** 2 + (coordinate.y - center_y) ** 2
            fz = round(9.0 * math.exp(-distance2 / 18.0), 2)
            fx = round((coordinate.x - center_x) * fz * 0.035, 2)
            fy = round((coordinate.y - center_y) * fz * 0.025, 2)
            points.append(PointForce(coordinate, Force(fx, fy, fz)))
        resultant = Force(
            round(sum(point.force.fx for point in points), 2),
            round(sum(point.force.fy for point in points), 2),
            round(sum(point.force.fz for point in points), 2),
        )
        state.update(SensorFrame(time.time(), 3, resultant, tuple(points)))
        stop_event.wait(interval)


class DashboardHandler(BaseHTTPRequestHandler):
    state: DashboardState

    def do_GET(self) -> None:  # noqa: N802 - BaseHTTPRequestHandler API
        path = urlparse(self.path).path
        if path == "/api/frame":
            payload = json.dumps(self.state.snapshot(), ensure_ascii=False).encode("utf-8")
            self._send(200, "application/json; charset=utf-8", payload, no_cache=True)
            return

        assets = {
            "/": ("index.html", "text/html; charset=utf-8"),
            "/index.html": ("index.html", "text/html; charset=utf-8"),
            "/app.js": ("app.js", "text/javascript; charset=utf-8"),
            "/style.css": ("style.css", "text/css; charset=utf-8"),
        }
        asset = assets.get(path)
        if asset is None:
            self._send(404, "text/plain; charset=utf-8", b"Not found")
            return
        filename, content_type = asset
        try:
            payload = ASSET_DIRECTORY.joinpath(filename).read_bytes()
        except OSError as exc:
            self._send(500, "text/plain; charset=utf-8", str(exc).encode())
            return
        self._send(200, content_type, payload)

    def _send(
        self,
        status: int,
        content_type: str,
        payload: bytes,
        *,
        no_cache: bool = False,
    ) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(payload)))
        if no_cache:
            self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(payload)

    def log_message(self, format: str, *args: object) -> None:
        return


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="S1813-Elite local web dashboard")
    parser.add_argument("--port", help="serial device; omitted means auto-discover")
    parser.add_argument("--address", type=int, help="UART address; omitted means probe 1..6")
    parser.add_argument(
        "--interval",
        type=float,
        default=1.0 / SENSOR_OUTPUT_HZ,
        help=f"sensor polling interval (default: 1/{SENSOR_OUTPUT_HZ} s)",
    )
    parser.add_argument("--host", default="127.0.0.1", help="HTTP bind address")
    parser.add_argument("--web-port", type=int, default=8765, help="HTTP port")
    parser.add_argument("--demo", action="store_true", help="show generated data without hardware")
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    if args.interval <= 0:
        raise SystemExit("--interval must be greater than zero")
    if not 1 <= args.web_port <= 65535:
        raise SystemExit("--web-port must be in 1..65535")

    state = DashboardState()
    stop_event = threading.Event()
    if args.demo:
        target = demo_worker
        worker_args = (state, stop_event, args.interval)
    else:
        target = sensor_worker
        worker_args = (state, stop_event, args.port, args.address, args.interval)
    worker = threading.Thread(target=target, args=worker_args, daemon=True)
    worker.start()

    handler = type("BoundDashboardHandler", (DashboardHandler,), {"state": state})
    try:
        server = ThreadingHTTPServer((args.host, args.web_port), handler)
    except OSError as exc:
        stop_event.set()
        print(f"Cannot start UI server: {exc}")
        return 1

    display_host = "localhost" if args.host in ("127.0.0.1", "0.0.0.0") else args.host
    print(f"S1813-Elite UI: http://{display_host}:{args.web_port}")
    print("Press Ctrl+C to stop.")
    try:
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        stop_event.set()
        server.server_close()
        worker.join(timeout=1.0)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
