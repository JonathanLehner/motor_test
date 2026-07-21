"""UART SDK for the PaXini PX-6AX GEN3 DP-S1813-Elite sensor."""

from __future__ import annotations

import struct
import threading
import time
from types import TracebackType
from typing import BinaryIO, Iterable

import serial
from serial.tools import list_ports

from .coordinates import S1813_ELITE_COORDINATES
from .models import Force, PointForce, PortInfo, SensorFrame


BAUDRATE = 921_600
SENSOR_OUTPUT_HZ = 83.3
READ_FUNCTION = 0xFB
RESULTANT_ADDRESS = 1008
DISTRIBUTED_ADDRESS = 1038
POINT_COUNT = 31
BYTES_PER_POINT = 3
FORCE_SCALE_N = 0.1
PAXINI_USB_VID = 0x1A86
PAXINI_USB_PID = 0x55D3


class PaxiniError(RuntimeError):
    """Base exception raised by this SDK."""


class ProtocolError(PaxiniError):
    """The sensor returned a malformed response."""


class DeviceNotFoundError(PaxiniError):
    """No unambiguous PaXini serial converter could be found."""


def calculate_lrc(data: bytes) -> int:
    """Calculate the protocol's 8-bit two's-complement LRC."""
    return (-sum(data)) & 0xFF


def decode_signed_byte(value: int) -> int:
    return value - 256 if value >= 128 else value


def build_read_request(device_address: int, start_address: int, length: int) -> bytes:
    """Build one raw UART application-area read request."""
    if not 1 <= device_address <= 0xFF:
        raise ValueError("device_address must be in 1..255")
    if not 0 <= start_address <= 0xFFFFFFFF:
        raise ValueError("start_address must fit in uint32")
    if not 1 <= length <= 0xFFFF:
        raise ValueError("length must be in 1..65535")
    frame = b"\x55\xAA" + struct.pack(
        "<HBBBIH", 9, device_address, 0, READ_FUNCTION, start_address, length
    )
    return frame + bytes((calculate_lrc(frame),))


def _read_exact(port: BinaryIO, size: int) -> bytes:
    data = bytearray()
    while len(data) < size:
        chunk = port.read(size - len(data))
        if not chunk:
            raise TimeoutError(f"response timed out ({len(data)}/{size} bytes received)")
        data.extend(chunk)
    return bytes(data)


def receive_response(port: BinaryIO) -> bytes:
    """Synchronize to the response header and receive one complete frame."""
    matched = 0
    while matched < 2:
        byte = _read_exact(port, 1)[0]
        if matched == 0:
            matched = 1 if byte == 0xAA else 0
        else:
            matched = 2 if byte == 0x55 else (1 if byte == 0xAA else 0)

    length_bytes = _read_exact(port, 2)
    body_length = int.from_bytes(length_bytes, "little")
    if body_length < 10 or body_length > 4096:
        raise ProtocolError(f"invalid response frame length: {body_length}")
    frame = b"\xAA\x55" + length_bytes + _read_exact(port, body_length + 1)
    expected_lrc = calculate_lrc(frame[:-1])
    if expected_lrc != frame[-1]:
        raise ProtocolError(
            f"LRC mismatch: calculated 0x{expected_lrc:02X}, received 0x{frame[-1]:02X}"
        )
    return frame


def parse_read_response(
    frame: bytes, expected_device: int, expected_address: int, expected_length: int
) -> bytes:
    """Validate a raw read response and return only its force payload."""
    if len(frame) < 15 or frame[:2] != b"\xAA\x55":
        raise ProtocolError("invalid response header")
    body_length = int.from_bytes(frame[2:4], "little")
    if len(frame) != 4 + body_length + 1:
        raise ProtocolError("response length does not match the frame header")
    if frame[4] != expected_device:
        raise ProtocolError(
            f"unexpected device address {frame[4]} (expected {expected_device})"
        )
    if frame[6] != READ_FUNCTION:
        raise ProtocolError(f"unexpected function code 0x{frame[6]:02X}")

    returned_address = int.from_bytes(frame[7:11], "little")
    returned_length = int.from_bytes(frame[11:13], "little")
    if returned_address != expected_address:
        raise ProtocolError(
            f"unexpected start address {returned_address} (expected {expected_address})"
        )
    if returned_length != expected_length:
        raise ProtocolError(
            f"unexpected data length {returned_length} (expected {expected_length})"
        )

    # frame[13] is an internal status byte, not part of the force payload.
    data = frame[14:-1]
    if len(data) != expected_length:
        raise ProtocolError(
            f"response contains {len(data)} data bytes (expected {expected_length})"
        )
    return data


def decode_force(data: bytes) -> Force:
    """Decode one Fx/Fy/Fz triplet into newtons."""
    if len(data) != 3:
        raise ValueError("one force sample must contain exactly three bytes")
    return Force(
        round(decode_signed_byte(data[0]) * FORCE_SCALE_N, 1),
        round(decode_signed_byte(data[1]) * FORCE_SCALE_N, 1),
        round(data[2] * FORCE_SCALE_N, 1),
    )


def discover_ports() -> tuple[PortInfo, ...]:
    """Return USB serial ports, with known PaXini converters sorted first."""
    ports = [
        PortInfo(
            device=item.device,
            description=item.description or "",
            vid=item.vid,
            pid=item.pid,
            serial_number=item.serial_number,
        )
        for item in list_ports.comports()
        if item.vid is not None or "USB" in (item.description or "").upper()
    ]
    ports.sort(
        key=lambda item: (
            0 if (item.vid, item.pid) == (PAXINI_USB_VID, PAXINI_USB_PID) else 1,
            item.device,
        )
    )
    return tuple(ports)


class S1813Elite:
    """High-level query interface for one S1813-Elite over USB UART.

    The class is a context manager. If ``port`` or ``device_address`` is
    omitted, the SDK discovers the USB converter and probes addresses 1..6.
    """

    def __init__(
        self,
        port: str | None = None,
        device_address: int | None = None,
        *,
        timeout: float = 0.25,
    ) -> None:
        if device_address is not None and not 1 <= device_address <= 0xFF:
            raise ValueError("device_address must be in 1..255")
        self.port_name = port
        self.device_address = device_address
        self.timeout = timeout
        self._serial: serial.Serial | None = None
        self._lock = threading.RLock()

    @property
    def is_connected(self) -> bool:
        return self._serial is not None and self._serial.is_open

    @staticmethod
    def list_ports() -> tuple[PortInfo, ...]:
        return discover_ports()

    def connect(self) -> "S1813Elite":
        """Open the port and auto-detect the UART device address if needed."""
        with self._lock:
            if self.is_connected:
                return self
            if self.port_name is None:
                candidates = discover_ports()
                known = [
                    item
                    for item in candidates
                    if (item.vid, item.pid) == (PAXINI_USB_VID, PAXINI_USB_PID)
                ]
                usable = known or list(candidates)
                if len(usable) != 1:
                    names = ", ".join(item.device for item in usable) or "none"
                    raise DeviceNotFoundError(
                        f"could not choose one USB serial port (candidates: {names})"
                    )
                self.port_name = usable[0].device

            try:
                self._serial = serial.Serial(
                    port=self.port_name,
                    baudrate=BAUDRATE,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=self.timeout,
                    write_timeout=self.timeout,
                    xonxoff=False,
                    rtscts=False,
                    dsrdtr=False,
                )
                if self.device_address is None:
                    self.device_address = self._find_device_address(range(1, 7))
            except Exception:
                self.close()
                raise
            return self

    def close(self) -> None:
        with self._lock:
            if self._serial is not None:
                self._serial.close()
                self._serial = None

    def __enter__(self) -> "S1813Elite":
        return self.connect()

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc_value: BaseException | None,
        traceback: TracebackType | None,
    ) -> None:
        self.close()

    def _require_connection(self) -> tuple[serial.Serial, int]:
        if not self.is_connected or self._serial is None or self.device_address is None:
            raise PaxiniError("sensor is not connected; call connect() first")
        return self._serial, self.device_address

    def _request_data(self, start_address: int, length: int) -> bytes:
        port, device_address = self._require_connection()
        port.reset_input_buffer()
        port.write(build_read_request(device_address, start_address, length))
        port.flush()
        response = receive_response(port)
        return parse_read_response(response, device_address, start_address, length)

    def _find_device_address(self, candidates: Iterable[int]) -> int:
        errors: list[str] = []
        for address in candidates:
            self.device_address = address
            try:
                self._request_data(RESULTANT_ADDRESS, 3)
                return address
            except (TimeoutError, ProtocolError, serial.SerialException) as exc:
                errors.append(f"{address}: {exc}")
        self.device_address = None
        raise DeviceNotFoundError("no sensor replied; attempts: " + "; ".join(errors))

    def query_resultant(self) -> Force:
        """Query the sensor-provided resultant Fx/Fy/Fz."""
        with self._lock:
            return decode_force(self._request_data(RESULTANT_ADDRESS, 3))

    def query_points(self) -> tuple[PointForce, ...]:
        """Query all 31 distributed forces in physical point order."""
        with self._lock:
            raw = self._request_data(DISTRIBUTED_ADDRESS, POINT_COUNT * BYTES_PER_POINT)
            return tuple(
                PointForce(coordinate, decode_force(raw[offset : offset + 3]))
                for offset, coordinate in zip(
                    range(0, len(raw), BYTES_PER_POINT), S1813_ELITE_COORDINATES
                )
            )

    def query_frame(self) -> SensorFrame:
        """Query resultant and distributed data as one application snapshot."""
        with self._lock:
            resultant = decode_force(self._request_data(RESULTANT_ADDRESS, 3))
            raw = self._request_data(DISTRIBUTED_ADDRESS, POINT_COUNT * BYTES_PER_POINT)
            points = tuple(
                PointForce(coordinate, decode_force(raw[offset : offset + 3]))
                for offset, coordinate in zip(
                    range(0, len(raw), BYTES_PER_POINT), S1813_ELITE_COORDINATES
                )
            )
            return SensorFrame(time.time(), self.device_address or 0, resultant, points)

    def query_snapshot(self) -> dict[str, object]:
        """Return ``query_frame()`` as a JSON-serializable dictionary."""
        return self.query_frame().to_dict()
