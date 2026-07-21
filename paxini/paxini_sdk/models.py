"""Public data models for the PaXini S1813-Elite SDK."""

from __future__ import annotations

from dataclasses import dataclass
from math import sqrt
from typing import Any


@dataclass(frozen=True)
class Force:
    """A three-axis force in newtons."""

    fx: float
    fy: float
    fz: float

    @property
    def tangential(self) -> float:
        return sqrt(self.fx * self.fx + self.fy * self.fy)

    @property
    def magnitude(self) -> float:
        return sqrt(self.fx * self.fx + self.fy * self.fy + self.fz * self.fz)

    def to_dict(self) -> dict[str, float]:
        return {
            "fx": self.fx,
            "fy": self.fy,
            "fz": self.fz,
            "tangential": self.tangential,
            "magnitude": self.magnitude,
        }


@dataclass(frozen=True)
class PointCoordinate:
    """Position of one sensing point in the vendor coordinate system (mm)."""

    index: int
    x: float
    y: float
    z: float

    def to_dict(self) -> dict[str, float | int]:
        return {"index": self.index, "x": self.x, "y": self.y, "z": self.z}


@dataclass(frozen=True)
class PointForce:
    """Force and physical coordinate for one sensing point."""

    coordinate: PointCoordinate
    force: Force

    @property
    def index(self) -> int:
        return self.coordinate.index

    def to_dict(self) -> dict[str, Any]:
        return {
            "index": self.index,
            "x": self.coordinate.x,
            "y": self.coordinate.y,
            "z": self.coordinate.z,
            **self.force.to_dict(),
        }


@dataclass(frozen=True)
class SensorFrame:
    """One combined resultant-force and distributed-force query."""

    timestamp: float
    device_address: int
    resultant: Force
    points: tuple[PointForce, ...]

    def to_dict(self) -> dict[str, Any]:
        return {
            "timestamp": self.timestamp,
            "device_address": self.device_address,
            "resultant": self.resultant.to_dict(),
            "points": [point.to_dict() for point in self.points],
        }


@dataclass(frozen=True)
class PortInfo:
    """A serial port that may host a PaXini converter."""

    device: str
    description: str
    vid: int | None = None
    pid: int | None = None
    serial_number: str | None = None

    def to_dict(self) -> dict[str, str | int | None]:
        return {
            "device": self.device,
            "description": self.description,
            "vid": self.vid,
            "pid": self.pid,
            "serial_number": self.serial_number,
        }
