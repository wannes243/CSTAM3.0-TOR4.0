from __future__ import annotations

from dataclasses import dataclass
from typing import Any
import numpy as np


@dataclass(frozen=True)
class WheelMeasurement:
    timestamp: float
    left_angle_rad: float
    right_angle_rad: float


@dataclass(frozen=True)
class IMUMeasurement:
    timestamp: float
    acceleration_mps2: np.ndarray
    angular_velocity_rps: np.ndarray
    roll_rad: float
    pitch_rad: float
    yaw_rad: float


@dataclass(frozen=True)
class LidarMeasurement:
    timestamp: float
    ranges_m: np.ndarray
    angles_rad: np.ndarray


@dataclass(frozen=True)
class PoseMeasurement:
    timestamp: float
    x_m: float
    y_m: float
    yaw_rad: float
    covariance: np.ndarray
    source: str


def as_array(value: Any, length: int) -> np.ndarray:
    arr = np.asarray(value, dtype=float).reshape(-1)
    if arr.size != length:
        raise ValueError(f"Expected {length} values, got {arr.size}")
    return arr
