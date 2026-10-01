from __future__ import annotations

from dataclasses import dataclass
import math

from .measurements import WheelMeasurement


@dataclass(frozen=True)
class WheelGeometry:
    radius_m: float
    track_width_m: float


@dataclass(frozen=True)
class WheelOdometry:
    timestamp: float
    left_distance_m: float
    right_distance_m: float
    distance_m: float
    yaw_delta_rad: float
    linear_velocity_mps: float
    angular_velocity_rps: float


class WheelOdometryEstimator:
    """Differential/skid-steer wheel odometry adapter.

    Encoder angles are converted to traveled wheel distances. Unequal wheel
    motion drives both translation and heading; heading is never replaced by
    an IMU yaw measurement here.
    """

    def __init__(self, geometry: WheelGeometry) -> None:
        if geometry.radius_m <= 0 or geometry.track_width_m <= 0:
            raise ValueError("Wheel radius and track width must be positive")
        self.geometry = geometry
        self._prev: WheelMeasurement | None = None

    def reset(self, measurement: WheelMeasurement | None = None) -> None:
        self._prev = measurement

    def update(self, measurement: WheelMeasurement) -> WheelOdometry | None:
        if self._prev is None:
            self._prev = measurement
            return None

        dt = measurement.timestamp - self._prev.timestamp
        if dt <= 0:
            self._prev = measurement
            return None

        dl = (measurement.left_angle_rad - self._prev.left_angle_rad) * self.geometry.radius_m
        dr = (measurement.right_angle_rad - self._prev.right_angle_rad) * self.geometry.radius_m
        ds = 0.5 * (dl + dr)
        dtheta = (dr - dl) / self.geometry.track_width_m

        self._prev = measurement
        return WheelOdometry(
            timestamp=measurement.timestamp,
            left_distance_m=dl,
            right_distance_m=dr,
            distance_m=ds,
            yaw_delta_rad=dtheta,
            linear_velocity_mps=ds / dt,
            angular_velocity_rps=dtheta / dt,
        )
