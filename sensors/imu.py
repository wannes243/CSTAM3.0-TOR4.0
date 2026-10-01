from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from .measurements import IMUMeasurement


@dataclass(frozen=True)
class IMUBias:
    gyro_z_rps: float = 0.0
    accel_xyz_mps2: tuple[float, float, float] = (0.0, 0.0, 0.0)


class IMUAdapter:
    """Normalizes Webots/real IMU readings into a hardware-independent record."""

    def __init__(self, gyro_bias: IMUBias | None = None) -> None:
        self.bias = gyro_bias or IMUBias()

    def make_measurement(
        self,
        timestamp: float,
        acceleration: list[float] | tuple[float, ...],
        angular_velocity: list[float] | tuple[float, ...],
        roll: float,
        pitch: float,
        yaw: float,
    ) -> IMUMeasurement:
        accel = np.asarray(acceleration, dtype=float)
        gyro = np.asarray(angular_velocity, dtype=float)
        if accel.shape != (3,) or gyro.shape != (3,):
            raise ValueError("IMU acceleration and angular velocity must be 3-vectors")
        return IMUMeasurement(
            timestamp=timestamp,
            acceleration_mps2=accel,
            angular_velocity_rps=gyro,
            roll_rad=float(roll),
            pitch_rad=float(pitch),
            yaw_rad=float(yaw),
        )
