from __future__ import annotations

from dataclasses import dataclass
import math
import numpy as np

from .motion_model import integrate_unicycle_state, wrap_angle
from sensors.wheel_encoder import WheelOdometry
from sensors.measurements import PoseMeasurement


@dataclass
class EKFConfig:
    initial_position_std_m: float = 0.02
    initial_yaw_std_rad: float = 0.05
    initial_velocity_std_mps: float = 0.05
    initial_angular_velocity_std_rps: float = 0.10
    initial_gyro_bias_std_rps: float = 0.02
    accel_drive_std_mps2: float = 0.12
    yaw_accel_std_rps2: float = 0.20
    gyro_bias_rw_std_rps: float = 0.0008
    encoder_distance_std_m: float = 0.002
    encoder_slip_std_m: float = 0.008
    gyro_std_rps: float = 0.003
    track_width_m: float = 0.394


class LocalizationEKF:
    """6-state EKF independent of Webots APIs.

    State = [x, y, yaw, v, omega, gyro_bias_z].
    Wheel odometry supplies v/omega observations. Gyro measures omega+bias.
    LiDAR/SLAM may provide an external x/y/yaw pose observation.
    """

    def __init__(self, config: EKFConfig | None = None) -> None:
        self.config = config or EKFConfig()
        self.x = np.zeros(6, dtype=float)
        self.P = np.diag([
            self.config.initial_position_std_m ** 2,
            self.config.initial_position_std_m ** 2,
            self.config.initial_yaw_std_rad ** 2,
            self.config.initial_velocity_std_mps ** 2,
            self.config.initial_angular_velocity_std_rps ** 2,
            self.config.initial_gyro_bias_std_rps ** 2,
        ])
        self.timestamp: float | None = None
        self.initialized = False

    def initialize(self, timestamp: float, x: float, y: float, yaw: float, gyro_bias: float = 0.0) -> None:
        self.x[:] = [x, y, wrap_angle(yaw), 0.0, 0.0, gyro_bias]
        self.timestamp = timestamp
        self.initialized = True

    @property
    def pose(self) -> tuple[float, float, float]:
        return float(self.x[0]), float(self.x[1]), float(self.x[2])

    @property
    def covariance_trace(self) -> float:
        return float(np.trace(self.P))

    def predict(self, timestamp: float) -> None:
        if not self.initialized:
            raise RuntimeError("EKF must be initialized before prediction")
        if self.timestamp is None:
            self.timestamp = timestamp
            return

        dt = timestamp - self.timestamp
        if dt <= 0:
            return
        dt = min(dt, 0.25)

        self.x, F = integrate_unicycle_state(self.x, dt)

        q = np.zeros((6, 6), dtype=float)
        q[0, 0] = (0.5 * self.config.accel_drive_std_mps2 * dt * dt) ** 2
        q[1, 1] = q[0, 0]
        q[2, 2] = (0.5 * self.config.yaw_accel_std_rps2 * dt * dt) ** 2
        q[3, 3] = (self.config.accel_drive_std_mps2 * math.sqrt(dt)) ** 2
        q[4, 4] = (self.config.yaw_accel_std_rps2 * math.sqrt(dt)) ** 2
        q[5, 5] = (self.config.gyro_bias_rw_std_rps * math.sqrt(dt)) ** 2
        self.P = F @ self.P @ F.T + q
        self.timestamp = timestamp


    def predict_wheel_odometry(self, odom: WheelOdometry) -> None:
        """Propagate planar pose directly from one differential-drive increment.

        The encoder increment is the authoritative short-term motion sample.
        Using ``yaw_delta_rad`` here avoids the previous one-cycle delay caused
        by predicting with the *old* angular-rate state and only afterwards
        updating that rate from the encoders.
        """
        if not self.initialized:
            raise RuntimeError("EKF must be initialized before prediction")

        timestamp = float(odom.timestamp)
        if self.timestamp is None:
            self.timestamp = timestamp
            return

        dt = timestamp - self.timestamp
        if dt <= 0:
            return
        dt = min(dt, 0.25)

        ds = float(odom.distance_m)
        dtheta = float(odom.yaw_delta_rad)
        theta = float(self.x[2])
        theta_mid = theta + 0.5 * dtheta

        self.x[0] += ds * math.cos(theta_mid)
        self.x[1] += ds * math.sin(theta_mid)
        self.x[2] = wrap_angle(theta + dtheta)

        # Update the kinematic rates from the same sample so the state remains
        # internally consistent for the next cycle and for diagnostics.
        self.x[3] = float(odom.linear_velocity_mps)
        self.x[4] = float(odom.angular_velocity_rps)

        F = np.eye(6, dtype=float)
        F[0, 2] = -ds * math.sin(theta_mid)
        F[1, 2] = ds * math.cos(theta_mid)
        self.P = F @ self.P @ F.T

        q = np.zeros((6, 6), dtype=float)
        q[0, 0] = max(self.config.encoder_distance_std_m, 1e-4) ** 2
        q[1, 1] = q[0, 0]
        sigma_distance = max(self.config.encoder_distance_std_m + self.config.encoder_slip_std_m, 1e-4)
        sigma_yaw = math.sqrt(2.0) * sigma_distance / max(self.config.track_width_m, 1e-4)
        q[2, 2] = sigma_yaw ** 2
        q[3, 3] = (self.config.accel_drive_std_mps2 * math.sqrt(dt)) ** 2
        q[4, 4] = (self.config.yaw_accel_std_rps2 * math.sqrt(dt)) ** 2
        q[5, 5] = (self.config.gyro_bias_rw_std_rps * math.sqrt(dt)) ** 2
        self.P += q
        self.P = 0.5 * (self.P + self.P.T)
        self.timestamp = timestamp

    def update_yaw(self, yaw_rad: float, std_rad: float | None = None) -> None:
        """Fuse an absolute planar yaw measurement in the local map frame."""
        sigma = float(std_rad if std_rad is not None else self.config.gyro_std_rps)
        sigma = max(sigma, 1e-5)
        z = np.array([wrap_angle(float(yaw_rad))], dtype=float)
        h = np.array([self.x[2]], dtype=float)
        H = np.zeros((1, 6), dtype=float)
        H[0, 2] = 1.0
        R = np.array([[sigma * sigma]], dtype=float)
        self.update(z, h, H, R, angle_indices=(0,))

    def update(self, z: np.ndarray, h: np.ndarray, H: np.ndarray, R: np.ndarray, angle_indices: tuple[int, ...] = ()) -> None:
        innovation = z - h
        for idx in angle_indices:
            innovation[idx] = wrap_angle(float(innovation[idx]))

        S = H @ self.P @ H.T + R
        PHt = self.P @ H.T
        try:
            K = np.linalg.solve(S, PHt.T).T
        except np.linalg.LinAlgError:
            S = S + np.eye(S.shape[0]) * 1e-9
            K = np.linalg.solve(S, PHt.T).T

        self.x = self.x + K @ innovation
        self.x[2] = wrap_angle(float(self.x[2]))

        I = np.eye(self.P.shape[0])
        # Joseph form improves numerical stability and guarantees symmetry better.
        IKH = I - K @ H
        self.P = IKH @ self.P @ IKH.T + K @ R @ K.T
        self.P = 0.5 * (self.P + self.P.T)

    def update_wheel_odometry(self, odom: WheelOdometry) -> None:
        z = np.array([odom.linear_velocity_mps, odom.angular_velocity_rps], dtype=float)
        h = self.x[[3, 4]].copy()
        H = np.zeros((2, 6), dtype=float)
        H[0, 3] = 1.0
        H[1, 4] = 1.0

        sigma_v = self.config.encoder_distance_std_m + self.config.encoder_slip_std_m
        sigma_w = sigma_v / max(abs(odom.distance_m), 0.02) if abs(odom.distance_m) > 0.02 else 0.25
        R = np.diag([max(sigma_v, 1e-4) ** 2, max(sigma_w, 0.02) ** 2])
        self.update(z, h, H, R)

    def update_gyro(self, gyro_z: float) -> None:
        z = np.array([gyro_z], dtype=float)
        h = np.array([self.x[4] + self.x[5]], dtype=float)
        H = np.zeros((1, 6), dtype=float)
        H[0, 4] = 1.0
        H[0, 5] = 1.0
        R = np.array([[max(self.config.gyro_std_rps, 1e-5) ** 2]], dtype=float)
        self.update(z, h, H, R)

    def update_pose(self, measurement: PoseMeasurement) -> None:
        z = np.array([measurement.x_m, measurement.y_m, measurement.yaw_rad], dtype=float)
        h = self.x[:3].copy()
        H = np.zeros((3, 6), dtype=float)
        H[:, :3] = np.eye(3)
        self.update(z, h, H, np.asarray(measurement.covariance, dtype=float), angle_indices=(2,))

    def set_pose_exact(self, measurement: PoseMeasurement) -> None:
        """Commit a trusted global relocalization pose without blending."""
        self.x[0] = float(measurement.x_m)
        self.x[1] = float(measurement.y_m)
        self.x[2] = wrap_angle(float(measurement.yaw_rad))
        covariance = np.asarray(measurement.covariance, dtype=float)
        if covariance.shape == (3, 3) and np.all(np.isfinite(covariance)):
            self.P[:3, :3] = covariance
            self.P[:3, 3:] = 0.0
            self.P[3:, :3] = 0.0
        self.P = 0.5 * (self.P + self.P.T)

    def is_valid(self, max_xy_variance: float = 4.0, max_yaw_variance: float = 1.0) -> bool:
        vals = np.diag(self.P)
        return bool(
            np.all(np.isfinite(self.x))
            and np.all(np.isfinite(self.P))
            and vals[0] <= max_xy_variance
            and vals[1] <= max_xy_variance
            and vals[2] <= max_yaw_variance
        )
