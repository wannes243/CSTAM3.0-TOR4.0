import math
import numpy as np

from localization.ekf import EKFConfig, LocalizationEKF
from sensors.measurements import PoseMeasurement
from sensors.wheel_encoder import WheelOdometry


def test_ekf_prediction_moves_forward():
    ekf = LocalizationEKF(EKFConfig())
    ekf.initialize(0.0, 0.0, 0.0, 0.0)
    ekf.x[3] = 1.0
    ekf.predict(0.25)
    assert math.isclose(float(ekf.x[0]), 0.25, rel_tol=1e-6)
    assert math.isclose(float(ekf.x[1]), 0.0, abs_tol=1e-6)


def test_gyro_bias_is_observable():
    ekf = LocalizationEKF(EKFConfig(gyro_std_rps=0.01))
    ekf.initialize(0.0, 0.0, 0.0, 0.0, gyro_bias=0.05)
    before = float(ekf.x[5])
    for _ in range(20):
        ekf.predict(0.05 * (_ + 1))
        ekf.update_gyro(0.05)
    assert abs(float(ekf.x[5]) - 0.05) < abs(before - 0.05) + 1e-9


def test_pose_update_corrects_state():
    ekf = LocalizationEKF(EKFConfig())
    ekf.initialize(0.0, 0.0, 0.0, 0.0)
    ekf.x[0] = 1.0
    z = PoseMeasurement(
        timestamp=1.0,
        x_m=0.0,
        y_m=0.0,
        yaw_rad=0.0,
        covariance=np.diag([0.01, 0.01, 0.01]),
        source="test",
    )
    ekf.update_pose(z)
    assert float(ekf.x[0]) < 1.0


def test_exact_global_relocalization_sets_pose_without_blending():
    ekf = LocalizationEKF(EKFConfig())
    ekf.initialize(0.0, 5.0, -4.0, 1.0)
    measurement = PoseMeasurement(
        timestamp=1.0, x_m=-1.25, y_m=2.5, yaw_rad=-0.75,
        covariance=np.diag([0.01, 0.01, 0.01]), source="known_map_global_localizer",
    )
    ekf.set_pose_exact(measurement)
    assert ekf.pose == (-1.25, 2.5, -0.75)


def test_wheel_odometry_prediction_applies_same_cycle_rotation():
    ekf = LocalizationEKF(EKFConfig(track_width_m=0.394))
    ekf.initialize(0.0, 0.0, 0.0, 0.0)
    odom = WheelOdometry(
        timestamp=0.1,
        left_distance_m=-0.02,
        right_distance_m=0.02,
        distance_m=0.0,
        yaw_delta_rad=0.1015228426,
        linear_velocity_mps=0.0,
        angular_velocity_rps=1.015228426,
    )
    ekf.predict_wheel_odometry(odom)
    assert math.isclose(float(ekf.x[2]), 0.1015228426, rel_tol=1e-6)
    assert math.isclose(float(ekf.x[0]), 0.0, abs_tol=1e-12)
    assert math.isclose(float(ekf.x[1]), 0.0, abs_tol=1e-12)


def test_imu_yaw_update_cannot_leave_large_heading_error():
    ekf = LocalizationEKF(EKFConfig())
    ekf.initialize(0.0, 0.0, 0.0, 0.0)
    ekf.x[2] = 0.05
    ekf.update_yaw(1.0, 0.01)
    assert abs(float(ekf.x[2]) - 1.0) < 0.04
