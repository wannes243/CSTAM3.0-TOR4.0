import math
import numpy as np

from sensors.wheel_encoder import WheelGeometry, WheelMeasurement, WheelOdometryEstimator
from localization.motion_model import wrap_angle


def test_straight_motion():
    est = WheelOdometryEstimator(WheelGeometry(0.1, 0.4))
    est.update(WheelMeasurement(0.0, 0.0, 0.0))
    out = est.update(WheelMeasurement(1.0, 10.0, 10.0))
    assert out is not None
    assert math.isclose(out.distance_m, 1.0, rel_tol=1e-9)
    assert math.isclose(out.yaw_delta_rad, 0.0, abs_tol=1e-12)


def test_pure_rotation():
    est = WheelOdometryEstimator(WheelGeometry(0.1, 0.4))
    est.update(WheelMeasurement(0.0, 0.0, 0.0))
    out = est.update(WheelMeasurement(1.0, -2.0, 2.0))
    assert out is not None
    assert math.isclose(out.distance_m, 0.0, abs_tol=1e-12)
    assert math.isclose(out.yaw_delta_rad, 1.0, rel_tol=1e-9)


def test_angle_wrap():
    assert math.isclose(wrap_angle(math.pi + 0.1), -math.pi + 0.1, abs_tol=1e-12)
