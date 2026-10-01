import math

import numpy as np

from localization.scan_matcher import LidarScanMatcher, ScanMatcherConfig


def test_scan_matcher_recovers_small_relative_motion():
    previous = np.array([
        [math.cos(a) * (1.0 + 0.3 * math.sin(3 * a)),
         math.sin(a) * (1.0 + 0.3 * math.sin(3 * a))]
        for a in np.linspace(-math.pi, math.pi, 120, endpoint=False)
    ])
    dx, dy, dtheta = 0.06, -0.04, 0.05
    c, s = math.cos(dtheta), math.sin(dtheta)
    shifted = previous - np.array([dx, dy])
    # In a static world, current-frame points are the inverse transform of
    # the robot motion expressed in the previous frame.
    current = np.column_stack((
        c * shifted[:, 0] + s * shifted[:, 1],
        -s * shifted[:, 0] + c * shifted[:, 1],
    ))

    matcher = LidarScanMatcher(ScanMatcherConfig(max_points=120, sigma_xy_m=0.1, sigma_yaw_rad=0.1))
    assert matcher.match(previous, (0.0, 0.0, 0.0), 0.0) is None
    # The prediction is intentionally slightly imperfect. If odometry already
    # explains the scan exactly, a correction is correctly rejected because it
    # provides no measurable improvement.
    measurement = matcher.match(current, (0.03, -0.02, 0.02), 0.1)

    assert measurement is not None
    assert abs(measurement.x_m - dx) < 0.03
    assert abs(measurement.y_m - dy) < 0.03
    assert abs(math.atan2(math.sin(measurement.yaw_rad - dtheta),
                          math.cos(measurement.yaw_rad - dtheta))) < 0.03


def test_scan_matcher_rejects_stationary_correction_but_keeps_reference():
    points = [[math.cos(a), math.sin(a)] for a in np.linspace(-math.pi, math.pi, 60, endpoint=False)]
    matcher = LidarScanMatcher(ScanMatcherConfig(max_points=60))
    assert matcher.match(points, (0.0, 0.0, 0.0), 0.0,
                         linear_velocity_mps=0.0,
                         angular_velocity_rps=0.0,
                         gyro_z_rps=0.0) is None
    assert matcher.last_status == "rejected_stationary"
    assert matcher.previous_points is not None


def test_scan_matcher_rejects_motion_inconsistent_with_odometry():
    points = [[math.cos(a), math.sin(a)] for a in np.linspace(-math.pi, math.pi, 60, endpoint=False)]
    matcher = LidarScanMatcher(ScanMatcherConfig(max_points=60))
    matcher.match(points, (0.0, 0.0, 0.0), 0.0)
    measurement = matcher.match(points, (0.0, 0.0, 0.0), 0.1,
                                 linear_velocity_mps=0.2,
                                 angular_velocity_rps=0.0,
                                 gyro_z_rps=0.0)
    assert measurement is None
    assert matcher.last_status in {"rejected_near_zero_motion", "rejected_odom_inconsistent",
                                   "rejected_no_score_improvement"}


def test_scan_matcher_handles_empty_scan():
    matcher = LidarScanMatcher()
    assert matcher.match([], (0.0, 0.0, 0.0), 0.0) is None
    assert matcher.last_status == "insufficient_points"
