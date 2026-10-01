import math

import numpy as np

from localization.map_localizer import KnownMapLocalizer, KnownMapLocalizerConfig


def test_known_map_localizer_global_search_recovers_pose_without_odom_hint():
    data = np.full((80, 80), -1, dtype=np.int8)
    # An asymmetric L-shaped landmark at x=1 and y=1.
    data[30:51, 50] = 100
    data[50, 30:51] = 100
    localizer = KnownMapLocalizer(
        data, resolution=0.1, radius=4.0,
        config=KnownMapLocalizerConfig(
            min_points=4, global_xy_step_m=0.4, global_yaw_step_rad=0.3,
            global_refine_xy_radius_m=0.3, global_refine_yaw_radius_rad=0.3,
            global_refine_xy_step_m=0.05, global_refine_yaw_step_rad=0.05,
            min_static_hit_fraction=0.5,
            global_min_free_space_fraction=0.0,
        ),
    )
    points = [[1.0, 0.0], [1.0, 0.5], [0.0, 1.0], [-0.5, 1.0]]
    # The odometry pose is intentionally far from the true origin.
    measurement = localizer.match(points, (3.0, -3.0, 2.5), 1.0)
    assert measurement is not None
    assert measurement.source == "known_map_global_localizer"
    assert abs(measurement.x_m) < 0.3
    assert abs(measurement.y_m) < 0.3
    assert localizer.global_localized


def test_known_map_localizer_retries_global_match_after_rejection():
    data = np.full((40, 40), -1, dtype=np.int8)
    localizer = KnownMapLocalizer(
        data, resolution=0.1, radius=2.0,
        config=KnownMapLocalizerConfig(min_points=4, min_static_hit_fraction=0.5),
    )
    points = [[1.0, 0.0], [1.0, 0.2], [1.0, -0.2], [0.8, 0.0]]
    assert localizer.match(points, (0.0, 0.0, 0.0), 1.0) is None
    assert localizer.last_status == "rejected_global_match"
    assert not localizer.global_localized


def test_known_map_global_match_requires_free_space_consistency():
    # Every endpoint matches an occupied cell, but the rays also pass through
    # occupied cells, so this is not a valid first-scan localization.
    data = np.full((40, 40), 100, dtype=np.int8)
    localizer = KnownMapLocalizer(
        data, resolution=0.1, radius=2.0,
        config=KnownMapLocalizerConfig(
            min_points=4, global_min_static_hit_fraction=0.9,
            global_min_free_space_fraction=0.8,
        ),
    )
    points = [[1.0, 0.0], [1.0, 0.2], [1.0, -0.2], [0.8, 0.0]]
    assert localizer.match(points, (0.0, 0.0, 0.0), 1.0) is None
    assert localizer.last_status == "rejected_global_match"


def test_known_map_localizer_identifies_non_static_return():
    data = np.full((20, 20), -1, dtype=np.int8)
    localizer = KnownMapLocalizer(data, 0.1, 1.0)
    assert not localizer.is_static_hit(0.2, 0.2)
