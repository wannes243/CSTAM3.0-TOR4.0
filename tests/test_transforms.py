import math
import sys
from pathlib import Path

from localization.frames import lidar_polar_to_robot, robot_to_world_point, lidar_return_to_world

sys.path.insert(0, str(Path(__file__).parents[1] / "ros_ws" / "src" / "fabtino_core"))
from fabtino_core.localization.frames import interpolate_pose as runtime_interpolate_pose
from fabtino_core.localization.frames import relative_yaw


def transform_robot_point(x, y, yaw, px, py):
    c, s = math.cos(yaw), math.sin(yaw)
    return x + c * px - s * py, y + s * px + c * py


def test_robot_to_map_transform_is_single_transform():
    x, y = transform_robot_point(1.0, 2.0, math.pi / 2, 1.0, 0.0)
    assert math.isclose(x, 1.0, abs_tol=1e-12)
    assert math.isclose(y, 3.0, abs_tol=1e-12)


def lidar_to_map(x, y, yaw, distance, sensor_angle):
    """Webots X/Z LiDAR sample to the viewer's right-handed X/Y map."""
    forward = distance * math.cos(sensor_angle)
    left = -distance * math.sin(sensor_angle)
    c, s = math.cos(yaw), math.sin(yaw)
    return x + c * forward - s * left, y + s * forward + c * left


def test_lidar_forward_matches_robot_arrow_heading():
    x, y = lidar_to_map(1.0, 2.0, 0.0, 3.0, 0.0)
    assert math.isclose(x, 4.0, abs_tol=1e-12)
    assert math.isclose(y, 2.0, abs_tol=1e-12)


def test_lidar_rotation_remains_in_world_frame():
    x, y = lidar_to_map(0.0, 0.0, math.pi / 2, 2.0, 0.0)
    assert math.isclose(x, 0.0, abs_tol=1e-12)
    assert math.isclose(y, 2.0, abs_tol=1e-12)


def test_polar_return_is_first_converted_to_robot_frame():
    x_r, y_r = lidar_polar_to_robot(2.0, math.pi / 2)
    assert math.isclose(x_r, 0.0, abs_tol=1e-12)
    assert math.isclose(y_r, -2.0, abs_tol=1e-12)


def test_robot_to_world_transform_is_explicit_and_composable():
    point = robot_to_world_point((10.0, 5.0, math.pi / 2), (2.0, 0.0))
    assert math.isclose(point[0], 10.0, abs_tol=1e-12)
    assert math.isclose(point[1], 7.0, abs_tol=1e-12)


def test_lidar_return_world_position_changes_with_robot_yaw_not_wall_frame():
    p0 = lidar_return_to_world((0.0, 0.0, 0.0), 2.0, 0.0)
    p1 = lidar_return_to_world((0.0, 0.0, math.pi / 2), 2.0, 0.0)
    assert p0 == (2.0, 0.0)
    assert math.isclose(p1[0], 0.0, abs_tol=1e-12)
    assert math.isclose(p1[1], 2.0, abs_tol=1e-12)


def test_same_wall_stays_fixed_when_robot_turns_left_in_place():
    # Initial pose: the wall is 2 m to the robot's left.
    initial = lidar_return_to_world((0.0, 0.0, 0.0), 2.0, -math.pi / 2)
    # After a +90-degree left turn, that same wall is directly in front.
    after_turn = lidar_return_to_world((0.0, 0.0, math.pi / 2), 2.0, 0.0)
    assert math.isclose(initial[0], 0.0, abs_tol=1e-12)
    assert math.isclose(initial[1], 2.0, abs_tol=1e-12)
    assert math.isclose(after_turn[0], initial[0], abs_tol=1e-12)
    assert math.isclose(after_turn[1], initial[1], abs_tol=1e-12)


def test_runtime_pose_interpolation_wraps_yaw_shortest_path():
    pose = runtime_interpolate_pose(
        (0.0, 1.0, 2.0, math.pi - 0.1),
        (1.0, 3.0, 4.0, -math.pi + 0.1),
        0.5,
    )
    assert math.isclose(pose[0], 2.0, abs_tol=1e-12)
    assert math.isclose(pose[1], 3.0, abs_tol=1e-12)
    assert math.isclose(abs(abs(pose[2]) - math.pi), 0.0, abs_tol=0.11)


def test_startup_relative_yaw_wraps_across_pi():
    assert math.isclose(relative_yaw(-math.pi + 0.1, math.pi - 0.1), 0.2, abs_tol=1e-12)
