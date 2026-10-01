from __future__ import annotations

import math
from typing import Sequence


def lidar_polar_to_robot(
    range_m: float,
    ray_angle_rad: float,
    angle_sign: float = -1.0,
) -> tuple[float, float]:
    """Convert a LiDAR polar return to the robot base frame.

    Robot frame convention:
      +x: forward, +y: left, +yaw: counter-clockwise.

    Webots' Pioneer horizontal scan angle is treated as positive toward the
    robot's right, therefore the default ``angle_sign=-1`` converts it to the
    robot-frame +y-left convention.
    """
    if range_m < 0 or not math.isfinite(range_m):
        raise ValueError("range_m must be finite and non-negative")
    if not math.isfinite(ray_angle_rad) or angle_sign not in (-1.0, 1.0):
        raise ValueError("invalid LiDAR angle")
    return (
        range_m * math.cos(ray_angle_rad),
        angle_sign * range_m * math.sin(ray_angle_rad),
    )


def robot_to_world_point(
    pose: Sequence[float],
    point_robot: Sequence[float],
) -> tuple[float, float]:
    """Transform a 2D robot-frame point using world pose [x, y, yaw]."""
    if len(pose) < 3 or len(point_robot) < 2:
        raise ValueError("pose must have 3 and point_robot must have 2 values")
    x_w, y_w, yaw = (float(pose[0]), float(pose[1]), float(pose[2]))
    x_r, y_r = (float(point_robot[0]), float(point_robot[1]))
    c = math.cos(yaw)
    s = math.sin(yaw)
    return x_w + c * x_r - s * y_r, y_w + s * x_r + c * y_r


def interpolate_pose(
    before: Sequence[float],
    after: Sequence[float],
    timestamp: float,
) -> tuple[float, float, float]:
    """Interpolate ``(timestamp, x, y, yaw)`` in the fixed map frame.

    Position is linearly interpolated. Yaw follows the shortest wrapped
    angular path so interpolation remains correct across the +/- pi boundary.
    """
    if len(before) < 4 or len(after) < 4:
        raise ValueError("poses must contain timestamp, x, y and yaw")
    t0, x0, y0, yaw0 = map(float, before[:4])
    t1, x1, y1, yaw1 = map(float, after[:4])
    if not all(math.isfinite(v) for v in (t0, t1, x0, y0, yaw0, x1, y1, yaw1)):
        raise ValueError("poses must be finite")
    if t1 <= t0:
        return x0, y0, yaw0
    alpha = min(1.0, max(0.0, (float(timestamp) - t0) / (t1 - t0)))
    yaw_delta = math.atan2(math.sin(yaw1 - yaw0), math.cos(yaw1 - yaw0))
    yaw = yaw0 + alpha * yaw_delta
    return x0 + alpha * (x1 - x0), y0 + alpha * (y1 - y0), math.atan2(math.sin(yaw), math.cos(yaw))


def relative_yaw(raw_yaw_rad: float, startup_yaw_rad: float) -> float:
    """Convert an absolute yaw observation to the startup-relative map yaw."""
    delta = float(raw_yaw_rad) - float(startup_yaw_rad)
    if not math.isfinite(delta):
        raise ValueError("yaw values must be finite")
    return math.atan2(math.sin(delta), math.cos(delta))


def lidar_return_to_world(
    pose: Sequence[float],
    range_m: float,
    ray_angle_rad: float,
    angle_sign: float = -1.0,
) -> tuple[float, float]:
    return robot_to_world_point(
        pose,
        lidar_polar_to_robot(range_m, ray_angle_rad, angle_sign),
    )
