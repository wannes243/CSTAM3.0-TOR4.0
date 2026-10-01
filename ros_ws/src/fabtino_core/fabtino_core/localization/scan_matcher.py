"""Lightweight 2-D correlative LiDAR scan matcher.

This is a scan-to-scan local matcher, not a global loop-closure system. It
estimates the robot motion between consecutive scans and produces an absolute
pose measurement by composing that motion with the previous EKF pose. The
controller must therefore provide motion signals so weak or contradictory
matches are not allowed to move a stationary robot.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

import numpy as np

from .motion_model import wrap_angle
from fabtino_core.sensors.measurements import PoseMeasurement


@dataclass(frozen=True)
class ScanMatcherConfig:
    enabled: bool = True
    min_points: int = 40
    max_points: int = 96
    max_match_error_m: float = 0.18
    sigma_xy_m: float = 0.04
    sigma_yaw_rad: float = 0.035
    stationary_linear_velocity_mps: float = 0.01
    stationary_angular_velocity_rps: float = 0.02
    min_translation_m: float = 0.01
    min_rotation_rad: float = 0.02
    max_odom_translation_disagreement_m: float = 0.05
    max_odom_rotation_disagreement_rad: float = 0.10
    min_score_improvement_m: float = 0.002


class LidarScanMatcher:
    def __init__(self, config: ScanMatcherConfig | None = None) -> None:
        self.config = config or ScanMatcherConfig()
        self.previous_points: np.ndarray | None = None
        self.previous_pose: tuple[float, float, float] | None = None
        self.last_error_m: float | None = None
        self.last_status = "waiting_for_first_scan"

    def reset_reference(self) -> None:
        """Drop scan-to-scan history after an external/global pose reset."""
        self.previous_points = None
        self.previous_pose = None
        self.last_error_m = None
        self.last_status = "waiting_for_first_scan"

    @staticmethod
    def _relative_motion(previous: Sequence[float], current: Sequence[float]) -> tuple[float, float, float]:
        px, py, pt = map(float, previous)
        cx, cy, ct = map(float, current)
        dx, dy = cx - px, cy - py
        c, s = math.cos(pt), math.sin(pt)
        return c * dx + s * dy, -s * dx + c * dy, wrap_angle(ct - pt)

    @staticmethod
    def _to_previous_frame(points: np.ndarray, motion: tuple[float, float, float]) -> np.ndarray:
        dx, dy, dtheta = motion
        c, s = math.cos(dtheta), math.sin(dtheta)
        # If the current robot pose in the previous frame is (dx, dy,
        # dtheta), a static-world point obeys:
        #   p_previous = R(dtheta) @ p_current + [dx, dy]
        # Translation is therefore added after rotating the current-frame
        # point; subtracting it would solve the inverse transform.
        return np.column_stack((c * points[:, 0] - s * points[:, 1] + dx,
                                s * points[:, 0] + c * points[:, 1] + dy))

    @staticmethod
    def _score(transformed: np.ndarray, reference: np.ndarray) -> float:
        if transformed.size == 0 or reference.size == 0:
            return float("inf")
        # The capped scan size keeps this deterministic and inexpensive on the
        # Webots controller thread; trimming rejects dynamic/outlier returns.
        diff = transformed[:, None, :] - reference[None, :, :]
        nearest = np.sum(diff * diff, axis=2).min(axis=1)
        nearest.sort()
        keep = max(8, int(nearest.size * 0.75))
        return float(math.sqrt(float(np.mean(nearest[:keep]))))

    def _downsample(self, points: Sequence[Sequence[float]]) -> np.ndarray:
        values = np.asarray(points, dtype=float)
        if values.size == 0:
            return np.empty((0, 2), dtype=float)
        if values.ndim != 2 or values.shape[1] < 2:
            return np.empty((0, 2), dtype=float)
        values = values[:, :2]
        values = values[np.all(np.isfinite(values), axis=1)]
        if values.shape[0] > self.config.max_points:
            indices = np.linspace(0, values.shape[0] - 1, self.config.max_points).astype(int)
            values = values[indices]
        return values

    def match(self, points: Sequence[Sequence[float]], pose: Sequence[float], timestamp: float,
              linear_velocity_mps: float | None = None,
              angular_velocity_rps: float | None = None,
              gyro_z_rps: float | None = None) -> PoseMeasurement | None:
        current = self._downsample(points)
        current_pose = tuple(map(float, pose[:3]))
        if not self.config.enabled:
            self.last_status = "disabled"
            return None
        if current.shape[0] < self.config.min_points:
            self.last_status = "insufficient_points"
            self.previous_points = current
            self.previous_pose = current_pose
            return None

        # Encoder and gyro measurements are the authority for detecting a
        # stopped robot. Keep the newest scan as the next reference, but never
        # inject a scan-matcher pose correction while stationary.
        if (linear_velocity_mps is not None and angular_velocity_rps is not None
                and gyro_z_rps is not None
                and abs(float(linear_velocity_mps)) < self.config.stationary_linear_velocity_mps
                and abs(float(angular_velocity_rps)) < self.config.stationary_angular_velocity_rps
                and abs(float(gyro_z_rps)) < self.config.stationary_angular_velocity_rps):
            self.previous_points = current
            self.previous_pose = current_pose
            self.last_status = "rejected_stationary"
            return None
        if self.previous_points is None or self.previous_pose is None:
            self.previous_points = current
            self.previous_pose = current_pose
            self.last_status = "waiting_for_second_scan"
            return None

        previous_pose = self.previous_pose
        reference_points = self.previous_points
        initial = self._relative_motion(previous_pose, current_pose)
        best = initial
        initial_score = self._score(self._to_previous_frame(current, best), reference_points)
        best_score = initial_score

        # Coarse-to-fine correlative search around the EKF-predicted motion.
        for xy_step, yaw_step in ((0.08, 0.10), (0.025, 0.035), (0.008, 0.012)):
            candidates = []
            for ix in range(-2, 3):
                for iy in range(-2, 3):
                    for ia in range(-2, 3):
                        candidate = (best[0] + ix * xy_step,
                                     best[1] + iy * xy_step,
                                     wrap_angle(best[2] + ia * yaw_step))
                        score = self._score(self._to_previous_frame(current, candidate), reference_points)
                        candidates.append((score, candidate))
            best_score, best = min(candidates, key=lambda item: item[0])

        self.previous_points = current
        self.previous_pose = current_pose
        self.last_error_m = best_score
        if not math.isfinite(best_score) or best_score > self.config.max_match_error_m:
            self.last_status = "rejected_match"
            return None

        motion_translation = math.hypot(best[0], best[1])
        if (motion_translation < self.config.min_translation_m
                and abs(best[2]) < self.config.min_rotation_rad):
            self.last_status = "rejected_near_zero_motion"
            return None

        # The scan matcher should refine the odometry prediction, not invent a
        # substantially different motion. ``initial`` is the wheel/gyro/EKF
        # prediction expressed in the same previous-robot frame as ``best``.
        translation_disagreement = math.hypot(best[0] - initial[0], best[1] - initial[1])
        rotation_disagreement = abs(wrap_angle(best[2] - initial[2]))
        if (translation_disagreement > self.config.max_odom_translation_disagreement_m
                or rotation_disagreement > self.config.max_odom_rotation_disagreement_rad):
            self.last_status = "rejected_odom_inconsistent"
            return None

        if not math.isfinite(initial_score) or initial_score - best_score < self.config.min_score_improvement_m:
            self.last_status = "rejected_no_score_improvement"
            return None

        px, py, pt = previous_pose
        dx, dy, dtheta = best
        c, s = math.cos(pt), math.sin(pt)
        x = px + c * dx - s * dy
        y = py + s * dx + c * dy
        yaw = wrap_angle(pt + dtheta)
        self.last_status = "accepted_match"
        return PoseMeasurement(
            timestamp=float(timestamp),
            x_m=x,
            y_m=y,
            yaw_rad=yaw,
            covariance=np.diag([
                self.config.sigma_xy_m ** 2,
                self.config.sigma_xy_m ** 2,
                self.config.sigma_yaw_rad ** 2,
            ]),
            source="lidar_scan_matcher",
        )
