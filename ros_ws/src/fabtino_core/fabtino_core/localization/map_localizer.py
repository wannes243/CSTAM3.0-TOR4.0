"""Global 2-D LiDAR scan-to-map localization against a static occupancy grid."""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

import numpy as np

from .motion_model import wrap_angle
from fabtino_core.sensors.measurements import PoseMeasurement


@dataclass(frozen=True)
class KnownMapLocalizerConfig:
    enabled: bool = True
    min_points: int = 20
    max_global_points: int = 100
    max_refine_points: int = 300
    search_xy_radius_m: float = 0.40
    search_yaw_radius_rad: float = 0.30
    coarse_xy_step_m: float = 0.10
    coarse_yaw_step_rad: float = 0.10
    fine_xy_step_m: float = 0.025
    fine_yaw_step_rad: float = 0.035
    occupied_tolerance_m: float = 0.10
    min_static_hit_fraction: float = 0.20
    sigma_xy_m: float = 0.06
    sigma_yaw_rad: float = 0.05
    global_xy_step_m: float = 0.40
    global_yaw_step_rad: float = 0.20
    global_refine_xy_radius_m: float = 0.30
    global_refine_yaw_radius_rad: float = 0.30
    global_refine_xy_step_m: float = 0.05
    global_refine_yaw_step_rad: float = 0.035
    global_min_static_hit_fraction: float = 0.90
    mean_distance_threshold_m: float = 0.06
    distance_sigma_m: float = 0.06
    global_min_free_space_fraction: float = 0.85
    sector_coverage_min: float = 0.80
    min_good_sectors: int = 6
    best_second_difference_min: float = 0.04
    free_space_sample_step_m: float = 0.10
    candidate_count: int = 8


class KnownMapLocalizer:
    """Coarse-to-fine correlative matcher with strict global verification."""

    def __init__(self, data: np.ndarray, resolution: float, radius: float,
                 config: KnownMapLocalizerConfig | None = None) -> None:
        self.data = np.asarray(data, dtype=np.int8)
        if self.data.ndim != 2 or self.data.size == 0:
            raise ValueError("known map must be a non-empty 2-D grid")
        self.resolution = float(resolution)
        self.radius = float(radius)
        self.config = config or KnownMapLocalizerConfig()
        self.last_status = "waiting_for_global_localization"
        self.last_error_m: float | None = None
        self.last_metrics: dict[str, float | int | None] = {}
        self.global_localized = False
        self._occupied = self.data == 100
        self._distance_m = self._build_distance_map(self._occupied)
        self._matching_mask = self._distance_m <= float(self.config.occupied_tolerance_m)

    def _build_distance_map(self, occupied: np.ndarray) -> np.ndarray:
        """Build an exact Euclidean distance map without a SciPy dependency."""
        inf = 1.0e12
        if not np.any(occupied):
            return np.full(occupied.shape, np.inf, dtype=float)

        def transform_1d(values: np.ndarray) -> np.ndarray:
            n = values.size
            v = np.empty(n, dtype=int)
            z = np.empty(n + 1, dtype=float)
            out = np.empty(n, dtype=float)
            k = 0
            v[0] = 0
            z[0] = -np.inf
            z[1] = np.inf
            for q in range(1, n):
                s = ((values[q] + q * q) - (values[v[k]] + v[k] * v[k])) / (2 * q - 2 * v[k])
                while s <= z[k]:
                    k -= 1
                    s = ((values[q] + q * q) - (values[v[k]] + v[k] * v[k])) / (2 * q - 2 * v[k])
                k += 1
                v[k] = q
                z[k] = s
                z[k + 1] = np.inf
            k = 0
            for q in range(n):
                while z[k + 1] < q:
                    k += 1
                out[q] = (q - v[k]) ** 2 + values[v[k]]
            return out

        base = np.where(occupied, 0.0, inf)
        horizontal = np.empty_like(base)
        for row in range(base.shape[0]):
            horizontal[row] = transform_1d(base[row])
        squared = np.empty_like(base)
        for col in range(base.shape[1]):
            squared[:, col] = transform_1d(horizontal[:, col])
        return np.sqrt(squared) * self.resolution

    def _cell(self, x: float, y: float) -> tuple[int, int] | None:
        gx = int(math.floor((x + self.radius) / self.resolution))
        gy = int(math.floor((y + self.radius) / self.resolution))
        if 0 <= gy < self.data.shape[0] and 0 <= gx < self.data.shape[1]:
            return gx, gy
        return None

    def is_static_hit(self, x: float, y: float) -> bool:
        cell = self._cell(float(x), float(y))
        if cell is None:
            return False
        gx, gy = cell
        return bool(self._matching_mask[gy, gx])

    @staticmethod
    def _transform(points: np.ndarray, pose: tuple[float, float, float]) -> np.ndarray:
        x, y, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        return np.column_stack((x + c * points[:, 0] - s * points[:, 1],
                                y + s * points[:, 0] + c * points[:, 1]))

    def _endpoint_distances(self, world: np.ndarray) -> np.ndarray:
        gx = np.floor((world[:, 0] + self.radius) / self.resolution).astype(int)
        gy = np.floor((world[:, 1] + self.radius) / self.resolution).astype(int)
        valid = ((gx >= 0) & (gx < self.data.shape[1]) &
                 (gy >= 0) & (gy < self.data.shape[0]))
        distances = np.full(len(world), max(self.radius, 1.0), dtype=float)
        distances[valid] = self._distance_m[gy[valid], gx[valid]]
        return distances

    def _free_space_fraction(self, points: np.ndarray, pose: tuple[float, float, float]) -> float:
        if len(points) == 0:
            return 0.0
        world = self._transform(points, pose)
        x0, y0 = float(pose[0]), float(pose[1])
        free = total = 0
        step = max(self.resolution, float(self.config.free_space_sample_step_m))
        for x1, y1 in world:
            count = max(1, int(math.ceil(math.hypot(x1 - x0, y1 - y0) / step)))
            for i in range(1, count):
                t = i / count
                cell = self._cell(x0 + (x1 - x0) * t, y0 + (y1 - y0) * t)
                if cell is None:
                    continue
                gx, gy = cell
                total += 1
                if self.data[gy, gx] == 0:
                    free += 1
        return free / total if total else 0.0

    def _evaluate(self, points: np.ndarray, pose: tuple[float, float, float],
                  include_free_space: bool = True) -> dict[str, float]:
        world = self._transform(points, pose)
        distances = self._endpoint_distances(world)
        overlap = float(np.mean(distances <= self.config.occupied_tolerance_m)) if len(points) else 0.0
        mean_distance = float(np.mean(distances)) if len(points) else float("inf")
        distance_score = math.exp(-mean_distance / max(1e-6, self.config.distance_sigma_m))
        angles = np.arctan2(points[:, 1], points[:, 0])
        sector_values = []
        for sector in range(8):
            mask = ((angles >= -math.pi + sector * math.pi / 4) &
                    (angles < -math.pi + (sector + 1) * math.pi / 4))
            if np.any(mask):
                sector_values.append(float(np.mean(distances[mask] <= self.config.occupied_tolerance_m)))
        good_sectors = sum(value >= self.config.sector_coverage_min for value in sector_values)
        # Ray traversal is useful for final verification but is much more
        # expensive than distance-map lookup. Refinement uses the fast score;
        # the accepted candidate is always evaluated again with ray checks.
        free_fraction = self._free_space_fraction(points, pose) if include_free_space else 0.0
        composite = 0.55 * overlap + 0.30 * distance_score + 0.15 * free_fraction
        return {
            "overlap": overlap,
            "mean_distance_m": mean_distance,
            "distance_score": distance_score,
            "free_space": free_fraction,
            "good_sectors": float(good_sectors),
            "composite": composite,
        }

    def _grid_values(self, start: float, stop: float, step: float) -> np.ndarray:
        step = max(float(step), self.resolution)
        return np.arange(start, stop + step * 0.5, step, dtype=float)

    def _coarse_candidates(self, points: np.ndarray) -> list[tuple[float, tuple[float, float, float]]]:
        xy = self._grid_values(-self.radius, self.radius, self.config.global_xy_step_m)
        yaws = self._grid_values(-math.pi, math.pi, self.config.global_yaw_step_rad)
        candidates: list[tuple[float, tuple[float, float, float]]] = []
        for yaw in yaws:
            c, s = math.cos(float(yaw)), math.sin(float(yaw))
            relative = np.column_stack((c * points[:, 0] - s * points[:, 1],
                                        s * points[:, 0] + c * points[:, 1]))
            for x in xy:
                world_x = relative[:, 0, None] + x
                world_y = relative[:, 1, None] + xy[None, :]
                gx = np.floor((world_x + self.radius) / self.resolution).astype(int)
                gy = np.floor((world_y + self.radius) / self.resolution).astype(int)
                valid = ((gx >= 0) & (gx < self.data.shape[1]) &
                         (gy >= 0) & (gy < self.data.shape[0]))
                safe_gx = np.clip(gx, 0, self.data.shape[1] - 1)
                safe_gy = np.clip(gy, 0, self.data.shape[0] - 1)
                scores = np.mean(self._matching_mask[safe_gy, safe_gx] & valid, axis=0)
                for index in np.argsort(scores)[-min(3, len(scores)):]:
                    candidates.append((float(scores[index]), (float(x), float(xy[index]), float(yaw))))
        candidates.sort(key=lambda item: item[0], reverse=True)
        unique: list[tuple[float, tuple[float, float, float]]] = []
        for score, pose in candidates:
            if all(math.hypot(pose[0] - other[1][0], pose[1] - other[1][1]) > self.config.global_xy_step_m * 0.5
                   or abs(wrap_angle(pose[2] - other[1][2])) > self.config.global_yaw_step_rad * 0.5
                   for other in unique):
                unique.append((score, pose))
            if len(unique) >= max(1, self.config.candidate_count):
                break
        return unique

    def _refine(self, points: np.ndarray, seed: tuple[float, float, float]) -> tuple[dict[str, float], tuple[float, float, float]]:
        best = seed
        best_metrics = self._evaluate(points, best, include_free_space=False)
        for ix in self._grid_values(-self.config.global_refine_xy_radius_m, self.config.global_refine_xy_radius_m, self.config.global_refine_xy_step_m):
            for iy in self._grid_values(-self.config.global_refine_xy_radius_m, self.config.global_refine_xy_radius_m, self.config.global_refine_xy_step_m):
                for ia in self._grid_values(-self.config.global_refine_yaw_radius_rad, self.config.global_refine_yaw_radius_rad, self.config.global_refine_yaw_step_rad):
                    pose = (seed[0] + float(ix), seed[1] + float(iy), wrap_angle(seed[2] + float(ia)))
                    metrics = self._evaluate(points, pose, include_free_space=False)
                    if metrics["composite"] > best_metrics["composite"]:
                        best, best_metrics = pose, metrics
        return best_metrics, best

    def _global_match(self, points: np.ndarray, timestamp: float) -> PoseMeasurement | None:
        seeds = self._coarse_candidates(points[:min(len(points), self.config.max_global_points)])
        if not seeds:
            self.last_status = "rejected_global_match"
            return None
        verification_points = points[:min(len(points), self.config.max_refine_points)]
        refined = [self._refine(verification_points, pose) for _, pose in seeds]
        refined.sort(key=lambda item: item[0]["composite"], reverse=True)
        _, best = refined[0]
        metrics = self._evaluate(verification_points, best, include_free_space=True)
        second = refined[1][0]["composite"] if len(refined) > 1 else -1.0
        separation = metrics["composite"] - second if second >= 0 else 1.0
        metrics["best_second_difference"] = separation
        self.last_metrics = {key: (int(value) if key == "good_sectors" else float(value)) for key, value in metrics.items()}
        self.last_error_m = metrics["mean_distance_m"]
        enough_sectors = len(verification_points) < 16 or metrics["good_sectors"] >= self.config.min_good_sectors
        unambiguous = len(verification_points) < 16 or separation >= self.config.best_second_difference_min
        accepted = (metrics["overlap"] >= self.config.global_min_static_hit_fraction and
                    metrics["mean_distance_m"] <= self.config.mean_distance_threshold_m and
                    metrics["free_space"] >= self.config.global_min_free_space_fraction and
                    enough_sectors and unambiguous)
        if not accepted:
            self.last_status = "ambiguous_global_match" if not unambiguous else "rejected_global_match"
            return None
        self.global_localized = True
        self.last_status = "accepted_global_match"
        return PoseMeasurement(
            timestamp=float(timestamp), x_m=best[0], y_m=best[1], yaw_rad=best[2],
            covariance=np.diag([self.config.sigma_xy_m ** 2, self.config.sigma_xy_m ** 2,
                                self.config.sigma_yaw_rad ** 2]),
            source="known_map_global_localizer",
        )

    def _local_match(self, points: np.ndarray, predicted_pose: Sequence[float], timestamp: float) -> PoseMeasurement | None:
        best = tuple(float(v) for v in predicted_pose[:3])
        current_metrics = self._evaluate(points, best, include_free_space=False)
        for xy_radius, yaw_radius, xy_step, yaw_step in (
            (self.config.search_xy_radius_m, self.config.search_yaw_radius_rad,
             self.config.coarse_xy_step_m, self.config.coarse_yaw_step_rad),
            (self.config.coarse_xy_step_m, self.config.coarse_yaw_step_rad,
             self.config.fine_xy_step_m, self.config.fine_yaw_step_rad),
        ):
            nxy = int(round(xy_radius / xy_step))
            nyaw = int(round(yaw_radius / yaw_step))
            candidates = []
            for ix in range(-nxy, nxy + 1):
                for iy in range(-nxy, nxy + 1):
                    for ia in range(-nyaw, nyaw + 1):
                        pose = (best[0] + ix * xy_step, best[1] + iy * xy_step,
                                wrap_angle(best[2] + ia * yaw_step))
                        candidates.append((self._evaluate(points, pose, include_free_space=False), pose))
            current_metrics, best = max(candidates, key=lambda item: item[0]["composite"])
        current_metrics = self._evaluate(points, best, include_free_space=True)
        self.last_metrics = current_metrics
        self.last_error_m = current_metrics["mean_distance_m"]
        if current_metrics["overlap"] < self.config.min_static_hit_fraction:
            self.last_status = "rejected_match"
            return None
        self.last_status = "accepted_match"
        return PoseMeasurement(
            timestamp=float(timestamp), x_m=best[0], y_m=best[1], yaw_rad=best[2],
            covariance=np.diag([self.config.sigma_xy_m ** 2, self.config.sigma_xy_m ** 2,
                                self.config.sigma_yaw_rad ** 2]),
            source="known_map_lidar_localizer",
        )

    def match(self, points: Sequence[Sequence[float]], predicted_pose: Sequence[float],
              timestamp: float) -> PoseMeasurement | None:
        values = np.asarray(points, dtype=float)
        if values.ndim != 2 or values.shape[1] < 2:
            values = np.empty((0, 2), dtype=float)
        else:
            values = values[:, :2]
            values = values[np.all(np.isfinite(values), axis=1)]
        if not self.config.enabled or len(values) < self.config.min_points:
            self.last_status = "insufficient_points"
            return None
        if len(values) > self.config.max_refine_points:
            values = values[np.linspace(0, len(values) - 1, self.config.max_refine_points).astype(int)]
        if not self.global_localized:
            return self._global_match(values, timestamp)
        return self._local_match(values, predicted_pose, timestamp)
