"""Small, dependency-light 2-D navigation stack for the Pioneer controller.

The planner operates in the same local map frame as the EKF: metres, +x
forward at startup and +y left.  UNKNOWN cells are allowed for A* by default. Frontier selection remains available for exploration.
"""
from __future__ import annotations

from dataclasses import dataclass
import heapq
import math
from typing import Iterable, Sequence

import numpy as np


@dataclass
class GridSpec:
    resolution: float = 0.05
    radius: float = 20.0


class OccupancyGrid:
    def __init__(self, resolution: float = 0.05, radius: float = 20.0) -> None:
        self.spec = GridSpec(float(resolution), float(radius))
        self._allocate()

    def _allocate(self) -> None:
        if self.spec.resolution <= 0 or self.spec.radius <= 0:
            raise ValueError("resolution and radius must be positive")
        self.size = int(math.ceil(2 * self.spec.radius / self.spec.resolution))
        self.data = np.full((self.size, self.size), -1, dtype=np.int8)
        self.static_data = self.data.copy()
        self.revision = getattr(self, "revision", 0) + 1

    @property
    def resolution(self) -> float:
        return self.spec.resolution

    @property
    def radius(self) -> float:
        return self.spec.radius

    def clear(self) -> None:
        self.data.fill(-1)
        self.static_data.fill(-1)
        self.revision += 1

    def cell(self, x: float, y: float) -> tuple[int, int] | None:
        gx = int(math.floor((float(x) + self.radius) / self.resolution))
        gy = int(math.floor((float(y) + self.radius) / self.resolution))
        if 0 <= gx < self.size and 0 <= gy < self.size:
            return gx, gy
        return None

    def point(self, cell: tuple[int, int]) -> tuple[float, float]:
        gx, gy = cell
        return ((gx + 0.5) * self.resolution - self.radius,
                (gy + 0.5) * self.resolution - self.radius)

    def set_cell(self, cell: tuple[int, int], value: int) -> None:
        gx, gy = cell
        if 0 <= gx < self.size and 0 <= gy < self.size:
            self.data[gy, gx] = int(value)

    def update_ray(self, x0: float, y0: float, x1: float, y1: float,
                   mark_endpoint=True, clear_occupied=False, protected_cells=()) -> list:
        """Insert a LiDAR ray using free space plus an occupied endpoint."""
        dx, dy = x1 - x0, y1 - y0
        steps = max(1, int(math.ceil(math.hypot(dx, dy) / (self.resolution * 0.7))))
        freed=[]
        for i in range(steps):
            t = i / steps
            cell = self.cell(x0 + dx * t, y0 + dy * t)
            if cell is not None and cell not in protected_cells and (clear_occupied or self.data[cell[1], cell[0]] != 100):
                self.data[cell[1], cell[0]] = 0
                freed.append(cell)
        endpoint = self.cell(x1, y1)
        if endpoint is not None and mark_endpoint:
            self.data[endpoint[1], endpoint[0]] = 100
        self.revision += 1
        return freed

    def load(self, resolution: float, radius: float, values: Sequence[int]) -> None:
        self.spec = GridSpec(float(resolution), float(radius))
        self._allocate()
        flat = np.asarray(list(values), dtype=np.int8)
        if flat.size != self.size * self.size:
            raise ValueError("navigation map size does not match resolution/radius")
        self.data[:, :] = flat.reshape((self.size, self.size))
        self.static_data = self.data.copy()
        self.revision += 1

    def restore_static(self) -> None:
        """Remove the previous scan's transient obstacle layer."""
        if not np.array_equal(self.data, self.static_data):
            self.data[:, :] = self.static_data
            self.revision += 1

    def set_dynamic_occupied(self, x: float, y: float) -> None:
        """Add a current-scan obstacle without modifying the static map."""
        cell = self.cell(x, y)
        if cell is None:
            return
        gx, gy = cell
        if self.static_data[gy, gx] != 100 and self.data[gy, gx] != 100:
            self.data[gy, gx] = 100
            self.revision += 1

    def inflated_obstacles(self, margin_m: float) -> np.ndarray:
        blocked = self.data == 100
        cells = int(math.ceil(max(0.0, margin_m) / self.resolution))
        if cells == 0:
            return blocked.copy()
        inflated = blocked.copy()
        occupied = np.argwhere(blocked)
        for gy, gx in occupied:
            y0, y1 = max(0, gy - cells), min(self.size, gy + cells + 1)
            x0, x1 = max(0, gx - cells), min(self.size, gx + cells + 1)
            inflated[y0:y1, x0:x1] = True
        return inflated

    def frontier_count(self) -> int:
        """Number of free cells touching at least one unknown cell."""
        if self.size < 3:
            return 0
        free = self.data == 0
        unknown = self.data < 0
        touching_unknown = np.zeros_like(unknown, dtype=bool)
        touching_unknown[1:, :] |= unknown[:-1, :]
        touching_unknown[:-1, :] |= unknown[1:, :]
        touching_unknown[:, 1:] |= unknown[:, :-1]
        touching_unknown[:, :-1] |= unknown[:, 1:]
        return int(np.count_nonzero(free & touching_unknown))

    def traversable(self, cell: tuple[int, int], blocked: np.ndarray, allow_unknown: bool = True) -> bool:
        gx, gy = cell
        if not (0 <= gx < self.size and 0 <= gy < self.size) or blocked[gy, gx]:
            return False
        return bool(self.data[gy, gx] == 0 or (allow_unknown and self.data[gy, gx] < 0))

    def nearest_free(self, cell: tuple[int, int], blocked: np.ndarray, limit: int = 30) -> tuple[int, int] | None:
        gx, gy = cell
        candidates = []
        for y in range(max(0, gy - limit), min(self.size, gy + limit + 1)):
            for x in range(max(0, gx - limit), min(self.size, gx + limit + 1)):
                if self.traversable((x, y), blocked):
                    candidates.append((abs(x - gx) + abs(y - gy), (x, y)))
        return min(candidates)[1] if candidates else None


def astar(grid: OccupancyGrid, start: tuple[int, int], goal: tuple[int, int], blocked: np.ndarray, allow_unknown: bool = True, unknown_cost: float = 1.15, edge_allowed=None) -> list[tuple[int, int]]:
    if not grid.traversable(start, blocked, allow_unknown) or not grid.traversable(goal, blocked, allow_unknown):
        return []
    directions = ((1, 0), (-1, 0), (0, 1), (0, -1),
                  (1, 1), (1, -1), (-1, 1), (-1, -1))
    open_set = [(0.0, start)]
    came: dict[tuple[int, int], tuple[int, int]] = {}
    cost = {start: 0.0}
    while open_set:
        _, current = heapq.heappop(open_set)
        if current == goal:
            path = [current]
            while current in came:
                current = came[current]
                path.append(current)
            return list(reversed(path))
        for dx, dy in directions:
            nxt = (current[0] + dx, current[1] + dy)
            if not grid.traversable(nxt, blocked, allow_unknown):
                continue
            if edge_allowed is not None and not edge_allowed(current,nxt):
                continue
            if dx and dy and (not grid.traversable((current[0] + dx, current[1]), blocked, allow_unknown)
                              or not grid.traversable((current[0], current[1] + dy), blocked, allow_unknown)):
                continue
            step = math.sqrt(2.0) if dx and dy else 1.0
            if grid.data[nxt[1], nxt[0]] < 0:
                step *= unknown_cost
            new_cost = cost[current] + step
            if new_cost < cost.get(nxt, float("inf")):
                cost[nxt] = new_cost
                h = math.hypot(goal[0] - nxt[0], goal[1] - nxt[1])
                heapq.heappush(open_set, (new_cost + h, nxt))
                came[nxt] = current
    return []


class NavigationController:
    """A* goal following and frontier exploration with a conservative stop."""
    def __init__(self, grid: OccupancyGrid, safety_margin_m: float = 0.03,
                 max_linear_mps: float = 0.25, max_angular_rps: float = 1.0,
                 allow_unknown: bool = True, unknown_cost: float = 1.15) -> None:
        self.grid = grid
        self.safety_margin_m = float(safety_margin_m)
        self.max_linear = float(max_linear_mps)
        self.max_angular = float(max_angular_rps)
        self.allow_unknown = bool(allow_unknown)
        self.unknown_cost = max(0.0, float(unknown_cost))
        self.mode = "idle"
        self.goal: tuple[float, float] | None = None
        self.path: list[tuple[int, int]] = []
        self.status = "idle"
        self._active_explore_goal: tuple[int, int] | None = None
        self._waiting_for_map_update = False
        self._plan_revision = grid.revision

    def stop(self, status: str = "stopped") -> None:
        self.mode, self.path, self.goal, self.status = "idle", [], None, status
        self._active_explore_goal = None
        self._waiting_for_map_update = False

    def set_goal(self, x: float, y: float) -> None:
        self.goal = (float(x), float(y))
        self.mode = "goal"
        self.status = "goal_pending"
        self.path = []
        self._active_explore_goal = None
        self._waiting_for_map_update = False

    def explore(self) -> None:
        self.mode = "explore"
        self.goal = None
        self.path = []
        self.status = "exploring"
        self._active_explore_goal = None
        self._waiting_for_map_update = False

    @property
    def active_target(self) -> tuple[float, float] | None:
        if self.goal is not None:
            return self.goal
        if self._active_explore_goal is not None:
            return self.grid.point(self._active_explore_goal)
        return None

    def load_map(self, resolution: float, radius: float, values: Sequence[int]) -> None:
        self.grid.load(resolution, radius, values)
        self.path = []
        self._waiting_for_map_update = False
        self._active_explore_goal = None

    def _frontier_candidates(self, start: tuple[int, int], blocked: np.ndarray) -> list[tuple[int, int]]:
        candidates = []
        for gy in range(1, self.grid.size - 1):
            for gx in range(1, self.grid.size - 1):
                cell = (gx, gy)
                if not self.grid.traversable(cell, blocked):
                    continue
                neighbours = [(gx + dx, gy + dy) for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1))]
                if not any(self.grid.data[y, x] < 0 for x, y in neighbours):
                    continue
                distance = math.hypot(gx - start[0], gy - start[1])
                # Prefer nearby frontiers, with a mild preference for larger exploration distance.
                candidates.append((distance, cell))
        candidates.sort(key=lambda item: item[0])
        return [cell for _, cell in candidates]

    def _plan(self, pose: Sequence[float]) -> None:
        blocked = self.grid.inflated_obstacles(self.safety_margin_m)
        start_raw = self.grid.cell(pose[0], pose[1])
        if start_raw is None:
            self.status, self.path, self.mode = "robot_outside_map", [], "idle"
            return
        start = self.grid.nearest_free(start_raw, blocked)
        if start is None:
            self.status, self.path, self.mode = "no_safe_start", [], "idle"
            return
        if self.mode == "goal":
            assert self.goal is not None
            goal_raw = self.grid.cell(*self.goal)
            if goal_raw is None:
                self.status, self.path, self.mode = "goal_outside_map", [], "idle"
                return
            goal = self.grid.nearest_free(goal_raw, blocked, limit=20)
            if goal is None:
                self.status, self.path, self.mode = "goal_blocked", [], "idle"
                return
        else:
            # Try frontiers in distance order. A frontier can be locally free
            # but unreachable after safety-margin inflation, so do not stop
            # exploration just because the first candidate is blocked.
            candidates = self._frontier_candidates(start, blocked)
            goal = None
            for candidate in candidates:
                candidate_path = astar(self.grid, start, candidate, blocked, self.allow_unknown, self.unknown_cost)
                if candidate_path:
                    goal = candidate
                    self.path = candidate_path
                    self._active_explore_goal = goal
                    self._plan_revision = self.grid.revision
                    break
            if goal is None:
                status = "exploration_complete" if not candidates else "no_reachable_frontier"
                self.status, self.path, self.mode = status, [], "idle"
                self._active_explore_goal = None
                return
        if self.mode == "goal":
            self.path = astar(self.grid, start, goal, blocked, self.allow_unknown, self.unknown_cost)
        self.status = "following" if self.path else "no_path"
        self._plan_revision = self.grid.revision
        if not self.path:
            self.mode = "idle"

    def _remaining_path_blocked(self) -> bool:
        """Check the existing route against the latest inflated occupancy."""
        if not self.path:
            return False
        blocked = self.grid.inflated_obstacles(self.safety_margin_m)
        return any(blocked[gy, gx] for gx, gy in self.path
                   if 0 <= gx < self.grid.size and 0 <= gy < self.grid.size)

    def command(self, pose: Sequence[float]) -> tuple[float, float]:
        if self.mode == "idle":
            return 0.0, 0.0
        if (self.mode == "goal" and self.path
                and self.grid.revision != self._plan_revision):
            if self._remaining_path_blocked():
                # LiDAR may have added a transient obstacle after A* planned
                # this route. Drop only the invalid route; the next block
                # replans the same goal against the updated grid.
                self.path = []
                self.status = "replanning"
            self._plan_revision = self.grid.revision
        if self.mode == "explore" and self._waiting_for_map_update:
            if self.grid.revision == self._plan_revision:
                self.status = "waiting_for_lidar"
                return 0.0, 0.0
            self._waiting_for_map_update = False
            self.path = []
            self.status = "replanning"
        if self.mode == "goal" and self.goal is not None:
            if math.hypot(pose[0] - self.goal[0], pose[1] - self.goal[1]) <= max(self.grid.resolution * 1.5, 0.12):
                self.stop("goal_reached")
                return 0.0, 0.0
        if not self.path:
            self._plan(pose)
        if not self.path:
            return 0.0, 0.0
        current = self.grid.cell(pose[0], pose[1])
        if current is None:
            self.stop("robot_outside_map")
            return 0.0, 0.0
        while len(self.path) > 1 and math.hypot(self.path[0][0] - current[0], self.path[0][1] - current[1]) <= 2.0:
            self.path.pop(0)
        if self.mode == "explore" and len(self.path) == 1 and math.hypot(self.path[0][0] - current[0], self.path[0][1] - current[1]) <= 2.0:
            # The frontier has been visited. Hold position until the next
            # LiDAR update changes the map, then force a fresh frontier search.
            self.path = []
            self._waiting_for_map_update = True
            self._plan_revision = self.grid.revision
            self.status = "frontier_reached"
            return 0.0, 0.0
        target = self.grid.point(self.path[min(3, len(self.path) - 1)])
        desired = math.atan2(target[1] - pose[1], target[0] - pose[0])
        error = math.atan2(math.sin(desired - pose[2]), math.cos(desired - pose[2]))
        omega = max(-self.max_angular, min(self.max_angular, 2.0 * error))
        alignment = max(0.0, math.cos(error))
        distance = math.hypot(target[0] - pose[0], target[1] - pose[1])
        v = min(self.max_linear, 0.8 * distance) * alignment
        if abs(error) > 1.0:
            v = 0.0
        return v, omega
