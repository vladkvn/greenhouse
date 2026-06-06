"""Планирование: глобальный маршрут до цели + локальный объезд препятствий.

Разделение на два уровня — стандарт навигации:
- GlobalPlanner: путь по карте от старта до цели (A*/Dijkstra по сетке занятости).
- LocalPlanner: на коротком горизонте превращает путь в команду скорости, реактивно
  объезжая то, чего нет на карте (внезапное препятствие из лидара).
- GoalNavigator: фасад, который связывает оба уровня и доводит робота до цели.
"""

from __future__ import annotations

import heapq
import math
from itertools import count
from typing import Literal, Protocol

from pydantic import BaseModel

from greenhouse.domain.errors import Failure, FailureCode
from greenhouse.domain.geometry import Pose2D, Twist2D
from greenhouse.domain.grid import CellState, MapMeta, OccupancyGrid
from greenhouse.sensing.interfaces import LidarScan


class Path(BaseModel, frozen=True):
    """Разреженная полилиния в системе координат карты."""

    waypoints: tuple[Pose2D, ...]


class PlanOk(BaseModel, frozen=True):
    kind: Literal["ok"] = "ok"
    path: Path


PlanResult = PlanOk | Failure
"""Дискриминированный результат планирования (по полю kind)."""


class GlobalPlanner(Protocol):
    """Строит путь по сетке занятости с учётом габарита робота.

    `grid` уже включает keep-out слой (карта ∪ зоны) — планировщик о зонах отдельно
    не знает, он видит единую модель занятости.
    """

    def plan(
        self, *, grid: OccupancyGrid, start: Pose2D, goal: Pose2D, robot_radius_m: float
    ) -> PlanResult: ...


class LocalPlanner(Protocol):
    """Реактивный слой: следующая команда скорости вдоль пути с объездом препятствий
    по свежему скану лидара."""

    def set_path(self, *, path: Path) -> None: ...

    def compute_command(self, *, pose: Pose2D, scan: LidarScan) -> Twist2D: ...

    def is_goal_reached(self, *, pose: Pose2D) -> bool: ...


class NavOutcome(BaseModel, frozen=True):
    kind: Literal["reached", "cancelled"]
    final_pose_error_m: float | None = None


NavResult = NavOutcome | Failure


class GoalNavigator(Protocol):
    """Фасад навигации к цели: владеет связкой global+local, отдаёт команды в control."""

    def navigate_to(self, *, goal: Pose2D) -> NavResult: ...

    def cancel(self) -> None: ...


# --- Реализации ---

_SQRT2 = math.sqrt(2.0)

# 8-связность: (drow, dcol, стоимость шага).
_NEIGHBORS: tuple[tuple[int, int, float], ...] = (
    (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
    (-1, -1, _SQRT2), (-1, 1, _SQRT2), (1, -1, _SQRT2), (1, 1, _SQRT2),
)


class AStarPlanner:
    """Реализует `GlobalPlanner`: A* по сетке занятости с инфляцией под радиус робота.

    Препятствия раздуваются на радиус робота, поэтому путь планируется для точки, но
    гарантирует зазор для габарита. Диагональные шаги запрещены «срезать» угол занятой
    ячейки. Эвристика — октайл-расстояние (допустимая для 8-связности).
    """

    def __init__(self, *, allow_unknown: bool = True) -> None:
        self._allow_unknown = allow_unknown

    def plan(
        self, *, grid: OccupancyGrid, start: Pose2D, goal: Pose2D, robot_radius_m: float
    ) -> PlanResult:
        meta = grid.meta
        blocked = _inflate(grid, robot_radius_m)
        start_cell = meta.world_to_cell(start.point)
        goal_cell = meta.world_to_cell(goal.point)
        if not self._free(grid, blocked, start_cell):
            return Failure(code=FailureCode.PLANNER_FAILED, message="старт в занятой ячейке")
        if not self._free(grid, blocked, goal_cell):
            return Failure(code=FailureCode.PLANNER_FAILED, message="цель недостижима (занята)")

        cells = self._astar(grid, blocked, start_cell, goal_cell)
        if cells is None:
            return Failure(code=FailureCode.PLANNER_FAILED, message="путь не найден")
        return PlanOk(path=Path(waypoints=_to_waypoints(meta, cells, start, goal)))

    def _free(self, grid: OccupancyGrid, blocked: set[tuple[int, int]], cell: tuple[int, int]) -> bool:
        row, col = cell
        if not grid.meta.in_bounds(row, col):
            return False
        if cell in blocked:
            return False
        if not self._allow_unknown and grid.at(row, col) is CellState.UNKNOWN:
            return False
        return True

    def _astar(
        self,
        grid: OccupancyGrid,
        blocked: set[tuple[int, int]],
        start: tuple[int, int],
        goal: tuple[int, int],
    ) -> list[tuple[int, int]] | None:
        counter = count()
        open_heap: list[tuple[float, int, tuple[int, int]]] = [
            (_octile(start, goal), next(counter), start)
        ]
        g_score: dict[tuple[int, int], float] = {start: 0.0}
        came_from: dict[tuple[int, int], tuple[int, int]] = {}
        closed: set[tuple[int, int]] = set()

        while open_heap:
            _, _, cur = heapq.heappop(open_heap)
            if cur == goal:
                return _reconstruct(came_from, cur)
            if cur in closed:
                continue
            closed.add(cur)
            r, c = cur
            for dr, dc, step in _NEIGHBORS:
                nb = (r + dr, c + dc)
                if nb in closed or not self._free(grid, blocked, nb):
                    continue
                if dr != 0 and dc != 0:  # не срезать угол занятой ячейки
                    if not self._free(grid, blocked, (r + dr, c)):
                        continue
                    if not self._free(grid, blocked, (r, c + dc)):
                        continue
                tentative = g_score[cur] + step
                if tentative < g_score.get(nb, math.inf):
                    g_score[nb] = tentative
                    came_from[nb] = cur
                    heapq.heappush(open_heap, (tentative + _octile(nb, goal), next(counter), nb))
        return None


class PurePursuitLocalPlanner:
    """Реализует `LocalPlanner`: ведёт робота вдоль пути (carrot / pure pursuit).

    В Инкременте 3 объезда нет — `scan` пока не используется (это слой Инкремента 4):
    планировщик доворачивает на ближайший непройденный путевой узел и едет к нему,
    притормаживая у цели.
    """

    def __init__(
        self,
        *,
        max_linear_m_s: float = 0.6,
        max_angular_rad_s: float = 1.5,
        goal_tol_m: float = 0.2,
        waypoint_tol_m: float = 0.3,
        turn_in_place_rad: float = 0.6,
    ) -> None:
        self._v_max = max_linear_m_s
        self._w_max = max_angular_rad_s
        self._goal_tol = goal_tol_m
        self._wp_tol = waypoint_tol_m
        self._turn = turn_in_place_rad
        self._waypoints: tuple[Pose2D, ...] = ()
        self._idx = 0

    def set_path(self, *, path: Path) -> None:
        self._waypoints = path.waypoints
        self._idx = 0

    def compute_command(self, *, pose: Pose2D, scan: LidarScan) -> Twist2D:
        if not self._waypoints:
            return Twist2D.stop()
        last = len(self._waypoints) - 1
        while self._idx < last and _dist(pose, self._waypoints[self._idx]) < self._wp_tol:
            self._idx += 1

        target = self._waypoints[self._idx]
        err = _wrap(math.atan2(target.y_m - pose.y_m, target.x_m - pose.x_m) - pose.theta_rad)
        w = _clamp(2.0 * err, -self._w_max, self._w_max)
        if abs(err) > self._turn:
            return Twist2D(linear_x_m_s=0.0, angular_z_rad_s=w)  # сначала довернуть
        v = min(self._v_max * math.cos(err), 1.5 * _dist(pose, self._waypoints[last]))
        return Twist2D(linear_x_m_s=max(0.0, v), angular_z_rad_s=w)

    def is_goal_reached(self, *, pose: Pose2D) -> bool:
        if not self._waypoints:
            return True
        return _dist(pose, self._waypoints[-1]) <= self._goal_tol


def _inflate(grid: OccupancyGrid, robot_radius_m: float) -> set[tuple[int, int]]:
    """Множество непроезжих ячеек: занятые, раздутые на радиус робота (в ячейках)."""
    meta = grid.meta
    r_cells = int(math.ceil(robot_radius_m / meta.resolution_m))
    blocked: set[tuple[int, int]] = set()
    for row in range(meta.height_px):
        for col in range(meta.width_px):
            if grid.at(row, col) is not CellState.OCCUPIED:
                continue
            for dr in range(-r_cells, r_cells + 1):
                for dc in range(-r_cells, r_cells + 1):
                    if dr * dr + dc * dc <= r_cells * r_cells:
                        blocked.add((row + dr, col + dc))
    return blocked


def _reconstruct(
    came_from: dict[tuple[int, int], tuple[int, int]], cur: tuple[int, int]
) -> list[tuple[int, int]]:
    path = [cur]
    while cur in came_from:
        cur = came_from[cur]
        path.append(cur)
    path.reverse()
    return path


def _to_waypoints(
    meta: MapMeta, cells: list[tuple[int, int]], start: Pose2D, goal: Pose2D
) -> tuple[Pose2D, ...]:
    """Сжать путь по ячейкам в разреженную полилинию (узлы только в точках поворота)."""
    if len(cells) <= 1:
        return (Pose2D(x_m=goal.x_m, y_m=goal.y_m, theta_rad=goal.theta_rad),)

    keep = [cells[0]]
    for i in range(1, len(cells) - 1):
        d_in = (cells[i][0] - cells[i - 1][0], cells[i][1] - cells[i - 1][1])
        d_out = (cells[i + 1][0] - cells[i][0], cells[i + 1][1] - cells[i][1])
        if d_in != d_out:  # направление изменилось — это поворотный узел
            keep.append(cells[i])
    keep.append(cells[-1])

    points = [meta.cell_to_world(r, c) for r, c in keep]
    points[0] = start.point     # реальные координаты старта/цели вместо центров ячеек
    points[-1] = goal.point

    poses: list[Pose2D] = []
    for i, p in enumerate(points):
        if i < len(points) - 1:
            nxt = points[i + 1]
            theta = math.atan2(nxt.y_m - p.y_m, nxt.x_m - p.x_m)
        else:
            theta = goal.theta_rad
        poses.append(Pose2D(x_m=p.x_m, y_m=p.y_m, theta_rad=theta))
    return tuple(poses)


def _octile(a: tuple[int, int], b: tuple[int, int]) -> float:
    dr = abs(a[0] - b[0])
    dc = abs(a[1] - b[1])
    return (dr + dc) + (_SQRT2 - 2.0) * min(dr, dc)


def _dist(pose: Pose2D, wp: Pose2D) -> float:
    return math.hypot(wp.x_m - pose.x_m, wp.y_m - pose.y_m)


def _wrap(angle_rad: float) -> float:
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def _clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))
