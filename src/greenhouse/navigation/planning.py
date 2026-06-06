"""Планирование: глобальный маршрут до цели + локальный объезд препятствий.

Разделение на два уровня — стандарт навигации:
- GlobalPlanner: путь по карте от старта до цели (A*/Dijkstra по сетке занятости).
- LocalPlanner: на коротком горизонте превращает путь в команду скорости, реактивно
  объезжая то, чего нет на карте (внезапное препятствие из лидара).
- GoalNavigator: фасад, который связывает оба уровня и доводит робота до цели.
"""

from __future__ import annotations

from typing import Literal, Protocol

from pydantic import BaseModel

from greenhouse.domain.errors import Failure
from greenhouse.domain.geometry import Pose2D, Twist2D
from greenhouse.domain.grid import OccupancyGrid
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

    def plan(self, *, grid: OccupancyGrid, start: Pose2D, goal: Pose2D, robot_radius_m: float) -> PlanResult: ...


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
