"""Локализация: где робот находится на карте.

Сейчас реализация может быть тривиальной (truth из симулятора). Цель — scan-matching /
particle filter поверх построенной сетки. Контракт от реализации не зависит.
"""

from __future__ import annotations

from typing import Protocol

from pydantic import BaseModel, Field

from greenhouse.domain.geometry import Pose2D
from greenhouse.domain.grid import OccupancyGrid
from greenhouse.sensing.interfaces import LidarScan, Odometry


class PoseEstimate(BaseModel, frozen=True):
    """Оценка позы с мерой уверенности."""

    pose: Pose2D
    confidence: float = Field(ge=0.0, le=1.0, description="0 = потеряна, 1 = уверенная.")
    stamp_s: float

    @property
    def is_lost(self) -> bool:
        return self.confidence <= 0.0


class Localizer(Protocol):
    """Оценивает позу робота на заданной карте по скану и одометрии."""

    def set_map(self, *, grid: OccupancyGrid) -> None: ...

    def set_initial_pose(self, *, pose: Pose2D) -> None: ...

    def update(self, *, scan: LidarScan, odometry: Odometry) -> PoseEstimate:
        """Слить наблюдение и движение в новую оценку позы."""
        ...

    def latest(self) -> PoseEstimate | None: ...
