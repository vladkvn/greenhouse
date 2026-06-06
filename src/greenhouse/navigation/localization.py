"""Локализация: где робот находится на карте.

Контракт `Localizer` не зависит от реализации. Заглушка — truth из симулятора
(`SimTruthLocalizer`); рабочая реализация — `ScanMatchLocalizer` (сопоставление скана с
сеткой занятости поверх одометрии), без подглядывания в истину симулятора.
"""

from __future__ import annotations

import math
from typing import Protocol

from pydantic import BaseModel, Field

from greenhouse.domain.geometry import Point2D, Pose2D
from greenhouse.domain.grid import CellState, OccupancyGrid
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


class ScanMatchLocalizer:
    """Реализует `Localizer` сопоставлением скана с сеткой занятости (scan matching).

    Шаг прогноза: к оценке прибавляется приращение одометрии (dead reckoning). Шаг
    коррекции: локальный поиск по смещению (x, y, theta) вокруг прогноза максимизирует
    долю концов лучей, попадающих в занятые ячейки карты, — поза «защёлкивается» на
    реальную геометрию, исправляя дрейф одометрии и начальное смещение. Уверенность —
    доля совпавших лучей; при несовпадении скана с картой она падает (потеря локализации).
    Истину симулятора реализация не использует.
    """

    def __init__(
        self,
        *,
        lost_confidence: float = 0.3,
        xy_step_m: float = 0.1,
        theta_step_rad: float = 0.05,
        refine_iters: int = 4,
    ) -> None:
        self._grid: OccupancyGrid | None = None
        self._est: Pose2D | None = None
        self._last_odom: Pose2D | None = None
        self._latest: PoseEstimate | None = None
        self._lost_conf = lost_confidence
        self._xy_step = xy_step_m
        self._theta_step = theta_step_rad
        self._iters = refine_iters

    def set_map(self, *, grid: OccupancyGrid) -> None:
        self._grid = grid

    def set_initial_pose(self, *, pose: Pose2D) -> None:
        self._est = pose
        self._last_odom = None

    def update(self, *, scan: LidarScan, odometry: Odometry) -> PoseEstimate:
        if self._est is None:
            self._est = odometry.pose
        # Прогноз: приращение одометрии (в общей системе координат симулятора).
        if self._last_odom is not None:
            self._est = Pose2D(
                x_m=self._est.x_m + (odometry.pose.x_m - self._last_odom.x_m),
                y_m=self._est.y_m + (odometry.pose.y_m - self._last_odom.y_m),
                theta_rad=self._est.theta_rad
                + _wrap(odometry.pose.theta_rad - self._last_odom.theta_rad),
            )
        self._last_odom = odometry.pose

        pose, confidence = self._match(scan, self._est)
        self._est = pose
        self._latest = PoseEstimate(pose=pose, confidence=confidence, stamp_s=scan.stamp_s)
        return self._latest

    def latest(self) -> PoseEstimate | None:
        return self._latest

    def _match(self, scan: LidarScan, seed: Pose2D) -> tuple[Pose2D, float]:
        grid = self._grid
        if grid is None:
            return seed, 0.0
        best = seed
        best_score = self._score(scan, seed, grid)
        xy, th = self._xy_step, self._theta_step
        for _ in range(self._iters):  # координатный спуск с уменьшением шага
            improved = False
            for dx in (-xy, 0.0, xy):
                for dy in (-xy, 0.0, xy):
                    for dth in (-th, 0.0, th):
                        cand = Pose2D(
                            x_m=best.x_m + dx, y_m=best.y_m + dy, theta_rad=best.theta_rad + dth
                        )
                        score = self._score(scan, cand, grid)
                        if score > best_score:
                            best, best_score, improved = cand, score, True
            if not improved:
                xy *= 0.5
                th *= 0.5
        valid = sum(1 for r in scan.ranges_m if math.isfinite(r) and r < scan.range_max_m)
        confidence = best_score / valid if valid else 0.0
        return best, confidence

    def _score(self, scan: LidarScan, pose: Pose2D, grid: OccupancyGrid) -> int:
        """Сколько концов лучей попадают в занятую ячейку (или её 8-соседство)."""
        meta = grid.meta
        hits = 0
        for i, r in enumerate(scan.ranges_m):
            if not math.isfinite(r) or r >= scan.range_max_m:
                continue
            angle = pose.theta_rad + scan.angle_min_rad + i * scan.angle_increment_rad
            row, col = meta.world_to_cell(
                Point2D(x_m=pose.x_m + r * math.cos(angle), y_m=pose.y_m + r * math.sin(angle))
            )
            if _occupied_near(grid, row, col):
                hits += 1
        return hits


def _occupied_near(grid: OccupancyGrid, row: int, col: int) -> bool:
    for dr in (-1, 0, 1):
        for dc in (-1, 0, 1):
            if grid.at(row + dr, col + dc) is CellState.OCCUPIED:
                return True
    return False


def _wrap(angle_rad: float) -> float:
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))
