"""Построение карты: сессия SLAM и хранилище карт.

Оркестрация владеет жизненным циклом сессии (start/stop). Конкретный алгоритм
(evidence-сетка по лидару сейчас, slam_toolbox через адаптер позже) скрыт за Protocol.
"""

from __future__ import annotations

import math
from collections.abc import Iterator
from typing import Protocol

from pydantic import BaseModel

from greenhouse.domain.geometry import Point2D
from greenhouse.domain.grid import CellState, MapMeta, OccupancyGrid
from greenhouse.sensing.interfaces import LidarScan


class MapBuilder(Protocol):
    """Инкрементальное построение карты из сканов лидара.

    Поза передаётся снаружи (из локализации/одометрии), чтобы строитель карты не
    зависел жёстко от конкретного локализатора.
    """

    def begin_session(self) -> None: ...

    def ingest_scan(self, *, scan: LidarScan, pose_xytheta: tuple[float, float, float]) -> None:
        """Добавить наблюдение в карту с известной (оценённой) позой робота."""
        ...

    def current_map(self) -> OccupancyGrid:
        """Текущее наилучшее представление карты."""
        ...

    def end_session(self) -> OccupancyGrid:
        """Завершить сессию и вернуть итоговую карту."""
        ...


class EvidenceGridMapper:
    """Реализует `MapBuilder`: лог-odds сетка занятости по лидар-рейкасту.

    Инверсная модель датчика: вдоль каждого луча ячейки до точки возврата получают
    свидетельство «свободно», а ячейка попадания — «занято». Свидетельства копятся в
    лог-odds, поэтому карта устойчива к шуму — повторные наблюдения усиливают уверенность,
    одиночный выброс не переключает ячейку. Луч растеризуется алгоритмом Брезенхэма по той
    же сетке, что потом увидят планировщик и проверка коллизий (единая модель занятости).

    Поза приходит снаружи как `(x, y, theta)` в координатах карты; углы скана считаются
    относительно курса робота, поэтому мировой угол луча = `theta + angle_min + i*incr`.
    """

    def __init__(
        self,
        *,
        meta: MapMeta,
        l_free: float = 0.4,
        l_occ: float = 0.85,
        l_clamp: float = 5.0,
        occ_threshold: float = 0.5,
        free_threshold: float = -0.35,
    ) -> None:
        self._meta = meta
        self._l_free = l_free
        self._l_occ = l_occ
        self._l_clamp = l_clamp
        self._occ_threshold = occ_threshold
        self._free_threshold = free_threshold
        self._logodds = [0.0] * (meta.width_px * meta.height_px)

    @property
    def meta(self) -> MapMeta:
        return self._meta

    def begin_session(self) -> None:
        self._logodds = [0.0] * (self._meta.width_px * self._meta.height_px)

    def ingest_scan(self, *, scan: LidarScan, pose_xytheta: tuple[float, float, float]) -> None:
        x, y, theta = pose_xytheta
        origin = self._meta.world_to_cell(Point2D(x_m=x, y_m=y))
        for i, r in enumerate(scan.ranges_m):
            angle = theta + scan.angle_min_rad + i * scan.angle_increment_rad
            hit = math.isfinite(r) and r < scan.range_max_m
            reach = r if hit else scan.range_max_m
            end = self._meta.world_to_cell(
                Point2D(x_m=x + reach * math.cos(angle), y_m=y + reach * math.sin(angle))
            )
            cells = list(_bresenham(origin, end))
            # Все ячейки до конечной — свободны; конечная — занята при реальном возврате.
            for cell in cells[:-1]:
                self._update(cell, -self._l_free)
            self._update(cells[-1], self._l_occ if hit else -self._l_free)

    def current_map(self) -> OccupancyGrid:
        return OccupancyGrid(meta=self._meta, cells=[self._classify(lo) for lo in self._logodds])

    def end_session(self) -> OccupancyGrid:
        return self.current_map()

    def _update(self, cell: tuple[int, int], delta: float) -> None:
        row, col = cell
        if not self._meta.in_bounds(row, col):
            return
        idx = row * self._meta.width_px + col
        self._logodds[idx] = _clamp(self._logodds[idx] + delta, -self._l_clamp, self._l_clamp)

    def _classify(self, logodds: float) -> CellState:
        if logodds > self._occ_threshold:
            return CellState.OCCUPIED
        if logodds < self._free_threshold:
            return CellState.FREE
        return CellState.UNKNOWN


def _bresenham(start: tuple[int, int], end: tuple[int, int]) -> Iterator[tuple[int, int]]:
    """Целочисленные ячейки `(row, col)` вдоль отрезка start→end включительно."""
    r0, c0 = start
    r1, c1 = end
    dr = abs(r1 - r0)
    dc = abs(c1 - c0)
    sr = 1 if r1 >= r0 else -1
    sc = 1 if c1 >= c0 else -1
    err = dc - dr
    r, c = r0, c0
    while True:
        yield r, c
        if r == r1 and c == c1:
            return
        e2 = 2 * err
        if e2 >= -dr:
            err -= dr
            c += sc
        if e2 <= dc:
            err += dc
            r += sr


def _clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


class StoredMap(BaseModel):
    label: str
    grid: OccupancyGrid


class MapStore(Protocol):
    """Загрузка/сохранение карт по семантическим меткам (например 'greenhouse_main')."""

    def save(self, *, label: str, grid: OccupancyGrid) -> None: ...

    def load(self, *, label: str) -> StoredMap | None: ...

    def list_labels(self) -> list[str]: ...
