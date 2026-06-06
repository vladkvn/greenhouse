"""Построение карты: сессия SLAM и хранилище карт.

Оркестрация владеет жизненным циклом сессии (start/stop). Конкретный алгоритм
(evidence-сетка по лидару сейчас, slam_toolbox через адаптер позже) скрыт за Protocol.
"""

from __future__ import annotations

from typing import Protocol

from pydantic import BaseModel

from greenhouse.domain.grid import OccupancyGrid
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


class StoredMap(BaseModel):
    label: str
    grid: OccupancyGrid


class MapStore(Protocol):
    """Загрузка/сохранение карт по семантическим меткам (например 'greenhouse_main')."""

    def save(self, *, label: str, grid: OccupancyGrid) -> None: ...

    def load(self, *, label: str) -> StoredMap | None: ...

    def list_labels(self) -> list[str]: ...
