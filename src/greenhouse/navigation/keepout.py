"""Закрытые зоны: оператор помечает область как запрещённую/медленную/предпочитаемую.

Зоны — полигоны в координатах карты. Реестр умеет растеризовать их в слой занятости,
который объединяется с построенной картой перед планированием.
"""

from __future__ import annotations

from enum import StrEnum
from typing import Protocol

from pydantic import BaseModel, Field

from greenhouse.domain.geometry import Point2D
from greenhouse.domain.grid import CellState, MapMeta, OccupancyGrid
from greenhouse.domain.identifiers import ZoneId

# Вклад зон в слой стоимости планирования (мультипликатор шага = 1 + cost).
_SLOW_COST = 4.0
_PREFERRED_COST = -0.3


class ZoneKind(StrEnum):
    KEEPOUT = "keepout"      # полностью непроезжая
    SLOW = "slow"            # проезжая, но с ограничением скорости
    PREFERRED = "preferred"  # поощряемая (снижает стоимость пути)


class Zone(BaseModel, frozen=True):
    zone_id: ZoneId
    kind: ZoneKind
    polygon: tuple[Point2D, ...] = Field(min_length=3, description="Вершины в координатах карты.")


class KeepoutRegistry(Protocol):
    """Хранит зоны и применяет их к карте.

    Редактируется во время работы: add_zone/remove_zone вызывает оператор;
    планировщик перечитывает результат apply_to при следующем перепланировании.
    """

    def add_zone(self, *, zone: Zone) -> None: ...

    def remove_zone(self, *, zone_id: ZoneId) -> None: ...

    def zones(self) -> list[Zone]: ...

    def apply_to(self, *, grid: OccupancyGrid) -> OccupancyGrid:
        """Вернуть карту с растеризованными KEEPOUT-зонами как занятыми ячейками."""
        ...

    def cost_layer(self, *, meta: MapMeta) -> list[float]:
        """Слой стоимости (SLOW/PREFERRED) для взвешенного планирования; row-major."""
        ...


class InMemoryKeepoutRegistry:
    """Реализует `KeepoutRegistry` в памяти. Редактируется на лету (add/remove зоны)."""

    def __init__(self) -> None:
        self._zones: dict[ZoneId, Zone] = {}

    def add_zone(self, *, zone: Zone) -> None:
        self._zones[zone.zone_id] = zone

    def remove_zone(self, *, zone_id: ZoneId) -> None:
        self._zones.pop(zone_id, None)

    def zones(self) -> list[Zone]:
        return list(self._zones.values())

    def apply_to(self, *, grid: OccupancyGrid) -> OccupancyGrid:
        meta = grid.meta
        keepouts = [z for z in self._zones.values() if z.kind is ZoneKind.KEEPOUT]
        if not keepouts:
            return grid
        cells = list(grid.cells)
        for row in range(meta.height_px):
            for col in range(meta.width_px):
                center = meta.cell_to_world(row, col)
                if any(_point_in_polygon(center, z.polygon) for z in keepouts):
                    cells[row * meta.width_px + col] = CellState.OCCUPIED
        return OccupancyGrid(meta=meta, cells=cells)

    def cost_layer(self, *, meta: MapMeta) -> list[float]:
        graded = [z for z in self._zones.values() if z.kind is not ZoneKind.KEEPOUT]
        layer = [0.0] * (meta.width_px * meta.height_px)
        if not graded:
            return layer
        for row in range(meta.height_px):
            for col in range(meta.width_px):
                center = meta.cell_to_world(row, col)
                cost = 0.0
                for z in graded:
                    if _point_in_polygon(center, z.polygon):
                        cost += _SLOW_COST if z.kind is ZoneKind.SLOW else _PREFERRED_COST
                layer[row * meta.width_px + col] = cost
        return layer


def _point_in_polygon(p: Point2D, polygon: tuple[Point2D, ...]) -> bool:
    """Тест «точка в полигоне» методом трассировки луча."""
    inside = False
    n = len(polygon)
    j = n - 1
    for i in range(n):
        xi, yi = polygon[i].x_m, polygon[i].y_m
        xj, yj = polygon[j].x_m, polygon[j].y_m
        if (yi > p.y_m) != (yj > p.y_m):
            x_cross = (xj - xi) * (p.y_m - yi) / (yj - yi) + xi
            if p.x_m < x_cross:
                inside = not inside
        j = i
    return inside
