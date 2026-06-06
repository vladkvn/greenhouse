"""Закрытые зоны: оператор помечает область как запрещённую/медленную/предпочитаемую.

Зоны — полигоны в координатах карты. Реестр умеет растеризовать их в слой занятости,
который объединяется с построенной картой перед планированием.
"""

from __future__ import annotations

from enum import Enum
from typing import Protocol

from pydantic import BaseModel, Field

from greenhouse.domain.geometry import Point2D
from greenhouse.domain.grid import MapMeta, OccupancyGrid
from greenhouse.domain.identifiers import ZoneId


class ZoneKind(str, Enum):
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
