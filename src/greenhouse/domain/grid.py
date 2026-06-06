"""Сеточная карта занятости — общая модель для mapping, planning и keep-out.

Единая модель занятости важна: и планировщик, и проверка коллизий работают с одной
и той же сеткой, иначе план и реальное движение расходятся.
"""

from __future__ import annotations

from enum import IntEnum

from pydantic import BaseModel, Field

from greenhouse.domain.geometry import Point2D


class CellState(IntEnum):
    """Состояние ячейки сетки занятости."""

    UNKNOWN = -1
    FREE = 0
    OCCUPIED = 1


class MapMeta(BaseModel, frozen=True):
    """Метаданные сетки: разрешение и привязка к системе координат карты."""

    resolution_m: float = Field(gt=0.0, description="Размер ячейки в метрах.")
    width_px: int = Field(gt=0)
    height_px: int = Field(gt=0)
    origin: Point2D = Field(description="Координаты карты для ячейки (0, 0).")

    def world_to_cell(self, p: Point2D) -> tuple[int, int]:
        col = int((p.x_m - self.origin.x_m) / self.resolution_m)
        row = int((p.y_m - self.origin.y_m) / self.resolution_m)
        return row, col

    def cell_to_world(self, row: int, col: int) -> Point2D:
        return Point2D(
            x_m=self.origin.x_m + (col + 0.5) * self.resolution_m,
            y_m=self.origin.y_m + (row + 0.5) * self.resolution_m,
        )

    def in_bounds(self, row: int, col: int) -> bool:
        return 0 <= row < self.height_px and 0 <= col < self.width_px


class OccupancyGrid(BaseModel):
    """Сетка занятости. `cells` — row-major список длиной width*height.

    Реализации алгоритмов могут работать с numpy внутри, но на границе модулей
    карта представлена этим валидируемым типом.
    """

    meta: MapMeta
    cells: list[CellState] = Field(description="Row-major, длина = width_px*height_px.")

    def at(self, row: int, col: int) -> CellState:
        if not self.meta.in_bounds(row, col):
            return CellState.OCCUPIED  # за границей карты считаем непроезжим
        return self.cells[row * self.meta.width_px + col]
