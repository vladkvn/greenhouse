"""Фабрики готовых миров для демо и тестов."""

from __future__ import annotations

import math

from greenhouse.adapters.sim.world import PolygonWorld, Segment
from greenhouse.domain.geometry import Point2D
from greenhouse.domain.grid import MapMeta


def _rect(x0: float, y0: float, x1: float, y1: float) -> tuple[Segment, ...]:
    p = [Point2D(x_m=x0, y_m=y0), Point2D(x_m=x1, y_m=y0),
         Point2D(x_m=x1, y_m=y1), Point2D(x_m=x0, y_m=y1)]
    return tuple(Segment(a=p[i], b=p[(i + 1) % 4]) for i in range(4))


def empty_room(width_m: float = 10.0, height_m: float = 6.0) -> PolygonWorld:
    """Пустая прямоугольная комната — для базовых тестов рейкаста и движения."""
    return PolygonWorld(
        walls=_rect(0.0, 0.0, width_m, height_m),
        bounds_min=Point2D(x_m=0.0, y_m=0.0),
        bounds_max=Point2D(x_m=width_m, y_m=height_m),
    )


def greenhouse_rows_world(
    *, rows: int = 3, row_length_m: float = 8.0, aisle_m: float = 2.0, bed_m: float = 1.0
) -> PolygonWorld:
    """Теплица: параллельные ряды грядок вдоль X с проходами между ними.

    Грядки — прямоугольные препятствия; робот ездит по проходам вдоль +X.
    """
    walls: list[Segment] = []
    height_m = rows * bed_m + (rows + 1) * aisle_m
    walls.extend(empty_room(row_length_m, height_m).walls)

    for r in range(rows):
        y0 = aisle_m + r * (bed_m + aisle_m)
        y1 = y0 + bed_m
        # грядка как препятствие, оставляя поля по краям для разворота
        walls.extend(_rect(1.0, y0, row_length_m - 1.0, y1))

    return PolygonWorld(
        walls=tuple(walls),
        bounds_min=Point2D(x_m=0.0, y_m=0.0),
        bounds_max=Point2D(x_m=row_length_m, y_m=height_m),
    )


def grid_meta_for_world(
    world: PolygonWorld, *, resolution_m: float = 0.1, padding_m: float = 0.5
) -> MapMeta:
    """`MapMeta`, покрывающая габариты мира с запасом `padding_m` по краям.

    Удобно для построения карты в симуляции: размер сетки и привязка координат
    выводятся прямо из `bounds` мира, без ручного подбора.
    """
    width_m = world.bounds_max.x_m - world.bounds_min.x_m + 2.0 * padding_m
    height_m = world.bounds_max.y_m - world.bounds_min.y_m + 2.0 * padding_m
    return MapMeta(
        resolution_m=resolution_m,
        width_px=int(math.ceil(width_m / resolution_m)),
        height_px=int(math.ceil(height_m / resolution_m)),
        origin=Point2D(
            x_m=world.bounds_min.x_m - padding_m,
            y_m=world.bounds_min.y_m - padding_m,
        ),
    )
