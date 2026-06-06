"""Тесты геометрии мира: рейкаст и проверка дистанции до стен."""

from __future__ import annotations

import math

from greenhouse.adapters.sim.worlds import empty_room
from greenhouse.domain.geometry import Point2D


def test_raycast_hits_known_wall() -> None:
    # Комната 10x6, луч из (1,3) вправо (+X) бьёт в правую стену x=10 → 9 м.
    world = empty_room(10.0, 6.0)
    d = world.raycast(origin=Point2D(x_m=1.0, y_m=3.0), angle_rad=0.0, max_range_m=20.0)
    assert math.isclose(d, 9.0, abs_tol=1e-6)


def test_raycast_up_hits_top_wall() -> None:
    world = empty_room(10.0, 6.0)
    d = world.raycast(origin=Point2D(x_m=2.0, y_m=1.0), angle_rad=math.pi / 2, max_range_m=20.0)
    assert math.isclose(d, 5.0, abs_tol=1e-6)


def test_raycast_beyond_range_is_inf() -> None:
    world = empty_room(10.0, 6.0)
    d = world.raycast(origin=Point2D(x_m=1.0, y_m=3.0), angle_rad=0.0, max_range_m=2.0)
    assert math.isinf(d)


def test_min_clearance_to_nearest_wall() -> None:
    world = empty_room(10.0, 6.0)
    # точка (1, 3): ближайшая стена — левая x=0 → 1 м.
    assert math.isclose(world.min_clearance(point=Point2D(x_m=1.0, y_m=3.0)), 1.0, abs_tol=1e-6)
