"""Полигональный мир симуляции: стены как отрезки + рейкаст луча.

Это «истинная» геометрия среды. Лидар стреляет лучами по этим же стенам, по которым
проверяется столкновение робота — единая модель занятости для наблюдений и физики.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from greenhouse.domain.geometry import Point2D


@dataclass(frozen=True)
class Segment:
    """Отрезок стены в координатах карты."""

    a: Point2D
    b: Point2D


@dataclass(frozen=True)
class PolygonWorld:
    """Мир как набор отрезков-стен и габаритный прямоугольник (для ограничения целей)."""

    walls: tuple[Segment, ...]
    bounds_min: Point2D
    bounds_max: Point2D

    def raycast(self, *, origin: Point2D, angle_rad: float, max_range_m: float) -> float:
        """Дистанция до ближайшей стены вдоль луча; inf, если ближе max_range стен нет.

        Решается пересечение параметрического луча P = O + t*d (t >= 0) с каждым
        отрезком A + u*(B-A), 0 <= u <= 1. Возвращается минимальное t в метрах.
        """
        dx = math.cos(angle_rad)
        dy = math.sin(angle_rad)
        best = math.inf

        for seg in self.walls:
            ex = seg.b.x_m - seg.a.x_m
            ey = seg.b.y_m - seg.a.y_m
            denom = dx * ey - dy * ex
            if abs(denom) < 1e-12:
                continue  # луч параллелен стене
            wx = seg.a.x_m - origin.x_m
            wy = seg.a.y_m - origin.y_m
            t = (wx * ey - wy * ex) / denom   # вдоль луча
            u = (wx * dy - wy * dx) / denom   # вдоль отрезка
            if t >= 0.0 and 0.0 <= u <= 1.0 and t < best:
                best = t

        if best <= max_range_m:
            return best
        return math.inf

    def min_clearance(self, *, point: Point2D) -> float:
        """Минимальное расстояние от точки до ближайшей стены (для проверки коллизий)."""
        return min(_dist_point_segment(point, s) for s in self.walls)


def _dist_point_segment(p: Point2D, s: Segment) -> float:
    ex = s.b.x_m - s.a.x_m
    ey = s.b.y_m - s.a.y_m
    length_sq = ex * ex + ey * ey
    if length_sq < 1e-12:
        return p.distance_to(s.a)
    t = ((p.x_m - s.a.x_m) * ex + (p.y_m - s.a.y_m) * ey) / length_sq
    t = max(0.0, min(1.0, t))
    proj = Point2D(x_m=s.a.x_m + t * ex, y_m=s.a.y_m + t * ey)
    return p.distance_to(proj)
