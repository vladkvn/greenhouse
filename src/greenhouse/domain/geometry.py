"""Планарные кинематические примитивы, общие для всех слоёв.

Чистый домен: без ROS, без numpy в публичном API, без побочных эффектов.
"""

from __future__ import annotations

import math

from pydantic import BaseModel, Field


class Point2D(BaseModel, frozen=True):
    """Точка на плоскости карты (метры)."""

    x_m: float
    y_m: float

    def distance_to(self, other: Point2D) -> float:
        return math.hypot(self.x_m - other.x_m, self.y_m - other.y_m)


class Pose2D(BaseModel, frozen=True):
    """Поза на плоскости: положение + ориентация (yaw)."""

    x_m: float = Field(description="Координата X (метры).")
    y_m: float = Field(description="Координата Y (метры).")
    theta_rad: float = Field(description="Курс (yaw) вокруг оси Z (радианы).")

    @property
    def point(self) -> Point2D:
        return Point2D(x_m=self.x_m, y_m=self.y_m)


class Twist2D(BaseModel, frozen=True):
    """Команда скорости для дифференциального привода."""

    linear_x_m_s: float = Field(description="Линейная скорость вперёд (м/с).")
    angular_z_rad_s: float = Field(description="Угловая скорость (рад/с).")

    @classmethod
    def stop(cls) -> Twist2D:
        return cls(linear_x_m_s=0.0, angular_z_rad_s=0.0)
