"""Слой управления: единственный слой, который физически двигает робота.

Принимает желаемую скорость и доводит её до привода (или до симулятора).
Бизнес-логики здесь нет — только исполнение команды с учётом ограничений.
"""

from __future__ import annotations

from typing import Protocol

from pydantic import BaseModel, Field

from greenhouse.domain.geometry import Twist2D


class MotionLimits(BaseModel, frozen=True):
    """Физические ограничения платформы."""

    max_linear_m_s: float = Field(gt=0.0)
    max_angular_rad_s: float = Field(gt=0.0)
    max_linear_accel_m_s2: float = Field(gt=0.0)


class MotionController(Protocol):
    """Доводит команду скорости до привода.

    Реализации: SimMotion (интеграция в состояние симулятора), Ros2Motion
    (публикация /cmd_vel), драйвер реального привода.
    """

    @property
    def limits(self) -> MotionLimits: ...

    def command(self, *, twist: Twist2D) -> None:
        """Задать желаемую скорость. Контроллер обязан сам обрезать её по limits."""
        ...

    def stop(self) -> None:
        """Немедленная безопасная остановка (нулевая скорость)."""
        ...
