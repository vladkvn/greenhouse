"""Следование за целью: режим Following.

`PersonFollower` превращает наблюдение цели в команду скорости — доворачивает на цель и
держит безопасную дистанцию (standoff). При потере цели немедленно командует стоп и через
несколько тиков сигнализирует о потере (безопасное поведение по умолчанию).
"""

from __future__ import annotations

from greenhouse.domain.geometry import Twist2D
from greenhouse.sensing.interfaces import TargetObservation


class PersonFollower:
    """Контроллер следования за целью с удержанием дистанции и безопасной остановкой."""

    def __init__(
        self,
        *,
        standoff_m: float = 0.8,
        max_linear_m_s: float = 0.6,
        max_angular_rad_s: float = 1.5,
        turn_in_place_rad: float = 0.5,
        lost_grace_ticks: int = 5,
        deadband_m: float = 0.1,
    ) -> None:
        self._standoff = standoff_m
        self._v_max = max_linear_m_s
        self._w_max = max_angular_rad_s
        self._turn = turn_in_place_rad
        self._lost_grace = lost_grace_ticks
        self._deadband = deadband_m
        self._lost_ticks = 0

    def update(self, *, observation: TargetObservation | None) -> Twist2D:
        if observation is None:
            self._lost_ticks += 1
            return Twist2D.stop()  # цель не видна — безопасная остановка
        self._lost_ticks = 0

        w = _clamp(2.0 * observation.bearing_rad, -self._w_max, self._w_max)
        if abs(observation.bearing_rad) > self._turn:
            return Twist2D(linear_x_m_s=0.0, angular_z_rad_s=w)  # сначала довернуть на цель

        gap = observation.range_m - self._standoff
        if abs(gap) <= self._deadband:
            v = 0.0  # на нужной дистанции — держим позицию
        else:
            v = _clamp(1.5 * gap, -self._v_max, self._v_max)  # дальше — догнать, ближе — отступить
        return Twist2D(linear_x_m_s=v, angular_z_rad_s=w)

    @property
    def target_lost(self) -> bool:
        """Цель считается потерянной, если её не видно дольше периода ожидания."""
        return self._lost_ticks > self._lost_grace


def _clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))
