"""Движущаяся цель (человек) в симуляции и её детектор.

`SimPerson` — истинное положение цели (мутабельно, как и состояние робота). `SimPersonDetector`
реализует `TargetDetector`: возвращает наблюдение, если цель в пределах дальности, в поле
зрения и не заслонена стеной (проверка прямой видимости рейкастом), иначе None.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from greenhouse.adapters.sim.clock import SimClock
from greenhouse.adapters.sim.state import SimState
from greenhouse.adapters.sim.world import PolygonWorld
from greenhouse.domain.geometry import Point2D
from greenhouse.sensing.interfaces import TargetObservation


@dataclass
class SimPerson:
    """Истинное положение цели. Метод `step` двигает её с заданной скоростью."""

    x_m: float
    y_m: float
    vx_m_s: float = 0.0
    vy_m_s: float = 0.0

    def step(self, *, dt_s: float) -> None:
        self.x_m += self.vx_m_s * dt_s
        self.y_m += self.vy_m_s * dt_s

    def point(self) -> Point2D:
        return Point2D(x_m=self.x_m, y_m=self.y_m)


class SimPersonDetector:
    """Реализует `greenhouse.sensing.TargetDetector` поверх истинного положения цели."""

    def __init__(
        self,
        *,
        world: PolygonWorld,
        robot_state: SimState,
        person: SimPerson,
        clock: SimClock,
        max_range_m: float = 5.0,
        fov_rad: float = math.radians(360.0),
    ) -> None:
        self._world = world
        self._state = robot_state
        self._person = person
        self._clock = clock
        self._max_range = max_range_m
        self._fov = fov_rad

    def detect(self) -> TargetObservation | None:
        s = self._state
        dx = self._person.x_m - s.x_m
        dy = self._person.y_m - s.y_m
        rng = math.hypot(dx, dy)
        if rng < 1e-6 or rng > self._max_range:
            return None
        bearing = _wrap(math.atan2(dy, dx) - s.theta_rad)
        if abs(bearing) > self._fov / 2.0:
            return None
        # Прямая видимость: стена ближе цели вдоль луча — цель заслонена.
        hit = self._world.raycast(
            origin=Point2D(x_m=s.x_m, y_m=s.y_m), angle_rad=math.atan2(dy, dx), max_range_m=rng
        )
        if math.isfinite(hit) and hit < rng - 1e-6:
            return None
        return TargetObservation(range_m=rng, bearing_rad=bearing, stamp_s=self._clock.now_s())


def _wrap(angle_rad: float) -> float:
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))
