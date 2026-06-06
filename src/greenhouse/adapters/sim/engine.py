"""SimEngine — физический шаг симуляции: интеграция diff-drive + расход батареи.

Коллизия: если шаг привёл бы центр робота ближе к стене, чем вписанный радиус,
движение откатывается (робот упирается). Это та же модель занятости, что у лидара.
"""

from __future__ import annotations

import math

from greenhouse.adapters.sim.clock import SimClock
from greenhouse.adapters.sim.state import SimState
from greenhouse.adapters.sim.world import PolygonWorld
from greenhouse.domain.geometry import Point2D


class SimEngine:
    def __init__(
        self,
        *,
        world: PolygonWorld,
        state: SimState,
        clock: SimClock,
        battery_drain_per_s: float = 0.0005,
    ) -> None:
        self._world = world
        self._state = state
        self._clock = clock
        self._drain = battery_drain_per_s

    def step(self, *, dt_s: float) -> None:
        s = self._state
        v = s.last_cmd.linear_x_m_s
        w = s.last_cmd.angular_z_rad_s

        new_theta = s.theta_rad + w * dt_s
        new_x = s.x_m + v * math.cos(new_theta) * dt_s
        new_y = s.y_m + v * math.sin(new_theta) * dt_s

        # Откат при столкновении: центр не ближе вписанного радиуса к стене.
        candidate = Point2D(x_m=new_x, y_m=new_y)
        if self._world.min_clearance(point=candidate) >= s.inscribed_radius_m:
            s.x_m, s.y_m = new_x, new_y
        s.theta_rad = new_theta  # поворот на месте всегда допустим

        s.battery_frac = max(0.0, s.battery_frac - self._drain * dt_s)
        self._clock.advance(dt_s=dt_s)
