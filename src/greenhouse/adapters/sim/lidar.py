"""SimLidar — реализация LidarSource: рейкаст по стенам мира из текущей позы."""

from __future__ import annotations

import math

from greenhouse.adapters.sim.clock import SimClock
from greenhouse.adapters.sim.state import SimState
from greenhouse.adapters.sim.world import PolygonWorld
from greenhouse.domain.geometry import Point2D
from greenhouse.sensing.interfaces import LidarScan


class SimLidar:
    """Реализует `greenhouse.sensing.LidarSource`."""

    def __init__(
        self,
        *,
        world: PolygonWorld,
        state: SimState,
        clock: SimClock,
        num_beams: int = 72,
        range_max_m: float = 8.0,
    ) -> None:
        self._world = world
        self._state = state
        self._clock = clock
        self._num_beams = num_beams
        self._range_max = range_max_m

    def read_scan(self) -> LidarScan:
        s = self._state
        origin = Point2D(x_m=s.x_m, y_m=s.y_m)
        increment = 2.0 * math.pi / self._num_beams
        ranges = tuple(
            self._world.raycast(
                origin=origin,
                angle_rad=s.theta_rad + i * increment,
                max_range_m=self._range_max,
            )
            for i in range(self._num_beams)
        )
        return LidarScan(
            angle_min_rad=s.theta_rad,
            angle_increment_rad=increment,
            range_max_m=self._range_max,
            ranges_m=ranges,
            stamp_s=self._clock.now_s(),
        )
