"""SimOdometry — реализация OdometrySource: истинная поза и скорость из состояния."""

from __future__ import annotations

from greenhouse.adapters.sim.clock import SimClock
from greenhouse.adapters.sim.state import SimState
from greenhouse.sensing.interfaces import Odometry


class SimOdometry:
    """Реализует `greenhouse.sensing.OdometrySource` (идеальная одометрия)."""

    def __init__(self, *, state: SimState, clock: SimClock) -> None:
        self._state = state
        self._clock = clock

    def read_odometry(self) -> Odometry:
        return Odometry(
            pose=self._state.pose(),
            velocity=self._state.last_cmd,
            stamp_s=self._clock.now_s(),
        )
