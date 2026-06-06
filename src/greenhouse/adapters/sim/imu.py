"""SimImu — реализация ImuSource: гироскоп/акселерометр из истинного состояния симулятора.

Гироскоп отдаёт ИСТИННУЮ угловую скорость робота (= поданная команда поворота, которую
движок применяет точно), поэтому, в отличие от одометрии с проскальзыванием колёс, курс по
IMU не «плывёт». Это и используется для прогноза курса в `ImuFusedLocalizer`.
"""

from __future__ import annotations

from greenhouse.adapters.sim.clock import SimClock
from greenhouse.adapters.sim.state import SimState
from greenhouse.sensing.interfaces import ImuSample


class SimImu:
    """Реализует `greenhouse.sensing.ImuSource`."""

    def __init__(self, *, state: SimState, clock: SimClock) -> None:
        self._state = state
        self._clock = clock

    def read_imu(self) -> ImuSample:
        return ImuSample(
            yaw_rate_rad_s=self._state.last_cmd.angular_z_rad_s,
            linear_accel_x_m_s2=0.0,
            stamp_s=self._clock.now_s(),
        )
