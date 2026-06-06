"""SimOdometry — реализация OdometrySource: поза и скорость из состояния.

С `yaw_slip > 0` моделируется проскальзывание колёс при повороте: одометрия недооценивает
поворот, и её курс «плывёт» относительно истинного. Положение остаётся истинным (фокус —
на дрейфе курса, который лечится слиянием с IMU-гироскопом, Инкремент 12)."""

from __future__ import annotations

import math

from greenhouse.adapters.sim.clock import SimClock
from greenhouse.adapters.sim.state import SimState
from greenhouse.domain.geometry import Pose2D
from greenhouse.sensing.interfaces import Odometry


class SimOdometry:
    """Реализует `greenhouse.sensing.OdometrySource`. `yaw_slip` ∈ [0, 1] — доля «потери» поворота."""

    def __init__(self, *, state: SimState, clock: SimClock, yaw_slip: float = 0.0) -> None:
        self._state = state
        self._clock = clock
        self._slip = yaw_slip
        self._last_true_theta = state.theta_rad
        self._odom_theta = state.theta_rad

    def read_odometry(self) -> Odometry:
        s = self._state
        # Накопить курс одометрии с недооценкой поворота (проскальзывание).
        d_true = math.atan2(
            math.sin(s.theta_rad - self._last_true_theta),
            math.cos(s.theta_rad - self._last_true_theta),
        )
        self._odom_theta += (1.0 - self._slip) * d_true
        self._last_true_theta = s.theta_rad
        pose = Pose2D(x_m=s.x_m, y_m=s.y_m, theta_rad=self._odom_theta)
        return Odometry(pose=pose, velocity=s.last_cmd, stamp_s=self._clock.now_s())
