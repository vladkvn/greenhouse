"""SimMotion — реализация MotionController: запоминает команду скорости с обрезкой
по физическим ограничениям. Интегрирует её в состояние SimEngine на шаге."""

from __future__ import annotations

from greenhouse.adapters.sim.state import SimState
from greenhouse.control.interfaces import MotionLimits
from greenhouse.domain.geometry import Twist2D


class SimMotion:
    """Реализует `greenhouse.control.MotionController`."""

    def __init__(self, *, state: SimState, limits: MotionLimits | None = None) -> None:
        self._state = state
        self._limits = limits or MotionLimits(
            max_linear_m_s=1.0, max_angular_rad_s=1.5, max_linear_accel_m_s2=1.0
        )

    @property
    def limits(self) -> MotionLimits:
        return self._limits

    def command(self, *, twist: Twist2D) -> None:
        v = _clamp(twist.linear_x_m_s, -self._limits.max_linear_m_s, self._limits.max_linear_m_s)
        w = _clamp(twist.angular_z_rad_s, -self._limits.max_angular_rad_s, self._limits.max_angular_rad_s)
        self._state.last_cmd = Twist2D(linear_x_m_s=v, angular_z_rad_s=w)

    def stop(self) -> None:
        self._state.last_cmd = Twist2D.stop()


def _clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))
