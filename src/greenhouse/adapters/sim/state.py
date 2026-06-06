"""Изменяемое состояние робота в симуляции (истина мира)."""

from __future__ import annotations

from dataclasses import dataclass

from greenhouse.domain.geometry import Pose2D, Twist2D


@dataclass
class SimState:
    """Истинное состояние робота. Мутабельно — это «реальность» симулятора."""

    x_m: float = 0.0
    y_m: float = 0.0
    theta_rad: float = 0.0
    battery_frac: float = 1.0
    last_cmd: Twist2D = Twist2D(linear_x_m_s=0.0, angular_z_rad_s=0.0)
    inscribed_radius_m: float = 0.25

    def pose(self) -> Pose2D:
        return Pose2D(x_m=self.x_m, y_m=self.y_m, theta_rad=self.theta_rad)
