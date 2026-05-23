"""Commanded twist applied on the next simulation sub-step."""

from __future__ import annotations

from fleet_contracts.geometry import Twist2D

from fleet_sim.state import SimState


class SimMotionController:
    def __init__(self, state: SimState) -> None:
        self._state = state

    def send_twist(self, twist: Twist2D, *, watchdog_deadline_s: float) -> None:
        _ = watchdog_deadline_s
        self._state.cmd_twist = twist

    def halt_immediate(self, *, reason_code: str) -> None:
        _ = reason_code
        self._state.cmd_twist = Twist2D(linear_x_m_s=0.0, angular_z_rad_s=0.0)
