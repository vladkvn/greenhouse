"""Motion output abstraction — shields Nav2/cmd_vel quirks."""

from __future__ import annotations

from typing import Protocol

from fleet_contracts.geometry import Twist2D


class MotionController(Protocol):
    """Bridges commanded twists to actuator backends."""

    def send_twist(self, twist: Twist2D, *, watchdog_deadline_s: float) -> None: ...

    def halt_immediate(self, *, reason_code: str) -> None: ...