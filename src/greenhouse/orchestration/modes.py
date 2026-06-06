"""Режимы работы робота. Оркестратор — единственная точка их переключения."""

from __future__ import annotations

from enum import StrEnum


class RobotMode(StrEnum):
    IDLE = "idle"
    MAPPING = "mapping"
    NAVIGATING = "navigating"
    FOLLOWING = "following"
    CHARGING = "charging"
