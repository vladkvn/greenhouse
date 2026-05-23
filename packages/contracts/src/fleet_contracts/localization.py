"""Pose estimates with localization health."""

from __future__ import annotations

from enum import StrEnum
from typing import Protocol

from pydantic import BaseModel

from fleet_contracts.geometry import Pose2D


class LocalizationStatus(StrEnum):
    OK = "ok"
    DEGRADED = "degraded"
    LOST = "lost"


class LocalizationEstimate(BaseModel):
    pose_map: Pose2D
    position_cov_xx: float


class LocalizationSnapshot(BaseModel):
    status: LocalizationStatus
    estimate: LocalizationEstimate | None


class Localizer(Protocol):
    """Wraps particle filter stack or future sensor fusion backends."""

    def snapshot(self) -> LocalizationSnapshot: ...

    def request_reset_to(self, seed: Pose2D) -> LocalizationSnapshot: ...
