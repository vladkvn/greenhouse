"""Ground-truth pose from the simulator (no SLAM)."""

from __future__ import annotations

from fleet_contracts.geometry import Pose2D
from fleet_contracts.localization import (
    LocalizationEstimate,
    LocalizationSnapshot,
    LocalizationStatus,
)

from fleet_sim.state import SimState


class SimTruthLocalizer:
    def __init__(self, state: SimState) -> None:
        self._state = state

    def snapshot(self) -> LocalizationSnapshot:
        est = LocalizationEstimate(
            pose_map=self._state.robot_pose,
            position_cov_xx=1e-9,
        )
        return LocalizationSnapshot(status=LocalizationStatus.OK, estimate=est)

    def request_reset_to(self, seed: Pose2D) -> LocalizationSnapshot:
        self._state.robot_pose = seed
        return self.snapshot()
