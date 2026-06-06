"""SimTruthLocalizer — реализация Localizer, возвращающая истинную позу из симулятора.

Заглушка для ранних инкрементов: позволяет проверять навигацию до появления
настоящего scan-matching (Инкремент 5 в roadmap).
"""

from __future__ import annotations

from greenhouse.adapters.sim.clock import SimClock
from greenhouse.adapters.sim.state import SimState
from greenhouse.domain.geometry import Pose2D
from greenhouse.domain.grid import OccupancyGrid
from greenhouse.navigation.localization import PoseEstimate
from greenhouse.sensing.interfaces import LidarScan, Odometry


class SimTruthLocalizer:
    """Реализует `greenhouse.navigation.Localizer`."""

    def __init__(self, *, state: SimState, clock: SimClock) -> None:
        self._state = state
        self._clock = clock
        self._latest: PoseEstimate | None = None

    def set_map(self, *, grid: OccupancyGrid) -> None:
        return None  # истине карта не нужна

    def set_initial_pose(self, *, pose: Pose2D) -> None:
        return None  # позу диктует симулятор

    def update(self, *, scan: LidarScan, odometry: Odometry) -> PoseEstimate:
        self._latest = PoseEstimate(
            pose=self._state.pose(), confidence=1.0, stamp_s=self._clock.now_s()
        )
        return self._latest

    def latest(self) -> PoseEstimate | None:
        return self._latest
