"""Synthetic lidar sourced from analytic walls."""

from __future__ import annotations

from fleet_contracts.perception import LaserScan

from fleet_sim.lidar_raycast import synth_lidar_scan
from fleet_sim.state import SimState
from fleet_sim.world import PolygonWorld, world_bounding_extent_diagonal_m


class SimLidarSource:
    def __init__(
        self,
        world: PolygonWorld,
        state: SimState,
        *,
        synth_range_max_m: float | None = None,
        synth_range_floor_m: float = 22.0,
        synth_diag_margin_m: float = 12.0,
    ) -> None:
        self._world = world
        self._state = state
        if synth_range_max_m is None:
            span = world_bounding_extent_diagonal_m(world)
            self._synth_range_max_m = max(synth_range_floor_m, span + synth_diag_margin_m)
        else:
            self._synth_range_max_m = float(synth_range_max_m)

    def read_scan(self) -> LaserScan:
        pose = self._state.robot_pose
        scan = synth_lidar_scan(
            self._world,
            ox_m=pose.x_m,
            oy_m=pose.y_m,
            heading_rad=pose.theta_rad,
            range_max_m=self._synth_range_max_m,
        )
        return scan.model_copy(update={"stamp_unix_s": self._state.sim_time_s})

    def is_available(self) -> bool:
        return True
