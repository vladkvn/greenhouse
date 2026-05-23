"""Synthetic lidar sourced from analytic walls."""

from __future__ import annotations

from fleet_contracts.perception import LaserScan

from fleet_sim.lidar_raycast import synth_lidar_scan
from fleet_sim.state import SimState
from fleet_sim.world import PolygonWorld


class SimLidarSource:
    def __init__(self, world: PolygonWorld, state: SimState) -> None:
        self._world = world
        self._state = state

    def read_scan(self) -> LaserScan:
        pose = self._state.robot_pose
        scan = synth_lidar_scan(
            self._world,
            ox_m=pose.x_m,
            oy_m=pose.y_m,
            heading_rad=pose.theta_rad,
        )
        return scan.model_copy(update={"stamp_unix_s": self._state.sim_time_s})

    def is_available(self) -> bool:
        return True
