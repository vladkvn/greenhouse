"""Time stepping for pose integration with simple collision rule."""

from __future__ import annotations

from fleet_sim.physics import integrate_twist
from fleet_sim.state import SimState
from fleet_sim.world import PolygonWorld


def simulation_step(world: PolygonWorld, state: SimState, dt_s: float) -> None:
    """Apply commanded twist; drop move if the new configuration leaves free space."""

    proposed = integrate_twist(state.robot_pose, state.cmd_twist, dt_s)
    if world.contains_point(proposed.x_m, proposed.y_m):
        state.robot_pose = proposed
    state.sim_time_s += dt_s
