"""Time stepping for pose integration with simple collision rule."""

from __future__ import annotations

from fleet_sim.footprint import footprint_circle_navigable
from fleet_sim.physics import integrate_twist
from fleet_sim.state import SimState
from fleet_sim.world import PolygonWorld


def simulation_step(world: PolygonWorld, state: SimState, dt_s: float) -> None:
    """Apply commanded twist; reject if inscribed disk hits room-boundary strokes."""

    proposed = integrate_twist(state.robot_pose, state.cmd_twist, dt_s)
    if footprint_circle_navigable(world, proposed.x_m, proposed.y_m, state.robot_inscribed_radius_m):
        state.robot_pose = proposed
    state.sim_time_s += dt_s
