"""Mutable simulation snapshot shared by mocks and stepping logic."""

from __future__ import annotations

from fleet_contracts.geometry import Pose2D, Twist2D


class SimState:
    """Single-thread toy state; robotics stack would split concerns across nodes."""

    def __init__(
        self,
        *,
        robot_pose: Pose2D,
        target_xy_m: tuple[float, float],
        sim_time_s: float = 0.0,
    ) -> None:
        self.robot_pose = robot_pose
        self.cmd_twist = Twist2D(linear_x_m_s=0.0, angular_z_rad_s=0.0)
        self.sim_time_s = sim_time_s
        self.target_xy_m = target_xy_m
