"""Mutable simulation snapshot shared by mocks and stepping logic."""

from __future__ import annotations

from fleet_contracts.geometry import Pose2D, Twist2D


class SimState:
    """Single-thread toy snapshot: pose + commands + inscribed circular footprint radius."""

    def __init__(
        self,
        *,
        robot_pose: Pose2D,
        target_xy_m: tuple[float, float],
        sim_time_s: float = 0.0,
        robot_inscribed_radius_m: float = 0.20,
    ) -> None:
        self.robot_pose = robot_pose
        self.cmd_twist = Twist2D(linear_x_m_s=0.0, angular_z_rad_s=0.0)
        self.sim_time_s = sim_time_s
        self.robot_inscribed_radius_m = float(robot_inscribed_radius_m)
        self.target_x_m = float(target_xy_m[0])
        self.target_y_m = float(target_xy_m[1])

    @property
    def target_xy_m(self) -> tuple[float, float]:
        return (self.target_x_m, self.target_y_m)

    def set_target_xy(self, x: float, y: float) -> None:
        """Update follow target coordinates (typically after UI clamp into free space)."""

        self.target_x_m = float(x)
        self.target_y_m = float(y)
