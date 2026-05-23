"""Proportional guidance toward a map point."""

from __future__ import annotations

import math

from fleet_contracts.geometry import Pose2D, Twist2D

from fleet_sim.physics import wrap_pi


def twist_follow_point(
    robot: Pose2D,
    target_x_m: float,
    target_y_m: float,
    *,
    max_linear_m_s: float,
    gain_omega: float,
    gain_linear: float,
) -> Twist2D:
    """Turn toward goal and translate when roughly aligned."""

    dx = target_x_m - robot.x_m
    dy = target_y_m - robot.y_m
    desired_heading = math.atan2(dy, dx)
    heading_err = wrap_pi(desired_heading - robot.theta_rad)
    omega = gain_omega * heading_err

    lateral_ok = abs(heading_err) < math.radians(18.0)
    linear = gain_linear * math.hypot(dx, dy) if lateral_ok else 0.0

    capped_linear = max(-max_linear_m_s, min(max_linear_m_s, linear))
    return Twist2D(linear_x_m_s=capped_linear, angular_z_rad_s=omega)
