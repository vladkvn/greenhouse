"""Unicycle kinematics."""

from __future__ import annotations

import math

from fleet_contracts.geometry import Pose2D, Twist2D


def wrap_pi(angle_rad: float) -> float:
    """Map angle to (-pi, pi]."""

    wrapped = (angle_rad + math.pi) % (2.0 * math.pi) - math.pi
    if wrapped <= -math.pi:
        return math.pi
    return wrapped


def integrate_twist(pose: Pose2D, twist: Twist2D, dt_s: float) -> Pose2D:
    """Euler integration in map frame; forward axis along robot heading."""

    theta = pose.theta_rad
    vx = twist.linear_x_m_s
    omega = twist.angular_z_rad_s
    x_new = pose.x_m + vx * math.cos(theta) * dt_s
    y_new = pose.y_m + vx * math.sin(theta) * dt_s
    theta_new = wrap_pi(theta + omega * dt_s)
    return Pose2D(x_m=x_new, y_m=y_new, theta_rad=theta_new)
