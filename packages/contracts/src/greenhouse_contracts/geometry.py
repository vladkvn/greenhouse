"""Planar kinematic primitives used across navigation and telemetry."""

from __future__ import annotations

from pydantic import BaseModel, Field


class Pose2D(BaseModel):
    """Pose in map or odometry plane (implicit frame chosen by adapters)."""

    x_m: float = Field(description="Along X axis (meters).")
    y_m: float = Field(description="Along Y axis (meters).")
    theta_rad: float = Field(description="Yaw around Z (radians).")


class Twist2D(BaseModel):
    """Differential-drive style command."""

    linear_x_m_s: float = Field(description="Forward linear velocity (m/s).")
    angular_z_rad_s: float = Field(description="Yaw rate (rad/s).")
