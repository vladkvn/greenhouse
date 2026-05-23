"""Sensor abstraction contracts — domain-facing models."""

from __future__ import annotations

from typing import Protocol

from pydantic import BaseModel, Field


class LaserScan(BaseModel):
    """Reduced 2D range scan usable without importing ROS."""

    frame_id: str = Field(description="TF frame referenced by adapters.")
    stamp_unix_s: float
    angle_min_rad: float = Field(description="Starting angle relative to scanner X.")
    angle_increment_rad: float
    range_min_m: float = Field(gt=0.0)
    range_max_m: float = Field(gt=0.0)
    ranges_m: list[float] = Field(description="Distance samples in meter.")


class CameraFrame(BaseModel):
    """Reference to buffered image pixels — adapters may mmap external storage."""

    frame_id: str
    stamp_unix_s: float
    width_px: int = Field(gt=0)
    height_px: int = Field(gt=0)
    pixel_format: str = Field(
        pattern=r"^(rgba8|bgr8|mono8)$",
        description="Hint for consumers; adapters document actual layout.",
    )
    opaque_handle: str = Field(
        description="Implementation-specific stable handle (shm id, mmap token, ROS pointer id)."
    )


class ImuSample(BaseModel):
    frame_id: str
    stamp_unix_s: float
    angular_velocity_rad_s_xy: tuple[float, float]
    acceleration_m_s2_xy: tuple[float, float]


class LidarSource(Protocol):
    """Protocol for planar lidars."""

    def read_scan(self) -> LaserScan: ...

    def is_available(self) -> bool: ...


class CameraSource(Protocol):
    """Protocol for monocular perception inputs."""

    def acquire_frame(self) -> CameraFrame: ...

    def is_available(self) -> bool: ...


class ImuSource(Protocol):
    """XY IMU excerpt for fused odometry stubs."""

    def read_sample(self) -> ImuSample: ...

    def is_available(self) -> bool: ...