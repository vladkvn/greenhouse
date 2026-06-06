"""Слой восприятия: интерфейсы датчиков и их наблюдения.

Только чтение «сырых» данных, без интерпретации. Реализации: SimLidar (сейчас),
Ros2Lidar / драйвер (позже) — за одним и тем же Protocol.
"""

from __future__ import annotations

from typing import Protocol

from pydantic import BaseModel, Field

from greenhouse.domain.geometry import Pose2D, Twist2D


class LidarScan(BaseModel, frozen=True):
    """Один оборот 2D-лидара. `ranges_m` — дистанции по равномерным углам."""

    angle_min_rad: float
    angle_increment_rad: float
    range_max_m: float = Field(gt=0.0)
    ranges_m: tuple[float, ...] = Field(description="inf/NaN = нет возврата.")
    stamp_s: float


class CameraFrame(BaseModel, frozen=True):
    """Кадр камеры. Пиксели хранятся вне доменной модели (ссылка/буфер у адаптера),
    здесь — метаданные, достаточные для детектора человека."""

    width_px: int = Field(gt=0)
    height_px: int = Field(gt=0)
    stamp_s: float
    frame_ref: str = Field(description="Идентификатор/ключ буфера кадра у адаптера.")


class Odometry(BaseModel, frozen=True):
    """Оценка движения по колёсам/энкодерам относительно старта."""

    pose: Pose2D
    velocity: Twist2D
    stamp_s: float


class ImuSample(BaseModel, frozen=True):
    yaw_rate_rad_s: float
    linear_accel_x_m_s2: float
    stamp_s: float


class LidarSource(Protocol):
    def read_scan(self) -> LidarScan: ...


class CameraSource(Protocol):
    def read_frame(self) -> CameraFrame: ...


class OdometrySource(Protocol):
    def read_odometry(self) -> Odometry: ...


class ImuSource(Protocol):
    def read_imu(self) -> ImuSample: ...
