"""Геометрия ровера из config/robot.yaml — общий модуль восприятия (follow + глубина).

Один источник (robot.yaml) → интринсики камеры (из HFOV), перевод пикселя в пеленг (follow)
и проекция пикселя на плоскость пола (дальность до низких препятствий: камера жёстко
закреплена, её поза известна из URDF/TF → любой пиксель пола однозначно даёт точку на полу).

Внешние параметры камеры (высота/наклон) НЕ дублируем — берём в рантайме из TF
(base_footprint→camera_optical_frame), которое robot_state_publisher строит из того же
robot.yaml. Геометрия остаётся в ОДНОМ месте.
"""
from __future__ import annotations

import math
import os

import yaml


def load_robot_config(path: str | None = None) -> dict:
    """Загрузить robot.yaml (по умолчанию — из share пакета rover_bringup)."""
    if path is None:
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(get_package_share_directory("rover_bringup"), "config", "robot.yaml")
    with open(path) as f:
        return yaml.safe_load(f)


def camera_intrinsics(
    cfg: dict, width: int | None = None, height: int | None = None
) -> tuple[float, float, float, float, int, int]:
    """(fx, fy, cx, cy, w, h) из HFOV. Если кадр публикуется в ДРУГОМ разрешении — передать
    width/height: fx масштабируется с шириной, а HFOV от масштаба не зависит."""
    cam = cfg["sensors"]["camera"]
    w = int(width) if width else int(cam["width"])
    h = int(height) if height else int(cam["height"])
    hfov = math.radians(float(cam["hfov_deg"]))
    fx = (w / 2.0) / math.tan(hfov / 2.0)
    fy = fx                       # квадратный пиксель (нет вертикальной калибровки)
    return fx, fy, w / 2.0, h / 2.0, w, h


def pixel_to_bearing(u: float, fx: float, cx: float) -> float:
    """Горизонтальный пеленг (рад, + ВЛЕВО как REP-103 +y) для пикселя-колонки u.

    Основа follow: bbox center-x → направление на цель. Корректная пинхол-модель вместо
    линейной (cx/w-0.5)·hfov из follow_me. fx,cx — из camera_intrinsics или CameraInfo.
    """
    return math.atan2(cx - u, fx)


def _quat_to_matrix(q: tuple[float, float, float, float]) -> list[list[float]]:
    """(x,y,z,w) → 3×3 матрица поворота."""
    x, y, z, w = q
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]


def ground_project(
    u: float,
    v: float,
    intr: tuple[float, float, float, float, int, int],
    cam_translation: tuple[float, float, float],
    cam_quat: tuple[float, float, float, float],
    *,
    floor_z: float = 0.0,
    max_range: float = 12.0,
) -> tuple[float, float] | None:
    """Пиксель (u,v) → точка (x,y) на полу в кадре base_footprint, либо None.

    cam_translation/cam_quat — TF base_footprint→camera_optical_frame (из URDF = из robot.yaml).
    Луч в оптическом кадре → поворот в base → пересечение с полом z=floor_z. Для дальности до
    низкого препятствия брать v = НИЖНЮЮ границу объекта (контакт с полом).
    """
    fx, fy, cx, cy, _, _ = intr
    rx, ry, rz = (u - cx) / fx, (v - cy) / fy, 1.0     # луч в оптич. кадре (z вперёд)
    R = _quat_to_matrix(cam_quat)
    dx = R[0][0] * rx + R[0][1] * ry + R[0][2] * rz
    dy = R[1][0] * rx + R[1][1] * ry + R[1][2] * rz
    dz = R[2][0] * rx + R[2][1] * ry + R[2][2] * rz
    ox, oy, oz = cam_translation
    if dz > -1e-6:                        # луч не идёт вниз → пола не достигает
        return None
    s = (floor_z - oz) / dz
    if s <= 0.0:
        return None
    x, y = ox + s * dx, oy + s * dy
    if math.hypot(x, y) > max_range:
        return None
    return x, y
