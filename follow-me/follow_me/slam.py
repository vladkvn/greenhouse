"""2D-SLAM по лидару (BreezySLAM): строит occupancy-карту и оценивает позу робота.

Вход: 360-бин скан лидара (метры, NaN=нет возврата) + курс с IMU (BNO085) как прайор
поворота. BreezySLAM (RMHC) уточняет позу scan-matching'ом. Одометрии колёс нет, поэтому
поворот берём с IMU (самое слабое место scan-matching), а смещение SLAM находит сам.

Опциональный импорт: если breezyslam не установлен, SlamMapper.available() == False и
follow_me работает без карты. На Jetson: pip install breezyslam (нужны build-tools под C-ext).

Система координат карты: квадрат map_size_pixels на map_size_meters. Мир (мм) с началом в
ЦЕНТРЕ карты. Перевод мир<->пиксель в world_to_pixel/pixel_to_world (используются и для
отрисовки робота, и для клика-цели — держим в одном месте, чтобы цель сходилась с картинкой).
"""

from __future__ import annotations

import logging
import math

import numpy as np

from .config import FollowMeConfig

log = logging.getLogger("follow_me.slam")


def _angle_diff_deg(a: float, b: float) -> float:
    """Кратчайшая разница углов a-b в градусах, в (-180, 180]."""
    d = (a - b + 180.0) % 360.0 - 180.0
    return d


class SlamMapper:
    def __init__(self, cfg: FollowMeConfig):
        self._cfg = cfg
        self._slam = None
        self._mapbytes = bytearray(cfg.map_size_pixels * cfg.map_size_pixels)
        self._pose_mm = (0.0, 0.0, 0.0)  # x, y, theta(deg)
        self._last_yaw: float | None = None
        try:
            from breezyslam.algorithms import RMHC_SLAM
            from breezyslam.sensors import Laser
            laser = Laser(cfg.slam_scan_size, cfg.slam_scan_rate_hz,
                          cfg.slam_detection_deg, cfg.slam_no_detection_mm)
            self._slam = RMHC_SLAM(
                laser, cfg.map_size_pixels, cfg.map_size_meters,
                map_quality=cfg.slam_map_quality,
                hole_width_mm=cfg.slam_hole_width_mm,
                sigma_xy_mm=cfg.slam_sigma_xy_mm,
                sigma_theta_degrees=cfg.slam_sigma_theta_deg,
                max_search_iter=cfg.slam_max_search_iter)
            log.info("BreezySLAM готов: карта %dpx / %.1fм",
                     cfg.map_size_pixels, cfg.map_size_meters)
        except Exception as exc:  # noqa: BLE001 - библиотека опциональна
            log.warning("BreezySLAM недоступен: %s -- карта выключена", exc)

    def available(self) -> bool:
        return self._slam is not None

    def update(self, scan_bins_m: np.ndarray, imu_yaw_deg: float | None,
               dt: float, fwd_pwm: float = 0.0) -> tuple[float, float, float]:
        """Один шаг SLAM. fwd_pwm -- средний ШИМ моторов (для одометрии смещения).

        Прайор движения для RMHC: поворот с IMU, смещение из команды ШИМ (энкодеров нет).
        Без прайора смещения SLAM не успевает за едущим роботом -> карта-облако.
        """
        # Скан в мм, 0 = нет возврата (как ждёт BreezySLAM).
        scan_mm = [0 if (v is None or math.isnan(v)) else int(v * 1000.0)
                   for v in scan_bins_m]

        dtheta = 0.0
        if imu_yaw_deg is not None and self._last_yaw is not None:
            dtheta = _angle_diff_deg(imu_yaw_deg, self._last_yaw)
            if self._cfg.slam_dtheta_invert:
                dtheta = -dtheta
        if imu_yaw_deg is not None:
            self._last_yaw = imu_yaw_deg

        # Поступательное смещение из ШИМ: (pwm/255)*скорость_при_полном*dt.
        # Инверсия: theta BreezySLAM на 180° от реального переда -> dxy идёт со знаком минус,
        # чтобы поза двигалась в реальном направлении движения робота.
        dxy_mm = (fwd_pwm / 255.0) * self._cfg.odom_speed_full_mps * dt * 1000.0
        if self._cfg.slam_odom_invert:
            dxy_mm = -dxy_mm
        self._slam.update(scan_mm, (dxy_mm, dtheta, dt))

        self._slam.getmap(self._mapbytes)
        x_mm, y_mm, theta_deg = self._slam.getpos()
        self._pose_mm = (x_mm, y_mm, theta_deg)
        return self._pose_mm

    @property
    def pose_mm(self) -> tuple[float, float, float]:
        return self._pose_mm

    def map_array(self) -> np.ndarray:
        """Карта как 2D uint8 (0=препятствие .. 255=свободно, 127=неизвестно)."""
        n = self._cfg.map_size_pixels
        return np.frombuffer(bytes(self._mapbytes), dtype=np.uint8).reshape((n, n))

    # ---- координаты: мир(мм) <-> пиксель(col,row) ----
    # BreezySLAM: getpos() отсчитывает от УГЛА карты (0..size_mm), робот стартует в центре
    # (size_mm/2). Байты карты row-major: index = row*size + col. Поэтому col~x, row~y.
    # ВНИМАНИЕ: знак theta и направление y проверить/откалибровать на железе (см. docs).
    def world_to_pixel(self, x_mm: float, y_mm: float) -> tuple[int, int]:
        ppm = self._cfg.map_size_pixels / self._cfg.map_size_meters  # пикс/метр
        col = int(round((x_mm / 1000.0) * ppm))
        row = int(round((y_mm / 1000.0) * ppm))
        n = self._cfg.map_size_pixels
        return max(0, min(n - 1, col)), max(0, min(n - 1, row))

    def pixel_to_world(self, col: int, row: int) -> tuple[float, float]:
        mpp = self._cfg.map_size_meters / self._cfg.map_size_pixels  # метр/пикс
        return col * mpp * 1000.0, row * mpp * 1000.0
