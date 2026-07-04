"""Hector-SLAM-стиль: scan-to-map matching по occupancy-grid (Gauss-Newton).

Ориентируемся, СОПОСТАВЛЯЯ текущий скан лидара с уже накопленной картой (а не доверяя
колёсам): для каждой позы считаем, насколько хорошо концы лучей ложатся на занятые ячейки,
и градиентным методом (Gauss-Newton по билинейно-интерполированной карте) точно доводим
позу. Это даёт чистые карты на одном лидаре, без энкодеров.

Внутренний кадр: x,y в МЕТРАХ (origin в углу карты, робот стартует в центре), theta в РАД
(0 = +x, против часовой). Карта -- вероятность занятости [0..1], 0.5 = неизвестно.
Пиксель: col = x/res, row = y/res (как world_to_pixel в мм/1000) -- интерфейс совместим с
SlamMapper (pose_mm, map_array, world_to_pixel, pixel_to_world), это drop-in замена.

v1: чистый scan matching от предыдущей позы (на медленной езде межкадровое смещение мало,
Gauss-Newton его находит). IMU/одометрия пока не нужны. Параметры в config (hector_*).
"""

from __future__ import annotations

import logging
import math

import numpy as np

from .config import FollowMeConfig

log = logging.getLogger("follow_me.hector")


def _angle_diff_deg(a: float, b: float) -> float:
    return (a - b + 180.0) % 360.0 - 180.0


class HectorSlam:
    def __init__(self, cfg: FollowMeConfig):
        self._cfg = cfg
        self.n = cfg.map_size_pixels
        self.res = cfg.map_size_meters / self.n  # м/пиксель
        self.prob = np.full((self.n, self.n), 0.5, dtype=np.float32)  # occupancy
        c = cfg.map_size_meters / 2.0   # старт в центре карты
        self.x = c
        self.y = c
        self.theta = 0.0                # рад
        self._scans = 0
        self._last_yaw = None           # для прайора поворота с IMU

    def available(self) -> bool:
        return True

    @property
    def pose_mm(self) -> tuple[float, float, float]:
        return self.x * 1000.0, self.y * 1000.0, math.degrees(self.theta)

    def map_array(self) -> np.ndarray:
        # prob: 1=занято -> 0(чёрный), 0=свободно -> 255(белый), 0.5 -> 127(серый).
        return ((1.0 - self.prob) * 255.0).astype(np.uint8)

    def world_to_pixel(self, x_mm: float, y_mm: float) -> tuple[int, int]:
        col = int(round(x_mm / 1000.0 / self.res))
        row = int(round(y_mm / 1000.0 / self.res))
        return (max(0, min(self.n - 1, col)), max(0, min(self.n - 1, row)))

    def pixel_to_world(self, col: int, row: int) -> tuple[float, float]:
        return col * self.res * 1000.0, row * self.res * 1000.0

    # ------------------------------------------------------------- scan -> точки
    def _scan_points(self, scan_bins_m: np.ndarray):
        """Точки скана в кадре робота (м) + их дистанции. None если пусто."""
        idx = np.where(~np.isnan(scan_bins_m))[0]
        if idx.size == 0:
            return None
        d = scan_bins_m[idx].astype(np.float64)
        ok = (d > 0.05) & (d < self._cfg.lidar_max_range_m)
        idx = idx[ok]
        d = d[ok]
        if d.size < 8:
            return None
        ang = np.radians(idx.astype(np.float64)) * self._cfg.hector_scan_dir
        pts = np.stack([d * np.cos(ang), d * np.sin(ang)], axis=1)  # Nx2
        return pts, d

    # ------------------------------------------------ билинейная M и её градиент
    @staticmethod
    def _interp(wx, wy, pmap, res):
        """Значение карты pmap и градиент (по миру, 1/м) в мировых точках (м)."""
        n = pmap.shape[0]
        cx = wx / res
        cy = wy / res
        x0 = np.clip(np.floor(cx).astype(np.int64), 0, n - 2)
        y0 = np.clip(np.floor(cy).astype(np.int64), 0, n - 2)
        x1 = x0 + 1
        y1 = y0 + 1
        dx = np.clip(cx - x0, 0.0, 1.0)
        dy = np.clip(cy - y0, 0.0, 1.0)
        m00 = pmap[y0, x0]; m10 = pmap[y0, x1]; m01 = pmap[y1, x0]; m11 = pmap[y1, x1]
        M = (m00 * (1 - dx) * (1 - dy) + m10 * dx * (1 - dy)
             + m01 * (1 - dx) * dy + m11 * dx * dy)
        dM_dcx = (m10 - m00) * (1 - dy) + (m11 - m01) * dy
        dM_dcy = (m01 - m00) * (1 - dx) + (m11 - m10) * dx
        return M, dM_dcx / res, dM_dcy / res

    def _coarse(self, scale: int):
        """Грубая карта (MAX-pool) + её res. Стена остаётся яркой (занято если занята
        любая мелкая ячейка) -> широкий градиент без размытия значения стены."""
        if scale <= 1:
            return self.prob, self.res
        m = self.n // scale
        k = m * scale
        pooled = self.prob[:k, :k].reshape(m, scale, m, scale).max(axis=(1, 3))
        return pooled.astype(np.float32), self.res * scale

    # ------------------------------------------------- scan-to-map (Gauss-Newton)
    def _gn(self, pts, pmap, res, x, y, th):
        # match_theta=False -> правим только x,y, курс берём с IMU (устойчиво у
        # вращательно-симметричных объектов, где скан-матчинг по углу неоднозначен).
        match_theta = self._cfg.hector_match_theta
        for _ in range(self._cfg.hector_iters):
            c, s = math.cos(th), math.sin(th)
            px, py = pts[:, 0], pts[:, 1]
            wx = x + c * px - s * py
            wy = y + s * px + c * py
            M, gx, gy = self._interp(wx, wy, pmap, res)
            r = 1.0 - M
            if match_theta:
                dpx = -s * px - c * py
                dpy = c * px - s * py
                J = np.stack([gx, gy, gx * dpx + gy * dpy], axis=1)   # Nx3
            else:
                J = np.stack([gx, gy], axis=1)                        # Nx2 (только x,y)
            H = J.T @ J
            b = J.T @ r
            lm = 1e-2 * np.diag(H) + 1e-4   # Levenberg-Marquardt демпфирование
            try:
                delta = np.linalg.solve(H + np.diag(lm), b)
            except np.linalg.LinAlgError:
                break
            delta = np.clip(delta, -0.2, 0.2)   # шаг на итерацию
            x += delta[0]; y += delta[1]
            if match_theta:
                th += delta[2]
            if np.sum(np.abs(delta)) < 1e-4:
                break
        return x, y, th

    def _match(self, pts: np.ndarray) -> None:
        x0, y0, th0 = self.x, self.y, self.theta
        x, y, th = x0, y0, th0
        # Грубо -> точно: на грубой карте широкий градиент ловит большое смещение.
        for scale in self._cfg.hector_scales:
            pmap, res = self._coarse(scale)
            x, y, th = self._gn(pts, pmap, res, x, y, th)
        # Защита от расхождения: межкадровое смещение мало; большой скачок -> отброс.
        if (abs(x - x0) > self._cfg.hector_max_jump_m
                or abs(y - y0) > self._cfg.hector_max_jump_m
                or abs(th - th0) > math.radians(self._cfg.hector_max_jump_deg)):
            return  # оставить прайор (предыдущую позу)
        lim = self._cfg.map_size_meters
        self.x = min(max(x, 0.0), lim)
        self.y = min(max(y, 0.0), lim)
        self.theta = th

    # ------------------------------------------------------------- обновление карты
    def _update_map(self, pts: np.ndarray, dists: np.ndarray) -> None:
        c, s = math.cos(self.theta), math.sin(self.theta)
        px, py = pts[:, 0], pts[:, 1]
        ex = (self.x + c * px - s * py) / self.res     # концы лучей, в ячейках
        ey = (self.y + s * px + c * py) / self.res
        rx = self.x / self.res
        ry = self.y / self.res
        n = self.n
        free_keep = 1.0 - self._cfg.hector_free_gain
        # свободное пространство вдоль лучей (до конца)
        for i in range(pts.shape[0]):
            steps = max(1, int(dists[i] / self.res))
            t = np.linspace(0.0, 1.0, steps, endpoint=False)
            fx = (rx + t * (ex[i] - rx)).astype(np.int64)
            fy = (ry + t * (ey[i] - ry)).astype(np.int64)
            ok = (fx >= 0) & (fx < n) & (fy >= 0) & (fy < n)
            self.prob[fy[ok], fx[ok]] *= free_keep      # к 0 (свободно)
        # концы лучей -> занято
        eix = ex.astype(np.int64); eiy = ey.astype(np.int64)
        ok = (eix >= 0) & (eix < n) & (eiy >= 0) & (eiy < n)
        p = self.prob[eiy[ok], eix[ok]]
        self.prob[eiy[ok], eix[ok]] = p + self._cfg.hector_occ_gain * (1.0 - p)
        np.clip(self.prob, 0.02, 0.98, out=self.prob)

    def _apply_motion_prior(self, imu_yaw_deg, dt: float, fwd_pwm: float) -> None:
        """Начальная догадка позы: поворот с IMU, смещение из ШИМ. matcher потом доводит.

        Без прайора поворота scan matcher не успевает за вращением -> карта рвётся.
        """
        cfg = self._cfg
        if cfg.hector_use_imu and imu_yaw_deg is not None:
            if self._last_yaw is not None:
                dyaw = _angle_diff_deg(imu_yaw_deg, self._last_yaw)
                self.theta += math.radians(dyaw * cfg.hector_imu_sign)
            self._last_yaw = imu_yaw_deg
        if cfg.hector_use_odom and fwd_pwm:
            dist = (fwd_pwm / 255.0) * cfg.odom_speed_full_mps * dt
            self.x += dist * math.cos(self.theta)
            self.y += dist * math.sin(self.theta)

    # ------------------------------------------------------------------- update
    def update(self, scan_bins_m: np.ndarray, imu_yaw_deg, dt: float,
               fwd_pwm: float = 0.0) -> tuple[float, float, float]:
        sp = self._scan_points(scan_bins_m)
        if sp is None:
            return self.pose_mm
        pts, dists = sp
        # Прайор движения (IMU поворот + ШИМ смещение) -- начальная догадка для matcher.
        self._apply_motion_prior(imu_yaw_deg, dt, fwd_pwm)
        # Прогрев: первые сканы только строят карту (матчить не по чему).
        if self._scans >= self._cfg.hector_warmup_scans:
            self._match(pts)        # доводим позу по совпадению скана с картой
        self._update_map(pts, dists)
        self._scans += 1
        return self.pose_mm
