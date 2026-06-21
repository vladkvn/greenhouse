"""Планирование пути (A*) и ведение (pure-pursuit) для езды в точку.

Чистые алгоритмы, не зависят от железа -> покрыты юнит-тестами (tests/test_navigation.py).
Планировщик работает в пикселях occupancy-карты, контроллер — в мире (мм). Склейка с
SLAM-координатами — в Navigator (main.py использует slam.world_to_pixel/pixel_to_world).

Карта (uint8): 0=препятствие .. 255=свободно, ~127=неизвестно. Проходимо при value>=free_thr;
неизвестное считаем непроходимым (не лезем в незакартированное). Препятствия раздуваем на
радиус робота.
"""

from __future__ import annotations

import heapq
import math
import threading

import numpy as np

# 8-связность: смещения и стоимость шага.
_NEIGHBORS = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
              (-1, -1, 1.41421356), (-1, 1, 1.41421356),
              (1, -1, 1.41421356), (1, 1, 1.41421356)]


def build_traversable(grid: np.ndarray, radius_px: int,
                      free_thr: int = 200, obstacle_thr: int = 60) -> np.ndarray:
    """bool-карта проходимости: свободно (value>=free_thr) и не у РЕАЛЬНОГО препятствия.

    Неизвестное (obstacle_thr..free_thr) непроходимо, но НЕ раздувается -- иначе инфляция
    шла бы по всей незакартированной карте (медленно). Раздуваем только стены (<obstacle_thr).
    """
    free = grid >= free_thr
    obstacle = grid < obstacle_thr  # только реальные стены, не неизвестное
    if radius_px > 0 and obstacle.any():
        from collections import deque
        h, w = grid.shape
        blocked = obstacle.copy()
        dq = deque()
        dist = np.full(grid.shape, -1, dtype=np.int32)
        for r, c in zip(*np.where(obstacle)):
            dist[r, c] = 0
            dq.append((r, c))
        while dq:
            r, c = dq.popleft()
            if dist[r, c] >= radius_px:
                continue
            for dr, dc, _ in _NEIGHBORS:
                nr, nc = r + dr, c + dc
                if 0 <= nr < h and 0 <= nc < w and dist[nr, nc] < 0:
                    dist[nr, nc] = dist[r, c] + 1
                    blocked[nr, nc] = True
                    dq.append((nr, nc))
        return free & ~blocked
    return free & ~obstacle


def plan_path(grid: np.ndarray, start_px: tuple[int, int], goal_px: tuple[int, int],
              radius_px: int, free_thr: int = 200) -> list[tuple[int, int]] | None:
    """A* по occupancy-карте. Координаты (col, row). Возвращает путь или None."""
    trav = build_traversable(grid, radius_px, free_thr)
    h, w = grid.shape
    sc, sr = start_px
    gc, gr = goal_px
    if not (0 <= sr < h and 0 <= sc < w and 0 <= gr < h and 0 <= gc < w):
        return None
    if not trav[gr, gc]:
        return None  # цель в препятствии/неизвестном
    start = (sr, sc)
    goal = (gr, gc)

    def heur(r, c):
        return math.hypot(r - gr, c - gc)

    open_heap = [(heur(sr, sc), 0.0, start)]
    came: dict = {}
    gscore = {start: 0.0}
    while open_heap:
        _, g, cur = heapq.heappop(open_heap)
        if cur == goal:
            path = [cur]
            while cur in came:
                cur = came[cur]
                path.append(cur)
            path.reverse()
            return [(c, r) for (r, c) in path]  # вернуть как (col, row)
        r, c = cur
        if g > gscore.get(cur, math.inf):
            continue
        for dr, dc, cost in _NEIGHBORS:
            nr, nc = r + dr, c + dc
            if not (0 <= nr < h and 0 <= nc < w) or not trav[nr, nc]:
                continue
            ng = g + cost
            nxt = (nr, nc)
            if ng < gscore.get(nxt, math.inf):
                gscore[nxt] = ng
                came[nxt] = cur
                heapq.heappush(open_heap, (ng + heur(nr, nc), ng, nxt))
    return None


class Navigator:
    """Связывает SLAM-позу/карту с планировщиком и pure-pursuit. Хранит цель и путь.

    Используется в режиме goto: set_goal_px() с клика по карте, step() каждый тик отдаёт
    команду моторам. Реплан раз в ~1с (карта/поза уточняются на ходу).
    """

    def __init__(self, cfg, slam):
        self._cfg = cfg
        self._slam = slam
        self.goal_px: tuple[int, int] | None = None
        self.path_px: list[tuple[int, int]] | None = None
        self._last_plan = -1.0
        self._planning = False  # идёт ли фоновый расчёт пути
        self._lock = threading.Lock()

    def set_goal_px(self, col: int, row: int) -> None:
        with self._lock:
            self.goal_px = (int(col), int(row))
            self.path_px = None
            self._last_plan = -1.0

    def clear(self) -> None:
        with self._lock:
            self.goal_px = None
            self.path_px = None

    @property
    def radius_px(self) -> int:
        # Радиус робота + зазор до стен -> путь держится от стен на этот клиренс.
        clearance_m = self._cfg.robot_radius_m + self._cfg.nav_wall_clearance_m
        return int(clearance_m / self._cfg.map_size_meters
                   * self._cfg.map_size_pixels)

    def _plan_worker(self, goal_px: tuple[int, int]) -> None:
        """Тяжёлый A* в фоне -- НЕ блокирует главный цикл (иначе просадка FPS).
        Планируем на ЗАГРУБЛЁННОЙ сетке (быстрее в scale^2 раз)."""
        try:
            sc = max(1, self._cfg.nav_plan_scale)
            x, y, _ = self._slam.pose_mm
            spx, srow = self._slam.world_to_pixel(x, y)
            grid = downsample_min(self._slam.map_array(), sc)
            start_c = (spx // sc, srow // sc)
            goal_c = (goal_px[0] // sc, goal_px[1] // sc)
            rad_c = max(1, self.radius_px // sc)
            pc = plan_path(grid, start_c, goal_c, rad_c)
            # обратно в полное разрешение (центр грубой ячейки)
            path = ([(c * sc + sc // 2, r * sc + sc // 2) for (c, r) in pc]
                    if pc else None)
            with self._lock:
                if self.goal_px == goal_px:   # цель не сменилась за время расчёта
                    self.path_px = path
        finally:
            self._planning = False

    def step(self, now_s: float) -> tuple[int, int, str]:
        cfg = self._cfg
        with self._lock:
            goal_px = self.goal_px
            path_px = self.path_px
        if goal_px is None:
            return 0, 0, "нет цели"
        x, y, _th = self._slam.pose_mm
        # Цель достигнута -> сбросить (иначе робот колеблется у точки).
        gx, gy = self._slam.pixel_to_world(*goal_px)
        if math.hypot(gx - x, gy - y) <= cfg.goal_tolerance_m * 1000.0:
            self.clear()
            return 0, 0, "цель достигнута -> стоп"
        # Реплан раз в секунду в ФОНЕ -- главный цикл не стоит.
        if not self._planning and now_s - self._last_plan > 1.0:
            self._planning = True
            self._last_plan = now_s
            threading.Thread(target=self._plan_worker, args=(goal_px,),
                             daemon=True).start()
        if not path_px:
            return 0, 0, "планирую путь..."
        path_world = [self._slam.pixel_to_world(c, r) for (c, r) in path_px]
        # Реальный перёд робота = SLAM theta + сдвиг (кадр карты y-вниз + лидар 180).
        heading = self._slam.pose_mm[2] + cfg.nav_heading_offset_deg
        left, right, st = pursuit_command(
            x, y, heading, path_world,
            cfg.lookahead_m * 1000.0, cfg.goal_tolerance_m * 1000.0,
            cfg.nav_speed, cfg.nav_turn_gain,
            deadband_deg=cfg.nav_heading_deadband_deg, arc_deg=cfg.nav_arc_deg,
            spin_deg=cfg.nav_spin_deg, slow_radius_mm=cfg.nav_slow_radius_m * 1000.0)
        # Физическая инверсия лево/право (зеркальный кадр карты).
        if cfg.nav_turn_invert:
            left, right = right, left
        return left, right, st


def downsample_min(grid: np.ndarray, scale: int) -> np.ndarray:
    """Загрубление карты для планирования (min-pool: ячейка занята/неизвестна, если
    ТАКОВА любая мелкая -> консервативно к препятствиям). A* на ней в scale^2 раз быстрее."""
    if scale <= 1:
        return grid
    n = grid.shape[0]
    m = n // scale
    k = m * scale
    return grid[:k, :k].reshape(m, scale, m, scale).min(axis=(1, 3))


def _norm_deg(d: float) -> float:
    return (d + 180.0) % 360.0 - 180.0


def pursuit_command(x_mm: float, y_mm: float, theta_deg: float,
                    path_world_mm: list[tuple[float, float]],
                    lookahead_mm: float, goal_tol_mm: float,
                    speed: int, turn_gain: float, max_speed: int = 255,
                    deadband_deg: float = 8.0, arc_deg: float = 50.0,
                    spin_deg: float = 110.0,
                    slow_radius_mm: float = 700.0) -> tuple[int, int, str]:
    """Pure-pursuit для skid-steer. Возвращает (left, right, статус).

    Конвенция (как в compute_drive): err>0 (цель слева) -> turn>0 -> левый борт МЕДЛЕННЕЕ.
    Физическую инверсию лево/право (зеркальный кадр карты) применяет Navigator.
    Поведение: мёртвая зона по курсу гасит маятник; большой угол -> дуга, очень большой ->
    разворот на месте; у цели замедляемся.
    """
    if not path_world_mm:
        return 0, 0, "нет пути -> стоп"

    gx, gy = path_world_mm[-1]
    dist_goal = math.hypot(gx - x_mm, gy - y_mm)
    if dist_goal <= goal_tol_mm:
        return 0, 0, f"цель достигнута ({dist_goal/1000:.2f} м) -> стоп"

    # Точка упреждения: первая точка пути на расстоянии >= lookahead, иначе цель.
    target = path_world_mm[-1]
    for px, py in path_world_mm:
        if math.hypot(px - x_mm, py - y_mm) >= lookahead_mm:
            target = (px, py)
            break

    tx, ty = target
    bearing = math.degrees(math.atan2(ty - y_mm, tx - x_mm))
    err = _norm_deg(bearing - theta_deg)

    # Доворот с мёртвой зоной (внутри -- не крутим, чтобы не было маятника).
    if abs(err) < deadband_deg:
        turn = 0.0
    else:
        turn = turn_gain * max(-1.0, min(1.0, err / 90.0)) * speed

    # Ход: прямо при малом угле, дугой при большом, разворот на месте при очень большом.
    if abs(err) > spin_deg:
        fwd = 0.0
    elif abs(err) > arc_deg:
        fwd = speed * 0.45
    else:
        fwd = float(speed)
    # Замедление у цели (гасит проскок/маятник на финише).
    if dist_goal < slow_radius_mm:
        fwd *= max(0.35, dist_goal / slow_radius_mm)

    left = int(round(max(-max_speed, min(max_speed, fwd - turn))))
    right = int(round(max(-max_speed, min(max_speed, fwd + turn))))
    return left, right, f"d={dist_goal/1000:.2f}м err={err:+.0f}° L={left} R={right}"
