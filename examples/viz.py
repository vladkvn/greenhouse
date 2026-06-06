"""Интерактивная 2D-сцена GreenHouse (окно pygame).

Робот живёт в мире-теплице. Кликни мышью в любой точке — туда ставится цель, A*
строит путь по карте, и робот едет к ней, объезжая грядки. Можно кликать на ходу —
маршрут перепланируется от текущей позы.

Управление:
    ЛКМ           — задать цель в этой точке
    R             — вернуть робота на старт
    ПРОБЕЛ        — стоп (сбросить цель)
    ESC / закрыть — выход

Запуск:
    pip install -e ".[viz]"        # один раз: поставить pygame-ce
    python examples/viz.py
"""

from __future__ import annotations

import math
import sys

import pygame

from greenhouse.adapters.sim import (
    PolygonWorld,
    build_sim_robot,
    greenhouse_rows_world,
    grid_meta_for_world,
)
from greenhouse.domain.geometry import Pose2D
from greenhouse.domain.grid import CellState, MapMeta, OccupancyGrid
from greenhouse.navigation.planning import AStarPlanner, Path, PlanOk, PurePursuitLocalPlanner
from greenhouse.orchestration.modes import RobotMode

PX_PER_M = 64           # масштаб: пикселей на метр
FPS = 60
DT_S = 1.0 / FPS
ROBOT_RADIUS_M = 0.25
START = (0.5, 1.0)      # старт робота (в проходе нижнего ряда)

# Палитра (R, G, B).
COL_BG = (24, 26, 30)
COL_FREE = (210, 214, 220)
COL_OCC = (40, 44, 52)
COL_PATH = (250, 196, 60)
COL_ROBOT = (60, 150, 245)
COL_GOAL = (235, 80, 80)
COL_TEXT = (235, 238, 242)
COL_BAD = (235, 130, 90)


def occupancy_from_world(world: PolygonWorld, meta: MapMeta) -> OccupancyGrid:
    """Растеризовать истинную геометрию мира в сетку занятости (для надёжного демо).

    Ячейка занята, если её центр ближе к стене, чем размер ячейки. Внутренности грядок
    запечатываются инфляцией планировщика под радиус робота.
    """
    cells: list[CellState] = []
    for row in range(meta.height_px):
        for col in range(meta.width_px):
            center = meta.cell_to_world(row, col)
            clear = world.min_clearance(point=center)
            cells.append(CellState.OCCUPIED if clear < meta.resolution_m else CellState.FREE)
    return OccupancyGrid(meta=meta, cells=cells)


class Scene:
    def __init__(self) -> None:
        self.world = greenhouse_rows_world()
        self.meta = grid_meta_for_world(self.world, resolution_m=0.1, padding_m=0.3)
        self.grid = occupancy_from_world(self.world, self.meta)
        self.planner = AStarPlanner(allow_unknown=True)
        self.local = PurePursuitLocalPlanner(
            max_linear_m_s=1.0, max_angular_rad_s=1.5, goal_tol_m=0.15
        )
        self.robot = build_sim_robot(world=self.world, start_x_m=START[0], start_y_m=START[1])
        self.robot.local_planner = self.local

        self.goal: Pose2D | None = None
        self.path: Path | None = None
        self.approx = False
        self.status = "Кликни, чтобы задать цель"

        self.screen_w = int(self.meta.width_px * self.meta.resolution_m * PX_PER_M)
        self.screen_h = int(self.meta.height_px * self.meta.resolution_m * PX_PER_M)

    # --- преобразования координат мир <-> экран (ось Y вверх в мире) ---
    def to_screen(self, x_m: float, y_m: float) -> tuple[int, int]:
        sx = (x_m - self.meta.origin.x_m) * PX_PER_M
        sy = self.screen_h - (y_m - self.meta.origin.y_m) * PX_PER_M
        return int(sx), int(sy)

    def to_world(self, sx: int, sy: int) -> tuple[float, float]:
        x = sx / PX_PER_M + self.meta.origin.x_m
        y = (self.screen_h - sy) / PX_PER_M + self.meta.origin.y_m
        return x, y

    def set_goal(self, sx: int, sy: int) -> None:
        gx, gy = self.to_world(sx, sy)
        goal = Pose2D(x_m=gx, y_m=gy, theta_rad=0.0)
        self.goal = goal
        result = self.planner.plan(
            grid=self.grid,
            start=self.robot.state.pose(),
            goal=goal,
            robot_radius_m=ROBOT_RADIUS_M,
            best_effort=True,  # если в саму точку нельзя — подъехать как можно ближе
        )
        if isinstance(result, PlanOk):
            self.path = result.path
            self.local.set_path(path=result.path)
            self.robot.mode = RobotMode.NAVIGATING
            # Конечная точка пути далеко от клика → цель была недостижима.
            stop = result.path.waypoints[-1].point
            self.approx = stop.distance_to(goal.point) > 0.25
            self.status = "Еду к ближайшей достижимой точке…" if self.approx else "Еду к цели…"
        else:
            self.path = None
            self.approx = False
            self.robot.mode = RobotMode.IDLE
            self.robot.motion.stop()
            self.status = f"Недостижимо: {result.message}"

    def stop(self) -> None:
        self.goal = None
        self.path = None
        self.robot.mode = RobotMode.IDLE
        self.robot.motion.stop()
        self.status = "Стоп. Кликни, чтобы задать цель"

    def reset(self) -> None:
        s = self.robot.state
        s.x_m, s.y_m, s.theta_rad = START[0], START[1], 0.0
        self.stop()

    def update(self) -> None:
        self.robot.tick(dt_s=DT_S)
        if self.robot.mode is RobotMode.NAVIGATING and self.local.is_goal_reached(
            pose=self.robot.state.pose()
        ):
            err = self.robot.state.pose().point.distance_to(
                self.goal.point if self.goal else self.robot.state.pose().point
            )
            self.robot.mode = RobotMode.IDLE
            self.robot.motion.stop()
            if self.approx:
                self.status = f"Подъехал максимально близко (до цели {err:.2f} м)"
            else:
                self.status = f"Цель достигнута (ошибка {err:.2f} м)"


def build_map_surface(scene: Scene) -> pygame.Surface:
    """Статичная подложка карты рендерится один раз (свободно / занято)."""
    surf = pygame.Surface((scene.screen_w, scene.screen_h))
    surf.fill(COL_BG)
    res_px = max(1, int(scene.meta.resolution_m * PX_PER_M + 0.999))
    m = scene.meta
    for row in range(m.height_px):
        for col in range(m.width_px):
            state = scene.grid.at(row, col)
            if state is CellState.OCCUPIED:
                color = COL_OCC
            elif state is CellState.FREE:
                color = COL_FREE
            else:
                continue
            center = m.cell_to_world(row, col)
            sx, sy = scene.to_screen(center.x_m, center.y_m)
            surf.fill(color, (sx - res_px // 2, sy - res_px // 2, res_px, res_px))
    return surf


def draw(scene: Scene, screen: pygame.Surface, map_surf: pygame.Surface, font: pygame.font.Font) -> None:
    screen.blit(map_surf, (0, 0))

    if scene.path is not None and len(scene.path.waypoints) >= 2:
        pts = [scene.to_screen(wp.x_m, wp.y_m) for wp in scene.path.waypoints]
        pygame.draw.lines(screen, COL_PATH, False, pts, 3)
        for px, py in pts:
            pygame.draw.circle(screen, COL_PATH, (px, py), 3)

    if scene.goal is not None:
        gx, gy = scene.to_screen(scene.goal.x_m, scene.goal.y_m)
        pygame.draw.circle(screen, COL_GOAL, (gx, gy), 8, 2)
        pygame.draw.line(screen, COL_GOAL, (gx - 10, gy), (gx + 10, gy), 2)
        pygame.draw.line(screen, COL_GOAL, (gx, gy - 10), (gx, gy + 10), 2)

    pose = scene.robot.state.pose()
    rx, ry = scene.to_screen(pose.x_m, pose.y_m)
    rpx = int(ROBOT_RADIUS_M * PX_PER_M)
    pygame.draw.circle(screen, COL_ROBOT, (rx, ry), rpx)
    hx = rx + int(math.cos(pose.theta_rad) * rpx * 1.6)
    hy = ry - int(math.sin(pose.theta_rad) * rpx * 1.6)
    pygame.draw.line(screen, COL_TEXT, (rx, ry), (hx, hy), 2)

    bad = scene.status.startswith("Недостижимо")
    lines = [
        scene.status,
        f"батарея: {scene.robot.state.battery_frac * 100:.1f}%   режим: {scene.robot.mode.value}",
        "ЛКМ — цель | ПРОБЕЛ — стоп | R — сброс | ESC — выход",
    ]
    colors = [COL_BAD if bad else COL_TEXT, COL_TEXT, COL_TEXT]
    for i, (text, color) in enumerate(zip(lines, colors, strict=True)):
        screen.blit(font.render(text, True, color), (10, 8 + i * 20))


def pick_font() -> pygame.font.Font:
    """Шрифт с кириллицей (на macOS обычно есть Arial)."""
    for name in ("arial", "helvetica", "applesymbols", "menlo"):
        path = pygame.font.match_font(name)
        if path:
            return pygame.font.Font(path, 15)
    return pygame.font.Font(None, 18)


def main(argv: list[str] | None = None) -> None:
    argv = sys.argv[1:] if argv is None else argv
    headless = "--selftest" in argv

    pygame.init()
    scene = Scene()
    screen = pygame.display.set_mode((scene.screen_w, scene.screen_h))
    pygame.display.set_caption("GreenHouse — кликни, чтобы задать цель роботу")
    font = pick_font()
    map_surf = build_map_surface(scene)
    clock = pygame.time.Clock()

    if headless:  # автотест: задать цель, прокрутить кадры, выйти
        sx, sy = scene.to_screen(7.0, 7.0)
        scene.set_goal(sx, sy)
        for _ in range(4000):
            scene.update()
            if scene.robot.mode is RobotMode.IDLE:
                break
        draw(scene, screen, map_surf, font)
        print(f"selftest: {scene.status}")
        pygame.quit()
        return

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                scene.set_goal(*event.pos)
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_r:
                    scene.reset()
                elif event.key == pygame.K_SPACE:
                    scene.stop()

        scene.update()
        draw(scene, screen, map_surf, font)
        pygame.display.flip()
        clock.tick(FPS)

    pygame.quit()


if __name__ == "__main__":
    main()
