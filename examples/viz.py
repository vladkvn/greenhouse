"""Интерактивная 2D-сцена GreenHouse — все возможности ядра в одном окне (pygame).

Два робота в теплице. Кликом задаёшь цель — A* строит путь, робот едет, объезжая грядки,
keep-out зоны и второго робота (перепланирование на лету). Можно переключить робота в режим
следования за целью, рисовать запретные зоны мышью, отправлять на зарядку; заряд батареи
тратится и восполняется на доке (при низком заряде робот сам едет заряжаться). Опционально
показывается оценка позы scan-matching локализацией.

Управление:
    ЛКМ              — задать цель выбранному роботу (в режиме зоны — угол зоны)
    ПКМ              — переместить цель-человека сюда
    1 / 2            — выбрать робота A / B
    F                — режим следования за человеком для выбранного робота
    M                — режим построения карты с нуля (кликай точки — карта строится по лидару)
    G                — режим навигации к цели (по умолчанию)
    C                — отправить выбранного робота на зарядку
    Z                — нарисовать keep-out зону (два клика по углам)
    X                — убрать все зоны
    L                — оверлей локализации (оценка позы vs истина)
    ПРОБЕЛ           — стоп выбранного робота
    R                — сброс сцены
    ESC / закрыть    — выход

Запуск:
    pip install -e ".[viz]"
    python examples/viz.py
"""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass, field

import pygame

from greenhouse.adapters.sim.loop import SimRobot, build_sim_robot
from greenhouse.adapters.sim.person import SimPerson, SimPersonDetector
from greenhouse.adapters.sim.world import PolygonWorld
from greenhouse.adapters.sim.worlds import greenhouse_rows_world, grid_meta_for_world
from greenhouse.domain.geometry import Point2D, Pose2D, Twist2D
from greenhouse.domain.grid import CellState, MapMeta, OccupancyGrid
from greenhouse.navigation.keepout import InMemoryKeepoutRegistry, Zone, ZoneKind
from greenhouse.navigation.localization import ScanMatchLocalizer
from greenhouse.navigation.mapping import EvidenceGridMapper
from greenhouse.navigation.planning import AStarPlanner, Path, PlanOk, ReactiveLocalPlanner
from greenhouse.orchestration.modes import RobotMode
from greenhouse.sensing.interfaces import LidarScan

PX_PER_M = 100
FPS = 60
DT_S = 1.0 / FPS
RADIUS_M = 0.25
LOW_BATTERY = 0.30
FULL_BATTERY = 0.98
DRAIN_PER_S = 0.012
REPLAN_EVERY = 8
FOLLOW_STANDOFF = 0.8
DOCK_APPROACH_M = 0.9  # на сколько отступает точка подъезда от дока

COL_BG = (24, 26, 30)
COL_FREE = (210, 214, 220)
COL_OCC = (45, 49, 58)
COL_ZONE = (200, 70, 70)
COL_DOCK = (90, 200, 120)
COL_PERSON = (240, 200, 70)
COL_TEXT = (235, 238, 242)
COL_DIM = (150, 156, 165)
COL_EST = (180, 120, 240)
ROBOT_COLORS = {"robot-A": (60, 150, 245), "robot-B": (240, 120, 80)}


@dataclass
class Agent:
    name: str
    robot: SimRobot
    color: tuple[int, int, int]
    detector: SimPersonDetector
    path_follower: ReactiveLocalPlanner
    localizer: ScanMatchLocalizer
    mapper: EvidenceGridMapper
    planner: AStarPlanner = field(default_factory=AStarPlanner)
    mode: RobotMode = RobotMode.IDLE
    goal: Pose2D | None = None
    path: Path | None = None
    replan_timer: int = 0
    docking: bool = False  # финальный заезд на док по прямой
    last_person: Point2D | None = None  # последняя видимая позиция цели
    lost_ticks: int = 0


class App:
    def __init__(self) -> None:
        self.world = greenhouse_rows_world(rows=2, row_length_m=8.0, aisle_m=1.5, bed_m=1.0)
        self.meta = grid_meta_for_world(self.world, resolution_m=0.1, padding_m=0.3)
        self.base_cells = _occupancy_cells(self.world, self.meta)
        self.keepout = InMemoryKeepoutRegistry()
        self.dock = Point2D(x_m=0.9, y_m=0.75)
        # К доку подъезжаем с востока: точка подъезда правее дока, носом на запад.
        self.dock_approach = Point2D(x_m=self.dock.x_m + DOCK_APPROACH_M, y_m=self.dock.y_m)
        self.dock_heading = math.pi
        self.person = SimPerson(x_m=4.0, y_m=3.25)
        self.agents: list[Agent] = [self._make_agent("robot-A", 1.0, 0.75),
                                    self._make_agent("robot-B", 7.0, 5.75)]
        self.selected = 0
        self.show_loc = False
        self.zone_mode = False
        self.zone_corner: Point2D | None = None
        self.zone_count = 0
        self.status = "Кликни, чтобы задать цель роботу A"

        self.screen_w = int(self.meta.width_px * self.meta.resolution_m * PX_PER_M)
        self.screen_h = int(self.meta.height_px * self.meta.resolution_m * PX_PER_M) + 64

    def _make_agent(self, name: str, x: float, y: float) -> Agent:
        robot = build_sim_robot(
            world=self.world, robot_id=name, start_x_m=x, start_y_m=y,
            dock=self.dock, battery_drain_per_s=DRAIN_PER_S,
        )
        loc = ScanMatchLocalizer()
        loc.set_map(grid=OccupancyGrid(meta=self.meta, cells=self.base_cells))
        loc.set_initial_pose(pose=robot.state.pose())
        return Agent(
            name=name, robot=robot, color=ROBOT_COLORS[name],
            detector=SimPersonDetector(
                world=self.world, robot_state=robot.state, person=self.person,
                clock=robot.clock, max_range_m=6.0,
            ),
            path_follower=ReactiveLocalPlanner(
                robot_radius_m=RADIUS_M, max_linear_m_s=1.0, max_angular_rad_s=1.5, goal_tol_m=0.2
            ),
            localizer=loc,
            mapper=EvidenceGridMapper(meta=self.meta),
        )

    # --- координаты ---
    def to_screen(self, x_m: float, y_m: float) -> tuple[int, int]:
        gx, gy = self.meta.origin.x_m, self.meta.origin.y_m
        gh = self.meta.height_px * self.meta.resolution_m
        return int((x_m - gx) * PX_PER_M), int((gh - (y_m - gy)) * PX_PER_M)

    def to_world(self, sx: int, sy: int) -> Point2D:
        gx, gy = self.meta.origin.x_m, self.meta.origin.y_m
        gh = self.meta.height_px * self.meta.resolution_m
        return Point2D(x_m=sx / PX_PER_M + gx, y_m=gh - sy / PX_PER_M + gy)

    # --- планирование/поведение ---
    def planning_grid(self, agent: Agent) -> OccupancyGrid:
        cells = list(self.base_cells)
        for other in self.agents:  # другие роботы — динамические препятствия
            if other is not agent:
                _mark(cells, self.meta, other.robot.state.x_m, other.robot.state.y_m, 0.35)
        grid = OccupancyGrid(meta=self.meta, cells=cells)
        return self.keepout.apply_to(grid=grid)

    def plan(self, agent: Agent, goal: Pose2D) -> bool:
        grid = self.planning_grid(agent)
        cost = self.keepout.cost_layer(meta=self.meta)
        res = agent.planner.plan(
            grid=grid, start=agent.robot.state.pose(), goal=goal,
            robot_radius_m=RADIUS_M, best_effort=True, cost_layer=cost,
        )
        if isinstance(res, PlanOk):
            agent.path = res.path
            agent.path_follower.set_path(path=res.path)
            return True
        agent.path = None
        return False

    def update_agent(self, agent: Agent) -> None:
        robot = agent.robot
        scan = robot.lidar.read_scan()
        if self.show_loc:
            agent.localizer.update(scan=scan, odometry=robot.odometry.read_odometry())

        # Низкий заряд → автоматически на зарядку.
        if robot.state.battery_frac < LOW_BATTERY and agent.mode is not RobotMode.CHARGING:
            agent.mode = RobotMode.CHARGING
            agent.goal = Pose2D(x_m=self.dock.x_m, y_m=self.dock.y_m, theta_rad=0.0)
            agent.path, agent.docking = None, False

        pose = robot.state.pose()
        if agent.mode is RobotMode.CHARGING:
            self._charge_step(agent, pose, scan)
        elif agent.mode is RobotMode.NAVIGATING and agent.goal is not None:
            if pose.point.distance_to(agent.goal.point) <= 0.25:
                agent.mode = RobotMode.IDLE
                robot.motion.stop()
            else:
                self._drive(agent, agent.goal, scan)
        elif agent.mode is RobotMode.FOLLOWING:
            self._follow_step(agent, pose, scan)
        elif agent.mode is RobotMode.MAPPING:
            agent.mapper.ingest_scan(
                scan=scan, pose_xytheta=(pose.x_m, pose.y_m, pose.theta_rad)
            )  # строим карту из лучей
            if agent.goal is not None and pose.point.distance_to(agent.goal.point) > 0.25:
                self._drive(agent, agent.goal, scan)  # едем по кликнутым точкам, исследуя
            else:
                robot.motion.stop()
        else:
            robot.motion.stop()

        robot.engine.step(dt_s=DT_S)

    def _charge_step(self, agent: Agent, pose: Pose2D, scan: LidarScan) -> None:
        """Зарядка: навигация к точке подъезда → ровный заезд на док с нужной стороны → заряд.

        Заряжаемся только встав на док через фазу заезда — поэтому робот всегда подходит к
        станции с заданной стороны (с заданным курсом), а не случайно проезжая рядом.
        """
        robot = agent.robot
        if agent.docking:  # фаза финального заезда: строго по прямой носом в док
            if pose.point.distance_to(self.dock) <= 0.18:
                robot.motion.stop()  # встал на док — движок заряжает
                if robot.state.battery_frac >= FULL_BATTERY:
                    agent.mode, agent.docking = RobotMode.IDLE, False
            else:
                robot.motion.command(twist=_aim_drive(pose, self.dock, max_v=0.4))
        elif pose.point.distance_to(self.dock_approach) <= 0.3:
            agent.docking = True  # доехали до точки подъезда — начинаем заезд
        else:
            self._drive(agent, self.dock_approach_pose(), scan)  # к точке подъезда — с объездом

    def _follow_step(self, agent: Agent, pose: Pose2D, scan: LidarScan) -> None:
        """Следование к цели через планировщик (объезд препятствий).

        Видим цель — едем к ней в обход помех; на нужной дистанции держим позицию. Если цель
        скрылась за препятствием — короткое время едем к месту, где видели её последним, чтобы
        обогнуть помеху и снова поймать. Долго не видно — безопасная остановка.
        """
        obs = agent.detector.detect()
        if obs is not None:
            agent.lost_ticks = 0
            ang = pose.theta_rad + obs.bearing_rad
            agent.last_person = Point2D(
                x_m=pose.x_m + obs.range_m * math.cos(ang),
                y_m=pose.y_m + obs.range_m * math.sin(ang),
            )
            if obs.range_m <= FOLLOW_STANDOFF + 0.05:
                agent.robot.motion.stop()  # на нужной дистанции — держим позицию
                return
        else:
            agent.lost_ticks += 1
            if agent.lost_ticks > 80 or agent.last_person is None:
                agent.robot.motion.stop()  # цель давно не видно — безопасная остановка
                agent.path = None
                return

        target = agent.last_person
        if pose.point.distance_to(target) <= FOLLOW_STANDOFF + 0.05:
            agent.robot.motion.stop()
            return
        self._drive(agent, Pose2D(x_m=target.x_m, y_m=target.y_m, theta_rad=0.0), scan)

    def dock_approach_pose(self) -> Pose2D:
        return Pose2D(x_m=self.dock_approach.x_m, y_m=self.dock_approach.y_m, theta_rad=self.dock_heading)

    def _drive(self, agent: Agent, goal: Pose2D, scan: LidarScan) -> None:
        agent.replan_timer += 1
        if agent.path is None or agent.replan_timer >= REPLAN_EVERY:
            agent.replan_timer = 0
            self.plan(agent, goal)
        if agent.path is not None:
            cmd = agent.path_follower.compute_command(pose=agent.robot.state.pose(), scan=scan)
            agent.robot.motion.command(twist=cmd)
        else:
            agent.robot.motion.stop()

    # --- ввод ---
    def on_click(self, sx: int, sy: int) -> None:
        p = self.to_world(sx, sy)
        if self.zone_mode:
            if self.zone_corner is None:
                self.zone_corner = p
                self.status = "Зона: кликни второй угол"
            else:
                self._add_zone(self.zone_corner, p)
                self.zone_corner = None
                self.zone_mode = False
            return
        agent = self.agents[self.selected]
        agent.goal = Pose2D(x_m=p.x_m, y_m=p.y_m, theta_rad=0.0)
        if agent.mode is not RobotMode.MAPPING:  # в режиме картирования режим не меняем
            agent.mode = RobotMode.NAVIGATING
        agent.path = None
        ok = self.plan(agent, agent.goal)
        far = agent.path is not None and agent.path.waypoints[-1].point.distance_to(p) > 0.3
        self.status = (
            f"{agent.name}: недостижимо" if not ok
            else f"{agent.name}: еду к ближайшей точке" if far
            else f"{agent.name}: еду к цели"
        )

    def _add_zone(self, a: Point2D, b: Point2D) -> None:
        self.zone_count += 1
        x0, x1 = sorted((a.x_m, b.x_m))
        y0, y1 = sorted((a.y_m, b.y_m))
        self.keepout.add_zone(zone=Zone(
            zone_id=f"zone-{self.zone_count}", kind=ZoneKind.KEEPOUT,
            polygon=(Point2D(x_m=x0, y_m=y0), Point2D(x_m=x1, y_m=y0),
                     Point2D(x_m=x1, y_m=y1), Point2D(x_m=x0, y_m=y1)),
        ))
        for ag in self.agents:  # маршруты пересчитаются с учётом новой зоны
            ag.path = None
        self.status = "Зона добавлена"

    def reset(self) -> None:
        for ag, (x, y) in zip(self.agents, [(1.0, 0.75), (7.0, 5.75)], strict=True):
            s = ag.robot.state
            s.x_m, s.y_m, s.theta_rad, s.battery_frac = x, y, 0.0, 1.0
            ag.mode, ag.goal, ag.path, ag.docking = RobotMode.IDLE, None, None, False
            ag.robot.motion.stop()
        self.person.x_m, self.person.y_m = 4.0, 3.25
        self.status = "Сброс"

    # --- цикл/рендер ---
    def run(self, *, selftest: bool = False) -> None:
        pygame.init()
        screen = pygame.display.set_mode((self.screen_w, self.screen_h))
        pygame.display.set_caption("GreenHouse — интерактивная симуляция")
        font = _pick_font(15)
        small = _pick_font(13)
        map_surf = self._map_surface()
        clock = pygame.time.Clock()

        if selftest:
            self.agents[0].mode = RobotMode.NAVIGATING
            self.agents[0].goal = Pose2D(x_m=7.0, y_m=3.25, theta_rad=0.0)
            for _ in range(600):
                for ag in self.agents:
                    self.update_agent(ag)
            self.draw(screen, map_surf, font, small)
            print("selftest ok:", round(self.agents[0].robot.state.x_m, 2))
            pygame.quit()
            return

        running = True
        while running:
            for event in pygame.event.get():
                running = self._handle(event, map_surf) and running
            self.person.step(dt_s=DT_S)
            for ag in self.agents:
                self.update_agent(ag)
            self.draw(screen, map_surf, font, small)
            pygame.display.flip()
            clock.tick(FPS)
        pygame.quit()

    def _handle(self, event: pygame.event.Event, map_surf: pygame.Surface) -> bool:
        if event.type == pygame.QUIT:
            return False
        if event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 1:
                self.on_click(*event.pos)
            elif event.button == 3:
                p = self.to_world(*event.pos)
                self.person.x_m, self.person.y_m = p.x_m, p.y_m
        elif event.type == pygame.KEYDOWN:
            a = self.agents[self.selected]
            if event.key == pygame.K_ESCAPE:
                return False
            elif event.key == pygame.K_1:
                self.selected = 0
            elif event.key == pygame.K_2:
                self.selected = 1
            elif event.key == pygame.K_f:
                a.mode, a.path, a.docking = RobotMode.FOLLOWING, None, False
                self.status = f"{a.name}: следую за человеком"
            elif event.key == pygame.K_g:
                a.mode, a.docking = RobotMode.IDLE, False
                self.status = f"{a.name}: навигация (кликни цель)"
            elif event.key == pygame.K_m:
                a.mode, a.goal, a.path = RobotMode.MAPPING, None, None
                a.mapper.begin_session()  # карта с нуля
                self.status = f"{a.name}: картирование — кликай точки, карта строится"
            elif event.key == pygame.K_c:
                a.mode, a.path, a.docking = RobotMode.CHARGING, None, False
                a.goal = Pose2D(x_m=self.dock.x_m, y_m=self.dock.y_m, theta_rad=0.0)
                self.status = f"{a.name}: на зарядку"
            elif event.key == pygame.K_z:
                self.zone_mode, self.zone_corner = True, None
                self.status = "Зона: кликни первый угол"
            elif event.key == pygame.K_x:
                for z in self.keepout.zones():
                    self.keepout.remove_zone(zone_id=z.zone_id)
                for ag in self.agents:
                    ag.path = None
                self.status = "Зоны очищены"
            elif event.key == pygame.K_l:
                self.show_loc = not self.show_loc
                self.status = f"Локализация: {'вкл' if self.show_loc else 'выкл'}"
            elif event.key == pygame.K_SPACE:
                a.mode, a.path, a.docking = RobotMode.IDLE, None, False
                a.robot.motion.stop()
                self.status = f"{a.name}: стоп"
            elif event.key == pygame.K_r:
                self.reset()
        return True

    def _blit_built_map(self, screen: pygame.Surface, agent: Agent) -> None:
        """Нарисовать карту, построенную роботом: свободно/занято/неизвестно (фон)."""
        screen.fill(COL_BG)
        grid = agent.mapper.current_map()
        res_px = max(1, int(self.meta.resolution_m * PX_PER_M + 0.999))
        for row in range(self.meta.height_px):
            for col in range(self.meta.width_px):
                state = grid.at(row, col)
                if state is CellState.FREE:
                    color = COL_FREE
                elif state is CellState.OCCUPIED:
                    color = COL_OCC
                else:
                    continue  # UNKNOWN — оставляем фон
                center = self.meta.cell_to_world(row, col)
                sx, sy = self.to_screen(center.x_m, center.y_m)
                screen.fill(color, (sx - res_px // 2, sy - res_px // 2, res_px, res_px))

    def _map_surface(self) -> pygame.Surface:
        surf = pygame.Surface((self.screen_w, self.screen_h))
        surf.fill(COL_BG)
        res_px = max(1, int(self.meta.resolution_m * PX_PER_M + 0.999))
        grid = OccupancyGrid(meta=self.meta, cells=self.base_cells)
        for row in range(self.meta.height_px):
            for col in range(self.meta.width_px):
                state = grid.at(row, col)
                color = COL_OCC if state is CellState.OCCUPIED else COL_FREE
                center = self.meta.cell_to_world(row, col)
                sx, sy = self.to_screen(center.x_m, center.y_m)
                surf.fill(color, (sx - res_px // 2, sy - res_px // 2, res_px, res_px))
        return surf

    def draw(self, screen: pygame.Surface, map_surf: pygame.Surface,
             font: pygame.font.Font, small: pygame.font.Font) -> None:
        mapping = next((a for a in self.agents if a.mode is RobotMode.MAPPING), None)
        if mapping is not None:
            self._blit_built_map(screen, mapping)  # показываем карту «как видит робот»
        else:
            screen.blit(map_surf, (0, 0))

        for z in self.keepout.zones():  # keep-out зоны
            pts = [self.to_screen(p.x_m, p.y_m) for p in z.polygon]
            s = pygame.Surface((self.screen_w, self.screen_h), pygame.SRCALPHA)
            pygame.draw.polygon(s, (*COL_ZONE, 90), pts)
            pygame.draw.polygon(s, COL_ZONE, pts, 2)
            screen.blit(s, (0, 0))

        dx, dy = self.to_screen(self.dock.x_m, self.dock.y_m)  # док
        ax, ay = self.to_screen(self.dock_approach.x_m, self.dock_approach.y_m)
        pygame.draw.line(screen, COL_DOCK, (ax, ay), (dx, dy), 1)   # ось подъезда
        pygame.draw.circle(screen, COL_DOCK, (ax, ay), 4, 1)
        pygame.draw.rect(screen, COL_DOCK, (dx - 12, dy - 12, 24, 24), 2)
        screen.blit(small.render("DOCK", True, COL_DOCK), (dx - 16, dy + 12))

        px, py = self.to_screen(self.person.x_m, self.person.y_m)  # человек-цель
        pygame.draw.circle(screen, COL_PERSON, (px, py), 7)
        pygame.draw.circle(screen, COL_BG, (px, py), 3)

        for i, ag in enumerate(self.agents):
            self._draw_agent(screen, ag, selected=i == self.selected)

        self._draw_panel(screen, font, small)

    def _draw_agent(self, screen: pygame.Surface, ag: Agent, *, selected: bool) -> None:
        if ag.path is not None and len(ag.path.waypoints) >= 2:
            pts = [self.to_screen(w.x_m, w.y_m) for w in ag.path.waypoints]
            pygame.draw.lines(screen, ag.color, False, pts, 2)
        if ag.goal is not None and ag.mode in (RobotMode.NAVIGATING, RobotMode.CHARGING):
            gx, gy = self.to_screen(ag.goal.x_m, ag.goal.y_m)
            pygame.draw.circle(screen, ag.color, (gx, gy), 6, 2)

        pose = ag.robot.state.pose()
        rx, ry = self.to_screen(pose.x_m, pose.y_m)
        rpx = int(RADIUS_M * PX_PER_M)
        if selected:
            pygame.draw.circle(screen, COL_TEXT, (rx, ry), rpx + 4, 2)
        pygame.draw.circle(screen, ag.color, (rx, ry), rpx)
        hx = rx + int(math.cos(pose.theta_rad) * rpx * 1.6)
        hy = ry - int(math.sin(pose.theta_rad) * rpx * 1.6)
        pygame.draw.line(screen, COL_BG, (rx, ry), (hx, hy), 2)

        if self.show_loc and (est := ag.localizer.latest()) is not None:
            ex, ey = self.to_screen(est.pose.x_m, est.pose.y_m)
            pygame.draw.circle(screen, COL_EST, (ex, ey), rpx + 2, 2)

    def _draw_panel(self, screen: pygame.Surface, font: pygame.font.Font,
                    small: pygame.font.Font) -> None:
        y0 = self.screen_h - 60
        pygame.draw.rect(screen, (18, 20, 24), (0, y0, self.screen_w, 60))
        screen.blit(font.render(self.status, True, COL_TEXT), (10, y0 + 6))
        for i, ag in enumerate(self.agents):  # индикаторы заряда
            bx = 10 + i * 150
            frac = ag.robot.state.battery_frac
            col = (90, 200, 120) if frac > LOW_BATTERY else (235, 90, 90)
            screen.blit(small.render(f"{ag.name} {ag.mode.value}", True, ag.color), (bx, y0 + 28))
            pygame.draw.rect(screen, COL_DIM, (bx, y0 + 44, 100, 8), 1)
            pygame.draw.rect(screen, col, (bx, y0 + 44, int(100 * frac), 8))
        hint = "ЛКМ цель | ПКМ чел | 1/2 робот | F след | M карта | C заряд | Z зона | X очист | L локал"
        screen.blit(small.render(hint, True, COL_DIM), (330, y0 + 8))


def _occupancy_cells(world: PolygonWorld, meta: MapMeta) -> list[CellState]:
    return [
        CellState.OCCUPIED
        if world.min_clearance(point=meta.cell_to_world(r, c)) < meta.resolution_m
        else CellState.FREE
        for r in range(meta.height_px)
        for c in range(meta.width_px)
    ]


def _aim_drive(pose: Pose2D, target: Point2D, *, max_v: float) -> Twist2D:
    """Ровно ехать на точку: довернуть носом и двигаться по прямой (финальный заезд на док)."""
    err = math.atan2(target.y_m - pose.y_m, target.x_m - pose.x_m) - pose.theta_rad
    err = math.atan2(math.sin(err), math.cos(err))
    w = max(-1.5, min(1.5, 2.0 * err))
    v = 0.0 if abs(err) > 0.4 else min(max_v, 1.5 * pose.point.distance_to(target))
    return Twist2D(linear_x_m_s=max(0.0, v), angular_z_rad_s=w)


def _mark(cells: list[CellState], meta: MapMeta, x_m: float, y_m: float, radius_m: float) -> None:
    rc = int(math.ceil(radius_m / meta.resolution_m))
    r0, c0 = meta.world_to_cell(Point2D(x_m=x_m, y_m=y_m))
    for dr in range(-rc, rc + 1):
        for dc in range(-rc, rc + 1):
            if dr * dr + dc * dc <= rc * rc and meta.in_bounds(r0 + dr, c0 + dc):
                cells[(r0 + dr) * meta.width_px + (c0 + dc)] = CellState.OCCUPIED


def _pick_font(size: int) -> pygame.font.Font:
    for name in ("arial", "helvetica", "applesymbols", "menlo"):
        path = pygame.font.match_font(name)
        if path:
            return pygame.font.Font(path, size)
    return pygame.font.Font(None, size + 3)


def main() -> None:
    App().run(selftest="--selftest" in sys.argv[1:])


if __name__ == "__main__":
    main()
