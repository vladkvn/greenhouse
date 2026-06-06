"""SimExplorer — автономное исследование (frontier-based) для построения карты с нуля.

Без априорных данных робот строит evidence-сетку по лидару и едет к ближайшей открытой
границе «известное↔неизвестное», планируя ТОЛЬКО по уже построенной карте (через неизвестное
не ходим). Достигнув границы, видит дальше — карта растёт. Застрявшие/недостижимые границы
заносятся в чёрный список. Когда открытых границ не осталось — достижимое пространство
разведано. Та же связка (mapper + planner + reactive follower), что и в остальном ядре.
"""

from __future__ import annotations

from greenhouse.adapters.sim.loop import SimRobot
from greenhouse.domain.geometry import Point2D, Pose2D, Twist2D
from greenhouse.domain.grid import CellState, OccupancyGrid
from greenhouse.navigation.mapping import EvidenceGridMapper, nearest_frontier
from greenhouse.navigation.planning import AStarPlanner, Path, PlanOk, PurePursuitLocalPlanner
from greenhouse.orchestration.modes import RobotMode


class SimExplorer:
    """Ведёт `SimRobot` по логике границ, наполняя `EvidenceGridMapper` картой."""

    def __init__(
        self,
        *,
        robot: SimRobot,
        mapper: EvidenceGridMapper,
        robot_radius_m: float = 0.25,
        frontier_min_m: float = 0.8,
        replan_every: int = 12,
        stuck_ticks: int = 40,
        dt_s: float = 0.1,
    ) -> None:
        self._robot = robot
        self._mapper = mapper
        self._radius = robot_radius_m
        self._dt = dt_s
        self._replan_every = replan_every
        self._stuck_limit = stuck_ticks
        meta = mapper.meta
        self._min_cells = max(1, int(frontier_min_m / meta.resolution_m))
        self._clear_rad = int(robot_radius_m / meta.resolution_m) + 1
        self._planner = AStarPlanner(allow_unknown=False)
        # Путь A* (по построенной карте, с инфляцией) уже безопасен → достаточно ехать по
        # точкам; реактивный объезд тут не нужен и только заклинивал бы в углах.
        self._follower = PurePursuitLocalPlanner(
            max_linear_m_s=robot.motion.limits.max_linear_m_s,
            max_angular_rad_s=robot.motion.limits.max_angular_rad_s,
        )
        self._goal: Pose2D | None = None
        self._path: Path | None = None
        self._timer = 0
        self._stuck = 0
        self._last_xy: Point2D | None = None
        self._blacklist: set[tuple[int, int]] = set()
        self._recover = 0
        self._recover_turn = 1.0

    def start(self) -> None:
        """Начать исследование с нуля: очистить карту и состояние, перейти в режим MAPPING."""
        self._mapper.begin_session()
        self._robot.mode = RobotMode.MAPPING
        self._goal = None
        self._path = None
        self._timer = 0
        self._stuck = 0
        self._last_xy = None
        self._blacklist = set()
        self._recover = 0
        self._recover_turn = 1.0

    @property
    def goal(self) -> Pose2D | None:
        return self._goal

    def current_map(self) -> OccupancyGrid:
        return self._mapper.current_map()

    def update(self) -> bool:
        """Один тик исследования (без шага физики). True — пространство разведано."""
        robot = self._robot
        pose = robot.state.pose()
        scan = robot.lidar.read_scan()
        self._mapper.ingest_scan(scan=scan, pose_xytheta=(pose.x_m, pose.y_m, pose.theta_rad))
        built = self._mapper.current_map()

        # Recovery-манёвр: застряли — отъезжаем назад с поворотом, чтобы выбраться из угла.
        if self._recover > 0:
            self._recover -= 1
            robot.motion.command(twist=Twist2D(linear_x_m_s=-0.35, angular_z_rad_s=self._recover_turn))
            return False

        moving_cmd = abs(robot.state.last_cmd.linear_x_m_s) > 0.05
        if moving_cmd and self._last_xy is not None and pose.point.distance_to(self._last_xy) < 0.01:
            self._stuck += 1
        else:
            self._stuck = 0
        self._last_xy = pose.point
        if self._stuck > self._stuck_limit:  # застряли — в чёрный список и отъезд
            if self._goal is not None:
                self._blacklist.add(built.meta.world_to_cell(self._goal.point))
            self._goal, self._stuck, self._path = None, 0, None
            self._recover, self._recover_turn = 35, -self._recover_turn
            return False

        reached = self._goal is not None and pose.point.distance_to(self._goal.point) <= 0.35
        self._timer += 1
        if self._goal is None or reached or self._timer >= self._replan_every:
            self._timer = 0
            cell = nearest_frontier(
                built, built.meta.world_to_cell(pose.point),
                min_cells=self._min_cells, clear_rad=self._clear_rad, blacklist=self._blacklist,
            )
            if cell is None:  # границ нет — готово
                robot.motion.stop()
                robot.mode = RobotMode.IDLE
                return True
            wp = built.meta.cell_to_world(*cell)
            self._goal = Pose2D(x_m=wp.x_m, y_m=wp.y_m, theta_rad=0.0)
            result = self._planner.plan(
                grid=built, start=pose, goal=self._goal, robot_radius_m=self._radius, best_effort=True
            )
            self._path = result.path if isinstance(result, PlanOk) else None
            if self._path is not None:
                self._follower.set_path(path=self._path)

        if self._path is not None:
            robot.motion.command(twist=self._follower.compute_command(pose=pose, scan=scan))
        else:
            robot.motion.stop()
        return False

    def explore(self, *, max_ticks: int = 5000) -> OccupancyGrid:
        """Прогнать исследование до конца (с шагом физики). Вернуть построенную карту."""
        self.start()
        for _ in range(max_ticks):
            done = self.update()
            self._robot.engine.step(dt_s=self._dt)
            if done:
                break
        return self._mapper.current_map()

    def coverage_known(self) -> float:
        cells = self._mapper.current_map().cells
        return sum(c is not CellState.UNKNOWN for c in cells) / len(cells)
