"""MappingBehavior — режим MAPPING: построить карту с нуля frontier-исследованием.

Пошаговый аналог SimExplorer.update, но на уровне ядра: поза берётся из localizer.latest()
(не из истины симулятора), поэтому поведение годится и для железа. Робот наполняет
EvidenceGridMapper по лидару и едет к ближайшей открытой границе «известное↔неизвестное»,
планируя ТОЛЬКО по уже построенной карте (через неизвестное не ходим). Когда границ не
осталось — достижимое пространство разведано, поведение запрашивает IDLE.
"""

from __future__ import annotations

from greenhouse.domain.geometry import Point2D, Pose2D, Twist2D
from greenhouse.domain.grid import OccupancyGrid
from greenhouse.navigation.mapping import EvidenceGridMapper, MapStore, nearest_frontier
from greenhouse.navigation.planning import AStarPlanner, Path, PlanOk, PurePursuitLocalPlanner
from greenhouse.navigation.protocols import RobotLike
from greenhouse.orchestration.modes import RobotMode


class MappingBehavior:
    """Реализует Protocol `Behavior` для режима MAPPING (frontier-исследование)."""

    def __init__(
        self,
        *,
        robot: RobotLike,
        mapper: EvidenceGridMapper,
        robot_radius_m: float = 0.25,
        frontier_min_m: float = 0.8,
        replan_every: int = 12,
        stuck_ticks: int = 40,
        map_store: MapStore | None = None,
        map_label: str = "map",
    ) -> None:
        self._robot = robot
        self._mapper = mapper
        self._radius = robot_radius_m
        self._map_store = map_store
        self._map_label = map_label
        self._replan_every = replan_every
        self._stuck_limit = stuck_ticks
        meta = mapper.meta
        self._min_cells = max(1, int(frontier_min_m / meta.resolution_m))
        self._clear_rad = int(robot_radius_m / meta.resolution_m) + 1
        self._planner = AStarPlanner(allow_unknown=False)
        self._follower = PurePursuitLocalPlanner(
            max_linear_m_s=robot.motion.limits.max_linear_m_s,
            max_angular_rad_s=robot.motion.limits.max_angular_rad_s,
        )
        self._begun = False
        self._goal: Pose2D | None = None
        self._path: Path | None = None
        self._timer = 0
        self._stuck = 0
        self._last_xy: Point2D | None = None
        self._last_cmd_v = 0.0
        self._blacklist: set[tuple[int, int]] = set()
        self._recover = 0
        self._recover_turn = 1.0

    @property
    def mode(self) -> RobotMode:
        return RobotMode.MAPPING

    def current_map(self) -> OccupancyGrid:
        return self._mapper.current_map()

    def step(self) -> RobotMode | None:
        est = self._robot.localizer.latest()
        if est is None or est.is_lost:
            return None  # без позы карту не строим
        if not self._begun:
            self._mapper.begin_session()
            self._begun = True

        pose = est.pose
        scan = self._robot.lidar.read_scan()
        self._mapper.ingest_scan(scan=scan, pose_xytheta=(pose.x_m, pose.y_m, pose.theta_rad))
        built = self._mapper.current_map()

        # Recovery: застряли — отъезжаем назад с поворотом, чтобы выбраться из угла.
        if self._recover > 0:
            self._recover -= 1
            self._command(Twist2D(linear_x_m_s=-0.35, angular_z_rad_s=self._recover_turn))
            return None

        moving = abs(self._last_cmd_v) > 0.05
        if moving and self._last_xy is not None and pose.point.distance_to(self._last_xy) < 0.01:
            self._stuck += 1
        else:
            self._stuck = 0
        self._last_xy = pose.point
        if self._stuck > self._stuck_limit:  # застряли — в чёрный список и отъезд
            if self._goal is not None:
                self._blacklist.add(built.meta.world_to_cell(self._goal.point))
            self._goal, self._stuck, self._path = None, 0, None
            self._recover, self._recover_turn = 35, -self._recover_turn
            return None

        reached = self._goal is not None and pose.point.distance_to(self._goal.point) <= 0.35
        self._timer += 1
        if self._goal is None or reached or self._timer >= self._replan_every:
            self._timer = 0
            cell = nearest_frontier(
                built, built.meta.world_to_cell(pose.point),
                min_cells=self._min_cells, clear_rad=self._clear_rad, blacklist=self._blacklist,
            )
            if cell is None:  # границ нет — пространство разведано
                self._stop()
                if self._map_store is not None:
                    self._map_store.save(label=self._map_label, grid=built)  # карта переживёт перезапуск
                return RobotMode.IDLE
            wp = built.meta.cell_to_world(*cell)
            self._goal = Pose2D(x_m=wp.x_m, y_m=wp.y_m, theta_rad=0.0)
            result = self._planner.plan(
                grid=built, start=pose, goal=self._goal, robot_radius_m=self._radius, best_effort=True
            )
            self._path = result.path if isinstance(result, PlanOk) else None
            if self._path is not None:
                self._follower.set_path(path=self._path)

        if self._path is not None:
            self._command(self._follower.compute_command(pose=pose, scan=scan))
        else:
            self._stop()
        return None

    def _command(self, twist: Twist2D) -> None:
        self._robot.motion.command(twist=twist)
        self._last_cmd_v = twist.linear_x_m_s

    def _stop(self) -> None:
        self._robot.motion.stop()
        self._last_cmd_v = 0.0
