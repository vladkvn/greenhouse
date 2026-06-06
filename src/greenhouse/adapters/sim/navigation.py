"""SimGoalNavigator — реализация GoalNavigator поверх симуляции.

Связывает глобальный планировщик (A* по карте) и локальный следователь пути, затем
крутит цикл управления до достижения цели. Препятствия, которых нет на карте, лидар
видит на ходу: навигатор сливает их в рабочую копию сетки занятости и периодически
перепланирует маршрут — глобальный планировщик надёжно строит обход (та же единая модель
занятости, что и в остальном ядре). Реактивный слой сглаживает следование. Эта же
структура (локальный костмап + перепланирование) переносится на реальный стек/ROS 2.
"""

from __future__ import annotations

import math

from greenhouse.adapters.sim.loop import SimRobot
from greenhouse.domain.errors import Failure, FailureCode
from greenhouse.domain.geometry import Point2D, Pose2D
from greenhouse.domain.grid import CellState, OccupancyGrid
from greenhouse.navigation.planning import (
    AStarPlanner,
    GlobalPlanner,
    NavOutcome,
    NavResult,
    PlanOk,
    PlanResult,
    ReactiveLocalPlanner,
)
from greenhouse.orchestration.modes import RobotMode


class SimGoalNavigator:
    """Реализует `greenhouse.navigation.GoalNavigator` для симулированного робота."""

    def __init__(
        self,
        *,
        robot: SimRobot,
        grid: OccupancyGrid,
        robot_radius_m: float = 0.3,
        planner: GlobalPlanner | None = None,
        goal_tol_m: float = 0.2,
        dt_s: float = 0.1,
        max_ticks: int = 3000,
        replan_every: int = 8,
    ) -> None:
        self._robot = robot
        self._grid = grid
        self._radius = robot_radius_m
        self._planner = planner or AStarPlanner()
        self._local = ReactiveLocalPlanner(
            robot_radius_m=robot_radius_m,
            max_linear_m_s=robot.motion.limits.max_linear_m_s,
            max_angular_rad_s=robot.motion.limits.max_angular_rad_s,
            goal_tol_m=goal_tol_m,
        )
        self._goal_tol = goal_tol_m
        self._dt = dt_s
        self._max_ticks = max_ticks
        self._replan_every = replan_every
        # Рабочая копия карты: база + наблюдаемые на ходу препятствия.
        self._cells = list(grid.cells)

    def navigate_to(self, *, goal: Pose2D) -> NavResult:
        result = self._plan_to(goal)
        if not isinstance(result, PlanOk):
            return result  # Failure от планировщика пробрасываем как есть
        self._local.set_path(path=result.path)
        self._robot.local_planner = self._local
        self._robot.mode = RobotMode.NAVIGATING

        for t in range(self._max_ticks):
            self._fuse_scan()  # подмешать в карту то, что видит лидар
            if t > 0 and t % self._replan_every == 0:
                replanned = self._plan_to(goal)
                if isinstance(replanned, PlanOk):
                    self._local.set_path(path=replanned.path)
            self._robot.tick(dt_s=self._dt)
            if self._local.is_goal_reached(pose=self._robot.state.pose()):
                self._stop()
                error = self._robot.state.pose().point.distance_to(goal.point)
                return NavOutcome(kind="reached", final_pose_error_m=error)

        self._stop()
        return Failure(
            code=FailureCode.OBSTACLE_BLOCKED,
            message="цель не достигнута за отведённые тики (застрял?)",
        )

    def cancel(self) -> None:
        self._stop()

    def _plan_to(self, goal: Pose2D) -> PlanResult:
        grid = OccupancyGrid(meta=self._grid.meta, cells=self._cells)
        return self._planner.plan(
            grid=grid, start=self._robot.state.pose(), goal=goal,
            robot_radius_m=self._radius, best_effort=True,
        )

    def _fuse_scan(self) -> None:
        """Отметить занятыми ячейки, куда сейчас попадает луч лидара (видимые препятствия)."""
        scan = self._robot.lidar.read_scan()
        pose = self._robot.state.pose()
        meta = self._grid.meta
        for i, r in enumerate(scan.ranges_m):
            if not math.isfinite(r) or r >= scan.range_max_m:
                continue
            angle = pose.theta_rad + scan.angle_min_rad + i * scan.angle_increment_rad
            hit = Point2D(x_m=pose.x_m + r * math.cos(angle), y_m=pose.y_m + r * math.sin(angle))
            row, col = meta.world_to_cell(hit)
            if meta.in_bounds(row, col):
                self._cells[row * meta.width_px + col] = CellState.OCCUPIED

    def _stop(self) -> None:
        self._robot.motion.stop()
        self._robot.mode = RobotMode.IDLE
