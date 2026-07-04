"""StepwiseNavigator — навигация к цели одним шагом за тик (неблокирующая).

Та же связка, что у блокирующего SimGoalNavigator (глобальный A* + реактивный объезд +
домешивание видимых лидаром препятствий + периодическое перепланирование), но БЕЗ
собственного цикла: `begin(goal)` строит маршрут, каждый `step()` делает один тик и сам
выдаёт команду в привод, читая позу из `localizer.latest()`. Это позволяет ОДНОМУ
навигатору обслуживать GoTo, Following.GOTO_LAST_SEEN и Charging.GOTO_DOCK — объезд и учёт
габарита достаются всем трём, а режим Following может во время следования породить под-цель
навигации (чего блокирующий фасад не позволяет).

Габарит робота учитывается инфляцией препятствий в A* и стоп-зазором реактивного слоя.
Блокирующий фасад SimGoalNavigator оставлен как есть для обратной совместимости тестов.
"""

from __future__ import annotations

import math

from greenhouse.domain.errors import Failure, FailureCode
from greenhouse.domain.geometry import Point2D, Pose2D
from greenhouse.domain.grid import CellState, OccupancyGrid
from greenhouse.navigation.keepout import KeepoutRegistry
from greenhouse.navigation.planning import (
    AStarPlanner,
    GlobalPlanner,
    NavOutcome,
    NavResult,
    PlanOk,
    PlanResult,
    ReactiveLocalPlanner,
)
from greenhouse.navigation.protocols import RobotLike
from greenhouse.sensing.interfaces import LidarScan


class StepwiseNavigator:
    """Пошаговая навигация к цели: begin/step/cancel. Водит привод напрямую."""

    def __init__(
        self,
        *,
        robot: RobotLike,
        grid: OccupancyGrid,
        robot_radius_m: float = 0.25,
        planner: GlobalPlanner | None = None,
        keepout: KeepoutRegistry | None = None,
        goal_tol_m: float = 0.2,
        replan_every: int = 8,
        max_steps: int = 3000,
    ) -> None:
        self._robot = robot
        self._grid = grid
        self._radius = robot_radius_m
        self._keepout = keepout
        self._planner = planner or AStarPlanner()
        self._local = ReactiveLocalPlanner(
            robot_radius_m=robot_radius_m,
            max_linear_m_s=robot.motion.limits.max_linear_m_s,
            max_angular_rad_s=robot.motion.limits.max_angular_rad_s,
            goal_tol_m=goal_tol_m,
        )
        self._replan_every = replan_every
        self._max_steps = max_steps
        self._cells = list(grid.cells)  # рабочая копия: карта + видимые на ходу препятствия
        self._goal: Pose2D | None = None
        self._steps = 0
        self._active = False

    @property
    def is_active(self) -> bool:
        return self._active

    @property
    def goal(self) -> Pose2D | None:
        return self._goal

    # ---------------------------------------------------------------- управление
    def begin(self, *, goal: Pose2D) -> PlanResult:
        """Спланировать маршрут к цели. Активирует step(), если план построен."""
        self._goal = goal
        self._cells = list(self._grid.cells)
        self._steps = 0
        start = self._current_pose()
        if start is None:
            self._active = False
            return Failure(code=FailureCode.PLANNER_FAILED, message="поза ещё не определена")
        result = self._plan_to(goal, start)
        if isinstance(result, PlanOk):
            self._local.set_path(path=result.path)
            self._active = True
        else:
            self._active = False
        return result

    def step(self) -> NavResult | None:
        """Один тик навигации. None — едем дальше; NavOutcome/Failure — навигация завершена."""
        if not self._active or self._goal is None:
            return None
        pose = self._current_pose()
        if pose is None:
            return None  # потеряли позу — ждём, движение не выдаём (привод не трогаем)

        if self._local.is_goal_reached(pose=pose):
            self._stop()
            return NavOutcome(kind="reached", final_pose_error_m=pose.point.distance_to(self._goal.point))
        if self._steps >= self._max_steps:
            self._stop()
            return Failure(code=FailureCode.OBSTACLE_BLOCKED, message="не доехал за отведённые шаги")

        scan: LidarScan = self._robot.lidar.read_scan()
        self._fuse_scan(pose, scan)
        if self._steps > 0 and self._steps % self._replan_every == 0:
            replanned = self._plan_to(self._goal, pose)
            if isinstance(replanned, PlanOk):
                self._local.set_path(path=replanned.path)
        self._steps += 1
        self._robot.motion.command(twist=self._local.compute_command(pose=pose, scan=scan))
        return None

    def cancel(self) -> None:
        self._active = False
        self._robot.motion.stop()

    # ------------------------------------------------------------------- внутр.
    def _current_pose(self) -> Pose2D | None:
        est = self._robot.localizer.latest()
        if est is None or est.is_lost:
            return None
        return est.pose

    def _plan_to(self, goal: Pose2D, start: Pose2D) -> PlanResult:
        grid = OccupancyGrid(meta=self._grid.meta, cells=self._cells)
        cost_layer = None
        if self._keepout is not None:
            grid = self._keepout.apply_to(grid=grid)  # KEEPOUT-зоны → занятые ячейки
            cost_layer = self._keepout.cost_layer(meta=grid.meta)  # SLOW/PREFERRED
        return self._planner.plan(
            grid=grid, start=start, goal=goal,
            robot_radius_m=self._radius, best_effort=True, cost_layer=cost_layer,
        )

    def _fuse_scan(self, pose: Pose2D, scan: LidarScan) -> None:
        """Отметить занятыми ячейки, куда сейчас попадает луч лидара (видимые препятствия)."""
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
        self._active = False
        self._robot.motion.stop()
