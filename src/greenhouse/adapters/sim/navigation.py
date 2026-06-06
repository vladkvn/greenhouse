"""SimGoalNavigator — реализация GoalNavigator поверх симуляции.

Связывает глобальный планировщик (A* по карте) и локальный следователь пути, затем
крутит цикл управления робота до достижения цели или исчерпания бюджета тиков. Та же
структура, что и на железе: планируем по карте, ведём по пути, упираемся в лимиты.
"""

from __future__ import annotations

from greenhouse.adapters.sim.loop import SimRobot
from greenhouse.domain.errors import Failure, FailureCode
from greenhouse.domain.geometry import Pose2D
from greenhouse.domain.grid import OccupancyGrid
from greenhouse.navigation.planning import (
    AStarPlanner,
    GlobalPlanner,
    NavOutcome,
    NavResult,
    PlanOk,
    PurePursuitLocalPlanner,
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
    ) -> None:
        self._robot = robot
        self._grid = grid
        self._radius = robot_radius_m
        self._planner = planner or AStarPlanner()
        self._local = PurePursuitLocalPlanner(
            max_linear_m_s=robot.motion.limits.max_linear_m_s,
            max_angular_rad_s=robot.motion.limits.max_angular_rad_s,
            goal_tol_m=goal_tol_m,
        )
        self._goal_tol = goal_tol_m
        self._dt = dt_s
        self._max_ticks = max_ticks

    def navigate_to(self, *, goal: Pose2D) -> NavResult:
        start = self._robot.state.pose()
        result = self._planner.plan(
            grid=self._grid, start=start, goal=goal, robot_radius_m=self._radius
        )
        if not isinstance(result, PlanOk):
            return result  # Failure от планировщика пробрасываем как есть

        self._local.set_path(path=result.path)
        self._robot.local_planner = self._local
        self._robot.mode = RobotMode.NAVIGATING

        for _ in range(self._max_ticks):
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

    def _stop(self) -> None:
        self._robot.motion.stop()
        self._robot.mode = RobotMode.IDLE
