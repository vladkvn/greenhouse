"""SimChargeController — автономная зарядка: при низком заряде робот сам едет на док.

Политика проста и переносима: если заряд упал ниже порога — перейти в режим CHARGING,
доехать до док-точки (тем же планировщиком, что и обычная навигация), встать на док и
заряжаться, пока батарея не наполнится. На доке движок симуляции восполняет заряд.
"""

from __future__ import annotations

from greenhouse.adapters.sim.loop import SimRobot
from greenhouse.adapters.sim.navigation import SimGoalNavigator
from greenhouse.domain.errors import Failure
from greenhouse.domain.geometry import Point2D, Pose2D
from greenhouse.domain.grid import OccupancyGrid
from greenhouse.orchestration.modes import RobotMode


class SimChargeController:
    """Решает, когда ехать заряжаться, и доводит цикл зарядки до конца."""

    def __init__(
        self,
        *,
        robot: SimRobot,
        grid: OccupancyGrid,
        dock: Point2D,
        robot_radius_m: float = 0.25,
        low_battery: float = 0.3,
        full_battery: float = 0.95,
        dt_s: float = 0.1,
        max_ticks: int = 4000,
    ) -> None:
        self._robot = robot
        self._grid = grid
        self._dock = dock
        self._radius = robot_radius_m
        self._low = low_battery
        self._full = full_battery
        self._dt = dt_s
        self._max_ticks = max_ticks

    def needs_charge(self) -> bool:
        return self._robot.state.battery_frac < self._low

    def charge_cycle(self) -> bool:
        """Доехать до дока и зарядиться до полного. True — успех, False — не доехал."""
        nav = SimGoalNavigator(
            robot=self._robot, grid=self._grid, robot_radius_m=self._radius,
            goal_tol_m=0.25, dt_s=self._dt, max_ticks=self._max_ticks,
        )
        outcome = nav.navigate_to(
            goal=Pose2D(x_m=self._dock.x_m, y_m=self._dock.y_m, theta_rad=0.0)
        )
        if isinstance(outcome, Failure):
            return False

        self._robot.mode = RobotMode.CHARGING
        self._robot.motion.stop()
        for _ in range(self._max_ticks):
            if self._robot.state.battery_frac >= self._full:
                self._robot.mode = RobotMode.IDLE
                return True
            self._robot.engine.step(dt_s=self._dt)  # стоит на доке — заряжается
        self._robot.mode = RobotMode.IDLE
        return self._robot.state.battery_frac >= self._full
