"""Тесты NavigatingBehavior (инкр.7, навигация) через ИНТЕГРИРОВАННЫЙ путь.

Оркестратор подключён в SimRobot.tick (robot.orchestrator), команда GoTo переводит в
NAVIGATING, поведение доводит робота до точки тем же StepwiseNavigator и по достижении
запрашивает IDLE. Тик робота сам делает sensing+localization, поэтому планирование идёт
по свежей позе.
"""

from __future__ import annotations

from greenhouse.adapters.sim.loop import build_sim_robot
from greenhouse.adapters.sim.worlds import empty_room
from greenhouse.domain.geometry import Point2D, Pose2D
from greenhouse.domain.grid import CellState, MapMeta, OccupancyGrid
from greenhouse.navigation.navigator import StepwiseNavigator
from greenhouse.orchestration import (
    BehaviorRegistry,
    GoTo,
    IdleBehavior,
    NavigatingBehavior,
    Orchestrator,
    RobotMode,
)


def _free_grid(w: int, h: int, *, res: float = 0.5) -> OccupancyGrid:
    meta = MapMeta(resolution_m=res, width_px=w, height_px=h, origin=Point2D(x_m=0.0, y_m=0.0))
    return OccupancyGrid(meta=meta, cells=[CellState.FREE] * (w * h))


def _pose(x: float, y: float) -> Pose2D:
    return Pose2D(x_m=x, y_m=y, theta_rad=0.0)


def _orchestrated(grid: OccupancyGrid):
    robot = build_sim_robot(world=empty_room(10.0, 6.0), start_x_m=2.0, start_y_m=3.0)
    nav = StepwiseNavigator(robot=robot, grid=grid, robot_radius_m=0.25, goal_tol_m=0.25)
    registry = BehaviorRegistry()
    registry.register(IdleBehavior(robot=robot))
    registry.register(NavigatingBehavior(navigator=nav))
    robot.orchestrator = Orchestrator(robot=robot, registry=registry)
    return robot


def _run_until_idle(robot, *, max_ticks: int = 800) -> int:
    for i in range(max_ticks):
        robot.tick(dt_s=0.1)
        if robot.mode is RobotMode.IDLE:
            return i
    return -1


def test_goto_reaches_goal_then_idles() -> None:
    robot = _orchestrated(_free_grid(20, 12))
    assert robot.orchestrator is not None
    robot.orchestrator.handle(command=GoTo(goal=_pose(8.0, 3.0)))
    assert robot.mode is RobotMode.NAVIGATING

    assert _run_until_idle(robot) >= 0, "не доехал/не перешёл в IDLE"
    assert robot.mode is RobotMode.IDLE
    assert robot.state.pose().point.distance_to(Point2D(x_m=8.0, y_m=3.0)) <= 0.3
    assert robot.state.last_cmd.linear_x_m_s == 0.0


def test_goto_unreachable_falls_back_to_idle() -> None:
    grid = _free_grid(20, 12)
    grid.cells[:] = [CellState.OCCUPIED] * len(grid.cells)  # всё занято -> план не построить
    robot = _orchestrated(grid)
    assert robot.orchestrator is not None
    robot.orchestrator.handle(command=GoTo(goal=_pose(8.0, 3.0)))

    # За несколько тиков поведение обнаружит непостроимый план и вернёт IDLE.
    assert _run_until_idle(robot, max_ticks=10) >= 0
    assert robot.mode is RobotMode.IDLE
