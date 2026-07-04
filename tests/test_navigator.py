"""Тесты StepwiseNavigator (инкремент 3): неблокирующая навигация к цели.

Гарнес повторяет интегрированный тик робота вручную (sensing → localizer.update →
navigator.step → engine.step), не трогая SimRobot.tick. Проверяется: доезд до цели с
учётом габарита, недостижимая цель → Failure, отмена, безопасное поведение при
неопределённой позе. Объезд непанесённых препятствий обеспечивает переиспользуемый
ReactiveLocalPlanner (его собственные тесты — в test_obstacle_avoidance).
"""

from __future__ import annotations

from greenhouse.adapters.sim.loop import SimRobot, build_sim_robot
from greenhouse.adapters.sim.worlds import empty_room
from greenhouse.domain.errors import Failure
from greenhouse.domain.geometry import Point2D, Pose2D
from greenhouse.domain.grid import CellState, MapMeta, OccupancyGrid
from greenhouse.navigation.navigator import StepwiseNavigator
from greenhouse.navigation.planning import NavOutcome, PlanOk


def _free_grid(width_px: int, height_px: int, *, resolution_m: float = 0.5) -> OccupancyGrid:
    meta = MapMeta(
        resolution_m=resolution_m, width_px=width_px, height_px=height_px,
        origin=Point2D(x_m=0.0, y_m=0.0),
    )
    return OccupancyGrid(meta=meta, cells=[CellState.FREE] * (width_px * height_px))


def _pose(x_m: float, y_m: float) -> Pose2D:
    return Pose2D(x_m=x_m, y_m=y_m, theta_rad=0.0)


def _localize(robot: SimRobot) -> None:
    """Один шаг восприятия+локализации (обновляет localizer.latest())."""
    scan = robot.lidar.read_scan()
    odom = robot.odometry.read_odometry()
    robot.localizer.update(scan=scan, odometry=odom)


def _drive(robot: SimRobot, nav: StepwiseNavigator, *, max_iter: int = 800, dt_s: float = 0.1):
    for _ in range(max_iter):
        _localize(robot)            # latest() = текущая поза для этого шага
        outcome = nav.step()        # навигатор выдаёт команду в привод
        robot.engine.step(dt_s=dt_s)  # физика применяет команду
        if outcome is not None:
            return outcome
    return None


def test_navigator_reaches_goal() -> None:
    world = empty_room(10.0, 6.0)
    robot = build_sim_robot(world=world, start_x_m=2.0, start_y_m=3.0)
    grid = _free_grid(20, 12)
    nav = StepwiseNavigator(robot=robot, grid=grid, robot_radius_m=0.25, goal_tol_m=0.2)

    _localize(robot)                         # поза должна быть известна до begin()
    assert isinstance(nav.begin(goal=_pose(8.0, 3.0)), PlanOk)
    assert nav.is_active
    outcome = _drive(robot, nav)

    assert isinstance(outcome, NavOutcome)
    assert outcome.kind == "reached"
    assert outcome.final_pose_error_m is not None and outcome.final_pose_error_m <= 0.3
    assert not nav.is_active                  # по достижении навигатор деактивирован
    assert robot.state.last_cmd.linear_x_m_s == 0.0  # привод остановлен


def test_navigator_already_at_goal_reaches_immediately() -> None:
    world = empty_room(10.0, 6.0)
    robot = build_sim_robot(world=world, start_x_m=4.0, start_y_m=3.0)
    nav = StepwiseNavigator(robot=robot, grid=_free_grid(20, 12), goal_tol_m=0.3)
    _localize(robot)
    nav.begin(goal=_pose(4.0, 3.0))
    _localize(robot)
    outcome = nav.step()
    assert isinstance(outcome, NavOutcome) and outcome.kind == "reached"


def test_navigator_unreachable_goal_fails_at_begin() -> None:
    world = empty_room(10.0, 6.0)
    robot = build_sim_robot(world=world, start_x_m=2.0, start_y_m=3.0)
    grid = _free_grid(20, 12)
    grid.cells[:] = [CellState.OCCUPIED] * len(grid.cells)  # всё занято
    nav = StepwiseNavigator(robot=robot, grid=grid)
    _localize(robot)
    assert isinstance(nav.begin(goal=_pose(8.0, 3.0)), Failure)
    assert not nav.is_active
    assert nav.step() is None  # неактивный навигатор не двигает привод


def test_navigator_without_pose_does_not_drive() -> None:
    world = empty_room(10.0, 6.0)
    robot = build_sim_robot(world=world, start_x_m=2.0, start_y_m=3.0)
    nav = StepwiseNavigator(robot=robot, grid=_free_grid(20, 12))
    # localizer.latest() ещё None (update не вызывали) -> begin не может спланировать.
    assert isinstance(nav.begin(goal=_pose(8.0, 3.0)), Failure)
    assert not nav.is_active


def test_navigator_cancel_stops_and_deactivates() -> None:
    world = empty_room(10.0, 6.0)
    robot = build_sim_robot(world=world, start_x_m=2.0, start_y_m=3.0)
    nav = StepwiseNavigator(robot=robot, grid=_free_grid(20, 12))
    _localize(robot)
    nav.begin(goal=_pose(8.0, 3.0))
    _localize(robot)
    nav.step()
    nav.cancel()
    assert not nav.is_active
    assert robot.state.last_cmd.linear_x_m_s == 0.0
