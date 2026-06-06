"""Тесты планирования и навигации: A* по сетке (с инфляцией и обходом препятствий)
и доводка робота из A в B по построенной карте через SimGoalNavigator."""

from __future__ import annotations

from greenhouse.adapters.sim.loop import build_sim_robot
from greenhouse.adapters.sim.navigation import SimGoalNavigator
from greenhouse.adapters.sim.worlds import empty_room, grid_meta_for_world
from greenhouse.domain.errors import Failure
from greenhouse.domain.geometry import Point2D, Pose2D, Twist2D
from greenhouse.domain.grid import CellState, MapMeta, OccupancyGrid
from greenhouse.navigation.mapping import EvidenceGridMapper
from greenhouse.navigation.planning import AStarPlanner, Path, PlanOk
from greenhouse.orchestration.modes import RobotMode


def _free_grid(width_px: int, height_px: int, *, resolution_m: float = 0.5) -> OccupancyGrid:
    meta = MapMeta(
        resolution_m=resolution_m,
        width_px=width_px,
        height_px=height_px,
        origin=Point2D(x_m=0.0, y_m=0.0),
    )
    return OccupancyGrid(meta=meta, cells=[CellState.FREE] * (width_px * height_px))


def _set_occupied(grid: OccupancyGrid, x_m: float, y_m: float) -> None:
    row, col = grid.meta.world_to_cell(Point2D(x_m=x_m, y_m=y_m))
    grid.cells[row * grid.meta.width_px + col] = CellState.OCCUPIED


def _at(grid: OccupancyGrid, pose: Pose2D) -> CellState:
    row, col = grid.meta.world_to_cell(pose.point)
    return grid.at(row, col)


def _path_length(path: Path) -> float:
    pts = path.waypoints
    return sum(pts[i].point.distance_to(pts[i + 1].point) for i in range(len(pts) - 1))


def _pose(x_m: float, y_m: float) -> Pose2D:
    return Pose2D(x_m=x_m, y_m=y_m, theta_rad=0.0)


def test_plan_straight_line_on_free_grid() -> None:
    grid = _free_grid(20, 12)
    result = AStarPlanner().plan(
        grid=grid, start=_pose(0.5, 0.5), goal=_pose(9.0, 5.0), robot_radius_m=0.0
    )
    assert isinstance(result, PlanOk)
    wp = result.path.waypoints
    assert wp[0].point.distance_to(Point2D(x_m=0.5, y_m=0.5)) < 1e-6
    assert wp[-1].point.distance_to(Point2D(x_m=9.0, y_m=5.0)) < 1e-6


def test_plan_fails_when_goal_occupied() -> None:
    grid = _free_grid(20, 12)
    _set_occupied(grid, 9.0, 5.0)
    result = AStarPlanner().plan(
        grid=grid, start=_pose(0.5, 0.5), goal=_pose(9.0, 5.0), robot_radius_m=0.0
    )
    assert isinstance(result, Failure)
    assert result.code.value == "planner_failed"


def test_plan_routes_around_obstacle() -> None:
    # Вертикальная стена x=5 перекрывает прямую; проход оставлен сверху (y>=5).
    grid = _free_grid(20, 12)
    for k in range(10):  # y от 0 до 4.5 — стена; выше свободно
        _set_occupied(grid, 5.0, k * 0.5)
    result = AStarPlanner().plan(
        grid=grid, start=_pose(1.0, 1.0), goal=_pose(9.0, 1.0), robot_radius_m=0.0
    )
    assert isinstance(result, PlanOk)
    # Путь длиннее прямой (был обход) и ни один узел не попал в занятую ячейку.
    assert _path_length(result.path) > _pose(1.0, 1.0).point.distance_to(Point2D(x_m=9.0, y_m=1.0))
    assert all(_at(grid, wp) is not CellState.OCCUPIED for wp in result.path.waypoints)


def test_inflation_blocks_gap_too_narrow_for_robot() -> None:
    # Узкий проход в один свободный ряд между двумя стенами.
    def gapped() -> OccupancyGrid:
        g = _free_grid(20, 12)
        for k in range(12):
            y = k * 0.5
            if abs(y - 3.0) > 0.6:  # оставляем свободным только ряд около y=3
                _set_occupied(g, 5.0, y)
        return g

    planner = AStarPlanner()
    thin = planner.plan(grid=gapped(), start=_pose(1.0, 3.0), goal=_pose(9.0, 3.0), robot_radius_m=0.1)
    fat = planner.plan(grid=gapped(), start=_pose(1.0, 3.0), goal=_pose(9.0, 3.0), robot_radius_m=0.6)
    assert isinstance(thin, PlanOk)       # узкому роботу проход есть
    assert isinstance(fat, Failure)       # широкому инфляция перекрывает зазор


def test_best_effort_approaches_unreachable_goal() -> None:
    # Цель в занятой ячейке: best_effort ведёт к ближайшей свободной рядом, а не Failure.
    grid = _free_grid(20, 12)
    _set_occupied(grid, 9.0, 5.0)
    planner = AStarPlanner()

    strict = planner.plan(grid=grid, start=_pose(1.0, 1.0), goal=_pose(9.0, 5.0), robot_radius_m=0.0)
    assert isinstance(strict, Failure)  # по умолчанию — отказ

    soft = planner.plan(
        grid=grid, start=_pose(1.0, 1.0), goal=_pose(9.0, 5.0), robot_radius_m=0.0, best_effort=True
    )
    assert isinstance(soft, PlanOk)
    stop = soft.path.waypoints[-1]
    assert _at(grid, stop) is not CellState.OCCUPIED          # сам не встал в занятую ячейку
    assert stop.point.distance_to(Point2D(x_m=9.0, y_m=5.0)) < 1.0  # но рядом с целью


def test_navigate_reaches_goal_on_built_map() -> None:
    # Критерий инкремента: робот доезжает из A в B по построенной карте.
    world = empty_room(10.0, 6.0)
    mapper = EvidenceGridMapper(meta=grid_meta_for_world(world, resolution_m=0.1))
    robot = build_sim_robot(world=world, start_x_m=2.0, start_y_m=3.0, map_builder=mapper)

    # Сначала строим карту проездом, затем планируем по ней.
    robot.mode = RobotMode.MAPPING
    robot.motion.command(twist=Twist2D(linear_x_m_s=0.6, angular_z_rad_s=0.0))
    for _ in range(120):
        robot.tick(dt_s=0.1)
    grid = mapper.end_session()

    # Возвращаем робота в исходную точку и навигируем к цели.
    robot.state.x_m, robot.state.y_m, robot.state.theta_rad = 2.0, 3.0, 0.0
    nav = SimGoalNavigator(robot=robot, grid=grid, robot_radius_m=0.25, goal_tol_m=0.2)
    outcome = nav.navigate_to(goal=_pose(8.0, 3.0))

    assert not isinstance(outcome, Failure)
    assert outcome.kind == "reached"
    assert outcome.final_pose_error_m is not None and outcome.final_pose_error_m <= 0.25
    assert robot.mode is RobotMode.IDLE


def test_navigate_unreachable_goal_returns_failure() -> None:
    world = empty_room(10.0, 6.0)
    grid = EvidenceGridMapper(meta=grid_meta_for_world(world)).current_map()
    # Делаем карту полностью занятой → любая цель недостижима.
    grid.cells[:] = [CellState.OCCUPIED] * len(grid.cells)
    robot = build_sim_robot(world=world, start_x_m=2.0, start_y_m=3.0)
    nav = SimGoalNavigator(robot=robot, grid=grid)

    outcome = nav.navigate_to(goal=_pose(8.0, 3.0))
    assert isinstance(outcome, Failure)
    assert robot.mode is RobotMode.IDLE
