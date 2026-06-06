"""Тесты реактивного объезда: ReactiveLocalPlanner уводит от препятствия по скану,
а робот доезжает до цели в обход коробки, которой нет на карте планирования."""

from __future__ import annotations

import math

from greenhouse.adapters.sim.loop import build_sim_robot
from greenhouse.adapters.sim.navigation import SimGoalNavigator
from greenhouse.adapters.sim.world import PolygonWorld, Segment
from greenhouse.adapters.sim.worlds import empty_room, grid_meta_for_world
from greenhouse.domain.errors import Failure
from greenhouse.domain.geometry import Point2D, Pose2D
from greenhouse.domain.grid import CellState, OccupancyGrid
from greenhouse.navigation.planning import Path, ReactiveLocalPlanner
from greenhouse.sensing.interfaces import LidarScan


def _rect(x0: float, y0: float, x1: float, y1: float) -> list[Segment]:
    p = [Point2D(x_m=x0, y_m=y0), Point2D(x_m=x1, y_m=y0),
         Point2D(x_m=x1, y_m=y1), Point2D(x_m=x0, y_m=y1)]
    return [Segment(a=p[i], b=p[(i + 1) % 4]) for i in range(4)]


def _world_with_box() -> PolygonWorld:
    """Пустая комната 10x6 с коробкой-препятствием по центру пути (вокруг (5,3))."""
    walls = list(empty_room(10.0, 6.0).walls) + _rect(4.6, 2.6, 5.4, 3.4)
    return PolygonWorld(
        walls=tuple(walls),
        bounds_min=Point2D(x_m=0.0, y_m=0.0),
        bounds_max=Point2D(x_m=10.0, y_m=6.0),
    )


def _free_grid_for(world: PolygonWorld) -> OccupancyGrid:
    """Карта планирования БЕЗ коробки: только периметр комнаты занят."""
    meta = grid_meta_for_world(empty_room(10.0, 6.0))
    cells: list[CellState] = []
    clean = empty_room(10.0, 6.0)
    for row in range(meta.height_px):
        for col in range(meta.width_px):
            center = meta.cell_to_world(row, col)
            occ = clean.min_clearance(point=center) < meta.resolution_m
            cells.append(CellState.OCCUPIED if occ else CellState.FREE)
    return OccupancyGrid(meta=meta, cells=cells)


def _scan(ranges: tuple[float, ...]) -> LidarScan:
    n = len(ranges)
    return LidarScan(
        angle_min_rad=0.0,
        angle_increment_rad=2.0 * math.pi / n,
        range_max_m=8.0,
        ranges_m=ranges,
        stamp_s=0.0,
    )


def _forward_obstacle_scan() -> LidarScan:
    """Скан: препятствие в ~0.4 м прямо по курсу (±15°), по бокам свободно."""
    n = 72
    inc = 2.0 * math.pi / n
    out = []
    for i in range(n):
        a = math.atan2(math.sin(i * inc), math.cos(i * inc))  # в [-pi, pi]
        out.append(0.4 if abs(a) <= math.radians(15.0) else 8.0)
    return _scan(tuple(out))


def test_reactive_steers_away_from_frontal_obstacle() -> None:
    planner = ReactiveLocalPlanner(robot_radius_m=0.25, max_linear_m_s=0.6)
    planner.set_path(path=Path(waypoints=(Pose2D(x_m=5.0, y_m=0.0, theta_rad=0.0),)))
    cmd = planner.compute_command(
        pose=Pose2D(x_m=0.0, y_m=0.0, theta_rad=0.0), scan=_forward_obstacle_scan()
    )
    # Цель прямо по курсу, но впереди преграда → планировщик доворачивает в сторону.
    assert abs(cmd.angular_z_rad_s) > 0.1


def test_reactive_goes_straight_when_clear() -> None:
    planner = ReactiveLocalPlanner(robot_radius_m=0.25, max_linear_m_s=0.6)
    planner.set_path(path=Path(waypoints=(Pose2D(x_m=5.0, y_m=0.0, theta_rad=0.0),)))
    cmd = planner.compute_command(
        pose=Pose2D(x_m=0.0, y_m=0.0, theta_rad=0.0), scan=_scan((8.0,) * 72)
    )
    # Всё свободно, цель по курсу → едем прямо.
    assert cmd.linear_x_m_s > 0.3
    assert abs(cmd.angular_z_rad_s) < 0.05


def test_robot_reaches_goal_around_unmapped_box() -> None:
    # Критерий инкремента: робот объезжает препятствие, которого нет на карте.
    world = _world_with_box()
    grid = _free_grid_for(world)  # карта не знает о коробке
    robot = build_sim_robot(world=world, start_x_m=2.0, start_y_m=3.0)
    nav = SimGoalNavigator(robot=robot, grid=grid, robot_radius_m=0.25, goal_tol_m=0.25, max_ticks=4000)

    outcome = nav.navigate_to(goal=Pose2D(x_m=8.0, y_m=3.0, theta_rad=0.0))

    # Чистый follower встал бы носом в коробку и не доехал; реактивный — объезжает.
    assert not isinstance(outcome, Failure)
    assert outcome.kind == "reached"
    assert robot.state.pose().point.distance_to(Point2D(x_m=8.0, y_m=3.0)) <= 0.3
    # И ни разу не въехал в коробку (движок держит зазор >= вписанного радиуса).
    assert world.min_clearance(point=robot.state.pose().point) >= robot.state.inscribed_radius_m - 1e-6
