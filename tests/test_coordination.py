"""Тесты координации флота: взаимное исключение на сегментах, атомарное резервирование
маршрута против тупиков, освобождение, и несколько роботов в одной симуляции."""

from __future__ import annotations

from greenhouse.adapters.sim.loop import build_sim_robot
from greenhouse.adapters.sim.navigation import SimGoalNavigator
from greenhouse.adapters.sim.worlds import empty_room, grid_meta_for_world
from greenhouse.coordination.coordinator import InMemoryTrafficCoordinator
from greenhouse.coordination.interfaces import ReservationGranted
from greenhouse.domain.errors import Failure
from greenhouse.domain.geometry import Point2D, Pose2D
from greenhouse.domain.grid import CellState, OccupancyGrid


def _free_grid() -> OccupancyGrid:
    world = empty_room(12.0, 6.0)
    meta = grid_meta_for_world(world)
    cells = [
        CellState.OCCUPIED
        if world.min_clearance(point=meta.cell_to_world(r, c)) < meta.resolution_m
        else CellState.FREE
        for r in range(meta.height_px)
        for c in range(meta.width_px)
    ]
    return OccupancyGrid(meta=meta, cells=cells)


def test_lane_mutual_exclusion_no_head_on() -> None:
    coord = InMemoryTrafficCoordinator()
    a = coord.request(robot_id="robot-A", segment_id="lane-1")
    b = coord.request(robot_id="robot-B", segment_id="lane-1")
    assert isinstance(a, ReservationGranted)
    assert not isinstance(b, ReservationGranted)  # второй робот в узкий ряд не пущен
    assert coord.holder_of(segment_id="lane-1") == "robot-A"

    coord.release(token=a.token)  # A вышел
    b2 = coord.request(robot_id="robot-B", segment_id="lane-1")
    assert isinstance(b2, ReservationGranted)  # liveness: B проезжает следом
    assert coord.holder_of(segment_id="lane-1") == "robot-B"


def test_request_route_atomic_prevents_deadlock() -> None:
    # Встречные маршруты с пересечением в обратном порядке — классический тупик.
    coord = InMemoryTrafficCoordinator()
    a = coord.request_route(robot_id="robot-A", segments=("s1", "s2"))
    assert isinstance(a, ReservationGranted)
    assert coord.holder_of(segment_id="s1") == "robot-A"
    assert coord.holder_of(segment_id="s2") == "robot-A"

    b = coord.request_route(robot_id="robot-B", segments=("s2", "s1"))
    assert not isinstance(b, ReservationGranted)  # B ждёт у входа, а не застревает
    # Атомарность: при отказе B не захватил ни одного сегмента.
    assert coord.holder_of(segment_id="s1") == "robot-A"
    assert coord.holder_of(segment_id="s2") == "robot-A"

    coord.release(token=a.token)
    b2 = coord.request_route(robot_id="robot-B", segments=("s2", "s1"))
    assert isinstance(b2, ReservationGranted)  # после освобождения тупика нет — B едет


def test_release_frees_whole_route() -> None:
    coord = InMemoryTrafficCoordinator()
    granted = coord.request_route(robot_id="robot-A", segments=("s1", "s2", "s3"))
    assert isinstance(granted, ReservationGranted)
    coord.release(token=granted.token)
    assert coord.holder_of(segment_id="s1") is None
    assert coord.holder_of(segment_id="s2") is None
    assert coord.holder_of(segment_id="s3") is None


def test_same_robot_request_is_idempotent() -> None:
    coord = InMemoryTrafficCoordinator()
    assert isinstance(coord.request(robot_id="r", segment_id="seg"), ReservationGranted)
    assert isinstance(coord.request(robot_id="r", segment_id="seg"), ReservationGranted)
    assert coord.holder_of(segment_id="seg") == "r"


def test_two_robots_navigate_in_one_simulation() -> None:
    # Несколько роботов в одном мире на разных проходах едут навстречу, не мешая друг другу.
    world = empty_room(12.0, 6.0)
    grid = _free_grid()
    a = build_sim_robot(world=world, robot_id="robot-A", start_x_m=2.0, start_y_m=2.0)
    b = build_sim_robot(world=world, robot_id="robot-B", start_x_m=10.0, start_y_m=4.0)
    nav_a = SimGoalNavigator(robot=a, grid=grid, robot_radius_m=0.25, goal_tol_m=0.3)
    nav_b = SimGoalNavigator(robot=b, grid=grid, robot_radius_m=0.25, goal_tol_m=0.3)

    out_a = nav_a.navigate_to(goal=Pose2D(x_m=10.0, y_m=2.0, theta_rad=0.0))
    out_b = nav_b.navigate_to(goal=Pose2D(x_m=2.0, y_m=4.0, theta_rad=0.0))

    assert not isinstance(out_a, Failure)
    assert not isinstance(out_b, Failure)
    assert a.state.pose().point.distance_to(Point2D(x_m=10.0, y_m=2.0)) <= 0.3
    assert b.state.pose().point.distance_to(Point2D(x_m=2.0, y_m=4.0)) <= 0.3
