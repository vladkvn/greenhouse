"""Тесты build_orchestrated_sim_robot — сквозная проверка композиции единого FSM.

Тот же реестр/оркестратор/поведения, что повторяет build_jetson_robot: полная миссия
(GoTo → следование → стоп) и проверка, что все режимы зарегистрированы и достижимы.
"""

from __future__ import annotations

from greenhouse.adapters.sim.builder import build_orchestrated_sim_robot
from greenhouse.adapters.sim.person import SimPerson
from greenhouse.adapters.sim.worlds import empty_room, grid_meta_for_world
from greenhouse.domain.geometry import Point2D, Pose2D
from greenhouse.domain.grid import CellState, MapMeta, OccupancyGrid
from greenhouse.navigation.mapping import EvidenceGridMapper
from greenhouse.orchestration import (
    EmergencyStop,
    FollowPerson,
    GoCharge,
    GoTo,
    RobotMode,
    StartMapping,
    Stop,
)


def _free_grid(w: int = 20, h: int = 12, *, res: float = 0.5) -> OccupancyGrid:
    meta = MapMeta(resolution_m=res, width_px=w, height_px=h, origin=Point2D(x_m=0.0, y_m=0.0))
    return OccupancyGrid(meta=meta, cells=[CellState.FREE] * (w * h))


def test_full_mission_goto_follow_stop() -> None:
    world = empty_room(10.0, 6.0)
    person = SimPerson(x_m=6.0, y_m=3.0)
    robot = build_orchestrated_sim_robot(
        world=world, grid=_free_grid(), start_x_m=2.0, start_y_m=3.0, person=person,
    )
    orch = robot.orchestrator
    assert orch is not None

    # 1) Доехать в точку -> вернуться в IDLE.
    orch.handle(command=GoTo(goal=Pose2D(x_m=8.0, y_m=3.0, theta_rad=0.0)))
    for _ in range(800):
        robot.tick(dt_s=0.1)
        if robot.mode is RobotMode.IDLE:
            break
    assert robot.mode is RobotMode.IDLE
    assert robot.state.pose().point.distance_to(Point2D(x_m=8.0, y_m=3.0)) <= 0.3

    # 2) Следовать -> приблизиться к человеку (стоит в 2 м позади).
    dist0 = robot.state.pose().point.distance_to(person.point())
    orch.handle(command=FollowPerson())
    for _ in range(120):
        robot.tick(dt_s=0.1)
    assert robot.mode is RobotMode.FOLLOWING
    assert robot.state.pose().point.distance_to(person.point()) < dist0 - 0.3  # подъехал ближе

    # 3) Стоп -> IDLE.
    orch.handle(command=Stop())
    assert robot.mode is RobotMode.IDLE


def test_all_modes_registered_and_reachable() -> None:
    world = empty_room(10.0, 6.0)
    robot = build_orchestrated_sim_robot(
        world=world, grid=_free_grid(), start_x_m=2.0, start_y_m=3.0,
        person=SimPerson(x_m=5.0, y_m=3.0), dock=Pose2D(x_m=8.0, y_m=4.5, theta_rad=0.0),
        mapper=EvidenceGridMapper(meta=grid_meta_for_world(world)),
    )
    orch = robot.orchestrator
    assert orch is not None
    goto = GoTo(goal=Pose2D(x_m=7.0, y_m=3.0, theta_rad=0.0))
    assert orch.handle(command=StartMapping()).mode is RobotMode.MAPPING
    assert orch.handle(command=goto).mode is RobotMode.NAVIGATING
    assert orch.handle(command=FollowPerson()).mode is RobotMode.FOLLOWING
    assert orch.handle(command=GoCharge()).mode is RobotMode.CHARGING
    assert orch.handle(command=EmergencyStop()).mode is RobotMode.IDLE
