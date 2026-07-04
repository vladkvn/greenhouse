"""Тесты ChargingBehavior (инкр.8) через интегрированный путь оркестратора.

Команда GoCharge (или автоматический preempt по низкому заряду в Supervisor) переводит в
CHARGING; поведение доезжает на док тем же StepwiseNavigator, стоит и заряжается до полного,
затем запрашивает IDLE. На доке движок симуляции восполняет заряд.
"""

from __future__ import annotations

from greenhouse.adapters.sim.loop import build_sim_robot
from greenhouse.adapters.sim.worlds import empty_room, grid_meta_for_world
from greenhouse.domain.geometry import Point2D, Pose2D
from greenhouse.domain.grid import CellState, OccupancyGrid
from greenhouse.navigation.navigator import StepwiseNavigator
from greenhouse.orchestration import (
    BehaviorRegistry,
    ChargingBehavior,
    GoCharge,
    IdleBehavior,
    Orchestrator,
    RobotMode,
    Supervisor,
)


def _free_grid(world) -> OccupancyGrid:
    meta = grid_meta_for_world(world)
    cells = [
        CellState.OCCUPIED
        if world.min_clearance(point=meta.cell_to_world(r, c)) < meta.resolution_m
        else CellState.FREE
        for r in range(meta.height_px)
        for c in range(meta.width_px)
    ]
    return OccupancyGrid(meta=meta, cells=cells)


def _charging_robot(*, battery: float):
    world = empty_room(10.0, 6.0)
    dock = Point2D(x_m=8.0, y_m=4.5)
    robot = build_sim_robot(world=world, start_x_m=1.5, start_y_m=1.5, dock=dock)
    robot.state.battery_frac = battery
    nav = StepwiseNavigator(robot=robot, grid=_free_grid(world), robot_radius_m=0.25, goal_tol_m=0.25)
    registry = BehaviorRegistry()
    registry.register(IdleBehavior(robot=robot))
    registry.register(ChargingBehavior(
        robot=robot, navigator=nav, dock=Pose2D(x_m=8.0, y_m=4.5, theta_rad=0.0), full_battery_frac=0.95,
    ))
    robot.orchestrator = Orchestrator(
        robot=robot, registry=registry, supervisor=Supervisor(low_battery_frac=0.3)
    )
    return robot, dock


def _run_until_idle(robot, *, max_ticks: int = 3000) -> bool:
    for _ in range(max_ticks):
        robot.tick(dt_s=0.1)
        if robot.mode is RobotMode.IDLE:
            return True
    return False


def test_go_charge_drives_to_dock_and_charges_full() -> None:
    robot, dock = _charging_robot(battery=0.5)
    assert robot.orchestrator is not None
    robot.orchestrator.handle(command=GoCharge())
    assert robot.mode is RobotMode.CHARGING

    assert _run_until_idle(robot), "не доехал/не зарядился"
    assert robot.state.battery_frac >= 0.95
    assert robot.state.pose().point.distance_to(dock) <= 0.3


def test_low_battery_auto_charges_via_supervisor() -> None:
    # Никакой команды — Supervisor сам форсит CHARGING при заряде ниже порога.
    robot, dock = _charging_robot(battery=0.25)
    assert robot.mode is RobotMode.IDLE
    robot.tick(dt_s=0.1)                       # первый тик: supervisor preempt -> CHARGING
    assert robot.mode is RobotMode.CHARGING

    assert _run_until_idle(robot)
    assert robot.state.battery_frac >= 0.95
    assert robot.state.pose().point.distance_to(dock) <= 0.3
