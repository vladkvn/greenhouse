"""Тесты зарядки: при низком заряде робот сам едет на док и заряжается до полного;
движок заряжает на доке и разряжает вне него."""

from __future__ import annotations

from greenhouse.adapters.sim.charging import SimChargeController
from greenhouse.adapters.sim.loop import build_sim_robot
from greenhouse.adapters.sim.worlds import empty_room, grid_meta_for_world
from greenhouse.domain.geometry import Point2D
from greenhouse.domain.grid import CellState, OccupancyGrid


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


def test_engine_charges_on_dock_and_drains_off_dock() -> None:
    world = empty_room(10.0, 6.0)
    dock = Point2D(x_m=2.0, y_m=2.0)
    # На доке заряд растёт.
    on = build_sim_robot(world=world, start_x_m=2.0, start_y_m=2.0, dock=dock)
    on.state.battery_frac = 0.5
    for _ in range(20):
        on.engine.step(dt_s=0.1)
    assert on.state.battery_frac > 0.5

    # Вне дока — падает.
    off = build_sim_robot(world=world, start_x_m=8.0, start_y_m=4.0, dock=dock)
    off.state.battery_frac = 0.5
    for _ in range(20):
        off.engine.step(dt_s=0.1)
    assert off.state.battery_frac < 0.5


def test_low_battery_triggers_drive_to_dock_and_recharge() -> None:
    # Критерий инкремента: при низком заряде робот сам едет на зарядку.
    world = empty_room(10.0, 6.0)
    grid = _free_grid(world)
    dock = Point2D(x_m=8.0, y_m=4.5)
    robot = build_sim_robot(world=world, start_x_m=1.5, start_y_m=1.5, dock=dock)
    robot.state.battery_frac = 0.25  # ниже порога

    ctrl = SimChargeController(
        robot=robot, grid=grid, dock=dock, robot_radius_m=0.25, low_battery=0.3, full_battery=0.95
    )
    assert ctrl.needs_charge()

    assert ctrl.charge_cycle() is True
    assert robot.state.battery_frac >= 0.95
    assert robot.state.pose().point.distance_to(dock) <= 0.3   # доехал до дока
    assert not ctrl.needs_charge()


def test_full_battery_does_not_need_charge() -> None:
    world = empty_room(10.0, 6.0)
    grid = _free_grid(world)
    dock = Point2D(x_m=8.0, y_m=4.5)
    robot = build_sim_robot(world=world, start_x_m=1.5, start_y_m=1.5, dock=dock)
    robot.state.battery_frac = 0.9
    ctrl = SimChargeController(robot=robot, grid=grid, dock=dock, low_battery=0.3)
    assert not ctrl.needs_charge()
