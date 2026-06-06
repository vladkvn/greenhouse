"""Тесты цикла управления: робот в Idle стоит, телеметрия идёт, лидар конечный."""

from __future__ import annotations

import math

from greenhouse.adapters.sim.loop import build_sim_robot, run
from greenhouse.adapters.sim.worlds import greenhouse_rows_world
from greenhouse.orchestration.modes import RobotMode


def test_idle_robot_stays_put() -> None:
    world = greenhouse_rows_world()
    robot = build_sim_robot(world=world, start_x_m=0.5, start_y_m=1.0)
    statuses = run(robot, ticks=50, dt_s=0.1)
    assert len(statuses) == 50
    assert robot.state.x_m == 0.5
    assert robot.state.y_m == 1.0


def test_telemetry_stream_well_formed() -> None:
    world = greenhouse_rows_world()
    robot = build_sim_robot(world=world, start_x_m=0.5, start_y_m=1.0)
    statuses = run(robot, ticks=10, dt_s=0.1)
    for st in statuses:
        assert st.robot_id == "robot-01"
        assert st.mode is RobotMode.IDLE
        assert st.pose is not None
        assert 0.0 <= st.battery_frac <= 1.0
        assert st.last_error is None


def test_battery_drains_over_time() -> None:
    world = greenhouse_rows_world()
    robot = build_sim_robot(world=world)
    run(robot, ticks=100, dt_s=0.1)
    assert robot.state.battery_frac < 1.0


def test_lidar_returns_finite_ranges_inside_world() -> None:
    world = greenhouse_rows_world()
    robot = build_sim_robot(world=world, start_x_m=0.5, start_y_m=1.0)
    scan = robot.lidar.read_scan()
    finite = [r for r in scan.ranges_m if math.isfinite(r)]
    assert len(finite) > 0  # внутри теплицы стены видны хотя бы по части лучей
    assert all(r <= scan.range_max_m for r in finite)
