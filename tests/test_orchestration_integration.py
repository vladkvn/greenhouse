"""Сквозные тесты единого FSM (трек A): переключение режимов командами через SimRobot.tick.

Покрывает то, чего нет в тестах отдельных поведений: переходы между режимами и
надрежимный preempt Supervisor по ходу движения. Робот целиком под оркестратором
(robot.orchestrator), команды подаёт «оператор».
"""

from __future__ import annotations

from greenhouse.adapters.sim.loop import build_sim_robot
from greenhouse.adapters.sim.person import SimPerson, SimPersonDetector
from greenhouse.adapters.sim.worlds import empty_room
from greenhouse.domain.geometry import Point2D, Pose2D
from greenhouse.domain.grid import CellState, MapMeta, OccupancyGrid
from greenhouse.navigation.following import PersonFollower
from greenhouse.navigation.navigator import StepwiseNavigator
from greenhouse.orchestration import (
    BehaviorRegistry,
    ChargingBehavior,
    EmergencyStop,
    FollowingBehavior,
    FollowPerson,
    GoTo,
    IdleBehavior,
    NavigatingBehavior,
    Orchestrator,
    RobotMode,
    StartMapping,
    Stop,
    Supervisor,
)


def _free_grid(w: int = 20, h: int = 12, *, res: float = 0.5) -> OccupancyGrid:
    meta = MapMeta(resolution_m=res, width_px=w, height_px=h, origin=Point2D(x_m=0.0, y_m=0.0))
    return OccupancyGrid(meta=meta, cells=[CellState.FREE] * (w * h))


def test_goto_then_stop_cancels_navigation() -> None:
    robot = build_sim_robot(world=empty_room(10.0, 6.0), start_x_m=2.0, start_y_m=3.0)
    nav = StepwiseNavigator(robot=robot, grid=_free_grid(), robot_radius_m=0.25, goal_tol_m=0.25)
    registry = BehaviorRegistry()
    registry.register(IdleBehavior(robot=robot))
    registry.register(NavigatingBehavior(navigator=nav))
    robot.orchestrator = Orchestrator(robot=robot, registry=registry)

    robot.orchestrator.handle(command=GoTo(goal=Pose2D(x_m=8.0, y_m=3.0, theta_rad=0.0)))
    for _ in range(30):
        robot.tick(dt_s=0.1)
    assert robot.mode is RobotMode.NAVIGATING
    assert robot.state.x_m > 2.3  # реально поехал

    robot.orchestrator.handle(command=Stop())  # оператор отменяет
    assert robot.mode is RobotMode.IDLE
    x_after = robot.state.x_m
    for _ in range(10):
        robot.tick(dt_s=0.1)
    assert abs(robot.state.x_m - x_after) < 0.05  # стоит на месте


def test_follow_then_emergency_stop() -> None:
    robot = build_sim_robot(world=empty_room(10.0, 6.0), start_x_m=2.0, start_y_m=3.0)
    person = SimPerson(x_m=6.0, y_m=3.0)
    detector = SimPersonDetector(
        world=empty_room(10.0, 6.0), robot_state=robot.state, person=person,
        clock=robot.clock, max_range_m=8.0,
    )
    nav = StepwiseNavigator(robot=robot, grid=_free_grid(), robot_radius_m=0.25)
    registry = BehaviorRegistry()
    registry.register(IdleBehavior(robot=robot))
    registry.register(FollowingBehavior(
        robot=robot, detector=detector, follower=PersonFollower(standoff_m=0.8, max_linear_m_s=1.0),
        navigator=nav, clock=robot.clock,
    ))
    robot.orchestrator = Orchestrator(robot=robot, registry=registry)

    robot.orchestrator.handle(command=FollowPerson())
    for _ in range(25):
        robot.tick(dt_s=0.1)
    assert robot.mode is RobotMode.FOLLOWING
    assert robot.state.x_m > 2.3  # поехал за человеком

    robot.orchestrator.handle(command=EmergencyStop())
    assert robot.mode is RobotMode.IDLE
    assert robot.state.last_cmd.linear_x_m_s == 0.0


def test_low_battery_preempts_active_navigation() -> None:
    robot = build_sim_robot(
        world=empty_room(10.0, 6.0), start_x_m=2.0, start_y_m=3.0, dock=Point2D(x_m=8.0, y_m=4.5)
    )
    robot.state.battery_frac = 0.25  # ниже порога
    nav = StepwiseNavigator(robot=robot, grid=_free_grid(), robot_radius_m=0.25)
    dock_nav = StepwiseNavigator(robot=robot, grid=_free_grid(), robot_radius_m=0.25)
    registry = BehaviorRegistry()
    registry.register(IdleBehavior(robot=robot))
    registry.register(NavigatingBehavior(navigator=nav))
    registry.register(ChargingBehavior(
        robot=robot, navigator=dock_nav, dock=Pose2D(x_m=8.0, y_m=4.5, theta_rad=0.0),
    ))
    robot.orchestrator = Orchestrator(
        robot=robot, registry=registry, supervisor=Supervisor(low_battery_frac=0.3)
    )

    robot.orchestrator.handle(command=GoTo(goal=Pose2D(x_m=8.0, y_m=3.0, theta_rad=0.0)))
    assert robot.mode is RobotMode.NAVIGATING
    robot.tick(dt_s=0.1)  # Supervisor видит низкий заряд -> форсит CHARGING поверх навигации
    assert robot.mode is RobotMode.CHARGING


def test_mode_switch_idle_navigate_idle_sequence() -> None:
    robot = build_sim_robot(world=empty_room(10.0, 6.0), start_x_m=2.0, start_y_m=3.0)
    nav = StepwiseNavigator(robot=robot, grid=_free_grid(), robot_radius_m=0.25, goal_tol_m=0.25)
    registry = BehaviorRegistry()
    registry.register(IdleBehavior(robot=robot))
    registry.register(NavigatingBehavior(navigator=nav))
    robot.orchestrator = Orchestrator(robot=robot, registry=registry)

    assert robot.mode is RobotMode.IDLE
    robot.orchestrator.handle(command=StartMapping())   # нет MappingBehavior -> просто смена режима
    assert robot.mode is RobotMode.MAPPING
    robot.orchestrator.handle(command=GoTo(goal=Pose2D(x_m=7.0, y_m=3.0, theta_rad=0.0)))
    assert robot.mode is RobotMode.NAVIGATING
    for _ in range(800):
        robot.tick(dt_s=0.1)
        if robot.mode is RobotMode.IDLE:
            break
    assert robot.mode is RobotMode.IDLE  # доехал -> сам вернулся в IDLE
