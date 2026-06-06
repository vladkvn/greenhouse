"""Тесты следования за человеком: робот держит дистанцию до движущейся цели,
детектор учитывает дальность/прямую видимость, при потере цели робот безопасно стоит."""

from __future__ import annotations

import math

from greenhouse.adapters.sim.loop import build_sim_robot
from greenhouse.adapters.sim.person import SimPerson, SimPersonDetector
from greenhouse.adapters.sim.worlds import empty_room
from greenhouse.navigation.following import PersonFollower
from greenhouse.orchestration.modes import RobotMode


def _setup(person: SimPerson, *, start=(2.0, 3.0), standoff=0.8):
    world = empty_room(10.0, 6.0)
    robot = build_sim_robot(world=world, start_x_m=start[0], start_y_m=start[1])
    robot.person_detector = SimPersonDetector(
        world=world, robot_state=robot.state, person=person, clock=robot.clock, max_range_m=5.0
    )
    robot.follower = PersonFollower(standoff_m=standoff, max_linear_m_s=1.0)
    robot.mode = RobotMode.FOLLOWING
    return robot


def _dist(robot, person: SimPerson) -> float:
    return robot.state.pose().point.distance_to(person.point())


def test_follower_approaches_static_target_to_standoff() -> None:
    person = SimPerson(x_m=5.0, y_m=3.0)
    robot = _setup(person, standoff=0.8)
    for _ in range(80):
        robot.tick(dt_s=0.1)
    assert abs(_dist(robot, person) - 0.8) < 0.2  # встал на дистанции standoff


def test_follower_tracks_moving_target() -> None:
    person = SimPerson(x_m=4.0, y_m=3.0, vx_m_s=0.3)
    robot = _setup(person, standoff=0.8)
    for _ in range(120):
        person.step(dt_s=0.1)
        robot.tick(dt_s=0.1)
    # Робот держится рядом с движущейся целью (не отстал и не наехал).
    assert 0.4 < _dist(robot, person) < 1.6


def test_follower_stops_safely_when_target_lost() -> None:
    person = SimPerson(x_m=4.0, y_m=3.0)
    robot = _setup(person)
    for _ in range(20):
        robot.tick(dt_s=0.1)
    person.x_m, person.y_m = 20.0, 20.0  # цель ушла далеко за пределы дальности
    for _ in range(10):
        robot.tick(dt_s=0.1)
    assert robot.follower is not None and robot.follower.target_lost
    assert robot.state.last_cmd.linear_x_m_s == 0.0
    assert robot.state.last_cmd.angular_z_rad_s == 0.0


def test_detector_respects_range_and_line_of_sight() -> None:
    world = empty_room(10.0, 6.0)
    robot = build_sim_robot(world=world, start_x_m=2.0, start_y_m=3.0)
    far = SimPerson(x_m=9.0, y_m=3.0)  # 7 м > max_range 5
    near = SimPerson(x_m=4.0, y_m=3.0)
    det_far = SimPersonDetector(
        world=world, robot_state=robot.state, person=far, clock=robot.clock, max_range_m=5.0
    )
    det_near = SimPersonDetector(
        world=world, robot_state=robot.state, person=near, clock=robot.clock, max_range_m=5.0
    )
    assert det_far.detect() is None                 # вне дальности
    obs = det_near.detect()
    assert obs is not None
    assert math.isclose(obs.range_m, 2.0, abs_tol=1e-6)
    assert math.isclose(obs.bearing_rad, 0.0, abs_tol=1e-6)  # цель прямо по курсу


def test_detector_blocked_by_wall_returns_none() -> None:
    # Стена между роботом и целью: цель должна быть не видна.
    world = empty_room(10.0, 6.0)
    robot = build_sim_robot(world=world, start_x_m=2.0, start_y_m=3.0)
    behind_wall = SimPerson(x_m=-1.0, y_m=3.0)  # за левой стеной комнаты (x=0)
    det = SimPersonDetector(
        world=world, robot_state=robot.state, person=behind_wall, clock=robot.clock, max_range_m=8.0
    )
    assert det.detect() is None
