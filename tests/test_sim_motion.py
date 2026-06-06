"""Тесты движения: интеграция diff-drive, обрезка по limits, откат при столкновении."""

from __future__ import annotations

import math

from greenhouse.adapters.sim.clock import SimClock
from greenhouse.adapters.sim.engine import SimEngine
from greenhouse.adapters.sim.motion import SimMotion
from greenhouse.adapters.sim.state import SimState
from greenhouse.adapters.sim.worlds import empty_room
from greenhouse.domain.geometry import Twist2D


def _engine() -> tuple[SimState, SimEngine, SimMotion]:
    state = SimState(x_m=1.0, y_m=3.0, theta_rad=0.0)
    clock = SimClock()
    engine = SimEngine(world=empty_room(10.0, 6.0), state=state, clock=clock)
    motion = SimMotion(state=state)
    return state, engine, motion


def test_forward_motion_integrates() -> None:
    state, engine, motion = _engine()
    motion.command(twist=Twist2D(linear_x_m_s=0.5, angular_z_rad_s=0.0))
    for _ in range(10):  # 1 секунда при dt=0.1 → 0.5 м
        engine.step(dt_s=0.1)
    assert math.isclose(state.x_m, 1.5, abs_tol=1e-6)
    assert math.isclose(state.y_m, 3.0, abs_tol=1e-6)


def test_command_clamped_to_limits() -> None:
    state, _engine_, motion = _engine()
    motion.command(twist=Twist2D(linear_x_m_s=99.0, angular_z_rad_s=0.0))
    assert state.last_cmd.linear_x_m_s == motion.limits.max_linear_m_s


def test_collision_blocks_motion() -> None:
    state, engine, motion = _engine()
    # Едем влево к стене x=0; вписанный радиус 0.25 → центр упрётся на ~0.25.
    motion.command(twist=Twist2D(linear_x_m_s=-1.0, angular_z_rad_s=0.0))
    for _ in range(50):
        engine.step(dt_s=0.1)
    assert state.x_m >= state.inscribed_radius_m - 1e-6
    assert state.x_m < 0.5  # подъехал близко, но не прошёл сквозь стену


def test_stop_zeroes_command() -> None:
    state, _e, motion = _engine()
    motion.command(twist=Twist2D(linear_x_m_s=0.5, angular_z_rad_s=0.3))
    motion.stop()
    assert state.last_cmd.linear_x_m_s == 0.0
    assert state.last_cmd.angular_z_rad_s == 0.0
