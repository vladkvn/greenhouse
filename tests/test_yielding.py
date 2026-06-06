"""Тесты децентрализованного разъезда (Инкремент 13): согласованная функция приоритета,
anti-starvation, fallback без связи и реальный разъезд двух роботов в узком ряду."""

from __future__ import annotations

import math

from greenhouse.adapters.sim.loop import build_sim_robot
from greenhouse.adapters.sim.world import PolygonWorld, Segment
from greenhouse.coordination.yielding import (
    RobotIntent,
    backoff_ticks,
    is_head_on,
    should_i_yield,
    who_yields,
)
from greenhouse.domain.geometry import Point2D, Twist2D


def _intent(rid: str, x: float, gx: float, **kw: object) -> RobotIntent:
    return RobotIntent(
        robot_id=rid, position=Point2D(x_m=x, y_m=3.0), goal=Point2D(x_m=gx, y_m=3.0), **kw  # type: ignore[arg-type]
    )


# --- детерминированная функция приоритета ---

def test_loaded_keeps_lane_empty_yields() -> None:
    loaded = _intent("A", 2.0, 9.0, loaded=True)
    empty = _intent("B", 7.0, 1.0, loaded=False)
    assert who_yields(loaded, empty) == "B"
    assert should_i_yield(me=empty, peer=loaded) is True
    assert should_i_yield(me=loaded, peer=empty) is False


def test_both_robots_agree_on_who_yields() -> None:
    a = _intent("A", 2.0, 9.0, battery_frac=0.8)
    b = _intent("B", 7.0, 1.0, battery_frac=0.5)
    # Оба считают одинаково → ровно один уступает.
    assert should_i_yield(me=a, peer=b) != should_i_yield(me=b, peer=a)


def test_anti_starvation_alternates() -> None:
    # Два одинаковых порожних робота: тот, кто уступал реже, уступает следующим.
    a = _intent("A", 2.0, 9.0)
    b = _intent("B", 7.0, 1.0)
    first = who_yields(a, b)
    # тот, кто уступил, увеличивает счётчик — следующим уступает другой
    if first == "A":
        a = a.model_copy(update={"yields_done": 1})
    else:
        b = b.model_copy(update={"yields_done": 1})
    assert who_yields(a, b) != first


def test_tie_break_is_deterministic_by_id() -> None:
    a = _intent("robot-A", 2.0, 9.0)
    b = _intent("robot-B", 7.0, 1.0)
    assert who_yields(a, b) == who_yields(b, a)  # симметрично, без зависимости от порядка


def test_head_on_detection() -> None:
    a = _intent("A", 4.0, 9.0)   # едет вправо
    b = _intent("B", 5.0, 1.0)   # едет влево, прямо навстречу, рядом
    assert is_head_on(me=a, peer=b) is True
    same_dir = _intent("C", 5.0, 9.0)  # едет туда же
    assert is_head_on(me=a, peer=same_dir) is False
    far = _intent("D", 9.5, 1.0)
    assert is_head_on(me=a, peer=far, range_m=2.0) is False


def test_backoff_breaks_symmetry_without_comms() -> None:
    # Разные id → разные задержки (симметрия ломается), стабильно между запусками.
    assert backoff_ticks(robot_id="robot-A", attempt=0) != backoff_ticks(robot_id="robot-B", attempt=0)
    assert backoff_ticks(robot_id="robot-A", attempt=0) == backoff_ticks(robot_id="robot-A", attempt=0)


# --- интеграция: два робота разъезжаются в узком ряду ---

def _corridor_world() -> PolygonWorld:
    """Узкий горизонтальный ряд y∈[2.5,3.5] с карманом-разъездом сверху при x∈[4.5,5.5]."""
    p = Point2D
    s = Segment
    walls = [
        s(a=p(x_m=0.0, y_m=2.5), b=p(x_m=10.0, y_m=2.5)),     # низ
        s(a=p(x_m=0.0, y_m=3.5), b=p(x_m=4.5, y_m=3.5)),      # верх слева
        s(a=p(x_m=4.5, y_m=3.5), b=p(x_m=4.5, y_m=4.5)),      # карман
        s(a=p(x_m=4.5, y_m=4.5), b=p(x_m=5.5, y_m=4.5)),
        s(a=p(x_m=5.5, y_m=4.5), b=p(x_m=5.5, y_m=3.5)),
        s(a=p(x_m=5.5, y_m=3.5), b=p(x_m=10.0, y_m=3.5)),     # верх справа
        s(a=p(x_m=0.0, y_m=2.5), b=p(x_m=0.0, y_m=3.5)),      # торцы
        s(a=p(x_m=10.0, y_m=2.5), b=p(x_m=10.0, y_m=3.5)),
    ]
    return PolygonWorld(walls=tuple(walls), bounds_min=p(x_m=0.0, y_m=2.5),
                        bounds_max=p(x_m=10.0, y_m=4.5))


def _drive_to(robot, target: Point2D, *, v_max: float = 0.7) -> None:
    pose = robot.state.pose()
    err = math.atan2(target.y_m - pose.y_m, target.x_m - pose.x_m) - pose.theta_rad
    err = math.atan2(math.sin(err), math.cos(err))
    w = max(-1.5, min(1.5, 2.0 * err))
    v = 0.0 if abs(err) > 0.6 else min(v_max, 1.2 * pose.point.distance_to(target))
    robot.motion.command(twist=Twist2D(linear_x_m_s=max(0.0, v), angular_z_rad_s=w))


def test_two_robots_pass_in_narrow_row_via_yielding() -> None:
    world = _corridor_world()
    alcove = Point2D(x_m=5.0, y_m=4.0)
    a = build_sim_robot(world=world, robot_id="robot-A", start_x_m=1.0, start_y_m=3.0)
    b = build_sim_robot(
        world=world, robot_id="robot-B", start_x_m=9.0, start_y_m=3.0, start_theta_rad=math.pi
    )
    goal = {"robot-A": Point2D(x_m=9.0, y_m=3.0), "robot-B": Point2D(x_m=1.0, y_m=3.0)}
    loaded = {"robot-A": True, "robot-B": False}  # A гружён → уступает B
    robots = {"robot-A": a, "robot-B": b}

    bay_lane = Point2D(x_m=5.0, y_m=3.0)  # карман на уровне ряда (въезд)

    def in_lane(pt: Point2D) -> bool:
        return abs(pt.y_m - 3.0) < 0.4  # ещё в ряду (не отъехал в карман)

    phase = {"robot-A": 0, "robot-B": 0}  # 0 — едем; 1 — в карман по ряду; 2 — вверх; 3 — назад
    min_sep = 9.0
    reached = {"robot-A": False, "robot-B": False}
    for _ in range(1500):
        intents = {
            rid: RobotIntent(robot_id=rid, position=robots[rid].state.pose().point,
                             goal=goal[rid], loaded=loaded[rid])
            for rid in robots
        }
        for rid, robot in robots.items():
            me, peer = intents[rid], intents["robot-B" if rid == "robot-A" else "robot-A"]
            gdir_x = goal[rid].x_m - me.position.x_m
            peer_ahead = gdir_x * (peer.position.x_m - me.position.x_m) > 0
            peer_near = me.position.distance_to(peer.position) < 2.2

            if phase[rid] == 1:                      # едем по ряду к карману
                _drive_to(robot, bay_lane)
                if abs(me.position.x_m - 5.0) < 0.4:
                    phase[rid] = 2
            elif phase[rid] == 2:                    # поднимаемся в карман, ждём проезда соседа
                _drive_to(robot, alcove)
                if not peer_ahead:
                    phase[rid] = 3
            elif phase[rid] == 3:                    # сосед проехал — спускаемся обратно в ряд
                _drive_to(robot, bay_lane)
                if in_lane(me.position):
                    phase[rid] = 0
            elif is_head_on(me=me, peer=peer, range_m=4.0) and should_i_yield(me=me, peer=peer):
                phase[rid] = 1                        # решил уступить
                _drive_to(robot, bay_lane)
            elif peer_ahead and peer_near and in_lane(peer.position):
                robot.motion.stop()                  # сосед ещё в ряду — пропускаю, не лезу
            else:
                _drive_to(robot, goal[rid])
        a.engine.step(dt_s=0.1)
        b.engine.step(dt_s=0.1)
        min_sep = min(min_sep, a.state.pose().point.distance_to(b.state.pose().point))
        reached = {rid: robots[rid].state.pose().point.distance_to(goal[rid]) <= 0.3
                   for rid in robots}
        if all(reached.values()):
            break

    assert all(reached.values())   # оба доехали (нет тупика/зацикливания)
    assert min_sep > 0.5           # ни разу не «слиплись» (нет лобовой)
