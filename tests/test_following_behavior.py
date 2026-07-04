"""Тесты FollowingBehavior (инкремент 5, флагман): под-FSM «за угол → last-seen → ожидание».

Детерминированно, в симуляции: управляемый детектор (видим цель N тиков, затем нет)
+ реальный SimRobot (поза/привод/лидар/навигатор). Гарнес повторяет интегрированный тик
(localize → behavior.step → engine.step). Проверяется порядок под-состояний, проекция
last_seen, мгновенный ре-захват цели и путь без last_seen.
"""

from __future__ import annotations

import math

from greenhouse.adapters.sim.loop import SimRobot, build_sim_robot
from greenhouse.adapters.sim.worlds import empty_room
from greenhouse.domain.geometry import Point2D
from greenhouse.domain.grid import CellState, MapMeta, OccupancyGrid
from greenhouse.navigation.following import PersonFollower
from greenhouse.navigation.navigator import StepwiseNavigator
from greenhouse.orchestration.behaviors.following import FollowingBehavior, FollowSub
from greenhouse.orchestration.modes import RobotMode
from greenhouse.sensing.interfaces import TargetObservation


def _free_grid(width_px: int, height_px: int, *, resolution_m: float = 0.5) -> OccupancyGrid:
    meta = MapMeta(
        resolution_m=resolution_m, width_px=width_px, height_px=height_px,
        origin=Point2D(x_m=0.0, y_m=0.0),
    )
    return OccupancyGrid(meta=meta, cells=[CellState.FREE] * (width_px * height_px))


def _localize(robot: SimRobot) -> None:
    robot.localizer.update(scan=robot.lidar.read_scan(), odometry=robot.odometry.read_odometry())


class ScriptedDetector:
    """Детектор по сценарию: список (сколько_тиков, видна_ли_цель). Цель всегда в 2 м по курсу."""

    def __init__(self, script: list[tuple[int, bool]]) -> None:
        self._seq: list[bool] = []
        for count, present in script:
            self._seq.extend([present] * count)
        self._i = 0

    def detect(self) -> TargetObservation | None:
        present = self._seq[self._i] if self._i < len(self._seq) else False
        self._i += 1
        if not present:
            return None
        return TargetObservation(range_m=2.0, bearing_rad=0.0, stamp_s=0.0)


def _behavior(
    robot: SimRobot,
    detector: ScriptedDetector,
    *,
    lost_grace_s: float = 0.3,
    search_timeout_s: float = 0.5,
) -> FollowingBehavior:
    nav = StepwiseNavigator(robot=robot, grid=_free_grid(20, 12), robot_radius_m=0.25, goal_tol_m=0.25)
    return FollowingBehavior(
        robot=robot,
        detector=detector,
        follower=PersonFollower(standoff_m=0.8, max_linear_m_s=1.0),
        navigator=nav,
        clock=robot.clock,
        lost_grace_s=lost_grace_s,
        search_timeout_s=search_timeout_s,
    )


def _run(robot: SimRobot, behavior: FollowingBehavior, *, ticks: int, dt_s: float = 0.1) -> list[FollowSub]:
    subs: list[FollowSub] = []
    for _ in range(ticks):
        _localize(robot)
        assert behavior.step() is None   # из FOLLOWING сам не выходит
        subs.append(behavior.sub)
        robot.engine.step(dt_s=dt_s)
    return subs


# -------------------------------------------------------- проекция last_seen
def test_last_seen_projection_along_heading() -> None:
    robot = build_sim_robot(world=empty_room(10.0, 6.0), start_x_m=2.0, start_y_m=3.0)
    behavior = _behavior(robot, ScriptedDetector([(1, True)]))
    _localize(robot)
    behavior.step()  # TRACKING: цель в 2 м по курсу (theta=0) -> (4, 3)
    ls = behavior.last_seen
    assert ls is not None
    assert math.isclose(ls.x_m, 4.0, abs_tol=1e-6) and math.isclose(ls.y_m, 3.0, abs_tol=1e-6)


def test_last_seen_projection_respects_orientation() -> None:
    robot = build_sim_robot(world=empty_room(10.0, 6.0), start_x_m=2.0, start_y_m=3.0)
    robot.state.theta_rad = math.pi / 2.0  # смотрим вдоль +Y
    behavior = _behavior(robot, ScriptedDetector([(1, True)]))
    _localize(robot)
    behavior.step()
    ls = behavior.last_seen
    assert ls is not None
    assert math.isclose(ls.x_m, 2.0, abs_tol=1e-6) and math.isclose(ls.y_m, 5.0, abs_tol=1e-6)


# ------------------------------------------------- полный путь до ожидания
def test_full_progression_corner_to_wait() -> None:
    # Видим цель 5 тиков, дальше она «зашла за угол» (детектор пуст).
    robot = build_sim_robot(world=empty_room(10.0, 6.0), start_x_m=2.0, start_y_m=3.0)
    behavior = _behavior(robot, ScriptedDetector([(5, True), (600, False)]))
    subs = _run(robot, behavior, ticks=600)

    order = [
        FollowSub.TRACKING, FollowSub.TARGET_LOST, FollowSub.GOTO_LAST_SEEN,
        FollowSub.SEARCH, FollowSub.IDLE_WAIT,
    ]
    first = {s: subs.index(s) for s in order if s in subs}
    assert set(first) == set(order), f"посещены не все под-состояния: {sorted(first)}"
    assert first[FollowSub.TRACKING] < first[FollowSub.TARGET_LOST] < first[FollowSub.GOTO_LAST_SEEN]
    assert first[FollowSub.GOTO_LAST_SEEN] < first[FollowSub.SEARCH] < first[FollowSub.IDLE_WAIT]
    assert subs[-1] is FollowSub.IDLE_WAIT          # закончили в ожидании
    assert robot.state.last_cmd.linear_x_m_s == 0.0  # в ожидании привод стоит


def test_reacquire_returns_to_tracking() -> None:
    # Видим 5, теряем 15 (входим в GOTO/SEARCH), затем цель снова появляется.
    robot = build_sim_robot(world=empty_room(10.0, 6.0), start_x_m=2.0, start_y_m=3.0)
    behavior = _behavior(robot, ScriptedDetector([(5, True), (15, False), (20, True)]))
    subs = _run(robot, behavior, ticks=40)
    assert FollowSub.GOTO_LAST_SEEN in subs or FollowSub.SEARCH in subs  # уходили искать
    assert behavior.sub is FollowSub.TRACKING       # и вернулись в слежение
    assert not behavior._navigator.is_active        # навигация отменена при ре-захвате


def test_never_seen_goes_straight_to_search() -> None:
    # Цель не видели ни разу -> минуя GOTO_LAST_SEEN сразу осмотр, затем ожидание.
    robot = build_sim_robot(world=empty_room(10.0, 6.0), start_x_m=2.0, start_y_m=3.0)
    behavior = _behavior(robot, ScriptedDetector([(600, False)]))
    subs = _run(robot, behavior, ticks=60)
    assert FollowSub.GOTO_LAST_SEEN not in subs     # ехать некуда — last_seen нет
    assert FollowSub.SEARCH in subs
    assert subs[-1] is FollowSub.IDLE_WAIT
    assert behavior.last_seen is None


def test_mode_is_following() -> None:
    robot = build_sim_robot(world=empty_room(10.0, 6.0))
    behavior = _behavior(robot, ScriptedDetector([(1, True)]))
    assert behavior.mode is RobotMode.FOLLOWING
