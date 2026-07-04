"""FollowingBehavior — следование за человеком как вложенная FSM (требование «за угол»).

RobotMode всё время остаётся FOLLOWING; FollowSub — внутренняя детализация (в телеметрию
через .sub). Надстраивает над низкоуровневым PersonFollower память о точке последнего
видения и поиск, НЕ меняя сам PersonFollower (он last-seen не помнит — following.py).

  TRACKING ──нет цели > lost_grace_s──▶ TARGET_LOST ──есть last_seen──▶ GOTO_LAST_SEEN
     ▲                                                                       │ доехал/тупик
     │ цель снова видна (с ЛЮБОЙ фазы)                                       ▼
     └────────────────── IDLE_WAIT ◀──таймаут поиска── SEARCH ◀─────────────┘

GOTO_LAST_SEEN делегирует тот же StepwiseNavigator, что и GoTo: едет в спроецированную в
карту точку last_seen С ОБЪЕЗДОМ препятствий и учётом габарита, доезжая до угла, откуда
снова появляется прямая видимость. Все пороги — в СЕКУНДАХ через инжектированный Clock.
"""

from __future__ import annotations

import math
from enum import StrEnum

from greenhouse.domain.geometry import Pose2D, Twist2D
from greenhouse.navigation.following import PersonFollower
from greenhouse.navigation.navigator import StepwiseNavigator
from greenhouse.navigation.planning import PlanOk
from greenhouse.navigation.protocols import RobotLike
from greenhouse.orchestration.modes import RobotMode
from greenhouse.runtime.clock import Clock
from greenhouse.sensing.interfaces import TargetDetector, TargetObservation


class FollowSub(StrEnum):
    TRACKING = "tracking"
    TARGET_LOST = "target_lost"
    GOTO_LAST_SEEN = "goto_last_seen"
    SEARCH = "search"
    IDLE_WAIT = "idle_wait"


class FollowingBehavior:
    """Реализует Protocol `Behavior` для режима FOLLOWING с под-FSM поиска цели."""

    def __init__(
        self,
        *,
        robot: RobotLike,
        detector: TargetDetector,
        follower: PersonFollower,
        navigator: StepwiseNavigator,
        clock: Clock,
        lost_grace_s: float = 0.5,
        search_timeout_s: float = 6.0,
        search_turn_rad_s: float = 0.6,
    ) -> None:
        self._robot = robot
        self._detector = detector
        self._follower = follower
        self._navigator = navigator
        self._clock = clock
        self._lost_grace_s = lost_grace_s
        self._search_timeout_s = search_timeout_s
        self._search_turn = search_turn_rad_s
        self._sub = FollowSub.TRACKING
        self._last_seen: Pose2D | None = None
        self._lost_since: float | None = None
        self._search_since = 0.0

    @property
    def mode(self) -> RobotMode:
        return RobotMode.FOLLOWING

    @property
    def sub(self) -> FollowSub:
        return self._sub

    @property
    def last_seen(self) -> Pose2D | None:
        return self._last_seen

    def step(self) -> RobotMode | None:
        obs = self._detector.detect()
        now = self._clock.now_s()

        # Цель видна — (ре)захват в ЛЮБОЙ фазе: отменяем навигацию и возвращаемся в TRACKING.
        if obs is not None and self._sub is not FollowSub.TRACKING:
            self._navigator.cancel()
            self._sub = FollowSub.TRACKING

        if self._sub is FollowSub.TRACKING:
            self._step_tracking(obs, now)
        elif self._sub is FollowSub.TARGET_LOST:
            self._step_target_lost(now)
        elif self._sub is FollowSub.GOTO_LAST_SEEN:
            self._step_goto(now)
        elif self._sub is FollowSub.SEARCH:
            self._step_search(now)
        else:  # IDLE_WAIT
            self._robot.motion.stop()

        return None  # из FOLLOWING выходим только по команде оператора (orchestrator)

    # ----------------------------------------------------------------- под-фазы
    def _step_tracking(self, obs: TargetObservation | None, now: float) -> None:
        if obs is None:
            if self._lost_since is None:
                self._lost_since = now
            self._robot.motion.stop()  # на время grace тормозим (дебаунс мерцания YOLO)
            if now - self._lost_since >= self._lost_grace_s:
                self._sub = FollowSub.TARGET_LOST
            return
        self._lost_since = None
        self._update_last_seen(obs)
        self._robot.motion.command(twist=self._follower.update(observation=obs))

    def _step_target_lost(self, now: float) -> None:
        if self._last_seen is not None and isinstance(
            self._navigator.begin(goal=self._last_seen), PlanOk
        ):
            self._sub = FollowSub.GOTO_LAST_SEEN
        else:
            self._enter_search(now)  # некуда ехать или план не построен — осмотримся

    def _step_goto(self, now: float) -> None:
        outcome = self._navigator.step()
        if outcome is not None:  # доехал до last_seen или тупик — переходим к осмотру
            self._enter_search(now)

    def _step_search(self, now: float) -> None:
        self._robot.motion.command(twist=Twist2D(linear_x_m_s=0.0, angular_z_rad_s=self._search_turn))
        if now - self._search_since >= self._search_timeout_s:
            self._sub = FollowSub.IDLE_WAIT
            self._robot.motion.stop()

    def _enter_search(self, now: float) -> None:
        self._navigator.cancel()
        self._sub = FollowSub.SEARCH
        self._search_since = now

    def _update_last_seen(self, obs: TargetObservation) -> None:
        """Спроецировать полярное наблюдение в точку карты по текущей оценке позы."""
        est = self._robot.localizer.latest()
        if est is None or est.is_lost:
            return
        p = est.pose
        ang = p.theta_rad + obs.bearing_rad  # абсолютный курс на цель
        self._last_seen = Pose2D(
            x_m=p.x_m + obs.range_m * math.cos(ang),
            y_m=p.y_m + obs.range_m * math.sin(ang),
            theta_rad=ang,
        )
