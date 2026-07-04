"""NavigatingBehavior — режим NAVIGATING: доехать в заданную точку через StepwiseNavigator.

Цель приходит от оркестратора (команда GoTo) через set_goal. Само планирование
откладывается до первого step() — он выполняется уже ПОСЛЕ локализации в тике робота,
поэтому поза для планирования свежая. По достижении цели или невозможности построить
маршрут запрашивает переход в IDLE.
"""

from __future__ import annotations

from greenhouse.domain.geometry import Pose2D
from greenhouse.navigation.navigator import StepwiseNavigator
from greenhouse.navigation.planning import PlanOk
from greenhouse.orchestration.modes import RobotMode


class NavigatingBehavior:
    """Реализует Protocol `Behavior` (и `GoalAccepting`) для режима NAVIGATING."""

    def __init__(self, *, navigator: StepwiseNavigator) -> None:
        self._navigator = navigator
        self._goal: Pose2D | None = None
        self._begun = False

    @property
    def mode(self) -> RobotMode:
        return RobotMode.NAVIGATING

    def set_goal(self, *, goal: Pose2D) -> None:
        self._goal = goal
        self._begun = False

    def step(self) -> RobotMode | None:
        if self._goal is None:
            return RobotMode.IDLE  # цель не задана — в ездовом режиме делать нечего
        if not self._begun:
            self._begun = True
            if not isinstance(self._navigator.begin(goal=self._goal), PlanOk):
                self._goal = None
                return RobotMode.IDLE  # маршрут не построен
            return None
        outcome = self._navigator.step()
        if outcome is not None:  # доехал (NavOutcome) или тупик (Failure)
            self._goal = None
            return RobotMode.IDLE
        return None
