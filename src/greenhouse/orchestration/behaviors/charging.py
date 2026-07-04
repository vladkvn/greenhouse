"""ChargingBehavior — режим CHARGING: доехать на док и зарядиться (под-FSM GOTO_DOCK→DOCKED).

Доезд до дока — тем же StepwiseNavigator, что и GoTo (объезд + габарит достаются бесплатно).
На доке робот стоит и ждёт, пока заряд не достигнет полного, затем запрашивает IDLE. Док —
фиксированная поза на карте, задаётся при сборке. Триггеры входа в CHARGING (команда
GoCharge или preempt по низкому заряду в Supervisor) — снаружи, в оркестраторе.
"""

from __future__ import annotations

from enum import StrEnum

from greenhouse.domain.errors import Failure
from greenhouse.domain.geometry import Pose2D
from greenhouse.navigation.navigator import StepwiseNavigator
from greenhouse.navigation.planning import NavOutcome
from greenhouse.orchestration.interfaces import RobotContext
from greenhouse.orchestration.modes import RobotMode


class ChargeSub(StrEnum):
    GOTO_DOCK = "goto_dock"
    DOCKED = "docked"


class ChargingBehavior:
    """Реализует Protocol `Behavior` для режима CHARGING."""

    def __init__(
        self,
        *,
        robot: RobotContext,
        navigator: StepwiseNavigator,
        dock: Pose2D,
        full_battery_frac: float = 0.95,
    ) -> None:
        self._robot = robot
        self._navigator = navigator
        self._dock = dock
        self._full = full_battery_frac
        self._sub = ChargeSub.GOTO_DOCK
        self._begun = False

    @property
    def mode(self) -> RobotMode:
        return RobotMode.CHARGING

    @property
    def sub(self) -> ChargeSub:
        return self._sub

    def step(self) -> RobotMode | None:
        if self._sub is ChargeSub.GOTO_DOCK:
            if not self._begun:
                self._begun = True
                if isinstance(self._navigator.begin(goal=self._dock), Failure):
                    return RobotMode.IDLE  # до дока не доехать — выходим
                return None
            outcome = self._navigator.step()
            if isinstance(outcome, NavOutcome) and outcome.kind == "reached":
                self._sub = ChargeSub.DOCKED
                self._robot.motion.stop()
            elif isinstance(outcome, Failure):
                return RobotMode.IDLE
            return None

        # DOCKED: стоим и заряжаемся, пока не наполнится.
        self._robot.motion.stop()
        if self._robot.battery_frac >= self._full:
            return RobotMode.IDLE
        return None
