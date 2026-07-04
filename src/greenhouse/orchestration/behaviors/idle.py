"""IdleBehavior — режим ожидания: держать привод в стопе, ждать команды."""

from __future__ import annotations

from greenhouse.orchestration.interfaces import RobotContext
from greenhouse.orchestration.modes import RobotMode


class IdleBehavior:
    """Реализует Protocol `Behavior` для режима IDLE."""

    def __init__(self, *, robot: RobotContext) -> None:
        self._robot = robot

    @property
    def mode(self) -> RobotMode:
        return RobotMode.IDLE

    def step(self) -> RobotMode | None:
        self._robot.motion.stop()
        return None  # из IDLE уходим только по команде оператора
