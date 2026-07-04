"""Реестр поведений: сопоставление RobotMode → Behavior.

Точка расширения «добавить режим» из контракта оркестрации: новый режим подключается
строкой `registry.register(NewBehavior(...))`, без правки самого оркестратора.
"""

from __future__ import annotations

from greenhouse.orchestration.interfaces import Behavior
from greenhouse.orchestration.modes import RobotMode


class BehaviorRegistry:
    """Декларативный реестр поведений по режимам."""

    def __init__(self) -> None:
        self._by_mode: dict[RobotMode, Behavior] = {}

    def register(self, behavior: Behavior) -> None:
        """Зарегистрировать поведение под его собственным режимом (`behavior.mode`)."""
        self._by_mode[behavior.mode] = behavior

    def get(self, mode: RobotMode) -> Behavior | None:
        return self._by_mode.get(mode)

    def modes(self) -> list[RobotMode]:
        return list(self._by_mode)
