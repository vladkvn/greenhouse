"""Supervisor — надрежимные правила безопасности (preempt поверх любого Behavior).

Каждый тик оркестратор спрашивает Supervisor ДО шага активного поведения. Это даёт
деградацию в стоп как первоклассное свойство: потеря локализации в ездовом режиме —
безопасная остановка; низкий заряд — форс перехода на зарядку.

Следование (FOLLOWING) переживает потерю локализации: оно работает в относительных
координатах цели и не зависит от карты, поэтому при is_lost его НЕ останавливаем (об этом
сообщает только телеметрия last_error). EmergencyStop приходит командой и обрабатывается
оркестратором напрямую, а не здесь.
"""

from __future__ import annotations

from dataclasses import dataclass

from greenhouse.orchestration.interfaces import RobotContext
from greenhouse.orchestration.modes import RobotMode

# Режимы, которым нельзя ехать вслепую при потере локализации.
_LOST_UNSAFE_MODES = (RobotMode.NAVIGATING, RobotMode.MAPPING, RobotMode.CHARGING)


@dataclass(frozen=True)
class Preempt:
    """Надрежимное вмешательство: форс-переход и/или удержание стопа на этот тик."""

    reason: str
    mode: RobotMode | None = None  # целевой режим перехода (None = режим не менять)
    force_stop: bool = False       # удержать привод в стопе на этом тике


class Supervisor:
    """Приоритетные safety-правила поверх режимов."""

    def __init__(self, *, low_battery_frac: float = 0.3) -> None:
        self._low_battery = low_battery_frac

    def check(self, *, robot: RobotContext) -> Preempt | None:
        if robot.localization_lost and robot.mode in _LOST_UNSAFE_MODES:
            return Preempt(reason="localization_lost", mode=RobotMode.IDLE, force_stop=True)
        if robot.battery_frac < self._low_battery and robot.mode is not RobotMode.CHARGING:
            return Preempt(reason="battery_low", mode=RobotMode.CHARGING)
        return None
