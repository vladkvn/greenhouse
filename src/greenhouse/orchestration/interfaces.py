"""Оркестрация: конечный автомат режимов и приём команд.

Оркестратор дёргает остальные слои только через их интерфейсы. Он переводит команды
оператора/человека и события (цель достигнута, цель потеряна, низкий заряд) в смену
режима и в безопасные остановки при потере локализации/цели.
"""

from __future__ import annotations

from typing import Literal, Protocol

from pydantic import BaseModel, Field

from greenhouse.domain.geometry import Pose2D
from greenhouse.domain.identifiers import RobotId
from greenhouse.orchestration.modes import RobotMode

# --- Команды (вход оркестратора) ---

class StartMapping(BaseModel, frozen=True):
    kind: Literal["start_mapping"] = "start_mapping"


class GoTo(BaseModel, frozen=True):
    kind: Literal["go_to"] = "go_to"
    goal: Pose2D


class FollowPerson(BaseModel, frozen=True):
    kind: Literal["follow_person"] = "follow_person"


class GoCharge(BaseModel, frozen=True):
    kind: Literal["go_charge"] = "go_charge"


class Stop(BaseModel, frozen=True):
    kind: Literal["stop"] = "stop"


class EmergencyStop(BaseModel, frozen=True):
    kind: Literal["emergency_stop"] = "emergency_stop"


Command = StartMapping | GoTo | FollowPerson | GoCharge | Stop | EmergencyStop
"""Дискриминированное объединение команд (по полю kind)."""


# --- Состояние робота (выход оркестратора, идёт в телеметрию) ---

class RobotStatus(BaseModel, frozen=True):
    robot_id: RobotId
    mode: RobotMode
    pose: Pose2D | None = None
    battery_frac: float = Field(ge=0.0, le=1.0)
    last_error: str | None = None


# --- Интерфейсы ---

class MissionHandler(Protocol):
    """Принимает команды и управляет режимом одного робота."""

    def handle(self, *, command: Command) -> RobotStatus: ...

    def status(self) -> RobotStatus: ...


class Behavior(Protocol):
    """Поведение одного режима: один тик его логики.

    Оркестратор вызывает step() активного поведения в цикле управления; поведение
    возвращает запрошенный переход (или None — остаться в режиме).
    """

    @property
    def mode(self) -> RobotMode: ...

    def step(self) -> RobotMode | None: ...
