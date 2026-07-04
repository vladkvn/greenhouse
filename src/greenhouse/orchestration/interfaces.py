"""Оркестрация: конечный автомат режимов и приём команд.

Оркестратор дёргает остальные слои только через их интерфейсы. Он переводит команды
оператора/человека и события (цель достигнута, цель потеряна, низкий заряд) в смену
режима и в безопасные остановки при потере локализации/цели.
"""

from __future__ import annotations

from typing import Literal, Protocol, runtime_checkable

from pydantic import BaseModel, Field

from greenhouse.control.interfaces import MotionController
from greenhouse.domain.geometry import Pose2D
from greenhouse.domain.identifiers import RobotId, ZoneId
from greenhouse.navigation.keepout import Zone
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


class EditKeepout(BaseModel, frozen=True):
    """Оператор ставит/снимает закрытую зону на лету (режим не меняется)."""

    kind: Literal["edit_keepout"] = "edit_keepout"
    op: Literal["add", "remove"]
    zone: Zone | None = None        # для op="add"
    zone_id: ZoneId | None = None   # для op="remove"


Command = StartMapping | GoTo | FollowPerson | GoCharge | Stop | EmergencyStop | EditKeepout
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


@runtime_checkable
class GoalAccepting(Protocol):
    """Поведение, принимающее целевую позу (например, NAVIGATING получает её из GoTo)."""

    def set_goal(self, *, goal: Pose2D) -> None: ...


class RobotContext(Protocol):
    """Вид робота, нужный оркестратору и поведениям: режим, привод и телеметрия.

    Структурный контракт (Protocol): и SimRobot, и будущий JetsonRobot удовлетворяют ему,
    поэтому один оркестратор работает над обоими. Поведения дёргают привод/датчики через
    эту же абстракцию, а не через конкретного робота.
    """

    robot_id: RobotId
    mode: RobotMode
    motion: MotionController

    @property
    def battery_frac(self) -> float: ...

    @property
    def latest_pose(self) -> Pose2D | None: ...

    @property
    def localization_lost(self) -> bool: ...
