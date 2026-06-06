"""Доменные ошибки. Возвращаются как типы результата, а не как сырые исключения,
там где сбой — ожидаемая часть контракта (потеря локализации, недостижимая цель)."""

from __future__ import annotations

from enum import Enum
from typing import Literal

from pydantic import BaseModel


class FailureCode(str, Enum):
    LOCALIZATION_LOST = "localization_lost"
    OBSTACLE_BLOCKED = "obstacle_blocked"
    PLANNER_FAILED = "planner_failed"
    TARGET_LOST = "target_lost"
    RESERVATION_DENIED = "reservation_denied"
    INTERNAL_ERROR = "internal_error"


class Failure(BaseModel, frozen=True):
    """Структурированный сбой операции."""

    kind: Literal["failure"] = "failure"
    code: FailureCode
    message: str
