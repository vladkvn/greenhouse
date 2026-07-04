"""Поведения режимов (реализации Protocol Behavior).

Каждое поведение — это логика одного RobotMode на один тик `step()`. Оркестратор крутит
`step()` активного режима; возвращённый RobotMode запрашивает переход (None = остаться).
"""

from greenhouse.orchestration.behaviors.charging import ChargeSub, ChargingBehavior
from greenhouse.orchestration.behaviors.following import FollowingBehavior, FollowSub
from greenhouse.orchestration.behaviors.idle import IdleBehavior
from greenhouse.orchestration.behaviors.mapping import MappingBehavior
from greenhouse.orchestration.behaviors.navigating import NavigatingBehavior

__all__ = [
    "ChargeSub",
    "ChargingBehavior",
    "FollowingBehavior",
    "FollowSub",
    "IdleBehavior",
    "MappingBehavior",
    "NavigatingBehavior",
]
