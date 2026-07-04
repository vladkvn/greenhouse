"""RobotLike — структурный контракт робота для слоя навигации.

Ослабляет зависимость с конкретного SimRobot до набора ПОРТОВ, которые нужны навигации:
привод, лидар и локализатор. Поза берётся из `localizer.latest()`, а не из истины
симулятора, поэтому `StepwiseNavigator` одинаково работает над SimRobot и будущим
JetsonRobot. Намеренно НЕ включает режим/команды/tick — это держит навигацию свободной от
слоя оркестрации (иначе возник бы цикл orchestration↔navigation); расширенный фасад робота
(с mode/tick) живёт уровнем выше.
"""

from __future__ import annotations

from typing import Protocol

from greenhouse.control.interfaces import MotionController
from greenhouse.navigation.localization import Localizer
from greenhouse.sensing.interfaces import LidarSource


class RobotLike(Protocol):
    """Минимальный вид робота для навигации: привод + лидар + локализатор."""

    motion: MotionController
    lidar: LidarSource
    localizer: Localizer
