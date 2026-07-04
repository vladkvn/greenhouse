"""Операторский ввод: текстовые команды → доменные Command (транспорт-независимо).

Парсер чистый и тестируемый; источники (UDP, stdin) — тонкие адаптеры поверх него.
Команды скармливаются `MissionHandler.handle`. Работает и для sim, и для железа.
"""

from greenhouse.adapters.operator.parser import parse_command
from greenhouse.adapters.operator.sources import (
    CommandSource,
    MultiCommandSource,
    StdinCommandSource,
    UdpCommandSource,
)

__all__ = [
    "parse_command",
    "CommandSource",
    "MultiCommandSource",
    "StdinCommandSource",
    "UdpCommandSource",
]
