"""Абстракция времени. Алгоритмы не зовут time.time() напрямую — это делает
детерминированными тесты и позволяет симуляции «ускорять» время."""

from __future__ import annotations

from typing import Protocol


class Clock(Protocol):
    """Источник монотонного времени в секундах."""

    def now_s(self) -> float: ...
