"""Часы симуляции: управляемое время вместо time.time() (детерминизм тестов)."""

from __future__ import annotations


class SimClock:
    """Реализация `greenhouse.runtime.Clock` с ручным продвижением времени."""

    def __init__(self, *, start_s: float = 0.0) -> None:
        self._t = start_s

    def now_s(self) -> float:
        return self._t

    def advance(self, *, dt_s: float) -> None:
        if dt_s < 0.0:
            raise ValueError("dt_s должно быть неотрицательным")
        self._t += dt_s
