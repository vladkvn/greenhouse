"""WallClock — реализация runtime.Clock на монотонных часах реального времени.

В отличие от SimClock (ручное продвижение времени для детерминизма тестов), на железе
время идёт само. Используется адаптерами Jetson (heartbeat привода, метки сканов).
"""

from __future__ import annotations

import time


class WallClock:
    """Реализует `greenhouse.runtime.Clock` поверх time.monotonic()."""

    def now_s(self) -> float:
        return time.monotonic()
