"""JetsonLidar — реализация LidarSource поверх боевого RPLIDAR-читателя follow_me.

Боевой `follow_me.lidar.LidarThread` крутит RPLIDAR A1 в фоне и держит последний оборот
как 360 одноградусных бинов (метры, NaN = нет возврата), индексированных СЫРЫМ углом
лидара. Этот адаптер приводит их к контракту ядра `LidarScan`, где луч 0 смотрит ВПЕРЁД,
углы растут против часовой (как у SimLidar: `angle_min_rad=0`, CCW).

Монтажный сдвиг и направление вращения лидара зависят от того, как он привинчен к шасси,
и подлежат КАЛИБРОВКЕ (см. тест конвенции углов). Параметры:
  * `forward_bin_deg` — какой сырой бин лидара смотрит по курсу робота;
  * `clockwise` — растёт ли сырой угол лидара по часовой (RPLIDAR A1 — да);
  * `masked_sectors` — секторы в системе робота (град.), которые лидар видит как
    собственный корпус (self-hit) → гасим в NaN.

Контракт: `greenhouse.sensing.LidarSource`.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from typing import Protocol

from greenhouse.runtime.clock import Clock
from greenhouse.sensing.interfaces import LidarScan

_NBINS = 360


class ScanSource(Protocol):
    """Источник сырого оборота: 360 бинов (метры, NaN = нет возврата). LidarThread это умеет."""

    def get_scan(self) -> Sequence[float]: ...


class JetsonLidar:
    """Реализует `greenhouse.sensing.LidarSource` поверх follow_me.LidarThread."""

    def __init__(
        self,
        *,
        clock: Clock,
        scan_source: ScanSource | None = None,
        port: str = "/dev/ttyUSB0",
        baud: int = 115200,
        max_range_m: float = 12.0,
        forward_bin_deg: float = 0.0,
        clockwise: bool = True,
        masked_sectors: Sequence[tuple[float, float]] = (),
    ) -> None:
        self._clock = clock
        self._source = scan_source
        self._port = port
        self._baud = baud
        self._max_range = max_range_m
        self._forward = forward_bin_deg
        self._sign = -1.0 if clockwise else 1.0
        self._masked = tuple(masked_sectors)
        self._increment = 2.0 * math.pi / _NBINS
        # Предрасчёт отображения «луч робота i → сырой бин лидара».
        self._raw_for_beam = [
            int(round(self._forward + self._sign * i)) % _NBINS for i in range(_NBINS)
        ]

    def start(self) -> None:
        """Поднять боевой LidarThread, если источник не внедрён извне (тесты внедряют свой)."""
        if self._source is None:
            self._source = _make_lidar_thread(self._port, self._baud, self._max_range)

    def read_scan(self) -> LidarScan:
        if self._source is None:
            self.start()
        assert self._source is not None
        bins = self._source.get_scan()
        ranges: list[float] = []
        for i in range(_NBINS):
            if self._is_masked(i):
                ranges.append(math.nan)
                continue
            d = float(bins[self._raw_for_beam[i]])
            ranges.append(d if math.isfinite(d) and 0.0 < d <= self._max_range else math.nan)
        return LidarScan(
            angle_min_rad=0.0,
            angle_increment_rad=self._increment,
            range_max_m=self._max_range,
            ranges_m=tuple(ranges),
            stamp_s=self._clock.now_s(),
        )

    def _is_masked(self, beam_deg: int) -> bool:
        for lo, hi in self._masked:
            if lo <= hi:
                if lo <= beam_deg <= hi:
                    return True
            elif beam_deg >= lo or beam_deg <= hi:  # сектор через шов 0°/360°
                return True
        return False


def _make_lidar_thread(port: str, baud: int, max_range_m: float) -> ScanSource:
    """Лениво поднять боевой LidarThread (доступен на Jetson из пакета follow_me)."""
    try:
        from follow_me.lidar import LidarThread  # type: ignore[import-not-found]
    except ImportError as exc:  # pragma: no cover - только вне Jetson
        raise ImportError(
            "Боевой пакет follow_me недоступен на PYTHONPATH. На Jetson он лежит в "
            "~/greenhouse/follow_me. Либо внедрите scan_source в JetsonLidar."
        ) from exc
    thread = LidarThread(port, baud, max_range_m)
    thread.start()
    thread.wait_until_ready(timeout=10.0)
    return thread  # type: ignore[no-any-return]
