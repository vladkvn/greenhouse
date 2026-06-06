"""InMemoryTrafficCoordinator — арбитр сегментов проездов для флота (in-process).

Взаимное исключение: сегмент держит не более одного робота. Предотвращение тупиков:
`request_route` резервирует весь нужный маршрут атомарно — робот либо получает все
сегменты сразу, либо ждёт у входа (а не застревает в середине занятого проезда).
За сетевым адаптером тот же интерфейс работает для распределённого флота.
"""

from __future__ import annotations

from greenhouse.coordination.interfaces import (
    Reservation,
    ReservationDenied,
    ReservationGranted,
    ReservationToken,
)
from greenhouse.domain.identifiers import RobotId, SegmentId
from greenhouse.runtime.clock import Clock


class InMemoryTrafficCoordinator:
    """Реализует `greenhouse.coordination.TrafficCoordinator` в одном процессе."""

    def __init__(self, *, clock: Clock | None = None) -> None:
        self._clock = clock
        self._holders: dict[SegmentId, RobotId] = {}

    def request(self, *, robot_id: RobotId, segment_id: SegmentId) -> Reservation:
        holder = self._holders.get(segment_id)
        if holder is not None and holder != robot_id:
            return ReservationDenied(segment_id=segment_id, held_by=holder)
        self._holders[segment_id] = robot_id
        return self._grant(robot_id, segment_id)

    def request_route(
        self, *, robot_id: RobotId, segments: tuple[SegmentId, ...]
    ) -> Reservation:
        for seg in segments:  # атомарность: сначала проверяем весь маршрут
            holder = self._holders.get(seg)
            if holder is not None and holder != robot_id:
                return ReservationDenied(segment_id=seg, held_by=holder)
        for seg in segments:
            self._holders[seg] = robot_id
        return self._grant(robot_id, segments[0])

    def release(self, *, token: ReservationToken) -> None:
        # Освобождаем все сегменты, удерживаемые этим роботом (весь его маршрут).
        for seg in [s for s, h in self._holders.items() if h == token.robot_id]:
            del self._holders[seg]

    def holder_of(self, *, segment_id: SegmentId) -> RobotId | None:
        return self._holders.get(segment_id)

    def _grant(self, robot_id: RobotId, segment_id: SegmentId) -> ReservationGranted:
        now = self._clock.now_s() if self._clock is not None else 0.0
        return ReservationGranted(
            token=ReservationToken(segment_id=segment_id, robot_id=robot_id, granted_at_s=now)
        )
