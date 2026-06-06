"""Координация флота: деление узких проездов через резервирование сегментов.

Проезды теплицы делятся на сегменты (участки между перекрёстками/расширениями).
Перед въездом в сегмент робот запрашивает эксклюзивный резерв. Это исключает
лобовые встречи в узких рядах. Координатор сначала in-process (симуляция нескольких
роботов в одном процессе), затем — за сетевым адаптером без смены интерфейса.
"""

from __future__ import annotations

from enum import StrEnum
from typing import Literal, Protocol

from pydantic import BaseModel

from greenhouse.domain.identifiers import RobotId, SegmentId


class SegmentKind(StrEnum):
    LANE = "lane"                    # узкий проезд: эксклюзивный
    PASSING_PLACE = "passing_place"  # расширение/разъезд: эксклюзив не нужен
    JUNCTION = "junction"            # перекрёсток


class ReservationToken(BaseModel, frozen=True):
    """Подтверждение резерва. Удерживается, пока робот в сегменте; затем release."""

    segment_id: SegmentId
    robot_id: RobotId
    granted_at_s: float


class ReservationDenied(BaseModel, frozen=True):
    kind: Literal["denied"] = "denied"
    segment_id: SegmentId
    held_by: RobotId | None = None
    eta_free_s: float | None = None


class ReservationGranted(BaseModel, frozen=True):
    kind: Literal["granted"] = "granted"
    token: ReservationToken


Reservation = ReservationGranted | ReservationDenied
"""Дискриминированный результат запроса (по полю kind)."""


class TrafficCoordinator(Protocol):
    """Арбитр доступа к сегментам проездов для всего флота.

    Гарантии: взаимное исключение на LANE/JUNCTION; предотвращение тупиков за счёт
    запроса всего нужного маршрута в согласованном порядке (робот ждёт у входа,
    а не застревает в середине занятого проезда).
    """

    def request(self, *, robot_id: RobotId, segment_id: SegmentId) -> Reservation: ...

    def request_route(self, *, robot_id: RobotId, segments: tuple[SegmentId, ...]) -> Reservation:
        """Атомарно зарезервировать последовательность сегментов (или отказ)."""
        ...

    def release(self, *, token: ReservationToken) -> None: ...

    def holder_of(self, *, segment_id: SegmentId) -> RobotId | None: ...
