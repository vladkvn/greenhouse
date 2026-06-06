"""Децентрализованный разъезд роботов (Инкремент 13) — без сервера.

Роботы напрямую обмениваются «намерениями» (`RobotIntent`). При лобовой встрече в узком ряду
оба считают ОДНУ И ТУ ЖЕ детерминированную функцию приоритета и приходят к одинаковому
решению, кто уступает: гружёный важнее порожнего; среди равных — anti-starvation (кто уступал
реже), затем кому дешевле отъехать, затем у кого больше заряд, затем тай-брейк по `robot_id`.
Уступающий отъезжает в точку разъезда, пропускает и возвращается. При полной потере связи
(нет намерения соседа) симметрия ломается детерминированным рандомизированным backoff.
"""

from __future__ import annotations

import zlib

from pydantic import BaseModel, Field

from greenhouse.domain.geometry import Point2D
from greenhouse.domain.identifiers import RobotId


class RobotIntent(BaseModel, frozen=True):
    """Намерение робота, которым он делится с соседями напрямую (peer-to-peer)."""

    robot_id: RobotId
    position: Point2D
    goal: Point2D
    loaded: bool = False
    battery_frac: float = Field(ge=0.0, le=1.0, default=1.0)
    yields_done: int = Field(ge=0, default=0)   # сколько раз уже уступал (anti-starvation)
    yield_cost: float = Field(ge=0.0, default=0.0)  # «цена» отъезда (дальше до места разъезда)


def _yield_key(r: RobotIntent) -> tuple[bool, int, float, float, str]:
    """Ключ «насколько робот должен уступить» (больше — скорее уступает).

    Порядок критериев соответствует roadmap: загрузка → anti-starvation → цена отъезда →
    заряд → детерминированный тай-брейк по id."""
    return (
        not r.loaded,        # порожний (True) уступает гружёному (False)
        -r.yields_done,      # уступавший реже (больше -yields) уступает; частый — нет
        -r.yield_cost,       # кому дешевле отъехать (меньше cost) — уступает
        r.battery_frac,      # у кого больше заряд — уступает (есть запас хода)
        r.robot_id,          # детерминированный тай-брейк (id уникальны)
    )


def who_yields(a: RobotIntent, b: RobotIntent) -> RobotId:
    """Кто из двоих уступает. Оба робота вычисляют одинаково → согласованное решение."""
    return a.robot_id if _yield_key(a) > _yield_key(b) else b.robot_id


def should_i_yield(*, me: RobotIntent, peer: RobotIntent) -> bool:
    return who_yields(me, peer) == me.robot_id


def is_head_on(*, me: RobotIntent, peer: RobotIntent, range_m: float = 2.5) -> bool:
    """Лобовая встреча: сосед близко, впереди по моему курсу, и едет навстречу мне."""
    tx, ty = peer.position.x_m - me.position.x_m, peer.position.y_m - me.position.y_m
    dist = (tx * tx + ty * ty) ** 0.5
    if dist < 1e-6 or dist > range_m:
        return False
    my = (me.goal.x_m - me.position.x_m, me.goal.y_m - me.position.y_m)
    pr = (peer.goal.x_m - peer.position.x_m, peer.goal.y_m - peer.position.y_m)
    ahead = my[0] * tx + my[1] * ty > 0.0          # сосед впереди по моему курсу
    facing = pr[0] * (-tx) + pr[1] * (-ty) > 0.0   # сосед едет в мою сторону
    return ahead and facing


def backoff_ticks(*, robot_id: RobotId, attempt: int, base: int = 8, spread: int = 24) -> int:
    """Детерминированный рандомизированный backoff (fallback без связи).

    Зависит от `robot_id` и номера попытки — два разных робота почти всегда получают разные
    задержки, что ломает симметрию и позволяет разъехаться даже без обмена намерениями.
    Стабилен между запусками (crc32, без зависимости от PYTHONHASHSEED)."""
    h = zlib.crc32(f"{robot_id}:{attempt}".encode())
    return base + h % spread
