"""Парсер текстовых команд оператора в доменные Command (чистая функция).

Грамматика (без учёта регистра, поля через пробел):
    go_to X Y [THETA]            -> ехать в точку (метры, курс рад)
    follow                       -> следовать за человеком
    map | start_mapping          -> строить карту
    charge | go_charge           -> ехать на зарядку
    stop                         -> штатная остановка
    estop | emergency_stop       -> аварийная остановка
    keepout add ID X0 Y0 X1 Y1   -> поставить прямоугольную KEEPOUT-зону
    keepout remove ID            -> снять зону
Непонятная строка -> None (вызывающий логирует и игнорирует).
"""

from __future__ import annotations

from greenhouse.domain.geometry import Point2D, Pose2D
from greenhouse.navigation.keepout import Zone, ZoneKind
from greenhouse.orchestration.interfaces import (
    Command,
    EditKeepout,
    EmergencyStop,
    FollowPerson,
    GoCharge,
    GoTo,
    StartMapping,
    Stop,
)


def parse_command(line: str) -> Command | None:
    parts = line.strip().split()
    if not parts:
        return None
    head, args = parts[0].lower(), parts[1:]
    try:
        if head == "go_to":
            theta = float(args[2]) if len(args) > 2 else 0.0
            return GoTo(goal=Pose2D(x_m=float(args[0]), y_m=float(args[1]), theta_rad=theta))
        if head == "follow":
            return FollowPerson()
        if head in ("map", "start_mapping"):
            return StartMapping()
        if head in ("charge", "go_charge"):
            return GoCharge()
        if head == "stop":
            return Stop()
        if head in ("estop", "emergency_stop"):
            return EmergencyStop()
        if head == "keepout":
            return _parse_keepout(args)
    except (IndexError, ValueError):
        return None
    return None


def _parse_keepout(args: list[str]) -> EditKeepout | None:
    if not args:
        return None
    op = args[0].lower()
    if op == "add":
        x0, y0, x1, y1 = float(args[2]), float(args[3]), float(args[4]), float(args[5])
        polygon = (
            Point2D(x_m=x0, y_m=y0), Point2D(x_m=x1, y_m=y0),
            Point2D(x_m=x1, y_m=y1), Point2D(x_m=x0, y_m=y1),
        )
        return EditKeepout(op="add", zone=Zone(zone_id=args[1], kind=ZoneKind.KEEPOUT, polygon=polygon))
    if op == "remove":
        return EditKeepout(op="remove", zone_id=args[1])
    return None
