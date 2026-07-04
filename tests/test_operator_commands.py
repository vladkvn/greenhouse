"""Тесты операторского ввода: парсер текстовых команд и UDP-источник (loopback)."""

from __future__ import annotations

import socket

from greenhouse.adapters.operator.parser import parse_command
from greenhouse.adapters.operator.sources import UdpCommandSource
from greenhouse.orchestration.interfaces import (
    EditKeepout,
    EmergencyStop,
    FollowPerson,
    GoCharge,
    GoTo,
    StartMapping,
    Stop,
)


def test_parse_go_to_with_and_without_theta() -> None:
    cmd = parse_command("go_to 3 4 1.57")
    assert isinstance(cmd, GoTo)
    assert cmd.goal.x_m == 3.0 and cmd.goal.y_m == 4.0 and abs(cmd.goal.theta_rad - 1.57) < 1e-9
    cmd2 = parse_command("go_to 5 6")
    assert isinstance(cmd2, GoTo) and cmd2.goal.theta_rad == 0.0


def test_parse_simple_commands() -> None:
    assert isinstance(parse_command("follow"), FollowPerson)
    assert isinstance(parse_command("map"), StartMapping)
    assert isinstance(parse_command("start_mapping"), StartMapping)
    assert isinstance(parse_command("charge"), GoCharge)
    assert isinstance(parse_command("go_charge"), GoCharge)
    assert isinstance(parse_command("stop"), Stop)
    assert isinstance(parse_command("estop"), EmergencyStop)
    assert isinstance(parse_command("emergency_stop"), EmergencyStop)


def test_parse_keepout_add_and_remove() -> None:
    add = parse_command("keepout add z1 1 2 3 4")
    assert isinstance(add, EditKeepout) and add.op == "add" and add.zone is not None
    assert add.zone.zone_id == "z1" and len(add.zone.polygon) == 4
    rem = parse_command("keepout remove z1")
    assert isinstance(rem, EditKeepout) and rem.op == "remove" and rem.zone_id == "z1"


def test_parse_case_insensitive() -> None:
    assert isinstance(parse_command("FOLLOW"), FollowPerson)
    assert isinstance(parse_command("Go_To 1 1"), GoTo)


def test_parse_invalid_returns_none() -> None:
    for bad in ("", "   ", "bogus", "go_to 1", "go_to a b", "keepout add z1 1 2", "keepout wat"):
        assert parse_command(bad) is None


def test_udp_source_receives_and_parses() -> None:
    src = UdpCommandSource(port=0, host="127.0.0.1")
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sender.sendto(b"go_to 2 3", ("127.0.0.1", src.port))
        sender.sendto(b"stop", ("127.0.0.1", src.port))
        got: list[object] = []
        for _ in range(200):  # busy-poll: loopback UDP приходит почти мгновенно
            got.extend(src.poll())
            if len(got) >= 2:
                break
        assert any(isinstance(c, GoTo) for c in got)
        assert any(isinstance(c, Stop) for c in got)
    finally:
        sender.close()
        src.close()
