"""Источники команд оператора: UDP-сокет и stdin. Оба неблокирующие (poll за тик цикла)."""

from __future__ import annotations

import select
import socket
import sys
from typing import Protocol

from greenhouse.adapters.operator.parser import parse_command
from greenhouse.orchestration.interfaces import Command


class CommandSource(Protocol):
    def poll(self) -> list[Command]: ...


class UdpCommandSource:
    """Слушает текстовые команды по UDP (симметрично протоколу моторов)."""

    def __init__(self, *, port: int, host: str = "0.0.0.0") -> None:
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setblocking(False)
        self._sock.bind((host, port))

    @property
    def port(self) -> int:
        """Фактический порт (полезно при bind на 0 — ОС выбирает свободный)."""
        return int(self._sock.getsockname()[1])

    def poll(self) -> list[Command]:
        cmds: list[Command] = []
        while True:
            try:
                data, _ = self._sock.recvfrom(512)
            except (BlockingIOError, OSError):
                break  # очередь пуста (или сокет закрыт)
            cmd = parse_command(data.decode(errors="ignore"))
            if cmd is not None:
                cmds.append(cmd)
        return cmds

    def close(self) -> None:
        self._sock.close()


class StdinCommandSource:
    """Читает команды со стандартного ввода (удобно по SSH)."""

    def poll(self) -> list[Command]:
        cmds: list[Command] = []
        while select.select([sys.stdin], [], [], 0.0)[0]:
            line = sys.stdin.readline()
            if not line:
                break
            cmd = parse_command(line)
            if cmd is not None:
                cmds.append(cmd)
        return cmds


class MultiCommandSource:
    """Объединяет несколько источников (например, UDP + stdin)."""

    def __init__(self, sources: list[CommandSource]) -> None:
        self._sources = sources

    def poll(self) -> list[Command]:
        out: list[Command] = []
        for source in self._sources:
            out.extend(source.poll())
        return out
