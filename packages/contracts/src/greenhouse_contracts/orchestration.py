"""Deterministic arbitration between subsystem contracts."""

from __future__ import annotations

from typing import Protocol

from greenhouse_contracts.messaging import CommandAck, CommandPayloadUnion, RobotMode


class RobotBehavior(Protocol):
    """Read-only façade for onboarding logic and diagnostics."""

    def active_mode(self) -> RobotMode: ...

    def is_safe_to_drive(self) -> bool: ...


class MissionHandler(Protocol):
    """Consumes MQTT/CLI commands routed by telemetry bridge."""

    def apply(self, command: CommandPayloadUnion) -> CommandAck: ...
