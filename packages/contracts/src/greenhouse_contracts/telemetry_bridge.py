"""Interfaces bridging ROS processes to MQTT / REST ingestion."""

from __future__ import annotations

from typing import Protocol

from pydantic import BaseModel, Field

from greenhouse_contracts.messaging import CommandAck, CommandPayloadUnion, TelemetryEnvelope


class TelemetryDispatchResult(BaseModel):
    dispatched: bool
    detail: str = Field(description="Carrier-level diagnostics (MQTT ack QoS semantics).")


class TelemetryPublisher(Protocol):
    """ROS-side publisher towards backend."""

    def publish(self, envelope: TelemetryEnvelope) -> TelemetryDispatchResult: ...


class CommandDispatchHandler(Protocol):
    """Executed when MQTT client receives payloads."""

    def __call__(self, command: CommandPayloadUnion) -> CommandAck: ...


class CommandSubscriber(Protocol):
    """MQTT client façade — implemented with paho/aiomqtt."""

    def start(self, handler: CommandDispatchHandler) -> None: ...

    def stop(self) -> None: ...
