"""MQTT and REST-aligned DTOs plus topic naming helpers."""

from __future__ import annotations

from enum import StrEnum
from typing import Annotated, Literal

from pydantic import BaseModel, Field

from greenhouse_contracts.geometry import Pose2D
from greenhouse_contracts.identifiers import RobotIdField

MQTT_TOPIC_PREFIX = "greenhouse"


class RobotMode(StrEnum):
    """High-level onboard mode surfaced in telemetry."""

    IDLE = "idle"
    MAPPING = "mapping"
    NAVIGATING = "navigating"
    FOLLOWING = "following"


def robot_base_topic(robot_id: str) -> str:
    """Return namespace segment `greenhouse/robots/<robot_id>`."""

    return f"{MQTT_TOPIC_PREFIX}/robots/{robot_id}"


def telemetry_topic(robot_id: str) -> str:
    return f"{robot_base_topic(robot_id)}/telemetry"


def commands_topic(robot_id: str) -> str:
    return f"{robot_base_topic(robot_id)}/commands"


def command_ack_topic(robot_id: str) -> str:
    return f"{robot_base_topic(robot_id)}/commands/ack"


class TelemetryEnvelope(BaseModel):
    """Periodic state published robot → backend."""

    robot_id: RobotIdField
    mode: RobotMode
    pose_map: Pose2D
    stamp_unix_s: float
    battery_fraction: float | None = None
    active_errors: list[str] = Field(default_factory=list)


class CommandCorrelation(BaseModel):
    """Propagated through MQTT for traceability."""

    correlation_id: str = Field(pattern=r"^[a-zA-Z0-9_.:-]+$")


class EmergencyStop(BaseModel):
    type: Literal["emergency_stop"] = "emergency_stop"
    meta: CommandCorrelation


class StartMapping(BaseModel):
    type: Literal["start_mapping"] = "start_mapping"
    meta: CommandCorrelation


class StopMapping(BaseModel):
    type: Literal["stop_mapping"] = "stop_mapping"
    meta: CommandCorrelation


class NavigateToPose(BaseModel):
    type: Literal["go_to_pose"] = "go_to_pose"
    meta: CommandCorrelation
    goal: Pose2D


class BeginFollow(BaseModel):
    type: Literal["follow_person"] = "follow_person"
    meta: CommandCorrelation
    track_id: str = Field(pattern=r"^[a-zA-Z0-9_.:-]+$")


class CancelActiveMission(BaseModel):
    type: Literal["cancel_mission"] = "cancel_mission"
    meta: CommandCorrelation


CommandPayloadUnion = (
    EmergencyStop
    | StartMapping
    | StopMapping
    | NavigateToPose
    | BeginFollow
    | CancelActiveMission
)

IncomingCommandEnvelope = Annotated[CommandPayloadUnion, Field(discriminator="type")]


class AckStatus(StrEnum):
    ACCEPTED = "accepted"
    REJECTED = "rejected"


class CommandAck(BaseModel):
    correlation_id: str
    robot_id: RobotIdField
    status: AckStatus
    detail: str
