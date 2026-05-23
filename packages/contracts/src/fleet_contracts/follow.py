"""Human-centric following contracts."""

from __future__ import annotations

from typing import Protocol

from pydantic import BaseModel, Field

from fleet_contracts.perception import CameraFrame


class BoundingBoxNormalized(BaseModel):
    """BBox in normalized image coordinates (origin top-left)."""

    xmin: float = Field(ge=0.0, le=1.0)
    ymin: float = Field(ge=0.0, le=1.0)
    xmax: float = Field(ge=0.0, le=1.0)
    ymax: float = Field(ge=0.0, le=1.0)


class PersonObservation(BaseModel):
    """Single detection hypothesis."""

    track_id: str
    confidence: float = Field(ge=0.0, le=1.0)
    bbox: BoundingBoxNormalized


class EmbeddingVector(BaseModel):
    values: tuple[float, ...]


class RankedObservation(BaseModel):
    track_id: str
    affinity: float = Field(ge=0.0, le=1.0)


class FollowGoal(BaseModel):
    """Operator or autonomy-selected track."""

    track_id: str


class FollowCommandAccepted(BaseModel):
    accepted: bool
    rationale: str


class PersonDetector(Protocol):
    def detect_persons(self, frame: CameraFrame) -> tuple[PersonObservation, ...]: ...


class PersonReIdentifier(Protocol):
    """Re-identification / embeddings for temporal consistency."""

    def rank_candidates(
        self,
        *,
        frame: CameraFrame,
        anchors: tuple[EmbeddingVector, ...],
    ) -> tuple[RankedObservation, ...]: ...


class FollowController(Protocol):
    """Produces motion intent while keeping target centred / within safety corridor."""

    def start_follow(self, goal: FollowGoal) -> FollowCommandAccepted: ...

    def cancel(self) -> None: ...
