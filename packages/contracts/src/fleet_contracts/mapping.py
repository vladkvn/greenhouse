"""Mapping session contracts — orthogonal to ROS map_server specifics."""

from __future__ import annotations

from typing import Protocol

from pydantic import BaseModel, Field


class MapExtents(BaseModel):
    resolution_m: float = Field(gt=0.0)
    width_px: int = Field(gt=0)
    height_px: int = Field(gt=0)


class PersistedMap(BaseModel):
    label: str
    extents: MapExtents


class MappingSessionStart(BaseModel):
    session_id: str


class PersistMapResult(BaseModel):
    persisted: PersistedMap | None = None
    reason_if_failed: str | None = None


class MapSessionResult(BaseModel):
    session: MappingSessionStart | None = None
    failure_detail: str | None = None


class MapStore(Protocol):
    """Loads / stores map artifacts keyed by semantic labels."""

    def load(self, *, label: str) -> PersistedMap | None: ...

    def list_labels(self) -> list[str]: ...


class MapBuilder(Protocol):
    """Owning mapping loop control (started/stopped by orchestration)."""

    def begin_session(self) -> MapSessionResult: ...

    def end_session(self) -> PersistMapResult: ...

    def ingest_scan(self, *, labeled: bool) -> None: ...