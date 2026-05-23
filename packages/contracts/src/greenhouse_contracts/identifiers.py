"""Stable identifiers exchanged between robot, simulator, and backend."""

from __future__ import annotations

from typing import Annotated

from pydantic import Field

RobotIdField = Annotated[
    str,
    Field(min_length=1, max_length=128, pattern=r"^[a-zA-Z0-9_-]+$"),
]
