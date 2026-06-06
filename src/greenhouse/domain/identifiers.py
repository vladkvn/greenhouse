"""Стабильные идентификаторы. Строки без пробелов; формат валидируется на границах."""

from __future__ import annotations

from typing import Annotated

from pydantic import Field

_ID = Field(pattern=r"^[a-zA-Z0-9_-]+$", min_length=1, max_length=64)

RobotId = Annotated[str, _ID]
"""Идентификатор робота, например 'robot-01'. Включается во все сообщения флота."""

ZoneId = Annotated[str, _ID]
"""Идентификатор зоны (keep-out / slow / preferred)."""

MissionId = Annotated[str, _ID]
"""Идентификатор миссии/задания."""

SegmentId = Annotated[str, _ID]
"""Идентификатор сегмента проезда (для координации флота)."""
