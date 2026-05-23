"""Axis-aligned rectangular rooms and wall segments for 2D ray casting."""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class LineSegment:
    x0: float
    y0: float
    x1: float
    y1: float


@dataclass(frozen=True, slots=True)
class Room:
    xmin: float
    xmax: float
    ymin: float
    ymax: float
    label: str

    def centroid(self) -> tuple[float, float]:
        return ((self.xmin + self.xmax) / 2.0, (self.ymin + self.ymax) / 2.0)


@dataclass(frozen=True, slots=True)
class PolygonWorld:
    """Free space is the union of axis-aligned room rectangles."""

    rooms: tuple[Room, ...]
    walls: tuple[LineSegment, ...]

    def contains_point(self, x: float, y: float) -> bool:
        return any(
            room.xmin <= x <= room.xmax and room.ymin <= y <= room.ymax for room in self.rooms
        )

    def room_by_label(self, label: str) -> Room:
        for room in self.rooms:
            if room.label == label:
                return room
        raise KeyError(label)


def _vertical_door_pair(
    x_wall: float,
    y_door_lo: float,
    y_door_hi: float,
    y_room_min: float,
    y_room_max: float,
) -> tuple[LineSegment, LineSegment]:
    """Two wall pieces on a vertical line, door gap between y_door_lo and y_door_hi."""

    low = LineSegment(x_wall, y_room_min, x_wall, y_door_lo)
    high = LineSegment(x_wall, y_door_hi, x_wall, y_room_max)
    return low, high


def three_rooms_line_world() -> PolygonWorld:
    """Three equal rooms in a row along +X, two vertical doorways at shared walls."""

    y_lo = 0.0
    y_hi = 4.0
    door_lo = 1.5
    door_hi = 2.5
    rooms = (
        Room(0.0, 4.0, y_lo, y_hi, "R0"),
        Room(4.0, 8.0, y_lo, y_hi, "R1"),
        Room(8.0, 12.0, y_lo, y_hi, "R2"),
    )
    w4a, w4b = _vertical_door_pair(4.0, door_lo, door_hi, y_lo, y_hi)
    w8a, w8b = _vertical_door_pair(8.0, door_lo, door_hi, y_lo, y_hi)

    outer = (
        LineSegment(0.0, y_lo, 12.0, y_lo),
        LineSegment(12.0, y_lo, 12.0, y_hi),
        LineSegment(12.0, y_hi, 0.0, y_hi),
        LineSegment(0.0, y_hi, 0.0, y_lo),
    )
    walls = outer + (w4a, w4b, w8a, w8b)
    return PolygonWorld(rooms=rooms, walls=tuple(walls))


def pairwise_room_adjacency(room_a: Room, room_b: Room) -> bool:
    """Adjacent if they touch on an axis-aligned edge (overlap of projection)."""

    if math.isclose(room_a.xmax, room_b.xmin) or math.isclose(room_a.xmin, room_b.xmax):
        y_overlap = min(room_a.ymax, room_b.ymax) - max(room_a.ymin, room_b.ymin)
        return y_overlap > 1e-3
    if math.isclose(room_a.ymax, room_b.ymin) or math.isclose(room_a.ymin, room_b.ymax):
        x_overlap = min(room_a.xmax, room_b.xmax) - max(room_a.xmin, room_b.xmin)
        return x_overlap > 1e-3
    return False


def adjacency_edges(world: PolygonWorld) -> tuple[tuple[str, str], ...]:
    pairs: list[tuple[str, str]] = []
    rooms_list = world.rooms
    for idx_i, ri in enumerate(rooms_list):
        for rj in rooms_list[idx_i + 1 :]:
            if pairwise_room_adjacency(ri, rj):
                pairs.append((ri.label, rj.label))
                pairs.append((rj.label, ri.label))
    return tuple(sorted(set(pairs), key=lambda item: (item[0], item[1])))
