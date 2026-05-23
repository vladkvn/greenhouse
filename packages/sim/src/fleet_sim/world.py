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

    def clamp_point_to_interior(self, x: float, y: float, *, margin_m: float = 0.02) -> tuple[float, float]:
        """Clamp (x,y) into the bounding box of all rooms shrinked by margin.

        For the default three-in-a-row layout the union fills that box, so the
        clamped point remains in free space. Degenerate layouts fall back to
        room centroid midpoints."""

        xmin = min(room.xmin for room in self.rooms)
        xmax = max(room.xmax for room in self.rooms)
        ymin = min(room.ymin for room in self.rooms)
        ymax = max(room.ymax for room in self.rooms)
        xm = xmin + margin_m
        xh = xmax - margin_m
        ym = ymin + margin_m
        yh = ymax - margin_m
        if xm > xh:
            xm = xh = (xmin + xmax) / 2.0
        if ym > yh:
            ym = yh = (ymin + ymax) / 2.0
        xc = min(max(float(x), xm), xh)
        yc = min(max(float(y), ym), yh)
        if self.contains_point(xc, yc):
            return (xc, yc)
        cxs: list[float] = []
        cys: list[float] = []
        for room in self.rooms:
            cx = (room.xmin + room.xmax) / 2.0
            cy = (room.ymin + room.ymax) / 2.0
            cxs.append(cx)
            cys.append(cy)
        return (sum(cxs) / len(cxs), sum(cys) / len(cys))


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


def _segments_on_horizontal_line(
    wall_y_m: float,
    x_span_lo: float,
    x_span_hi: float,
    door_openings_x_m: tuple[tuple[float, float], ...],
) -> tuple[LineSegment, ...]:
    """Pieces of a horizontal obstruction line minus rectangular door openings in X."""

    if x_span_hi <= x_span_lo + 1e-9:
        return ()
    carve: list[tuple[float, float]] = []
    for x_o_lo, x_o_hi in door_openings_x_m:
        clipped_lo = max(x_span_lo, min(x_o_lo, x_o_hi))
        clipped_hi = min(x_span_hi, max(x_o_lo, x_o_hi))
        if clipped_hi <= clipped_lo + 1e-9:
            continue
        carve.append((clipped_lo, clipped_hi))
    carve.sort(key=lambda interval: interval[0])
    merged_ranges: list[tuple[float, float]] = []
    pass_index = 0
    while pass_index < len(carve):
        run_lo = carve[pass_index][0]
        run_hi = carve[pass_index][1]
        next_index = pass_index + 1
        while next_index < len(carve) and carve[next_index][0] <= run_hi + 7e-2:
            run_hi = max(run_hi, carve[next_index][1])
            next_index += 1
        merged_ranges.append((run_lo, run_hi))
        pass_index = next_index
    out: list[LineSegment] = []
    crawl_x = x_span_lo
    merge_index = 0
    while merge_index < len(merged_ranges):
        gap_left, gap_right = merged_ranges[merge_index]
        if gap_left > crawl_x + 8e-3:
            out.append(LineSegment(crawl_x, wall_y_m, gap_left, wall_y_m))
        crawl_x = max(crawl_x, gap_right)
        merge_index += 1
    if crawl_x < x_span_hi - 8e-3:
        out.append(LineSegment(crawl_x, wall_y_m, x_span_hi, wall_y_m))
    return tuple(out)


def world_axis_aligned_bbox(world: PolygonWorld) -> tuple[float, float, float, float]:
    xmin = min(r.xmin for r in world.rooms)
    xmax = max(r.xmax for r in world.rooms)
    ymin = min(r.ymin for r in world.rooms)
    ymax = max(r.ymax for r in world.rooms)
    return xmin, xmax, ymin, ymax


def world_bounding_extent_diagonal_m(world: PolygonWorld) -> float:
    """Straight-line span of hull corners (lid / detection range heuristic)."""

    xmin, xmax, ymin, ymax = world_axis_aligned_bbox(world)
    return math.hypot(xmax - xmin, ymax - ymin)


def world_plot_bounds_xy(
    world: PolygonWorld,
    *,
    margin_left_m: float = 2.75,
    margin_right_m: float = 4.25,
    margin_bottom_m: float = 2.75,
    margin_top_m: float = 6.75,
) -> tuple[tuple[float, float], tuple[float, float]]:
    xmin, xmax, ymin, ymax = world_axis_aligned_bbox(world)
    return (
        (xmin - margin_left_m, xmax + margin_right_m),
        (ymin - margin_bottom_m, ymax + margin_top_m),
    )


def greenhouse_parallel_rows_world(
    *,
    length_x_m: float = 56.0,
    n_parallel_aisles: int = 6,
    aisle_width_m: float = 2.05,
    bed_strip_between_m: float = 0.38,
    door_centers_x_m: tuple[float, ...] = (9.75, 28.75, 47.95),
    door_half_width_m: float = 0.7,
    cross_passage_half_width_m: float = 0.76,
) -> PolygonWorld:
    """Toy greenhouse footprint: parallel long walkways (+X) with bed strips and door gaps.

    Each walkway is navigable rectangle ``[0, length_x_m] × aisles``. Narrow horizontal
    wall segments occupy the obstacle strip between walkways, carving aligned door gaps.
    Thin cross rectangles under doors connect neighbouring aisles.
    """

    if n_parallel_aisles < 2:
        raise ValueError("n_parallel_aisles must be >= 2")

    door_intervals_x: tuple[tuple[float, float], ...] = tuple(
        (cx - door_half_width_m, cx + door_half_width_m) for cx in door_centers_x_m
    )
    accumulated_rooms: list[Room] = []
    accumulated_walls: list[LineSegment] = []
    next_label = 0
    walker_y = 0.0
    for aisle_index in range(n_parallel_aisles):
        y_floor = walker_y
        y_ceiling = walker_y + aisle_width_m
        accumulated_rooms.append(
            Room(0.0, length_x_m, y_floor, y_ceiling, f"bay-{next_label:03d}"),
        )
        next_label += 1
        walker_y = y_ceiling
        if aisle_index == n_parallel_aisles - 1:
            break
        stripe_lo_y = walker_y
        stripe_hi_y = walker_y + bed_strip_between_m
        obstruction_y_wall = stripe_lo_y + 0.5 * bed_strip_between_m
        for door_cx in door_centers_x_m:
            lx = door_cx - cross_passage_half_width_m
            rx = door_cx + cross_passage_half_width_m
            accumulated_rooms.append(Room(lx, rx, stripe_lo_y, stripe_hi_y, f"lnk-{next_label:03d}"))
            next_label += 1
        accumulated_walls.extend(
            _segments_on_horizontal_line(obstruction_y_wall, 0.0, length_x_m, door_intervals_x),
        )
        walker_y = stripe_hi_y

    hull_x_min = 0.0
    hull_y_min = min(r.ymin for r in accumulated_rooms)
    hull_x_max = length_x_m
    hull_y_max = max(r.ymax for r in accumulated_rooms)
    outer_box = (
        LineSegment(hull_x_min, hull_y_min, hull_x_max, hull_y_min),
        LineSegment(hull_x_max, hull_y_min, hull_x_max, hull_y_max),
        LineSegment(hull_x_max, hull_y_max, hull_x_min, hull_y_max),
        LineSegment(hull_x_min, hull_y_max, hull_x_min, hull_y_min),
    )
    return PolygonWorld(
        rooms=tuple(accumulated_rooms),
        walls=tuple(list(outer_box) + accumulated_walls),
    )


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
