"""Robot footprint vs free union of rooms minus zero-thickness wall obstacles."""

from __future__ import annotations

import math

from fleet_sim.world import LineSegment, PolygonWorld


def _squared_distance_point_to_closed_segment(px: float, py: float, seg: LineSegment) -> float:
    ax, ay = seg.x0, seg.y0
    bx, by = seg.x1, seg.y1
    abx = bx - ax
    aby = by - ay
    apx = px - ax
    apy = py - ay
    ab_sq = abx * abx + aby * aby
    if ab_sq <= 1e-18:
        return apx * apx + apy * apy
    u = max(0.0, min(1.0, (apx * abx + apy * aby) / ab_sq))
    vx = ax + abx * u - px
    vy = ay + aby * u - py
    return vx * vx + vy * vy


def _disk_intersects_wall(
    xc_m: float,
    yc_m: float,
    radius_m: float,
    seg: LineSegment,
    *,
    eps_m_sq: float,
) -> bool:
    return _squared_distance_point_to_closed_segment(xc_m, yc_m, seg) <= radius_m * radius_m + eps_m_sq


def disk_overlaps_any_wall(world: PolygonWorld, xc_m: float, yc_m: float, radius_m: float) -> bool:
    """Treat each wall polyline segment as infinitely thin obstruction for the inscribed disk."""

    if radius_m <= 0.0:
        # Point robot: disallow lying on drawn wall strokes (thin guard).
        eps_rad = max(5e-4, 1e-6)
        thresh = eps_rad * eps_rad
        for seg in world.walls:
            if _squared_distance_point_to_closed_segment(xc_m, yc_m, seg) <= thresh:
                return True
        return False
    epsilon_abs_m = max(5e-4, radius_m * 5e-4)
    eps_sq = epsilon_abs_m * epsilon_abs_m
    for seg in world.walls:
        if _disk_intersects_wall(xc_m, yc_m, radius_m, seg, eps_m_sq=eps_sq):
            return True
    return False


def footprint_circle_navigable(
    world: PolygonWorld,
    xc_m: float,
    yc_m: float,
    radius_m: float,
) -> bool:
    """Disk fits in union-of-rooms freespace AND does not cross analytic wall strokes."""

    if not world.contains_point(xc_m, yc_m):
        return False
    return not disk_overlaps_any_wall(world, xc_m, yc_m, radius_m)


def chord_footprints_navigable(
    world: PolygonWorld,
    ax_m: float,
    ay_m: float,
    bx_m: float,
    by_m: float,
    radius_m: float,
    *,
    chord_spacing_m: float,
) -> bool:
    """Chord sweep: uniformly sample disk centres along A→B; each must stay navigable."""

    dist = math.hypot(bx_m - ax_m, by_m - ay_m)
    if dist <= 1e-9:
        return footprint_circle_navigable(world, ax_m, ay_m, radius_m)
    samples = max(2, math.ceil(dist / chord_spacing_m))
    for fraction in range(samples + 1):
        blend = fraction / samples
        cx = ax_m + blend * (bx_m - ax_m)
        cy = ay_m + blend * (by_m - ay_m)
        if not footprint_circle_navigable(world, cx, cy, radius_m):
            return False
    return True
