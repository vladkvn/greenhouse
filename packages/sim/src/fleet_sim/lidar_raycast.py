"""Ray intersection with line segments for synthetic lidar."""

from __future__ import annotations

import math
from typing import Final

from fleet_contracts.perception import LaserScan

from fleet_sim.physics import wrap_pi
from fleet_sim.world import LineSegment, PolygonWorld


def _ray_segment_intersection_distance(
    ox: float,
    oy: float,
    dx: float,
    dy: float,
    seg: LineSegment,
) -> float | None:
    """Return distance along unit ray (cos, sin) if hit, else None."""

    px, py = seg.x0, seg.y0
    sx = seg.x1 - seg.x0
    sy = seg.y1 - seg.y0
    qx = px - ox
    qy = py - oy
    denom = dx * sy - dy * sx
    if abs(denom) < 1e-12:
        return None
    t = (qx * sy - qy * sx) / denom
    u = (qx * dy - qy * dx) / denom
    if t >= 0.0 and 0.0 <= u <= 1.0:
        return t
    return None


_EVERY_DEG_STEP: Final[float] = 8.0


def synth_lidar_scan(
    world: PolygonWorld,
    *,
    ox_m: float,
    oy_m: float,
    heading_rad: float,
    range_min_m: float = 0.05,
    range_max_m: float = 10.0,
) -> LaserScan:
    """360 degree lidar in robot frame; beam 0 rad points along robot +X (forward)."""

    angle_min = -math.pi
    angle_inc = math.radians(_EVERY_DEG_STEP)
    n = int(round(2.0 * math.pi / angle_inc))
    angle_inc = 2.0 * math.pi / float(n)
    ranges: list[float] = []
    cos_h = math.cos(heading_rad)
    sin_h = math.sin(heading_rad)
    for index in range(n):
        angle_local = angle_min + float(index) * angle_inc
        lr = math.cos(angle_local)
        sr = math.sin(angle_local)
        dx = cos_h * lr - sin_h * sr
        dy = sin_h * lr + cos_h * sr
        inv_len = 1.0 / math.hypot(dx, dy)
        dx_u = dx * inv_len
        dy_u = dy * inv_len
        best: float | None = None
        for wall in world.walls:
            t_hit = _ray_segment_intersection_distance(ox_m, oy_m, dx_u, dy_u, wall)
            if t_hit is None:
                continue
            if best is None or t_hit < best:
                best = t_hit
        if best is None:
            hit = range_max_m
        else:
            hit = min(max(best, range_min_m), range_max_m)
        ranges.append(hit)

    return LaserScan(
        frame_id="sim_base",
        stamp_unix_s=0.0,
        angle_min_rad=angle_min,
        angle_increment_rad=angle_inc,
        range_min_m=range_min_m,
        range_max_m=range_max_m,
        ranges_m=ranges,
    )


def min_range_forward_cone(scan: LaserScan, *, cone_half_width_rad: float) -> float:
    """Minimum range among beams within plus-minus cone of robot forward (+X body axis)."""

    min_r = float("inf")
    for index, rng in enumerate(scan.ranges_m):
        beam_angle = scan.angle_min_rad + float(index) * scan.angle_increment_rad
        if abs(wrap_pi(beam_angle)) <= cone_half_width_rad:
            min_r = min(min_r, rng)
    return min_r if math.isfinite(min_r) else scan.range_max_m
