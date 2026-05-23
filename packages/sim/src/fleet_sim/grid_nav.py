"""Grid occupancy + A* respecting room union and analytic wall strokes (`PolygonWorld.walls`)."""

from __future__ import annotations

import heapq
import math
from collections import deque
from typing import Final

from fleet_sim.footprint import chord_footprints_navigable, footprint_circle_navigable
from fleet_sim.world import PolygonWorld

_NEIGHBOR_STEP: Final[tuple[tuple[int, int], ...]] = ((-1, 0), (1, 0), (0, -1), (0, 1))


def world_nav_bounds(world: PolygonWorld, *, margin_m: float) -> tuple[float, float, float, float]:
    """Interior axis-aligned bbox (usable for sampling grid origins)."""

    xmin = min(room.xmin for room in world.rooms)
    xmax = max(room.xmax for room in world.rooms)
    ymin = min(room.ymin for room in world.rooms)
    ymax = max(room.ymax for room in world.rooms)
    return (
        xmin + margin_m,
        ymin + margin_m,
        xmax - margin_m,
        ymax - margin_m,
    )


def simplify_polyline_through_free(
    world: PolygonWorld,
    path_m: tuple[tuple[float, float], ...],
    *,
    robot_inscribed_radius_m: float,
    spacing_m: float = 0.08,
) -> list[tuple[float, float]]:
    """Greedy shortening: chord allowed only if whole inscribed disk clears rooms and walls."""

    if len(path_m) <= 2:
        return list(path_m)
    out: list[tuple[float, float]] = [path_m[0]]
    anchor_idx = 0
    while anchor_idx < len(path_m) - 1:
        farthest = anchor_idx + 1
        for candidate in range(len(path_m) - 1, anchor_idx, -1):
            xa, ya = path_m[anchor_idx]
            xb, yb = path_m[candidate]
            if chord_footprints_navigable(
                world,
                xa,
                ya,
                xb,
                yb,
                robot_inscribed_radius_m,
                chord_spacing_m=spacing_m,
            ):
                farthest = candidate
                break
        out.append(path_m[farthest])
        anchor_idx = farthest
    return out


class _OccupancyGrid:
    __slots__ = (
        "cell_m",
        "nx",
        "ny",
        "origin_x_m",
        "origin_y_m",
        "robot_inscribed_radius_m",
        "world",
    )

    def __init__(
        self,
        world: PolygonWorld,
        *,
        cell_m: float,
        interior_margin_m: float,
        robot_inscribed_radius_m: float,
    ) -> None:
        self.world = world
        self.cell_m = cell_m
        self.robot_inscribed_radius_m = robot_inscribed_radius_m
        x0i, y0i, x1i, y1i = world_nav_bounds(world, margin_m=interior_margin_m)
        width_i = max(0.0, x1i - x0i)
        height_i = max(0.0, y1i - y0i)
        nx = max(1, math.ceil(width_i / cell_m))
        ny = max(1, math.ceil(height_i / cell_m))
        self.nx = nx
        self.ny = ny
        self.origin_x_m = x0i + 0.5 * cell_m
        self.origin_y_m = y0i + 0.5 * cell_m

    def center(self, ix: int, iy: int) -> tuple[float, float]:
        return (
            self.origin_x_m + float(ix) * self.cell_m,
            self.origin_y_m + float(iy) * self.cell_m,
        )

    def traversable_ij(self, ix: int, iy: int) -> bool:
        if not (0 <= ix < self.nx and 0 <= iy < self.ny):
            return False
        cx, cy = self.center(ix, iy)
        return footprint_circle_navigable(self.world, cx, cy, self.robot_inscribed_radius_m)

    def ij_from_xy(self, x_m: float, y_m: float) -> tuple[int, int]:
        ix = int(round((x_m - self.origin_x_m) / self.cell_m))
        iy = int(round((y_m - self.origin_y_m) / self.cell_m))
        return ix, iy

    def nearest_traversable_ij(self, ix: int, iy: int) -> tuple[int, int] | None:
        """Breadth-first from a clamped seed cell to nearest traversable centre."""

        cix = max(0, min(self.nx - 1, ix))
        ciy = max(0, min(self.ny - 1, iy))
        if self.traversable_ij(cix, ciy):
            return (cix, ciy)
        q: deque[tuple[int, int]] = deque()
        q.append((cix, ciy))
        seen: set[tuple[int, int]] = {(cix, ciy)}
        while q:
            ci, cj = q.popleft()
            if self.traversable_ij(ci, cj):
                return (ci, cj)
            for di, dj in _NEIGHBOR_STEP:
                ni, nj = ci + di, cj + dj
                if not (0 <= ni < self.nx and 0 <= nj < self.ny):
                    continue
                if (ni, nj) in seen:
                    continue
                seen.add((ni, nj))
                q.append((ni, nj))
        return None


def plan_path_through_free_space(
    world: PolygonWorld,
    start_xy_m: tuple[float, float],
    goal_xy_m: tuple[float, float],
    *,
    cell_m: float = 0.22,
    margin_m: float = 0.04,
    robot_inscribed_radius_m: float = 0.20,
    simplify_spacing_m: float = 0.08,
) -> list[tuple[float, float]]:
    """Discrete A* on cell centres; inscribed disk clears rooms and wall strokes.

    `margin_m` adds interior bbox clearance on top of the robot radius. If unreachable,
    returns a single waypoint at `goal_xy_m`.
    """

    interior_margin_m = margin_m + robot_inscribed_radius_m
    grid = _OccupancyGrid(
        world,
        cell_m=cell_m,
        interior_margin_m=interior_margin_m,
        robot_inscribed_radius_m=robot_inscribed_radius_m,
    )
    sx_raw, sy_raw = start_xy_m
    gx_raw, gy_raw = goal_xy_m
    start_ij = grid.nearest_traversable_ij(*grid.ij_from_xy(sx_raw, sy_raw))
    goal_ij = grid.nearest_traversable_ij(*grid.ij_from_xy(gx_raw, gy_raw))
    if start_ij is None or goal_ij is None:
        return [goal_xy_m]

    def manhattan_heuristic(a_ix: int, a_iy: int) -> float:
        return grid.cell_m * (abs(a_ix - goal_ij[0]) + abs(a_iy - goal_ij[1]))

    open_heap: list[tuple[float, int, tuple[int, int]]] = []
    tie = 0
    heapq.heappush(open_heap, (manhattan_heuristic(*start_ij), tie, start_ij))
    tie += 1
    came_from: dict[tuple[int, int], tuple[int, int]] = {}
    g_score: dict[tuple[int, int], float] = {start_ij: 0.0}
    closed: set[tuple[int, int]] = set()

    visited_goal: tuple[int, int] | None = None
    while open_heap:
        _, _, current = heapq.heappop(open_heap)
        if current in closed:
            continue
        closed.add(current)
        if current == goal_ij:
            visited_goal = current
            break
        c_ix, c_iy = current
        cg = g_score[current]
        for dx, dy in _NEIGHBOR_STEP:
            n_ix, n_iy = c_ix + dx, c_iy + dy
            neigh = (n_ix, n_iy)
            if not grid.traversable_ij(n_ix, n_iy):
                continue
            tentative = cg + grid.cell_m
            prior = g_score.get(neigh)
            if prior is None or tentative < prior:
                g_score[neigh] = tentative
                came_from[neigh] = current
                f_prio_entry = tentative + manhattan_heuristic(n_ix, n_iy)
                heapq.heappush(open_heap, (f_prio_entry, tie, neigh))
                tie += 1

    if visited_goal is None:
        return [goal_xy_m]

    cells: list[tuple[int, int]] = []
    crawl: tuple[int, int] | None = visited_goal
    while crawl is not None:
        cells.append(crawl)
        if crawl == start_ij:
            break
        crawl = came_from.get(crawl)
    if cells[-1] != start_ij:
        return [goal_xy_m]
    cells.reverse()
    points: list[tuple[float, float]] = [grid.center(ix, iy) for ix, iy in cells]
    simplified = simplify_polyline_through_free(
        world,
        tuple(points),
        robot_inscribed_radius_m=robot_inscribed_radius_m,
        spacing_m=simplify_spacing_m,
    )
    if simplified:
        simplified[-1] = goal_xy_m
    return simplified
