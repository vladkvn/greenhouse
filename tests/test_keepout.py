"""Тесты закрытых зон: KEEPOUT не пересекается маршрутом, зоны редактируются на лету,
SLOW-зона обходится при наличии альтернативы."""

from __future__ import annotations

from greenhouse.domain.geometry import Point2D, Pose2D
from greenhouse.domain.grid import CellState, MapMeta, OccupancyGrid
from greenhouse.navigation.keepout import InMemoryKeepoutRegistry, Zone, ZoneKind
from greenhouse.navigation.planning import AStarPlanner, PlanOk


def _free_grid(width_px: int, height_px: int, *, resolution_m: float = 0.5) -> OccupancyGrid:
    meta = MapMeta(
        resolution_m=resolution_m, width_px=width_px, height_px=height_px,
        origin=Point2D(x_m=0.0, y_m=0.0),
    )
    return OccupancyGrid(meta=meta, cells=[CellState.FREE] * (width_px * height_px))


def _rect_zone(zone_id: str, kind: ZoneKind, x0: float, y0: float, x1: float, y1: float) -> Zone:
    return Zone(
        zone_id=zone_id,
        kind=kind,
        polygon=(
            Point2D(x_m=x0, y_m=y0), Point2D(x_m=x1, y_m=y0),
            Point2D(x_m=x1, y_m=y1), Point2D(x_m=x0, y_m=y1),
        ),
    )


def _pose(x: float, y: float) -> Pose2D:
    return Pose2D(x_m=x, y_m=y, theta_rad=0.0)


def _in_zone(p: Pose2D, zone: Zone) -> bool:
    from greenhouse.navigation.keepout import _point_in_polygon

    return _point_in_polygon(p.point, zone.polygon)


def test_apply_to_marks_keepout_cells_occupied() -> None:
    grid = _free_grid(20, 12)
    reg = InMemoryKeepoutRegistry()
    reg.add_zone(zone=_rect_zone("z1", ZoneKind.KEEPOUT, 4.0, 2.0, 6.0, 4.0))
    blocked = reg.apply_to(grid=grid)
    row, col = blocked.meta.world_to_cell(Point2D(x_m=5.0, y_m=3.0))
    assert blocked.at(row, col) is CellState.OCCUPIED       # внутри зоны — занято
    assert grid.at(row, col) is CellState.FREE              # исходная карта не тронута


def test_keepout_route_avoids_zone_and_is_editable() -> None:
    planner = AStarPlanner()
    reg = InMemoryKeepoutRegistry()
    # Зона-перегородка поперёк прямого пути из (1,3) в (9,3), просвет сверху (y>5).
    reg.add_zone(zone=_rect_zone("wall", ZoneKind.KEEPOUT, 4.5, 0.0, 5.5, 5.0))

    grid = reg.apply_to(grid=_free_grid(20, 12))
    blocked_zone = reg.zones()[0]
    res = planner.plan(grid=grid, start=_pose(1.0, 3.0), goal=_pose(9.0, 3.0), robot_radius_m=0.0)
    assert isinstance(res, PlanOk)
    assert all(not _in_zone(wp, blocked_zone) for wp in res.path.waypoints)

    # Зону убрали на лету — маршрут снова можно вести напрямую.
    reg.remove_zone(zone_id="wall")
    grid2 = reg.apply_to(grid=_free_grid(20, 12))
    res2 = planner.plan(grid=grid2, start=_pose(1.0, 3.0), goal=_pose(9.0, 3.0), robot_radius_m=0.0)
    assert isinstance(res2, PlanOk)
    straight = _pose(1.0, 3.0).point.distance_to(Point2D(x_m=9.0, y_m=3.0))
    length = sum(
        res2.path.waypoints[i].point.distance_to(res2.path.waypoints[i + 1].point)
        for i in range(len(res2.path.waypoints) - 1)
    )
    assert length < straight + 0.5  # практически прямая


def test_slow_zone_is_avoided_when_detour_exists() -> None:
    planner = AStarPlanner()
    reg = InMemoryKeepoutRegistry()
    reg.add_zone(zone=_rect_zone("slow", ZoneKind.SLOW, 4.5, 2.0, 5.5, 4.0))
    grid = _free_grid(20, 12)
    cost = reg.cost_layer(meta=grid.meta)

    slow_zone = reg.zones()[0]
    res = planner.plan(
        grid=grid, start=_pose(1.0, 3.0), goal=_pose(9.0, 3.0), robot_radius_m=0.0, cost_layer=cost
    )
    assert isinstance(res, PlanOk)
    # Дорогую зону маршрут обходит (не ведёт узлы через её центр).
    assert all(not _in_zone(wp, slow_zone) for wp in res.path.waypoints)


def test_cost_layer_values() -> None:
    reg = InMemoryKeepoutRegistry()
    reg.add_zone(zone=_rect_zone("p", ZoneKind.PREFERRED, 0.0, 0.0, 2.0, 2.0))
    grid = _free_grid(20, 12)
    cost = reg.cost_layer(meta=grid.meta)
    row, col = grid.meta.world_to_cell(Point2D(x_m=1.0, y_m=1.0))
    assert cost[row * grid.meta.width_px + col] < 0.0    # PREFERRED дешевле
    far_row, far_col = grid.meta.world_to_cell(Point2D(x_m=8.0, y_m=5.0))
    assert cost[far_row * grid.meta.width_px + far_col] == 0.0
