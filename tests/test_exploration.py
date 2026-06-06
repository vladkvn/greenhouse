"""Тесты автономного исследования (Инкремент 10): робот без априорных данных строит карту
по логике границ (frontier-based), доводит исследование до конца и не сталкивается."""

from __future__ import annotations

from greenhouse.adapters.sim.exploration import SimExplorer
from greenhouse.adapters.sim.loop import build_sim_robot
from greenhouse.adapters.sim.worlds import empty_room, grid_meta_for_world
from greenhouse.domain.geometry import Point2D
from greenhouse.domain.grid import CellState, MapMeta, OccupancyGrid
from greenhouse.navigation.mapping import EvidenceGridMapper, nearest_frontier


def _grid(rows: list[str]) -> OccupancyGrid:
    """Сетка из ASCII: '#'=занято, '.'=свободно, '?'=неизвестно (rows[0] — верх)."""
    h, w = len(rows), len(rows[0])
    meta = MapMeta(resolution_m=1.0, width_px=w, height_px=h, origin=Point2D(x_m=0.0, y_m=0.0))
    glyph = {"#": CellState.OCCUPIED, ".": CellState.FREE, "?": CellState.UNKNOWN}
    cells = [glyph[rows[h - 1 - r][c]] for r in range(h) for c in range(w)]
    return OccupancyGrid(meta=meta, cells=cells)


def test_nearest_frontier_finds_open_boundary() -> None:
    # Слева известно-свободно, справа неизвестно; граница — на стыке.
    grid = _grid([
        "....??",
        "....??",
        "....??",
    ])
    cell = nearest_frontier(grid, (1, 0), min_cells=1, clear_rad=0)
    assert cell is not None
    row, col = cell
    assert grid.at(row, col) is CellState.FREE
    assert grid.at(row, col + 1) is CellState.UNKNOWN  # действительно у края неизвестного


def test_nearest_frontier_none_when_all_known() -> None:
    grid = _grid([
        "....",
        "....",
    ])
    assert nearest_frontier(grid, (0, 0), min_cells=1, clear_rad=0) is None


def test_nearest_frontier_skips_blacklisted() -> None:
    grid = _grid([
        "....??",
        "....??",
    ])
    # Заблокируем правый край — открытых границ вне чёрного списка не останется.
    black = {(0, 3), (1, 3)}
    assert nearest_frontier(grid, (0, 0), min_cells=1, clear_rad=0, blacklist=black) is None


def test_explorer_maps_room_from_scratch_and_terminates() -> None:
    # Критерий инкремента: без априорных данных робот сам разведывает помещение.
    world = empty_room(8.0, 6.0)
    mapper = EvidenceGridMapper(meta=grid_meta_for_world(world))
    robot = build_sim_robot(world=world, start_x_m=4.0, start_y_m=3.0)
    explorer = SimExplorer(robot=robot, mapper=mapper, robot_radius_m=0.25)

    grid = explorer.explore(max_ticks=4000)

    assert explorer.coverage_known() > 0.7          # бо́льшая часть комнаты разведана
    row, col = grid.meta.world_to_cell(Point2D(x_m=4.0, y_m=3.0))
    assert grid.at(row, col) is CellState.FREE       # центр — свободно
    # Где-то найдены стены.
    assert any(c is CellState.OCCUPIED for c in grid.cells)


def test_explorer_drive_stays_collision_free() -> None:
    world = empty_room(8.0, 6.0)
    mapper = EvidenceGridMapper(meta=grid_meta_for_world(world))
    robot = build_sim_robot(world=world, start_x_m=2.0, start_y_m=2.0)
    explorer = SimExplorer(robot=robot, mapper=mapper, robot_radius_m=0.25)
    explorer.start()

    min_clear = 9.0
    for _ in range(2000):
        done = explorer.update()
        robot.engine.step(dt_s=0.1)
        min_clear = min(min_clear, world.min_clearance(point=robot.state.pose().point))
        if done:
            break
    assert min_clear >= robot.state.inscribed_radius_m - 1e-6  # ни разу не въехал в стену
