"""Тесты непрерывной актуализации карты (Инкремент 11): обновление только при уверенной
позе, отражение появившихся/исчезнувших препятствий, слой динамики, слияние и сохранение."""

from __future__ import annotations

from pathlib import Path

from greenhouse.adapters.sim.clock import SimClock
from greenhouse.adapters.sim.lidar import SimLidar
from greenhouse.adapters.sim.state import SimState
from greenhouse.adapters.sim.world import PolygonWorld, Segment
from greenhouse.adapters.sim.worlds import empty_room, grid_meta_for_world
from greenhouse.domain.geometry import Point2D
from greenhouse.domain.grid import CellState, MapMeta, OccupancyGrid
from greenhouse.navigation.mapping import (
    ConfidenceGatedMapper,
    DynamicObstacleLayer,
    EvidenceGridMapper,
    FileMapStore,
    InMemoryMapStore,
    merge_occupancy,
)


def _box_world() -> PolygonWorld:
    walls = list(empty_room(10.0, 6.0).walls)
    p = [Point2D(x_m=4.7, y_m=3.8), Point2D(x_m=5.3, y_m=3.8),
         Point2D(x_m=5.3, y_m=4.2), Point2D(x_m=4.7, y_m=4.2)]
    walls += [Segment(a=p[i], b=p[(i + 1) % 4]) for i in range(4)]
    return PolygonWorld(walls=tuple(walls), bounds_min=Point2D(x_m=0.0, y_m=0.0),
                        bounds_max=Point2D(x_m=10.0, y_m=6.0))


def _cell(grid: OccupancyGrid, x: float, y: float) -> CellState:
    row, col = grid.meta.world_to_cell(Point2D(x_m=x, y_m=y))
    return grid.at(row, col)


def test_confidence_gate_skips_uncertain_scans() -> None:
    world = empty_room(10.0, 6.0)
    gated = ConfidenceGatedMapper(
        mapper=EvidenceGridMapper(meta=grid_meta_for_world(world)), min_confidence=0.5
    )
    state = SimState(x_m=5.0, y_m=3.0)
    scan = SimLidar(world=world, state=state, clock=SimClock()).read_scan()

    assert gated.maybe_ingest(scan=scan, pose=state.pose(), confidence=0.3) is False
    assert all(c is CellState.UNKNOWN for c in gated.current_map().cells)  # неуверенно → не трогаем

    assert gated.maybe_ingest(scan=scan, pose=state.pose(), confidence=0.9) is True
    assert any(c is not CellState.UNKNOWN for c in gated.current_map().cells)


def test_map_reflects_appearing_and_disappearing_obstacle() -> None:
    meta = grid_meta_for_world(empty_room(10.0, 6.0))
    gated = ConfidenceGatedMapper(mapper=EvidenceGridMapper(meta=meta), min_confidence=0.5)
    state = SimState(x_m=5.0, y_m=3.0)
    clock = SimClock()
    lidar_empty = SimLidar(world=empty_room(10.0, 6.0), state=state, clock=clock)
    lidar_box = SimLidar(world=_box_world(), state=state, clock=clock)

    for _ in range(5):  # сначала пусто → область над роботом свободна
        gated.maybe_ingest(scan=lidar_empty.read_scan(), pose=state.pose(), confidence=1.0)
    assert _cell(gated.current_map(), 5.0, 3.8) is CellState.FREE

    for _ in range(10):  # появилось препятствие → отражается как занято
        gated.maybe_ingest(scan=lidar_box.read_scan(), pose=state.pose(), confidence=1.0)
    assert _cell(gated.current_map(), 5.0, 3.8) is CellState.OCCUPIED

    for _ in range(30):  # препятствие убрали → лог-odds очищает ячейку обратно в свободно
        gated.maybe_ingest(scan=lidar_empty.read_scan(), pose=state.pose(), confidence=1.0)
    assert _cell(gated.current_map(), 5.0, 3.8) is CellState.FREE


def test_uncertain_pose_does_not_smear_map() -> None:
    meta = grid_meta_for_world(empty_room(10.0, 6.0))
    gated = ConfidenceGatedMapper(mapper=EvidenceGridMapper(meta=meta), min_confidence=0.5)
    state = SimState(x_m=5.0, y_m=3.0)
    clock = SimClock()
    for _ in range(5):  # построили стабильную карту (область свободна)
        gated.maybe_ingest(
            scan=SimLidar(world=empty_room(10.0, 6.0), state=state, clock=clock).read_scan(),
            pose=state.pose(), confidence=1.0,
        )
    # Скан с препятствием, но НИЗКАЯ уверенность позы → не подмешиваем, карта не «засвечена».
    box_scan = SimLidar(world=_box_world(), state=state, clock=clock).read_scan()
    for _ in range(10):
        gated.maybe_ingest(scan=box_scan, pose=state.pose(), confidence=0.2)
    assert _cell(gated.current_map(), 5.0, 3.8) is CellState.FREE


def test_dynamic_layer_marks_then_decays() -> None:
    meta = grid_meta_for_world(empty_room(10.0, 6.0))
    base = OccupancyGrid(meta=meta, cells=[CellState.FREE] * (meta.width_px * meta.height_px))
    layer = DynamicObstacleLayer(meta=meta, ttl_ticks=3)
    state = SimState(x_m=5.0, y_m=3.0)
    scan = SimLidar(world=_box_world(), state=state, clock=SimClock()).read_scan()

    layer.observe(scan=scan, pose=state.pose(), base=base)
    assert layer.cells()                                   # помеха зафиксирована
    assert _cell(layer.apply_to(base), 5.0, 3.8) is CellState.OCCUPIED

    for _ in range(3):  # выветривается за ttl
        layer.decay()
    assert not layer.cells()
    assert _cell(layer.apply_to(base), 5.0, 3.8) is CellState.FREE


def test_merge_keeps_base_where_update_unknown() -> None:
    meta = MapMeta(resolution_m=1.0, width_px=3, height_px=1, origin=Point2D(x_m=0.0, y_m=0.0))
    base = OccupancyGrid(meta=meta, cells=[CellState.OCCUPIED, CellState.FREE, CellState.OCCUPIED])
    update = OccupancyGrid(meta=meta, cells=[CellState.FREE, CellState.UNKNOWN, CellState.UNKNOWN])
    merged = merge_occupancy(base, update)
    assert merged.cells == [CellState.FREE, CellState.FREE, CellState.OCCUPIED]


def test_in_memory_map_store_roundtrip() -> None:
    meta = MapMeta(resolution_m=1.0, width_px=2, height_px=1, origin=Point2D(x_m=0.0, y_m=0.0))
    grid = OccupancyGrid(meta=meta, cells=[CellState.FREE, CellState.OCCUPIED])
    store = InMemoryMapStore()
    assert store.load(label="m") is None
    store.save(label="m", grid=grid)
    loaded = store.load(label="m")
    assert loaded is not None and loaded.grid.cells == grid.cells
    assert store.list_labels() == ["m"]


def test_file_map_store_survives_restart(tmp_path: Path) -> None:
    meta = MapMeta(resolution_m=0.5, width_px=2, height_px=2, origin=Point2D(x_m=1.0, y_m=2.0))
    grid = OccupancyGrid(
        meta=meta,
        cells=[CellState.FREE, CellState.OCCUPIED, CellState.UNKNOWN, CellState.FREE],
    )
    FileMapStore(directory=tmp_path).save(label="greenhouse_main", grid=grid)
    # Новый экземпляр store (как после перезапуска процесса) видит ту же карту.
    reopened = FileMapStore(directory=tmp_path)
    loaded = reopened.load(label="greenhouse_main")
    assert loaded is not None
    assert loaded.grid.cells == grid.cells
    assert loaded.grid.meta == meta
    assert reopened.list_labels() == ["greenhouse_main"]
