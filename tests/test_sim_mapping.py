"""Тесты построения карты: evidence-сетка по лидар-рейкасту строит корректную
OccupancyGrid — стены заняты, пройденное пространство свободно, невидимое — unknown."""

from __future__ import annotations

from greenhouse.adapters.sim.loop import build_sim_robot, run
from greenhouse.adapters.sim.worlds import (
    empty_room,
    greenhouse_rows_world,
    grid_meta_for_world,
)
from greenhouse.domain.geometry import Point2D, Twist2D
from greenhouse.domain.grid import CellState, OccupancyGrid
from greenhouse.navigation.mapping import EvidenceGridMapper
from greenhouse.orchestration.modes import RobotMode
from greenhouse.sensing.interfaces import LidarScan


def _cell_at(grid: OccupancyGrid, x_m: float, y_m: float) -> CellState:
    row, col = grid.meta.world_to_cell(Point2D(x_m=x_m, y_m=y_m))
    return grid.at(row, col)


def test_single_scan_marks_walls_occupied_and_interior_free() -> None:
    # Из центра пустой комнаты 10x6 один оборот лидара видит все 4 стены.
    world = empty_room(10.0, 6.0)
    mapper = EvidenceGridMapper(meta=grid_meta_for_world(world))
    robot = build_sim_robot(world=world, start_x_m=5.0, start_y_m=3.0)
    mapper.ingest_scan(scan=robot.lidar.read_scan(), pose_xytheta=(5.0, 3.0, 0.0))
    grid = mapper.current_map()

    assert _cell_at(grid, 10.05, 3.0) is CellState.OCCUPIED   # правая стена x=10
    assert _cell_at(grid, 0.03, 3.0) is CellState.OCCUPIED   # левая стена x=0
    assert _cell_at(grid, 5.0, 3.0) is CellState.FREE        # позиция робота
    assert _cell_at(grid, 7.0, 3.0) is CellState.FREE        # пройдено лучом вправо


def test_unobserved_area_stays_unknown() -> None:
    # Кольцо отступа вокруг мира лидар не достаёт сквозь стены → остаётся unknown.
    world = empty_room(10.0, 6.0)
    mapper = EvidenceGridMapper(meta=grid_meta_for_world(world, padding_m=1.0))
    robot = build_sim_robot(world=world, start_x_m=5.0, start_y_m=3.0)
    mapper.ingest_scan(scan=robot.lidar.read_scan(), pose_xytheta=(5.0, 3.0, 0.0))
    grid = mapper.current_map()

    assert _cell_at(grid, -0.5, 3.0) is CellState.UNKNOWN    # за левой стеной
    assert _cell_at(grid, 5.0, -0.5) is CellState.UNKNOWN    # за нижней стеной


def test_begin_session_resets_to_unknown() -> None:
    world = empty_room(10.0, 6.0)
    mapper = EvidenceGridMapper(meta=grid_meta_for_world(world))
    robot = build_sim_robot(world=world, start_x_m=5.0, start_y_m=3.0)
    mapper.ingest_scan(scan=robot.lidar.read_scan(), pose_xytheta=(5.0, 3.0, 0.0))
    assert any(c is not CellState.UNKNOWN for c in mapper.current_map().cells)

    mapper.begin_session()
    assert all(c is CellState.UNKNOWN for c in mapper.current_map().cells)


def test_evidence_accumulates_stable_classification() -> None:
    # Повтор того же скана не «расшатывает» карту: стена остаётся занятой, центр — свободным.
    world = empty_room(10.0, 6.0)
    mapper = EvidenceGridMapper(meta=grid_meta_for_world(world))
    robot = build_sim_robot(world=world, start_x_m=5.0, start_y_m=3.0)
    scan = robot.lidar.read_scan()
    for _ in range(30):
        mapper.ingest_scan(scan=scan, pose_xytheta=(5.0, 3.0, 0.0))
    grid = mapper.current_map()

    assert _cell_at(grid, 10.05, 3.0) is CellState.OCCUPIED
    assert _cell_at(grid, 5.0, 3.0) is CellState.FREE


def test_no_return_beam_clears_free_without_occupied() -> None:
    # Луч без возврата (range_max по всем лучам) метит свободным до края, ничего не занимает.
    world = empty_room(10.0, 6.0)
    mapper = EvidenceGridMapper(meta=grid_meta_for_world(world))
    blank = LidarScan(
        angle_min_rad=0.0,
        angle_increment_rad=3.14159 / 2,
        range_max_m=4.0,
        ranges_m=(float("inf"),) * 4,
        stamp_s=0.0,
    )
    mapper.ingest_scan(scan=blank, pose_xytheta=(5.0, 3.0, 0.0))
    grid = mapper.current_map()

    assert _cell_at(grid, 6.0, 3.0) is CellState.FREE        # вдоль луча вправо
    assert not any(c is CellState.OCCUPIED for c in grid.cells)


def test_drive_through_world_builds_map_via_mapping_mode() -> None:
    # Критерий инкремента: проезд по миру в режиме MAPPING строит карту со стенами.
    world = empty_room(10.0, 6.0)
    mapper = EvidenceGridMapper(meta=grid_meta_for_world(world))
    robot = build_sim_robot(world=world, start_x_m=2.0, start_y_m=3.0, map_builder=mapper)
    robot.mode = RobotMode.MAPPING
    robot.motion.command(twist=Twist2D(linear_x_m_s=0.5, angular_z_rad_s=0.0))

    run(robot, ticks=80, dt_s=0.1)
    grid = mapper.end_session()

    assert robot.state.x_m > 2.0                              # робот реально ехал
    assert sum(c is CellState.OCCUPIED for c in grid.cells) > 0
    assert _cell_at(grid, 10.05, 3.0) is CellState.OCCUPIED    # упёрся/увидел правую стену
    assert _cell_at(grid, 5.0, 3.0) is CellState.FREE         # пройденный проход свободен


def test_greenhouse_bed_edge_is_occupied() -> None:
    # В мире-теплице ближняя кромка грядки (препятствия) попадает в карту как занятая.
    world = greenhouse_rows_world()
    mapper = EvidenceGridMapper(meta=grid_meta_for_world(world))
    robot = build_sim_robot(world=world, start_x_m=4.0, start_y_m=1.0, map_builder=mapper)
    robot.mode = RobotMode.MAPPING
    run(robot, ticks=5, dt_s=0.1)  # стоит на месте, но крутит лидаром каждый тик
    grid = mapper.current_map()

    # Первая грядка занимает y∈[2,3] при x∈[1, row_length-1]; кромка y≈2 видна из прохода y=1.
    assert _cell_at(grid, 4.0, 2.02) is CellState.OCCUPIED
