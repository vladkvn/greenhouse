"""Тесты MappingBehavior (инкр.6) через интегрированный путь оркестратора.

Команда StartMapping переводит в MAPPING; поведение строит карту с нуля frontier-
исследованием (та же логика, что SimExplorer, но по localizer.latest()), и по исчерпании
границ запрашивает IDLE. Зеркалит test_exploration (empty_room, старт из центра).
"""

from __future__ import annotations

from greenhouse.adapters.sim.loop import build_sim_robot
from greenhouse.adapters.sim.worlds import empty_room, grid_meta_for_world
from greenhouse.domain.geometry import Point2D
from greenhouse.domain.grid import CellState, OccupancyGrid
from greenhouse.navigation.mapping import EvidenceGridMapper, InMemoryMapStore
from greenhouse.orchestration import (
    BehaviorRegistry,
    IdleBehavior,
    MappingBehavior,
    Orchestrator,
    RobotMode,
    StartMapping,
)


def _coverage(grid: OccupancyGrid) -> float:
    return sum(c is not CellState.UNKNOWN for c in grid.cells) / len(grid.cells)


def test_mapping_builds_room_then_idles() -> None:
    world = empty_room(8.0, 6.0)
    mapper = EvidenceGridMapper(meta=grid_meta_for_world(world))
    robot = build_sim_robot(world=world, start_x_m=4.0, start_y_m=3.0)
    registry = BehaviorRegistry()
    registry.register(IdleBehavior(robot=robot))
    registry.register(MappingBehavior(robot=robot, mapper=mapper, robot_radius_m=0.25))
    robot.orchestrator = Orchestrator(robot=robot, registry=registry)

    assert robot.orchestrator is not None
    robot.orchestrator.handle(command=StartMapping())
    assert robot.mode is RobotMode.MAPPING

    done = False
    for _ in range(4000):
        robot.tick(dt_s=0.1)
        if robot.mode is RobotMode.IDLE:
            done = True
            break
    assert done, "исследование не завершилось переходом в IDLE"

    grid = mapper.current_map()
    assert _coverage(grid) > 0.7  # бо́льшая часть комнаты разведана
    row, col = grid.meta.world_to_cell(Point2D(x_m=4.0, y_m=3.0))
    assert grid.at(row, col) is CellState.FREE
    assert any(c is CellState.OCCUPIED for c in grid.cells)


def test_mapping_saves_map_on_completion() -> None:
    world = empty_room(8.0, 6.0)
    mapper = EvidenceGridMapper(meta=grid_meta_for_world(world))
    store = InMemoryMapStore()
    robot = build_sim_robot(world=world, start_x_m=4.0, start_y_m=3.0)
    registry = BehaviorRegistry()
    registry.register(IdleBehavior(robot=robot))
    registry.register(MappingBehavior(
        robot=robot, mapper=mapper, robot_radius_m=0.25, map_store=store, map_label="room",
    ))
    robot.orchestrator = Orchestrator(robot=robot, registry=registry)

    assert robot.orchestrator is not None
    robot.orchestrator.handle(command=StartMapping())
    for _ in range(4000):
        robot.tick(dt_s=0.1)
        if robot.mode is RobotMode.IDLE:
            break

    saved = store.load(label="room")  # карта сохранена и переживёт перезапуск
    assert saved is not None and _coverage(saved.grid) > 0.7
