"""Тесты команды EditKeepout (требование 2): оператор ставит/снимает зону на лету.

Юнит: команда правит общий KeepoutRegistry. Интеграция: KEEPOUT-зона поперёк прямого
пути заставляет GoTo объехать её (тот же реестр у оркестратора и навигатора, навигатор
перечитывает зоны на перепланировании).
"""

from __future__ import annotations

from greenhouse.adapters.sim.loop import build_sim_robot
from greenhouse.adapters.sim.worlds import empty_room
from greenhouse.domain.geometry import Point2D, Pose2D
from greenhouse.domain.grid import CellState, MapMeta, OccupancyGrid
from greenhouse.navigation.keepout import InMemoryKeepoutRegistry, Zone, ZoneKind
from greenhouse.navigation.navigator import StepwiseNavigator
from greenhouse.orchestration import (
    BehaviorRegistry,
    EditKeepout,
    GoTo,
    IdleBehavior,
    NavigatingBehavior,
    Orchestrator,
    RobotMode,
)


def _free_grid(w: int, h: int, *, res: float = 0.5) -> OccupancyGrid:
    meta = MapMeta(resolution_m=res, width_px=w, height_px=h, origin=Point2D(x_m=0.0, y_m=0.0))
    return OccupancyGrid(meta=meta, cells=[CellState.FREE] * (w * h))


def _zone(zid: str, x0: float, y0: float, x1: float, y1: float, kind: ZoneKind = ZoneKind.KEEPOUT) -> Zone:
    poly = (
        Point2D(x_m=x0, y_m=y0), Point2D(x_m=x1, y_m=y0),
        Point2D(x_m=x1, y_m=y1), Point2D(x_m=x0, y_m=y1),
    )
    return Zone(zone_id=zid, kind=kind, polygon=poly)


def _build(grid: OccupancyGrid):
    keepout = InMemoryKeepoutRegistry()
    robot = build_sim_robot(world=empty_room(10.0, 6.0), start_x_m=2.0, start_y_m=3.0)
    nav = StepwiseNavigator(
        robot=robot, grid=grid, robot_radius_m=0.2, goal_tol_m=0.25, keepout=keepout, replan_every=6
    )
    registry = BehaviorRegistry()
    registry.register(IdleBehavior(robot=robot))
    registry.register(NavigatingBehavior(navigator=nav))
    robot.orchestrator = Orchestrator(robot=robot, registry=registry, keepout=keepout)
    return robot, keepout


def test_edit_keepout_add_and_remove_updates_registry() -> None:
    robot, keepout = _build(_free_grid(20, 12))
    assert robot.orchestrator is not None
    robot.orchestrator.handle(command=EditKeepout(op="add", zone=_zone("z1", 4.5, 0.0, 5.5, 4.0)))
    assert [z.zone_id for z in keepout.zones()] == ["z1"]
    robot.orchestrator.handle(command=EditKeepout(op="remove", zone_id="z1"))
    assert keepout.zones() == []


def test_keepout_zone_forces_detour() -> None:
    # Зона-«стена» x∈[4.5,5.5], y∈[0,4]; проход сверху (y>4). Прямой путь по y=3 перекрыт.
    robot, _ = _build(_free_grid(20, 12))
    assert robot.orchestrator is not None
    robot.orchestrator.handle(command=EditKeepout(op="add", zone=_zone("wall", 4.5, 0.0, 5.5, 4.0)))
    robot.orchestrator.handle(command=GoTo(goal=Pose2D(x_m=8.0, y_m=3.0, theta_rad=0.0)))

    max_y = 3.0
    reached = False
    for _ in range(1200):
        robot.tick(dt_s=0.1)
        max_y = max(max_y, robot.state.y_m)
        if robot.mode is RobotMode.IDLE:
            reached = True
            break
    assert reached
    assert robot.state.pose().point.distance_to(Point2D(x_m=8.0, y_m=3.0)) <= 0.35
    assert max_y > 3.8  # объехал зону сверху, а не поехал сквозь неё
