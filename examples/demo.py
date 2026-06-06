"""Наглядное демо ядра GreenHouse в терминале (ASCII).

Две сцены:
  1. Построение карты — робот едет в режиме MAPPING, лидар отрисовывает мир в сетку.
  2. Навигация A→B — A* строит путь по построенной карте, робот едет вдоль него.

Запуск:
    python examples/demo.py
    python examples/demo.py --no-anim     # без анимации, только итоговые кадры
    python examples/demo.py --fps 30      # скорость анимации
"""

from __future__ import annotations

import argparse
import sys
import time

from greenhouse.adapters.sim import (
    PolygonWorld,
    build_sim_robot,
    empty_room,
    grid_meta_for_world,
)
from greenhouse.domain.geometry import Point2D, Pose2D, Twist2D
from greenhouse.domain.grid import CellState, OccupancyGrid
from greenhouse.navigation import AStarPlanner, EvidenceGridMapper, Path, PlanOk
from greenhouse.navigation.planning import PurePursuitLocalPlanner
from greenhouse.orchestration.modes import RobotMode

_GLYPH = {CellState.OCCUPIED: "#", CellState.FREE: "·", CellState.UNKNOWN: " "}


def render(
    grid: OccupancyGrid,
    *,
    robot: tuple[float, float] | None = None,
    goal: tuple[float, float] | None = None,
    path: Path | None = None,
) -> str:
    """Кадр карты с наложенными путём (*), целью (G) и роботом (@)."""
    m = grid.meta
    rows = [[_GLYPH[grid.at(r, c)] for c in range(m.width_px)] for r in range(m.height_px)]

    def put(x_m: float, y_m: float, ch: str) -> None:
        r, c = m.world_to_cell(Point2D(x_m=x_m, y_m=y_m))
        if m.in_bounds(r, c):
            rows[r][c] = ch

    if path is not None:
        for wp in path.waypoints:
            put(wp.x_m, wp.y_m, "*")
    if goal is not None:
        put(goal[0], goal[1], "G")
    if robot is not None:
        put(robot[0], robot[1], "@")

    # Строки печатаем сверху вниз (y растёт вверх).
    return "\n".join("".join(row) for row in reversed(rows))


def _clear() -> None:
    sys.stdout.write("\033[H\033[J")  # курсор в начало + очистка экрана


def scene_mapping(*, anim: bool, frame_s: float) -> tuple[OccupancyGrid, PolygonWorld]:
    world = empty_room(10.0, 6.0)
    mapper = EvidenceGridMapper(meta=grid_meta_for_world(world, resolution_m=0.25))
    robot = build_sim_robot(world=world, start_x_m=2.0, start_y_m=3.0, map_builder=mapper)
    robot.mode = RobotMode.MAPPING
    robot.motion.command(twist=Twist2D(linear_x_m_s=0.7, angular_z_rad_s=0.0))

    for step in range(120):
        robot.tick(dt_s=0.1)
        if anim and step % 4 == 0:
            _clear()
            print("СЦЕНА 1 — построение карты (режим MAPPING)\n")
            print(render(mapper.current_map(), robot=(robot.state.x_m, robot.state.y_m)))
            time.sleep(frame_s)

    grid = mapper.end_session()
    _clear()
    print("СЦЕНА 1 — карта построена  (# стены, · свободно, ' ' неизвестно)\n")
    print(render(grid, robot=(robot.state.x_m, robot.state.y_m)))
    print()
    return grid, world


def scene_navigation(
    grid: OccupancyGrid, world: PolygonWorld, *, anim: bool, frame_s: float
) -> None:
    start = Pose2D(x_m=2.0, y_m=3.0, theta_rad=0.0)
    goal = Pose2D(x_m=8.0, y_m=4.5, theta_rad=0.0)
    radius = 0.25

    result = AStarPlanner().plan(grid=grid, start=start, goal=goal, robot_radius_m=radius)
    if not isinstance(result, PlanOk):
        print(f"Планировщик не справился: {result.message}")
        return
    path = result.path

    robot = build_sim_robot(world=world, start_x_m=start.x_m, start_y_m=start.y_m)
    local = PurePursuitLocalPlanner(max_linear_m_s=0.6, max_angular_rad_s=1.5, goal_tol_m=0.15)
    local.set_path(path=path)
    robot.local_planner = local
    robot.mode = RobotMode.NAVIGATING

    g = (goal.x_m, goal.y_m)
    for step in range(2000):
        robot.tick(dt_s=0.1)
        if local.is_goal_reached(pose=robot.state.pose()):
            break
        if anim and step % 3 == 0:
            _clear()
            print("СЦЕНА 2 — навигация A→B (A* + следование пути)\n")
            print(render(grid, robot=(robot.state.x_m, robot.state.y_m), goal=g, path=path))
            time.sleep(frame_s)

    err = robot.state.pose().point.distance_to(goal.point)
    _clear()
    print("СЦЕНА 2 — цель достигнута  (@ робот, G цель, * путь)\n")
    print(render(grid, robot=(robot.state.x_m, robot.state.y_m), goal=g, path=path))
    print(f"\nузлов пути: {len(path.waypoints)} | ошибка позы: {err:.3f} м")


def main() -> None:
    parser = argparse.ArgumentParser(description="Демо ядра GreenHouse в терминале.")
    parser.add_argument("--no-anim", action="store_true", help="без анимации, только итог")
    parser.add_argument("--fps", type=float, default=20.0, help="кадров в секунду (анимация)")
    args = parser.parse_args()

    anim = not args.no_anim
    frame_s = 1.0 / args.fps if args.fps > 0 else 0.0

    grid, world = scene_mapping(anim=anim, frame_s=frame_s)
    if anim:
        time.sleep(0.8)
    scene_navigation(grid, world, anim=anim, frame_s=frame_s)


if __name__ == "__main__":
    main()
