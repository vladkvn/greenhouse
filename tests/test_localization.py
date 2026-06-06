"""Тесты scan-matching локализации: оценка позы по скану и карте (без истины симулятора)
отслеживает робота, исправляет начальное смещение и корректно «теряется» при несовпадении."""

from __future__ import annotations

from greenhouse.adapters.sim.loop import build_sim_robot
from greenhouse.adapters.sim.world import PolygonWorld
from greenhouse.adapters.sim.worlds import empty_room, grid_meta_for_world
from greenhouse.domain.geometry import Point2D, Pose2D, Twist2D
from greenhouse.domain.grid import CellState, OccupancyGrid
from greenhouse.navigation.localization import ScanMatchLocalizer
from greenhouse.sensing.interfaces import LidarScan, Odometry


def _occupancy(world: PolygonWorld, *, resolution_m: float = 0.1) -> OccupancyGrid:
    meta = grid_meta_for_world(world, resolution_m=resolution_m)
    cells = [
        CellState.OCCUPIED
        if world.min_clearance(point=meta.cell_to_world(r, c)) < resolution_m
        else CellState.FREE
        for r in range(meta.height_px)
        for c in range(meta.width_px)
    ]
    return OccupancyGrid(meta=meta, cells=cells)


def test_scan_match_tracks_truth_while_driving() -> None:
    world = empty_room(10.0, 6.0)
    grid = _occupancy(world)
    robot = build_sim_robot(world=world, start_x_m=3.0, start_y_m=3.0)
    loc = ScanMatchLocalizer()
    loc.set_map(grid=grid)
    loc.set_initial_pose(pose=Pose2D(x_m=3.0, y_m=3.0, theta_rad=0.0))
    robot.motion.command(twist=Twist2D(linear_x_m_s=0.5, angular_z_rad_s=0.0))

    max_err = 0.0
    for _ in range(60):
        est = loc.update(scan=robot.lidar.read_scan(), odometry=robot.odometry.read_odometry())
        robot.engine.step(dt_s=0.1)
        truth = robot.state.pose()
        max_err = max(max_err, est.pose.point.distance_to(truth.point))
    assert max_err < 0.2            # оценка не отрывается от истины
    assert not est.is_lost


def test_scan_match_corrects_initial_offset() -> None:
    world = empty_room(10.0, 6.0)
    grid = _occupancy(world)
    robot = build_sim_robot(world=world, start_x_m=5.0, start_y_m=3.0)
    loc = ScanMatchLocalizer()
    loc.set_map(grid=grid)
    loc.set_initial_pose(pose=Pose2D(x_m=5.3, y_m=3.25, theta_rad=0.0))  # смещение ~0.4 м

    for _ in range(15):  # робот стоит; scan-match подтягивает оценку к истине
        est = loc.update(scan=robot.lidar.read_scan(), odometry=robot.odometry.read_odometry())
    assert est.pose.point.distance_to(Point2D(x_m=5.0, y_m=3.0)) < 0.15


def test_localizer_reports_lost_on_blank_scan() -> None:
    world = empty_room(10.0, 6.0)
    grid = _occupancy(world)
    loc = ScanMatchLocalizer()
    loc.set_map(grid=grid)
    loc.set_initial_pose(pose=Pose2D(x_m=5.0, y_m=3.0, theta_rad=0.0))

    blank = LidarScan(
        angle_min_rad=0.0,
        angle_increment_rad=2.0 * 3.14159 / 72,
        range_max_m=8.0,
        ranges_m=(float("inf"),) * 72,  # ни одного возврата
        stamp_s=0.0,
    )
    odom = Odometry(
        pose=Pose2D(x_m=5.0, y_m=3.0, theta_rad=0.0),
        velocity=Twist2D.stop(),
        stamp_s=0.0,
    )
    est = loc.update(scan=blank, odometry=odom)
    assert est.is_lost  # нечего сопоставлять с картой → уверенность 0
