"""Тесты слияния IMU в локализацию (Инкремент 12): при проскальзывании колёс одометрия
теряет курс, а локализатор с прогнозом по гироскопу держит курс точно."""

from __future__ import annotations

import math

from greenhouse.adapters.sim.imu import SimImu
from greenhouse.adapters.sim.loop import build_sim_robot
from greenhouse.adapters.sim.worlds import greenhouse_rows_world, grid_meta_for_world
from greenhouse.domain.geometry import Twist2D
from greenhouse.domain.grid import CellState, OccupancyGrid
from greenhouse.navigation.localization import ImuFusedLocalizer, ScanMatchLocalizer


def _occupancy(world, *, res: float = 0.1) -> OccupancyGrid:
    meta = grid_meta_for_world(world, resolution_m=res)
    cells = [
        CellState.OCCUPIED if world.min_clearance(point=meta.cell_to_world(r, c)) < res
        else CellState.FREE
        for r in range(meta.height_px)
        for c in range(meta.width_px)
    ]
    return OccupancyGrid(meta=meta, cells=cells)


def _herr(a: float, b: float) -> float:
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


def test_sim_imu_reads_true_yaw_rate() -> None:
    robot = build_sim_robot(world=greenhouse_rows_world(), start_x_m=2.0, start_y_m=3.25)
    imu = SimImu(state=robot.state, clock=robot.clock)
    robot.motion.command(twist=Twist2D(linear_x_m_s=0.0, angular_z_rad_s=1.2))
    assert math.isclose(imu.read_imu().yaw_rate_rad_s, 1.2)


def test_odometry_yaw_slips_under_slip() -> None:
    robot = build_sim_robot(world=greenhouse_rows_world(), start_x_m=2.0, start_y_m=3.25, yaw_slip=1.0)
    robot.motion.command(twist=Twist2D(linear_x_m_s=0.0, angular_z_rad_s=1.2))
    for _ in range(10):
        robot.engine.step(dt_s=0.1)
    assert abs(robot.state.theta_rad) > 1.0                 # реально повернулся
    assert abs(robot.odometry.read_odometry().pose.theta_rad) < 0.05  # одометрия «не заметила»


def test_imu_fusion_keeps_heading_under_slip() -> None:
    world = greenhouse_rows_world(rows=2, row_length_m=8.0, aisle_m=1.5, bed_m=1.0)
    grid = _occupancy(world)
    robot = build_sim_robot(world=world, start_x_m=2.0, start_y_m=3.25, yaw_slip=1.0)
    imu = SimImu(state=robot.state, clock=robot.clock)

    odom_loc = ScanMatchLocalizer()
    odom_loc.set_map(grid=grid)
    odom_loc.set_initial_pose(pose=robot.state.pose())
    imu_loc = ImuFusedLocalizer(imu=imu)
    imu_loc.set_map(grid=grid)
    imu_loc.set_initial_pose(pose=robot.state.pose())

    robot.motion.command(twist=Twist2D(linear_x_m_s=0.0, angular_z_rad_s=1.5))  # быстрый поворот
    e_odom = e_imu = None
    true = robot.state.theta_rad
    for _ in range(12):
        scan = robot.lidar.read_scan()
        od = robot.odometry.read_odometry()
        e_odom = odom_loc.update(scan=scan, odometry=od)
        e_imu = imu_loc.update(scan=scan, odometry=od)
        true = robot.state.theta_rad  # истинный курс в момент этой оценки
        robot.engine.step(dt_s=0.2)

    assert e_imu is not None and e_odom is not None
    err_imu = _herr(e_imu.pose.theta_rad, true)
    err_odom = _herr(e_odom.pose.theta_rad, true)
    assert err_imu < 0.25                 # IMU-фьюжн держит курс
    assert err_imu < err_odom - 0.3       # и заметно точнее одометрии под проскальзыванием


def test_imu_fusion_tracks_with_clean_odometry() -> None:
    # Без проскальзывания фьюжн тоже корректно ведёт (sanity).
    world = greenhouse_rows_world(rows=2, row_length_m=8.0, aisle_m=1.5, bed_m=1.0)
    grid = _occupancy(world)
    robot = build_sim_robot(world=world, start_x_m=2.0, start_y_m=3.25)
    imu = SimImu(state=robot.state, clock=robot.clock)
    loc = ImuFusedLocalizer(imu=imu)
    loc.set_map(grid=grid)
    loc.set_initial_pose(pose=robot.state.pose())

    robot.motion.command(twist=Twist2D(linear_x_m_s=0.5, angular_z_rad_s=0.3))
    est = None
    true_pose = robot.state.pose()
    for _ in range(20):
        est = loc.update(scan=robot.lidar.read_scan(), odometry=robot.odometry.read_odometry())
        true_pose = robot.state.pose()
        robot.engine.step(dt_s=0.1)
    assert est is not None
    assert est.pose.point.distance_to(true_pose.point) < 0.2
    assert _herr(est.pose.theta_rad, true_pose.theta_rad) < 0.2
