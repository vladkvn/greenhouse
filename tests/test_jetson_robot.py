"""Тесты каркаса трека B (без железа): детектор-конверсия, одометрия через привод,
сборка робота и цикл tick на внедрённых заглушках.

Боевые источники (RPLIDAR, YOLO-камера, ESP32) не нужны — проверяются ШВЫ:
bearing+лидар → TargetObservation, command→одометрия, композиция build_jetson_robot,
порядок sensing→localize→orchestrator в JetsonRobot.tick.
"""

from __future__ import annotations

import math

from greenhouse.adapters.jetson.detector import JetsonTargetDetector, range_at_bearing
from greenhouse.adapters.jetson.loop import JetsonConfig, JetsonRobot, build_jetson_robot
from greenhouse.adapters.jetson.motion import JetsonMotion
from greenhouse.adapters.jetson.odometry import DeadReckonOdometry
from greenhouse.adapters.sim.clock import SimClock
from greenhouse.control.interfaces import MotionLimits
from greenhouse.domain.geometry import Point2D, Pose2D, Twist2D
from greenhouse.domain.grid import CellState, MapMeta, OccupancyGrid
from greenhouse.navigation.localization import PoseEstimate
from greenhouse.orchestration import BehaviorRegistry, IdleBehavior, Orchestrator, RobotMode
from greenhouse.sensing.interfaces import LidarScan, Odometry

_LIMITS = MotionLimits(max_linear_m_s=0.4, max_angular_rad_s=1.5, max_linear_accel_m_s2=1.0)
_INC = 2.0 * math.pi / 360.0


def _scan(hits: dict[int, float]) -> LidarScan:
    ranges = tuple(hits.get(i, math.nan) for i in range(360))
    return LidarScan(
        angle_min_rad=0.0, angle_increment_rad=_INC, range_max_m=12.0, ranges_m=ranges, stamp_s=0.0
    )


# ---------------------------------------------------------- range_at_bearing
def test_range_at_bearing_takes_median_in_sector() -> None:
    scan = _scan({0: 2.0, 1: 2.2, 359: 1.8})  # вокруг курса 0
    r = range_at_bearing(scan, 0.0, math.radians(4.0))
    assert r is not None and 1.8 <= r <= 2.2


def test_range_at_bearing_none_when_empty() -> None:
    assert range_at_bearing(_scan({}), 0.0, math.radians(4.0)) is None


# ------------------------------------------------------ JetsonTargetDetector
class _FakeVision:
    def __init__(self, bearings: list[float]) -> None:
        self._b = bearings

    def detect_bearings(self) -> list[float]:
        return self._b


class _FakeLidar:
    def __init__(self, scan: LidarScan) -> None:
        self._scan = scan

    def read_scan(self) -> LidarScan:
        return self._scan


def test_detector_picks_nearest_ranged_person() -> None:
    scan = _scan({0: 3.0, 11: 1.5})  # курс 0 -> 3 м; ~0.19 рад -> 1.5 м
    det = JetsonTargetDetector(vision=_FakeVision([0.0, 0.2]), lidar=_FakeLidar(scan), clock=SimClock())
    obs = det.detect()
    assert obs is not None
    assert math.isclose(obs.range_m, 1.5)
    assert abs(obs.bearing_rad - 0.2) < 0.05


def test_detector_none_without_persons() -> None:
    det = JetsonTargetDetector(vision=_FakeVision([]), lidar=_FakeLidar(_scan({0: 2.0})), clock=SimClock())
    assert det.detect() is None


def test_detector_none_without_lidar_range() -> None:
    det = JetsonTargetDetector(vision=_FakeVision([1.0]), lidar=_FakeLidar(_scan({})), clock=SimClock())
    assert det.detect() is None  # человек виден, но дальности нет


# ------------------------------------------- command_sink -> dead-reckoning
class _FakeLink:
    def drive(self, *, left: int, right: int) -> None: ...
    def stop(self) -> None: ...
    def close(self) -> None: ...


def test_motion_feeds_dead_reckon_odometry() -> None:
    clock = SimClock()
    odom = DeadReckonOdometry(clock=clock)
    motion = JetsonMotion(link=_FakeLink(), clock=clock, limits=_LIMITS, command_sink=odom)
    motion.command(twist=Twist2D(linear_x_m_s=0.4, angular_z_rad_s=0.0))
    clock.advance(dt_s=2.0)
    assert math.isclose(odom.read_odometry().pose.x_m, 0.8, abs_tol=1e-9)  # 0.4 м/с * 2 с


# ------------------------------------------------------- build_jetson_robot
def _free_grid() -> OccupancyGrid:
    meta = MapMeta(resolution_m=0.5, width_px=20, height_px=12, origin=Point2D(x_m=0.0, y_m=0.0))
    return OccupancyGrid(meta=meta, cells=[CellState.FREE] * (20 * 12))


class _FakeMotorLink:
    def drive(self, *, left: int, right: int) -> None: ...
    def stop(self) -> None: ...
    def close(self) -> None: ...


def test_build_with_map_wires_drive_modes() -> None:
    robot = build_jetson_robot(
        config=JetsonConfig(), grid=_free_grid(), vision=_FakeVision([]), motor_link=_FakeMotorLink(),
    )
    assert robot.orchestrator is not None
    assert robot.person_detector is not None     # с картой собран детектор следования
    assert robot.battery_frac == 1.0             # без датчика заряда — полный (CALIBRATE)


def test_build_without_map_has_no_detector() -> None:
    robot = build_jetson_robot(config=JetsonConfig(), grid=None, motor_link=_FakeMotorLink())
    assert robot.orchestrator is not None
    assert robot.person_detector is None         # без карты ездовые режимы не регистрируются


# --------------------------------------------------------- JetsonRobot.tick
class _FakeLocalizer:
    def __init__(self, pose: Pose2D) -> None:
        self._est = PoseEstimate(pose=pose, confidence=1.0, stamp_s=0.0)

    def set_map(self, *, grid: OccupancyGrid) -> None: ...
    def set_initial_pose(self, *, pose: Pose2D) -> None: ...
    def update(self, *, scan: LidarScan, odometry: Odometry) -> PoseEstimate:
        return self._est
    def latest(self) -> PoseEstimate | None:
        return self._est


class _FakeOdometry:
    def read_odometry(self) -> Odometry:
        return Odometry(pose=Pose2D(x_m=0.0, y_m=0.0, theta_rad=0.0), velocity=Twist2D.stop(), stamp_s=0.0)


class _FakeMotion:
    def __init__(self) -> None:
        self.stops = 0

    @property
    def limits(self) -> MotionLimits:
        return _LIMITS

    def command(self, *, twist: Twist2D) -> None: ...
    def stop(self) -> None:
        self.stops += 1


def test_jetson_tick_runs_sensing_localize_orchestrator() -> None:
    pose = Pose2D(x_m=1.0, y_m=2.0, theta_rad=0.0)
    motion = _FakeMotion()
    robot = JetsonRobot(
        robot_id="rover-test", clock=SimClock(), lidar=_FakeLidar(_scan({0: 5.0})),
        odometry=_FakeOdometry(), localizer=_FakeLocalizer(pose), motion=motion,
    )
    registry = BehaviorRegistry()
    registry.register(IdleBehavior(robot=robot))
    robot.orchestrator = Orchestrator(robot=robot, registry=registry)

    status = robot.tick(dt_s=0.1)
    assert status.mode is RobotMode.IDLE
    assert status.pose == pose
    assert status.battery_frac == 1.0
    assert motion.stops > 0  # IDLE удержал привод в стопе через оркестратор
