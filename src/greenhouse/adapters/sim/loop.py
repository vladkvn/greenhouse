"""Цикл управления симуляции и сборка готового робота.

Один тик: прочитать датчики → локализоваться → выдать команду по режиму → шаг физики
→ собрать телеметрию (RobotStatus). В Инкременте 1 поведение тривиально — Idle (стоп);
последующие инкременты подключат планирование/следование, не меняя структуру цикла.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

from greenhouse.adapters.sim.clock import SimClock
from greenhouse.adapters.sim.engine import SimEngine
from greenhouse.adapters.sim.lidar import SimLidar
from greenhouse.adapters.sim.localization import SimTruthLocalizer
from greenhouse.adapters.sim.motion import SimMotion
from greenhouse.adapters.sim.odometry import SimOdometry
from greenhouse.adapters.sim.state import SimState
from greenhouse.adapters.sim.world import PolygonWorld
from greenhouse.control.interfaces import MotionController
from greenhouse.domain.geometry import Point2D, Pose2D
from greenhouse.domain.identifiers import RobotId
from greenhouse.navigation.following import PersonFollower
from greenhouse.navigation.localization import Localizer, PoseEstimate
from greenhouse.navigation.mapping import ConfidenceGatedMapper, MapBuilder
from greenhouse.navigation.planning import LocalPlanner
from greenhouse.orchestration.interfaces import RobotStatus
from greenhouse.orchestration.modes import RobotMode
from greenhouse.sensing.interfaces import LidarScan, LidarSource, OdometrySource, TargetDetector

if TYPE_CHECKING:
    from greenhouse.orchestration.orchestrator import Orchestrator


@dataclass
class SimRobot:
    """Собранный из адаптеров робот: всё, что нужно для одного тика цикла."""

    robot_id: RobotId
    state: SimState
    clock: SimClock
    engine: SimEngine
    lidar: LidarSource
    odometry: OdometrySource
    localizer: Localizer
    motion: MotionController
    mode: RobotMode = RobotMode.IDLE
    map_builder: MapBuilder | None = None
    local_planner: LocalPlanner | None = None
    person_detector: TargetDetector | None = None
    follower: PersonFollower | None = None
    live_mapper: ConfidenceGatedMapper | None = None  # непрерывная актуализация на ходу
    orchestrator: Orchestrator | None = None  # если задан — режимом правит единый FSM

    # SimRobot удовлетворяет RobotContext (read-only телеметрия для оркестратора).
    @property
    def battery_frac(self) -> float:
        return self.state.battery_frac

    @property
    def latest_pose(self) -> Pose2D | None:
        est = self.localizer.latest()
        return est.pose if est is not None else None

    @property
    def localization_lost(self) -> bool:
        est = self.localizer.latest()
        return est.is_lost if est is not None else False

    def tick(self, *, dt_s: float) -> RobotStatus:
        scan = self.lidar.read_scan()
        odom = self.odometry.read_odometry()
        estimate = self.localizer.update(scan=scan, odometry=odom)

        # Непрерывная актуализация карты в ездовых режимах (с защитой по уверенности позы).
        if self.live_mapper is not None and self.mode in (RobotMode.NAVIGATING, RobotMode.FOLLOWING):
            self.live_mapper.maybe_ingest(
                scan=scan, pose=estimate.pose, confidence=estimate.confidence
            )

        # Режимом правит либо единый FSM (Orchestrator), либо встроенный диспетчер (legacy).
        if self.orchestrator is not None:
            self.orchestrator.tick(dt_s=dt_s)
        else:
            self._dispatch_legacy(scan=scan, estimate=estimate)

        self.engine.step(dt_s=dt_s)

        return RobotStatus(
            robot_id=self.robot_id,
            mode=self.mode,
            pose=estimate.pose,
            battery_frac=self.state.battery_frac,
            last_error=None if not estimate.is_lost else "localization_lost",
        )

    def _dispatch_legacy(self, *, scan: LidarScan, estimate: PoseEstimate) -> None:
        """Встроенный диспетчер режимов (до оркестратора). Idle=стоп; Mapping=карта; Nav=путь."""
        if self.mode is RobotMode.IDLE:
            self.motion.stop()
        elif self.mode is RobotMode.MAPPING and self.map_builder is not None:
            p = estimate.pose
            self.map_builder.ingest_scan(scan=scan, pose_xytheta=(p.x_m, p.y_m, p.theta_rad))
        elif self.mode is RobotMode.NAVIGATING and self.local_planner is not None:
            if self.local_planner.is_goal_reached(pose=estimate.pose):
                self.motion.stop()
            else:
                self.motion.command(
                    twist=self.local_planner.compute_command(pose=estimate.pose, scan=scan)
                )
        elif (
            self.mode is RobotMode.FOLLOWING
            and self.follower is not None
            and self.person_detector is not None
        ):
            self.motion.command(twist=self.follower.update(observation=self.person_detector.detect()))


def build_sim_robot(
    *,
    world: PolygonWorld,
    robot_id: RobotId = "robot-01",
    start_x_m: float = 1.0,
    start_y_m: float = 1.0,
    start_theta_rad: float = 0.0,
    map_builder: MapBuilder | None = None,
    dock: Point2D | None = None,
    battery_drain_per_s: float = 0.0005,
    yaw_slip: float = 0.0,
) -> SimRobot:
    """Собрать робота со всеми sim-адаптерами в согласованном состоянии."""
    state = SimState(x_m=start_x_m, y_m=start_y_m, theta_rad=start_theta_rad)
    clock = SimClock()
    return SimRobot(
        robot_id=robot_id,
        state=state,
        clock=clock,
        engine=SimEngine(
            world=world, state=state, clock=clock, dock=dock,
            battery_drain_per_s=battery_drain_per_s,
        ),
        lidar=SimLidar(world=world, state=state, clock=clock),
        odometry=SimOdometry(state=state, clock=clock, yaw_slip=yaw_slip),
        localizer=SimTruthLocalizer(state=state, clock=clock),
        motion=SimMotion(state=state),
        map_builder=map_builder,
    )


def run(robot: SimRobot, *, ticks: int, dt_s: float = 0.1) -> list[RobotStatus]:
    """Прогнать цикл `ticks` шагов, вернуть поток телеметрии."""
    return [robot.tick(dt_s=dt_s) for _ in range(ticks)]
