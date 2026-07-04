"""JetsonRobot — боевой цикл управления и сборка робота из адаптеров.

Зеркало SimRobot, но БЕЗ шага физики (engine.step): мир реальный, его двигают моторы.
Один тик: датчики → локализация → (актуализация карты) → единый FSM (Orchestrator.tick) →
телеметрия. Тот же Orchestrator и те же поведения, что проверены в симуляции, — на железе
меняются только адаптеры-источники.

`build_jetson_robot` — композиционный корень: собирает реальные адаптеры и поведения.
Места, требующие замеров на конкретном шасси, помечены `# CALIBRATE`.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Literal

from greenhouse.adapters.jetson.clock import WallClock
from greenhouse.adapters.jetson.detector import JetsonTargetDetector, PersonVision, build_follow_me_vision
from greenhouse.adapters.jetson.imu import ZeroImu
from greenhouse.adapters.jetson.lidar import JetsonLidar
from greenhouse.adapters.jetson.motion import (
    Esp32SerialLink,
    Esp32UdpLink,
    JetsonMotion,
    MotorLink,
)
from greenhouse.adapters.jetson.odometry import DeadReckonOdometry
from greenhouse.control.interfaces import MotionController, MotionLimits
from greenhouse.domain.geometry import Pose2D
from greenhouse.domain.grid import OccupancyGrid
from greenhouse.domain.identifiers import RobotId
from greenhouse.navigation.following import PersonFollower
from greenhouse.navigation.keepout import InMemoryKeepoutRegistry, KeepoutRegistry
from greenhouse.navigation.localization import (
    ImuFusedLocalizer,
    Localizer,
    ScanMatchLocalizer,
)
from greenhouse.navigation.mapping import ConfidenceGatedMapper, EvidenceGridMapper, MapStore
from greenhouse.navigation.navigator import StepwiseNavigator
from greenhouse.orchestration.behaviors import (
    ChargingBehavior,
    FollowingBehavior,
    IdleBehavior,
    MappingBehavior,
    NavigatingBehavior,
)
from greenhouse.orchestration.interfaces import RobotStatus
from greenhouse.orchestration.modes import RobotMode
from greenhouse.orchestration.orchestrator import Orchestrator
from greenhouse.orchestration.registry import BehaviorRegistry
from greenhouse.orchestration.supervisor import Supervisor
from greenhouse.runtime.clock import Clock
from greenhouse.sensing.interfaces import BatterySource, LidarSource, OdometrySource, TargetDetector

if TYPE_CHECKING:
    from greenhouse.navigation.planning import LocalPlanner


@dataclass
class JetsonRobot:
    """Собранный из реальных адаптеров робот. Удовлетворяет RobotContext и RobotLike."""

    robot_id: RobotId
    clock: Clock
    lidar: LidarSource
    odometry: OdometrySource
    localizer: Localizer
    motion: MotionController
    mode: RobotMode = RobotMode.IDLE
    person_detector: TargetDetector | None = None
    live_mapper: ConfidenceGatedMapper | None = None
    battery: BatterySource | None = None
    local_planner: LocalPlanner | None = None
    orchestrator: Orchestrator | None = None

    # --- RobotContext (телеметрия для оркестратора) ---
    @property
    def battery_frac(self) -> float:
        return self.battery.read_battery() if self.battery is not None else 1.0  # CALIBRATE: датчик заряда

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

        if self.live_mapper is not None and self.mode in (RobotMode.NAVIGATING, RobotMode.FOLLOWING):
            self.live_mapper.maybe_ingest(
                scan=scan, pose=estimate.pose, confidence=estimate.confidence
            )

        if self.orchestrator is not None:
            self.orchestrator.tick(dt_s=dt_s)  # единый FSM правит режимом и приводом
        # БЕЗ engine.step — мир реальный

        return RobotStatus(
            robot_id=self.robot_id,
            mode=self.mode,
            pose=estimate.pose,
            battery_frac=self.battery_frac,
            last_error=None if not estimate.is_lost else "localization_lost",
        )


@dataclass
class JetsonConfig:
    """Параметры сборки. Помеченные CALIBRATE — измерить на конкретном шасси."""

    robot_id: RobotId = "rover-01"
    motor_transport: Literal["serial", "udp"] = "serial"  # ESP подключён по USB -> serial
    esp32_serial_port: str = "/dev/ttyUSB0"        # CALIBRATE: порт ESP (см. /dev/serial/by-id/, CH340)
    esp32_serial_baud: int = 115200
    esp32_ip: str = "192.168.1.50"                 # для motor_transport="udp"
    esp32_port: int = 4210
    lidar_port: str = "/dev/ttyUSB0"
    lidar_baud: int = 115200
    lidar_forward_bin_deg: float = 0.0           # CALIBRATE: какой сырой бин лидара смотрит вперёд
    lidar_clockwise: bool = True                 # CALIBRATE: направление вращения RPLIDAR
    lidar_masked_sectors: tuple[tuple[float, float], ...] = ()  # CALIBRATE: секторы self-hit корпуса
    lidar_max_range_m: float = 12.0
    wheel_base_m: float = 0.18                    # CALIBRATE: колея шасси
    max_linear_m_s: float = 0.4                   # CALIBRATE: безопасная макс. скорость
    max_angular_rad_s: float = 1.5                # CALIBRATE
    max_linear_accel_m_s2: float = 1.0            # CALIBRATE
    pwm_max: int = 220                            # CALIBRATE: ШИМ при max_linear
    pwm_min_move: int = 120                       # CALIBRATE: ШИМ ниже которого мотор не крутится
    robot_radius_m: float = 0.25                  # CALIBRATE: описанный радиус шасси + запас
    hfov_rad: float = 1.221                       # CALIBRATE: ~70° горизонтальный FOV камеры
    engine_path: str = "models/yolo11n.engine"
    camera_index: int = 0
    use_imu: bool = False                         # CALIBRATE: True когда заведён реальный IMU
    low_battery_frac: float = 0.3
    # Размер сетки для режима MAPPING (карта с нуля).
    map_resolution_m: float = 0.1
    map_width_m: float = 16.0                      # CALIBRATE: охват площадки
    map_height_m: float = 16.0                     # CALIBRATE
    map_origin_x_m: float = 0.0
    map_origin_y_m: float = 0.0


def build_jetson_robot(
    *,
    config: JetsonConfig | None = None,
    grid: OccupancyGrid | None = None,           # загруженная карта; None -> сначала MAPPING
    mapper: EvidenceGridMapper | None = None,    # нужен для режима MAPPING (карта с нуля)
    map_store: MapStore | None = None,           # куда сохранять построенную карту
    map_label: str = "map",
    live_mapper: ConfidenceGatedMapper | None = None,
    dock: Pose2D | None = None,                  # поза дока для CHARGING
    vision: PersonVision | None = None,          # None -> ленивый follow_me YOLO (только на Jetson)
    battery: BatterySource | None = None,
    keepout: KeepoutRegistry | None = None,
    motor_link: MotorLink | None = None,         # внедрить готовую связь (тесты); иначе по transport
) -> JetsonRobot:
    """Собрать боевого робота. Регистрирует поведения по доступным ресурсам.

    Idle — всегда. Mapping — если задан `mapper`. Navigating/Following/Charging — если задана
    карта `grid` (ездовые режимы планируют по ней). Камера-зрение для Following берётся из
    `vision` или лениво строится поверх follow_me на Jetson.
    """
    cfg = config or JetsonConfig()
    clock = WallClock()
    odometry = DeadReckonOdometry(clock=clock)
    limits = MotionLimits(
        max_linear_m_s=cfg.max_linear_m_s,
        max_angular_rad_s=cfg.max_angular_rad_s,
        max_linear_accel_m_s2=cfg.max_linear_accel_m_s2,
    )
    link = motor_link or _build_link(cfg)
    motion = JetsonMotion(
        link=link,
        clock=clock, limits=limits, wheel_base_m=cfg.wheel_base_m,
        pwm_max=cfg.pwm_max, pwm_min_move=cfg.pwm_min_move,
        command_sink=odometry,  # одометрия интегрирует поданные команды
    )
    lidar = JetsonLidar(
        clock=clock, port=cfg.lidar_port, baud=cfg.lidar_baud, max_range_m=cfg.lidar_max_range_m,
        forward_bin_deg=cfg.lidar_forward_bin_deg, clockwise=cfg.lidar_clockwise,
        masked_sectors=cfg.lidar_masked_sectors,
    )
    # Локализация без энкодеров: IMU-фьюжн (курс с гироскопа) либо чистый scan-match.
    localizer: Localizer = (
        ImuFusedLocalizer(imu=ZeroImu(clock=clock)) if cfg.use_imu else ScanMatchLocalizer()
    )
    if grid is not None:
        localizer.set_map(grid=grid)
    keepout = keepout or InMemoryKeepoutRegistry()

    detector: TargetDetector | None = None
    if grid is not None:
        person_vision = vision or build_follow_me_vision(
            engine_path=cfg.engine_path, camera_index=cfg.camera_index, hfov_rad=cfg.hfov_rad
        )
        detector = JetsonTargetDetector(vision=person_vision, lidar=lidar, clock=clock)

    robot = JetsonRobot(
        robot_id=cfg.robot_id, clock=clock, lidar=lidar, odometry=odometry,
        localizer=localizer, motion=motion, person_detector=detector,
        live_mapper=live_mapper, battery=battery,
    )

    registry = BehaviorRegistry()
    registry.register(IdleBehavior(robot=robot))
    if mapper is not None:
        registry.register(MappingBehavior(
            robot=robot, mapper=mapper, robot_radius_m=cfg.robot_radius_m,
            map_store=map_store, map_label=map_label,
        ))
    if grid is not None:
        registry.register(NavigatingBehavior(navigator=_nav(robot, grid, cfg, keepout)))
        if detector is not None:
            registry.register(FollowingBehavior(
                robot=robot, detector=detector, follower=PersonFollower(),  # CALIBRATE: standoff/скорости
                navigator=_nav(robot, grid, cfg, keepout), clock=clock,
            ))
        if dock is not None:
            registry.register(ChargingBehavior(
                robot=robot, navigator=_nav(robot, grid, cfg, keepout), dock=dock,
            ))

    robot.orchestrator = Orchestrator(
        robot=robot, registry=registry,
        supervisor=Supervisor(low_battery_frac=cfg.low_battery_frac), keepout=keepout,
    )
    return robot


def _build_link(cfg: JetsonConfig) -> MotorLink:
    if cfg.motor_transport == "serial":
        return Esp32SerialLink(port_path=cfg.esp32_serial_port, baud=cfg.esp32_serial_baud)
    return Esp32UdpLink(ip=cfg.esp32_ip, port=cfg.esp32_port)


def _nav(
    robot: JetsonRobot, grid: OccupancyGrid, cfg: JetsonConfig, keepout: KeepoutRegistry
) -> StepwiseNavigator:
    return StepwiseNavigator(
        robot=robot, grid=grid, robot_radius_m=cfg.robot_radius_m, keepout=keepout,
    )
