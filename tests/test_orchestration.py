"""Тесты оркестрации (инкремент 2): Orchestrator + BehaviorRegistry + Supervisor.

Проверяются на точном двойнике RobotContext (без sim-цикла, без железа):
команды переключают режим, EmergencyStop вытесняет из любого режима, Supervisor форсит
зарядку при низком заряде и безопасный стоп при потере локализации (а Following её
переживает), реестр позволяет добавить режим одной регистрацией, поведение само
запрашивает переход.
"""

from __future__ import annotations

from greenhouse.control.interfaces import MotionLimits
from greenhouse.domain.geometry import Pose2D, Twist2D
from greenhouse.orchestration import (
    BehaviorRegistry,
    IdleBehavior,
    Orchestrator,
    RobotMode,
    Supervisor,
)
from greenhouse.orchestration.interfaces import (
    EmergencyStop,
    FollowPerson,
    GoCharge,
    GoTo,
    StartMapping,
    Stop,
)

_LIMITS = MotionLimits(max_linear_m_s=0.4, max_angular_rad_s=1.5, max_linear_accel_m_s2=1.0)


# ------------------------------------------------------------------- двойники
class FakeMotion:
    """Двойник MotionController: считает команды и стопы."""

    def __init__(self) -> None:
        self.stops = 0
        self.commands = 0

    @property
    def limits(self) -> MotionLimits:
        return _LIMITS

    def command(self, *, twist: Twist2D) -> None:
        self.commands += 1

    def stop(self) -> None:
        self.stops += 1


class FakeRobot:
    """Двойник RobotContext."""

    def __init__(
        self,
        *,
        mode: RobotMode = RobotMode.IDLE,
        battery: float = 1.0,
        lost: bool = False,
    ) -> None:
        self.robot_id = "robot-test"
        self.mode = mode
        self.motion = FakeMotion()
        self._battery = battery
        self._lost = lost
        self._pose = Pose2D(x_m=1.0, y_m=2.0, theta_rad=0.5)

    @property
    def battery_frac(self) -> float:
        return self._battery

    @property
    def latest_pose(self) -> Pose2D | None:
        return self._pose

    @property
    def localization_lost(self) -> bool:
        return self._lost


class StubBehavior:
    """Поведение с программируемыми переходами (для проверки tick)."""

    def __init__(self, mode: RobotMode, returns: list[RobotMode | None] | None = None) -> None:
        self._mode = mode
        self._returns = list(returns or [])
        self.steps = 0

    @property
    def mode(self) -> RobotMode:
        return self._mode

    def step(self) -> RobotMode | None:
        self.steps += 1
        return self._returns.pop(0) if self._returns else None


def _orch(robot: FakeRobot, *, supervisor: Supervisor | None = None) -> Orchestrator:
    registry = BehaviorRegistry()
    registry.register(IdleBehavior(robot=robot))
    return Orchestrator(robot=robot, registry=registry, supervisor=supervisor)


# --------------------------------------------------------------------- команды
def test_commands_switch_mode() -> None:
    robot = FakeRobot()
    orch = _orch(robot)
    assert orch.handle(command=FollowPerson()).mode is RobotMode.FOLLOWING
    assert orch.handle(command=StartMapping()).mode is RobotMode.MAPPING
    assert orch.handle(command=GoCharge()).mode is RobotMode.CHARGING
    assert orch.handle(command=Stop()).mode is RobotMode.IDLE


def test_goto_stores_goal_and_navigates() -> None:
    robot = FakeRobot()
    orch = _orch(robot)
    goal = Pose2D(x_m=5.0, y_m=6.0, theta_rad=0.0)
    status = orch.handle(command=GoTo(goal=goal))
    assert status.mode is RobotMode.NAVIGATING
    assert orch.goal == goal


def test_emergency_stop_preempts_from_any_mode() -> None:
    robot = FakeRobot(mode=RobotMode.FOLLOWING)
    orch = _orch(robot)
    before = robot.motion.stops
    status = orch.handle(command=EmergencyStop())
    assert status.mode is RobotMode.IDLE
    assert robot.motion.stops > before  # привод остановлен


def test_mode_transition_stops_motion() -> None:
    robot = FakeRobot()
    orch = _orch(robot)
    before = robot.motion.stops
    orch.handle(command=FollowPerson())
    assert robot.motion.stops == before + 1  # безопасный выход из старого режима


# ----------------------------------------------------------------- supervisor
def test_low_battery_forces_charging() -> None:
    robot = FakeRobot(mode=RobotMode.IDLE, battery=0.2)
    orch = _orch(robot, supervisor=Supervisor(low_battery_frac=0.3))
    assert orch.tick(dt_s=0.1).mode is RobotMode.CHARGING


def test_lost_localization_safe_stops_while_navigating() -> None:
    robot = FakeRobot(mode=RobotMode.NAVIGATING, lost=True)
    orch = _orch(robot)
    before = robot.motion.stops
    status = orch.tick(dt_s=0.1)
    assert status.mode is RobotMode.IDLE
    assert status.last_error == "localization_lost"
    assert robot.motion.stops > before


def test_following_survives_lost_localization() -> None:
    robot = FakeRobot(mode=RobotMode.FOLLOWING, lost=True)
    orch = _orch(robot)
    status = orch.tick(dt_s=0.1)
    assert status.mode is RobotMode.FOLLOWING  # следование идёт в относительных координатах
    assert status.last_error == "localization_lost"


# ------------------------------------------------------------- behaviors/реестр
def test_behavior_requested_transition_is_applied() -> None:
    robot = FakeRobot(mode=RobotMode.MAPPING)
    registry = BehaviorRegistry()
    registry.register(StubBehavior(RobotMode.MAPPING, returns=[RobotMode.IDLE]))
    orch = Orchestrator(robot=robot, registry=registry)
    assert orch.tick(dt_s=0.1).mode is RobotMode.IDLE  # карта достроена -> IDLE


def test_registry_register_get_modes_is_extension_point() -> None:
    robot = FakeRobot()
    registry = BehaviorRegistry()
    registry.register(IdleBehavior(robot=robot))
    registry.register(StubBehavior(RobotMode.FOLLOWING))
    assert registry.get(RobotMode.IDLE) is not None
    assert set(registry.modes()) == {RobotMode.IDLE, RobotMode.FOLLOWING}
    assert registry.get(RobotMode.CHARGING) is None  # незарегистрированный режим


def test_idle_behavior_holds_stop() -> None:
    robot = FakeRobot(mode=RobotMode.IDLE)
    orch = _orch(robot)
    before = robot.motion.stops
    orch.tick(dt_s=0.1)
    assert robot.motion.stops > before  # IDLE держит привод в стопе


def test_status_reflects_telemetry() -> None:
    robot = FakeRobot(mode=RobotMode.IDLE, battery=0.77)
    orch = _orch(robot)
    status = orch.status()
    assert status.battery_frac == 0.77
    assert status.pose == Pose2D(x_m=1.0, y_m=2.0, theta_rad=0.5)
    assert status.last_error is None
