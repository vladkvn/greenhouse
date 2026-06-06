"""Цикл управления симуляции и сборка готового робота.

Один тик: прочитать датчики → локализоваться → выдать команду по режиму → шаг физики
→ собрать телеметрию (RobotStatus). В Инкременте 1 поведение тривиально — Idle (стоп);
последующие инкременты подключат планирование/следование, не меняя структуру цикла.
"""

from __future__ import annotations

from dataclasses import dataclass

from greenhouse.adapters.sim.clock import SimClock
from greenhouse.adapters.sim.engine import SimEngine
from greenhouse.adapters.sim.lidar import SimLidar
from greenhouse.adapters.sim.localization import SimTruthLocalizer
from greenhouse.adapters.sim.motion import SimMotion
from greenhouse.adapters.sim.odometry import SimOdometry
from greenhouse.adapters.sim.state import SimState
from greenhouse.adapters.sim.world import PolygonWorld
from greenhouse.domain.identifiers import RobotId
from greenhouse.orchestration.interfaces import RobotStatus
from greenhouse.orchestration.modes import RobotMode


@dataclass
class SimRobot:
    """Собранный из адаптеров робот: всё, что нужно для одного тика цикла."""

    robot_id: RobotId
    state: SimState
    clock: SimClock
    engine: SimEngine
    lidar: SimLidar
    odometry: SimOdometry
    localizer: SimTruthLocalizer
    motion: SimMotion
    mode: RobotMode = RobotMode.IDLE

    def tick(self, *, dt_s: float) -> RobotStatus:
        scan = self.lidar.read_scan()
        odom = self.odometry.read_odometry()
        estimate = self.localizer.update(scan=scan, odometry=odom)

        # Поведение по режиму. Инкремент 1: Idle = стоп.
        if self.mode is RobotMode.IDLE:
            self.motion.stop()

        self.engine.step(dt_s=dt_s)

        return RobotStatus(
            robot_id=self.robot_id,
            mode=self.mode,
            pose=estimate.pose,
            battery_frac=self.state.battery_frac,
            last_error=None if not estimate.is_lost else "localization_lost",
        )


def build_sim_robot(
    *,
    world: PolygonWorld,
    robot_id: RobotId = "robot-01",
    start_x_m: float = 1.0,
    start_y_m: float = 1.0,
    start_theta_rad: float = 0.0,
) -> SimRobot:
    """Собрать робота со всеми sim-адаптерами в согласованном состоянии."""
    state = SimState(x_m=start_x_m, y_m=start_y_m, theta_rad=start_theta_rad)
    clock = SimClock()
    return SimRobot(
        robot_id=robot_id,
        state=state,
        clock=clock,
        engine=SimEngine(world=world, state=state, clock=clock),
        lidar=SimLidar(world=world, state=state, clock=clock),
        odometry=SimOdometry(state=state, clock=clock),
        localizer=SimTruthLocalizer(state=state, clock=clock),
        motion=SimMotion(state=state),
    )


def run(robot: SimRobot, *, ticks: int, dt_s: float = 0.1) -> list[RobotStatus]:
    """Прогнать цикл `ticks` шагов, вернуть поток телеметрии."""
    return [robot.tick(dt_s=dt_s) for _ in range(ticks)]
