"""Orchestrator — реализация MissionHandler: единственная точка переключения режимов.

Неблокирующий конечный автомат поверх RobotMode. Каждый тик:
  1. Supervisor.check — надрежимный preempt (потеря локализации, низкий заряд);
  2. иначе step() активного Behavior из реестра;
  3. применить запрошенный переход (RobotMode | None).
Команды оператора (`handle`) переводят целевой режим. В отличие от блокирующих фасадов sim
(SimGoalNavigator крутит цикл сам), `tick` исполняет ОДИН шаг — его вызывает rate-loop робота,
поэтому режим Following может во время следования породить под-цель навигации.
"""

from __future__ import annotations

from typing import Literal

from greenhouse.domain.geometry import Pose2D
from greenhouse.domain.identifiers import ZoneId
from greenhouse.navigation.keepout import KeepoutRegistry, Zone
from greenhouse.orchestration.interfaces import (
    Command,
    EditKeepout,
    EmergencyStop,
    FollowPerson,
    GoalAccepting,
    GoCharge,
    GoTo,
    RobotContext,
    RobotStatus,
    StartMapping,
    Stop,
)
from greenhouse.orchestration.modes import RobotMode
from greenhouse.orchestration.registry import BehaviorRegistry
from greenhouse.orchestration.supervisor import Supervisor


class Orchestrator:
    """Реализует `greenhouse.orchestration.MissionHandler`."""

    def __init__(
        self,
        *,
        robot: RobotContext,
        registry: BehaviorRegistry,
        supervisor: Supervisor | None = None,
        keepout: KeepoutRegistry | None = None,
    ) -> None:
        self._robot = robot
        self._registry = registry
        self._supervisor = supervisor or Supervisor()
        self._keepout = keepout  # тот же реестр, что у навигатора — правки видны на replan
        self._goal: Pose2D | None = None  # последняя цель GoTo (для NavigatingBehavior)

    @property
    def goal(self) -> Pose2D | None:
        return self._goal

    # ------------------------------------------------------------- MissionHandler
    def handle(self, *, command: Command) -> RobotStatus:
        match command:
            case EmergencyStop():
                self._robot.motion.stop()
                self._enter(RobotMode.IDLE)
            case Stop():
                self._enter(RobotMode.IDLE)
            case StartMapping():
                self._enter(RobotMode.MAPPING)
            case GoTo(goal=goal):
                self._goal = goal
                self._enter(RobotMode.NAVIGATING)
                behavior = self._registry.get(RobotMode.NAVIGATING)
                if isinstance(behavior, GoalAccepting):
                    behavior.set_goal(goal=goal)
            case FollowPerson():
                self._enter(RobotMode.FOLLOWING)
            case GoCharge():
                self._enter(RobotMode.CHARGING)
            case EditKeepout(op=op, zone=zone, zone_id=zone_id):
                self._edit_keepout(op=op, zone=zone, zone_id=zone_id)  # режим не меняем
        return self.status()

    def status(self) -> RobotStatus:
        r = self._robot
        return RobotStatus(
            robot_id=r.robot_id,
            mode=r.mode,
            pose=r.latest_pose,
            battery_frac=r.battery_frac,
            last_error="localization_lost" if r.localization_lost else None,
        )

    # -------------------------------------------------------------------- цикл
    def tick(self, *, dt_s: float) -> RobotStatus:
        preempt = self._supervisor.check(robot=self._robot)
        if preempt is not None:
            if preempt.force_stop:
                self._robot.motion.stop()
            if preempt.mode is not None:
                self._enter(preempt.mode)
            return self.status()

        behavior = self._registry.get(self._robot.mode)
        if behavior is not None:
            requested = behavior.step()
            if requested is not None:
                self._enter(requested)
        return self.status()

    # ------------------------------------------------------------------- внутр.
    def _enter(self, mode: RobotMode) -> None:
        """Перейти в режим: безопасный выход из старого (стоп привода) и смена режима."""
        if mode is self._robot.mode:
            return
        self._robot.motion.stop()
        self._robot.mode = mode

    def _edit_keepout(
        self, *, op: Literal["add", "remove"], zone: Zone | None, zone_id: ZoneId | None
    ) -> None:
        if self._keepout is None:
            return
        if op == "add" and zone is not None:
            self._keepout.add_zone(zone=zone)
        elif op == "remove" and zone_id is not None:
            self._keepout.remove_zone(zone_id=zone_id)
