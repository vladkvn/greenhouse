"""build_orchestrated_sim_robot — sim-зеркало build_jetson_robot.

Собирает SimRobot под управлением единого Orchestrator с теми же поведениями, что и на
железе. Используется как сквозная проверка композиции (тот же реестр/оркестратор/поведения,
что повторяет build_jetson_robot), и как удобный способ гонять полные миссии в симуляции.
"""

from __future__ import annotations

from greenhouse.adapters.sim.loop import SimRobot, build_sim_robot
from greenhouse.adapters.sim.person import SimPerson, SimPersonDetector
from greenhouse.adapters.sim.world import PolygonWorld
from greenhouse.domain.geometry import Pose2D
from greenhouse.domain.grid import OccupancyGrid
from greenhouse.navigation.following import PersonFollower
from greenhouse.navigation.keepout import InMemoryKeepoutRegistry, KeepoutRegistry
from greenhouse.navigation.mapping import EvidenceGridMapper, MapStore
from greenhouse.navigation.navigator import StepwiseNavigator
from greenhouse.orchestration.behaviors import (
    ChargingBehavior,
    FollowingBehavior,
    IdleBehavior,
    MappingBehavior,
    NavigatingBehavior,
)
from greenhouse.orchestration.orchestrator import Orchestrator
from greenhouse.orchestration.registry import BehaviorRegistry
from greenhouse.orchestration.supervisor import Supervisor


def build_orchestrated_sim_robot(
    *,
    world: PolygonWorld,
    grid: OccupancyGrid | None = None,
    start_x_m: float = 1.0,
    start_y_m: float = 1.0,
    robot_radius_m: float = 0.25,
    person: SimPerson | None = None,
    dock: Pose2D | None = None,
    mapper: EvidenceGridMapper | None = None,
    map_store: MapStore | None = None,
    keepout: KeepoutRegistry | None = None,
    low_battery_frac: float = 0.3,
) -> SimRobot:
    """Собрать SimRobot под единым FSM. Поведения регистрируются по доступным ресурсам."""
    robot = build_sim_robot(
        world=world, start_x_m=start_x_m, start_y_m=start_y_m,
        dock=dock.point if dock is not None else None,
    )
    keepout = keepout or InMemoryKeepoutRegistry()

    registry = BehaviorRegistry()
    registry.register(IdleBehavior(robot=robot))
    if mapper is not None:
        registry.register(MappingBehavior(
            robot=robot, mapper=mapper, robot_radius_m=robot_radius_m, map_store=map_store,
        ))
    if grid is not None:
        registry.register(NavigatingBehavior(navigator=_nav(robot, grid, robot_radius_m, keepout)))
        if person is not None:
            detector = SimPersonDetector(
                world=world, robot_state=robot.state, person=person, clock=robot.clock, max_range_m=8.0
            )
            registry.register(FollowingBehavior(
                robot=robot, detector=detector, follower=PersonFollower(),
                navigator=_nav(robot, grid, robot_radius_m, keepout), clock=robot.clock,
            ))
        if dock is not None:
            registry.register(ChargingBehavior(
                robot=robot, navigator=_nav(robot, grid, robot_radius_m, keepout), dock=dock,
            ))

    robot.orchestrator = Orchestrator(
        robot=robot, registry=registry,
        supervisor=Supervisor(low_battery_frac=low_battery_frac), keepout=keepout,
    )
    return robot


def _nav(
    robot: SimRobot, grid: OccupancyGrid, robot_radius_m: float, keepout: KeepoutRegistry
) -> StepwiseNavigator:
    return StepwiseNavigator(
        robot=robot, grid=grid, robot_radius_m=robot_radius_m, keepout=keepout, goal_tol_m=0.25,
    )
