"""Симуляционные адаптеры: реализации интерфейсов ядра без железа и ROS.

Каждый класс удовлетворяет соответствующему Protocol из ядра:
SimLidar→LidarSource, SimMotion→MotionController, SimTruthLocalizer→Localizer,
SimOdometry→OdometrySource, SimClock→Clock.
"""

from greenhouse.adapters.sim.clock import SimClock
from greenhouse.adapters.sim.engine import SimEngine
from greenhouse.adapters.sim.lidar import SimLidar
from greenhouse.adapters.sim.localization import SimTruthLocalizer
from greenhouse.adapters.sim.loop import SimRobot, build_sim_robot, run
from greenhouse.adapters.sim.motion import SimMotion
from greenhouse.adapters.sim.navigation import SimGoalNavigator
from greenhouse.adapters.sim.odometry import SimOdometry
from greenhouse.adapters.sim.person import SimPerson, SimPersonDetector
from greenhouse.adapters.sim.state import SimState
from greenhouse.adapters.sim.world import PolygonWorld, Segment
from greenhouse.adapters.sim.worlds import (
    empty_room,
    greenhouse_rows_world,
    grid_meta_for_world,
)

__all__ = [
    "SimClock",
    "SimEngine",
    "SimLidar",
    "SimTruthLocalizer",
    "SimRobot",
    "build_sim_robot",
    "run",
    "SimMotion",
    "SimGoalNavigator",
    "SimPerson",
    "SimPersonDetector",
    "SimOdometry",
    "SimState",
    "PolygonWorld",
    "Segment",
    "empty_room",
    "greenhouse_rows_world",
    "grid_meta_for_world",
]
