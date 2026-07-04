"""Адаптеры реального робота (Jetson Orin Nano + ESP32-CAM 4WD + RPLIDAR A1).

Зеркало `adapters/sim`: каждый класс удовлетворяет тому же Protocol из ядра, что и его
sim-двойник, поэтому режимы и навигация, проверенные в симуляции, едут на железе без
изменений в ядре.

  WallClock   → runtime.Clock
  JetsonMotion→ control.MotionController   (Twist2D → "L R" по UDP к ESP32)
  JetsonLidar → sensing.LidarSource        (RPLIDAR A1 → LidarScan, луч 0 по курсу)

Инкремент 0 (телеоп) использует только эти три. Остальные адаптеры (детектор, одометрия,
IMU, композиционный корень) добавляются следующими инкрементами.
"""

from greenhouse.adapters.jetson.clock import WallClock
from greenhouse.adapters.jetson.detector import (
    JetsonTargetDetector,
    PersonVision,
    range_at_bearing,
)
from greenhouse.adapters.jetson.imu import ZeroImu
from greenhouse.adapters.jetson.lidar import JetsonLidar
from greenhouse.adapters.jetson.loop import JetsonConfig, JetsonRobot, build_jetson_robot
from greenhouse.adapters.jetson.motion import (
    DriveWatchdog,
    Esp32SerialLink,
    Esp32UdpLink,
    JetsonMotion,
    twist_to_lr,
)
from greenhouse.adapters.jetson.odometry import DeadReckonOdometry
from greenhouse.adapters.jetson.runner import run_jetson

__all__ = [
    "WallClock",
    "JetsonLidar",
    "JetsonMotion",
    "Esp32UdpLink",
    "Esp32SerialLink",
    "DriveWatchdog",
    "twist_to_lr",
    "DeadReckonOdometry",
    "ZeroImu",
    "JetsonTargetDetector",
    "PersonVision",
    "range_at_bearing",
    "JetsonConfig",
    "JetsonRobot",
    "build_jetson_robot",
    "run_jetson",
]
