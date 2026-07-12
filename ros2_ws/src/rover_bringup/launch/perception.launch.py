"""Восприятие + follow ровера: person_tracker (детекция → поза в карте) + follow_behavior (FSM).
[+ позже depth].

ДВА нюанса запуска person_tracker (иначе не заведётся):
  1. Интерпретатор — venv ~/greenhouse/.venv/bin/python: там torch, ultralytics лежит в
     ~/.local (user-site), rclpy виден через include-system-site-packages. Системный python3
     из `ros2 run` НЕ имеет torch/ultralytics.
  2. `unset PYTHONNOUSERSITE` — при PYTHONNOUSERSITE=1 (нужен для colcon) прячется ~/.local →
     ultralytics не импортируется. Поэтому person_tracker — через bash -c с unset.

follow_behavior — чистый rclpy (без torch) → обычный Node.
camera_node (владелец /dev/video) — в drivers.launch.
"""
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

VENV_PY = "/home/luki/greenhouse/.venv/bin/python"
MODEL = "/home/luki/greenhouse/models/yolo11n.engine"
DA_MODEL = "/home/luki/greenhouse/models/da_v2_small.onnx"


def generate_launch_description():
    person = (
        f"unset PYTHONNOUSERSITE; exec {VENV_PY} -m rover_drivers.person_tracker "
        f"--ros-args -p model_path:={MODEL} -p conf:=0.4 -p rate_hz:=12.0"
    )
    depth = (
        f"unset PYTHONNOUSERSITE; exec {VENV_PY} -m rover_drivers.depth_obstacles "
        f"--ros-args -p model_path:={DA_MODEL} -p rate_hz:=4.0"
    )
    return LaunchDescription([
        ExecuteProcess(cmd=["bash", "-c", person], output="screen", name="person_tracker"),
        ExecuteProcess(cmd=["bash", "-c", depth], output="screen", name="depth_obstacles"),
        Node(
            package="rover_drivers",
            executable="follow_behavior",
            name="follow_behavior",
            output="screen",
            parameters=[{
                "standoff_m": 1.5,
                "max_lin": 0.20,
                "max_ang": 1.2,
                "k_ang": 1.6,
                "search_ang": 0.9,
            }],
        ),
    ])
