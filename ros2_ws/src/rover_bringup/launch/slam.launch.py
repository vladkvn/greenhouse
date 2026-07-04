"""SLAM: локализация (rf2o+EKF) + slam_toolbox online_async (живое построение карты).

Запускать ПОВЕРХ drivers.launch.py. Даёт TF map→odom + /map. Катать робота телеопом —
карта строится и сходится по loop closure (поведение как на гифках slam_toolbox).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("rover_bringup")
    slam_params = os.path.join(pkg, "config", "slam_toolbox.yaml")

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, "launch", "localization.launch.py"))
    )

    slam_toolbox = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[slam_params, {"use_sim_time": False}],
    )

    return LaunchDescription([localization, slam_toolbox])
