"""Полный bring-up ровера одним файлом: drivers + SLAM + Nav2 + RViz (флагами).

Примеры:
  # карта (катать телеопом) + RViz, без навигации:
  ros2 launch rover_bringup bringup.launch.py use_nav:=false
  # полный автоном — строит карту И ездит в точку из RViz:
  ros2 launch rover_bringup bringup.launch.py
  # без RViz (на роботе по SSH), свои порты:
  ros2 launch rover_bringup bringup.launch.py use_rviz:=false lidar_port:=/dev/ttyUSB1 esp_port:=/dev/ttyUSB0
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("rover_bringup")
    launch_dir = os.path.join(pkg, "launch")
    rviz_cfg = os.path.join(pkg, "rviz", "rover_nav.rviz")

    use_slam = LaunchConfiguration("use_slam")
    use_nav = LaunchConfiguration("use_nav")
    use_rviz = LaunchConfiguration("use_rviz")
    lidar_port = LaunchConfiguration("lidar_port")
    esp_port = LaunchConfiguration("esp_port")
    use_imu = LaunchConfiguration("use_imu")

    drivers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "drivers.launch.py")),
        launch_arguments={
            "lidar_port": lidar_port,
            "esp_port": esp_port,
            "use_imu": use_imu,
        }.items(),
    )

    # SLAM включает localization (rf2o+EKF); если SLAM выключен, поднимаем хотя бы локализацию
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "slam.launch.py")),
        condition=IfCondition(use_slam),
    )
    localization_only = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "localization.launch.py")),
        condition=UnlessCondition(use_slam),   # если SLAM выкл — поднять хотя бы rf2o+EKF
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "nav2.launch.py")),
        condition=IfCondition(use_nav),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_cfg],
        output="screen",
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_slam", default_value="true"),
        DeclareLaunchArgument("use_nav", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("lidar_port", default_value="/dev/ttyUSB1"),
        DeclareLaunchArgument("esp_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("use_imu", default_value="true"),
        drivers,
        slam,
        localization_only,
        nav2,
        rviz,
    ])
