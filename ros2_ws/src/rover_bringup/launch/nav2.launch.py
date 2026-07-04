"""Nav2 (без amcl/map_server — карту и map→odom даёт slam_toolbox).

Запускать ПОВЕРХ slam.launch.py (нужны /map и TF map→odom). Контроллер публикует
прямо в /cmd_vel → его слушает esp32_cmd_vel_bridge. Цель ставится в RViz
инструментом «Nav2 Goal» или actions navigate_to_pose.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("rover_bringup")
    params = os.path.join(pkg, "config", "nav2.yaml")
    use_sim_time = {"use_sim_time": False}

    lifecycle_nodes = [
        "controller_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
    ]

    return LaunchDescription([
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[params, use_sim_time],
            # контроллер по умолчанию публикует /cmd_vel — его слушает мост ESP32
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[params, use_sim_time],
        ),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            parameters=[params, use_sim_time],
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=[params, use_sim_time],
        ),
        Node(
            package="nav2_waypoint_follower",
            executable="waypoint_follower",
            name="waypoint_follower",
            output="screen",
            parameters=[params, use_sim_time],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[{
                "use_sim_time": False,
                "autostart": True,
                "node_names": lifecycle_nodes,
            }],
        ),
    ])
