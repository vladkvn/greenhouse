"""Nav2 (без amcl/map_server — карту и map→odom даёт slam_toolbox).

Запускать ПОВЕРХ slam.launch.py (нужны /map и TF map→odom). Контроллер публикует
прямо в /cmd_vel → его слушает esp32_cmd_vel_bridge. Цель ставится в RViz
инструментом «Nav2 Goal» или actions navigate_to_pose.
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg = get_package_share_directory("rover_bringup")
    params = os.path.join(pkg, "config", "nav2.yaml")
    use_sim_time = {"use_sim_time": False}
    # Кастомный BT: recovery отъезжает назад ПЕРЕД разворотом (не цепляет углы в тесноте).
    bt_xml = os.path.join(pkg, "behavior_trees", "navigate_backup_recovery.xml")

    # Футпринт Nav2 — из ЕДИНОГО robot.yaml (не дублируем robot_radius руками в nav2.yaml).
    # RewrittenYaml подменяет значение КАЖДОГО ключа robot_radius (в local и global costmap).
    with open(os.path.join(pkg, "config", "robot.yaml")) as _f:
        _rcfg = yaml.safe_load(_f)
    configured_params = RewrittenYaml(
        source_file=params,
        param_rewrites={"robot_radius": str(float(_rcfg["footprint_radius"]))},
        convert_types=True,
    )

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
            parameters=[configured_params, use_sim_time],
            # контроллер по умолчанию публикует /cmd_vel — его слушает мост ESP32
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[configured_params, use_sim_time],
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
            parameters=[params, use_sim_time,
                        {"default_nav_to_pose_bt_xml": bt_xml}],
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
