"""Драйверы железа: URDF/TF (robot_state_publisher), RPLIDAR A1, мост cmd_vel→ESP32, BNO085.

Это нижний слой — публикует /scan, /imu/data, TF base_footprint→laser/imu_link и принимает
/cmd_vel. Поверх него запускаются localization / slam / nav2.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory("rover_bringup")
    xacro_file = os.path.join(pkg, "urdf", "rover.urdf.xacro")

    lidar_port = LaunchConfiguration("lidar_port")
    esp_port = LaunchConfiguration("esp_port")
    use_imu = LaunchConfiguration("use_imu")

    robot_description = ParameterValue(Command(["xacro ", xacro_file]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument("lidar_port", default_value="/dev/ttyUSB1",
                              description="RPLIDAR A1 (CP2102)"),
        DeclareLaunchArgument("esp_port", default_value="/dev/ttyUSB0",
                              description="ESP32 моторы (CH340)"),
        DeclareLaunchArgument("use_imu", default_value="true",
                              description="запускать узел BNO085"),

        # URDF → TF (фиксированные джойнты публикуются автоматически)
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description, "use_sim_time": False}],
        ),

        # RPLIDAR A1 → /scan через наш узел на библиотеке rplidar. sllidar_ros2 на этом
        # железе (A1 + CP2102) таймаутит из-за DTR-ресета при открытии порта. Разворот 180°
        # учтён в URDF (base_link→laser), скан НЕ роллим.
        Node(
            package="rover_drivers",
            executable="rplidar_scan_node",
            name="rplidar_scan_node",
            output="screen",
            parameters=[{
                "serial_port": lidar_port,
                "baud": 115200,
                "frame_id": "laser",
                "invert": True,    # RPLIDAR идёт CW, ROS ждёт CCW → зеркалим (замерено rf2o vs IMU)
            }],
        ),
        # Альтернатива sllidar_ros2 (если заведётся на другом лидаре):
        #   Node(package="sllidar_ros2", executable="sllidar_node", parameters=[{
        #     "channel_type":"serial","serial_port":lidar_port,"serial_baudrate":115200,
        #     "frame_id":"laser","angle_compensate":True,"scan_mode":"Standard"}])

        # /cmd_vel → "L R\n" на ESP32 (skid-steer, failsafe, slew-rate)
        Node(
            package="rover_drivers",
            executable="esp32_cmd_vel_bridge",
            name="esp32_cmd_vel_bridge",
            output="screen",
            parameters=[{
                "serial_port": esp_port,
                "baud": 115200,
                "wheel_base": 0.18,        # CALIBRATE
                "max_linear": 0.4,         # CALIBRATE
                "max_angular": 1.5,
                "pwm_max": 255,            # полный 8-бит: запас на срыв стикции при развороте на месте
                "pwm_min_move": 120,       # CALIBRATE
                "cmd_timeout": 0.4,
                "heartbeat_hz": 15.0,
                "max_pwm_step": 50,        # резче кик (было 20 — ШИМ слишком долго полз через мёртвую зону)
                "swap_sides": False,       # CALIBRATE: True если борта перепутаны
            }],
        ),

        # BNO085 → /imu/data (опционально)
        Node(
            package="rover_drivers",
            executable="bno085_imu_node",
            name="bno085_imu_node",
            output="screen",
            condition=IfCondition(use_imu),
            parameters=[{
                "i2c_bus": 7,
                "address": 0x4A,
                "frame_id": "imu_link",
                "rate_hz": 50.0,
                "publish_accel": True,
            }],
        ),
    ])
