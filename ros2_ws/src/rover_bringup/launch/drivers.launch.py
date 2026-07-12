"""Драйверы железа: URDF/TF (robot_state_publisher), RPLIDAR A1, привод cmd_vel→Dual VESC, BNO085, камера.

Это нижний слой — публикует /scan, /odom, /imu/data, TF base_footprint→laser/imu_link и принимает
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
    vesc_port = LaunchConfiguration("vesc_port")
    use_imu = LaunchConfiguration("use_imu")

    robot_description = ParameterValue(Command(["xacro ", xacro_file]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument("lidar_port", default_value="/dev/ttyUSB1",
                              description="RPLIDAR A1 (CP2102)"),
        DeclareLaunchArgument("esp_port", default_value="/dev/ttyUSB0",
                              description="ESP32 моторы (CH340) — старый привод, fallback"),
        DeclareLaunchArgument("vesc_port", default_value="/dev/ttyACM0",
                              description="Dual VESC локальная половина (USB-CDC)"),
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

        # /cmd_vel → Dual VESC (diff-drive, ERPM, колёсная одометрия /odom). Геометрия и электрика
        # привода — из robot.yaml (секция drivetrain); тут только порт и TF. Заменил мост ESP32.
        Node(
            package="rover_drivers",
            executable="vesc_diff_drive",
            name="vesc_diff_drive",
            output="screen",
            parameters=[{
                "port": vesc_port,
                "publish_tf": False,       # TF odom→base_footprint публикует EKF robot_localization
            }],
        ),
        # Fallback на старый привод ESP32 (щёточный 4WD) — раскомментировать вместо VESC:
        # Node(
        #     package="rover_drivers", executable="esp32_cmd_vel_bridge", name="esp32_cmd_vel_bridge",
        #     output="screen", parameters=[{
        #         "serial_port": esp_port, "baud": 115200, "wheel_base": 0.18, "max_linear": 0.4,
        #         "max_angular": 1.5, "pwm_max": 255, "pwm_min_move": 120, "cmd_timeout": 0.4,
        #         "heartbeat_hz": 15.0, "max_pwm_step": 50, "swap_sides": False}],
        # ),

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

        # Камера → /camera/image/compressed + /camera/camera_info (единый владелец /dev/video;
        # панель, детектор person и монокулярная глубина ПОДПИСЫВАЮТСЯ на этот топик)
        Node(
            package="rover_drivers",
            executable="camera_node",
            name="camera_node",
            output="screen",
            parameters=[{
                "pub_width": 640,
                "fps": 20.0,
                "jpeg_quality": 60,
                "frame_id": "camera_optical_frame",
            }],
        ),
    ])
