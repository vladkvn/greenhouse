"""Локализация: КОЛЁСНАЯ одометрия VESC (vx) + IMU (курс) через EKF.

Колёсную /odom публикует vesc_diff_drive (drivers.launch). EKF сливает /odom + /imu/data →
TF odom→base_footprint + /odometry/filtered — «одометрия», которую потребляют slam_toolbox и Nav2.

rf2o (лазерная одометрия) заменена колёсной — точнее на прямой и без дрейфа в коридорах, плюс
экономит CPU на Jetson. Оставлена опционально ДЛЯ СВЕРКИ при калибровке:
  use_laser_odom:=true → публикует /odom_rf2o (в EKF НЕ входит) — удобно сравнить масштаб колёсной.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("rover_bringup")
    rf2o_params = os.path.join(pkg, "config", "rf2o.yaml")
    ekf_params = os.path.join(pkg, "config", "ekf.yaml")
    use_laser_odom = LaunchConfiguration("use_laser_odom")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_laser_odom", default_value="false",
            description="rf2o /odom_rf2o для сверки с колёсной одометрией (в EKF не фьюзится)"),

        # rf2o — только для сравнения при калибровке; по умолчанию выключен
        Node(
            package="rf2o_laser_odometry",
            executable="rf2o_laser_odometry_node",
            name="rf2o_laser_odometry",
            output="screen",
            condition=IfCondition(use_laser_odom),
            parameters=[rf2o_params],
        ),
        # EKF: /odom (колёса, vx) + /imu/data (yaw, vyaw) → TF odom→base_footprint + /odometry/filtered
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[ekf_params, {"use_sim_time": False}],
        ),
    ])
