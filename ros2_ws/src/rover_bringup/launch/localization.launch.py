"""Локализация без энкодеров: rf2o (лазерная одометрия) + EKF (слияние с IMU).

rf2o: /scan → /odom_rf2o (без TF). EKF: /odom_rf2o + /imu/data → TF odom→base_footprint
+ /odometry/filtered. Это «одометрия», которую дальше потребляют slam_toolbox и Nav2.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("rover_bringup")
    rf2o_params = os.path.join(pkg, "config", "rf2o.yaml")
    ekf_params = os.path.join(pkg, "config", "ekf.yaml")

    return LaunchDescription([
        Node(
            package="rf2o_laser_odometry",
            executable="rf2o_laser_odometry_node",
            name="rf2o_laser_odometry",
            output="screen",
            parameters=[rf2o_params],
        ),
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[ekf_params, {"use_sim_time": False}],
        ),
    ])
