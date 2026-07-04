#!/usr/bin/env bash
# Установка ROS 2 Humble + стек навигации на Jetson (Ubuntu 22.04 / arm64).
# Идемпотентно — можно перезапускать после обрыва/ребута (Jetson нестабилен).
# Запуск как root:  echo <sudo_pw> | sudo -S bash install_ros2_jetson.sh
set -eo pipefail
trap 'echo "INSTALL_FAILED rc=$? at line $LINENO"' ERR

echo "=== [0/6] $(date) восстановление apt после возможного ребута ==="
dpkg --configure -a || true
apt-get -y --fix-broken install || true

echo "=== [1/6] базовые пакеты + universe ==="
apt-get update -y
apt-get install -y locales curl gnupg lsb-release software-properties-common
locale-gen en_US en_US.UTF-8 || true
add-apt-repository -y universe || true

echo "=== [2/6] apt-репозиторий ROS 2 (Humble) ==="
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" \
  > /etc/apt/sources.list.d/ros2.list
apt-get update -y

echo "=== [3/6] ROS 2 Humble base + стек (nav2 / slam_toolbox / robot_localization) ==="
DEBIAN_FRONTEND=noninteractive apt-get install -y \
  ros-humble-ros-base \
  ros-humble-slam-toolbox \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-robot-localization \
  ros-humble-robot-state-publisher ros-humble-xacro \
  ros-humble-teleop-twist-keyboard ros-humble-rviz2 \
  ros-humble-tf2-tools ros-humble-tf2-ros \
  python3-colcon-common-extensions python3-vcstool python3-rosdep python3-serial git

echo "=== [4/6] rosdep init ==="
rosdep init || true

echo "=== [5/6] python-библиотеки IMU (best-effort, узел опционален) ==="
pip3 install adafruit-blinka adafruit-circuitpython-bno08x adafruit-extended-bus \
  || pip3 install --break-system-packages adafruit-blinka adafruit-circuitpython-bno08x adafruit-extended-bus \
  || echo "WARN: IMU libs не поставились — позже, пока use_imu:=false"

echo "=== [6/6] готово ==="
echo "INSTALL_DONE_MARKER $(date)"
