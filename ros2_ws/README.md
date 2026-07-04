# ros2_ws — ROS 2 Humble стек ровера (карта + навигация)

Живое построение карты (slam_toolbox) и езда в точку (Nav2) на роботе без энкодеров:
лазерная одометрия **rf2o** + курс **BNO085**, слитые **EKF** (robot_localization).

Полный runbook (установка, сборка, bring-up, калибровка): [`docs/ROVER_ROS2_NAV.md`](../docs/ROVER_ROS2_NAV.md).

## Пакеты

- **`rover_drivers`** (ament_python) — мосты к железу:
  - `esp32_cmd_vel_bridge` — `/cmd_vel` → "L R\n" serial (skid-steer, failsafe, slew-rate);
  - `bno085_imu_node` — BNO085 (I2C-7) → `/imu/data`.
- **`rover_bringup`** (ament_cmake) — URDF/TF, launch и конфиги rf2o/EKF/slam_toolbox/Nav2, RViz.

## Быстрый старт

```bash
# 1) пакеты из исходников (sllidar_ros2, rf2o)
vcs import src < rover.repos
# 2) сборка
source /opt/ros/humble/setup.bash
colcon build --symlink-install && source install/setup.bash
# 3) карта (катать телеопом) + RViz:
ros2 launch rover_bringup bringup.launch.py use_nav:=false
ros2 run teleop_twist_keyboard teleop_twist_keyboard   # отдельный терминал
# 4) карта + навигация: запусти без use_nav:=false и ставь цель «Nav2 Goal» в RViz
ros2 launch rover_bringup bringup.launch.py
```

Launch-файлы: `drivers` (железо) · `localization` (rf2o+EKF) · `slam` (+slam_toolbox) ·
`nav2` (Nav2) · `bringup` (всё, флагами `use_slam/use_nav/use_rviz`).
