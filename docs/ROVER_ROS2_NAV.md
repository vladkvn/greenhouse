# ROS 2 карта + навигация в точку (rf2o + EKF + slam_toolbox + Nav2)

Полноценный ROS 2 Humble-стек для ровера: **живое построение карты и езда в точку**, как
на гифках [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox). Без энкодеров —
одометрия берётся с лидара (rf2o) и сливается с курсом IMU (EKF).

Воркспейс: [`ros2_ws/`](../ros2_ws). Это **отдельный ROS-стек** для карты/навигации; боевой
`follow_me` (следование за человеком, YOLO) остаётся как есть — общее только железо.

## Поток данных и TF

```
RPLIDAR A1 ──/scan──┬─▶ rf2o_laser_odometry ──/odom_rf2o (vx,vyaw)──┐
                    │                                                ├─▶ EKF ─▶ TF odom→base_footprint
BNO085 ──/imu/data──────────────────────(yaw, vyaw)─────────────────┘        + /odometry/filtered
                    │                                                         │
                    ├─▶ slam_toolbox (online async) ──▶ TF map→odom + /map ◀──┘
                    │
                    └─▶ Nav2 (costmaps/RPP) ──/cmd_vel──▶ esp32_cmd_vel_bridge ──"L R\n"──▶ ESP32

TF: map → odom → base_footprint → base_link → { laser (yaw=π, x=−0.13), imu_link }
```

Почему так: энкодеров нет → одометрию даёт **rf2o** (scan-to-scan). Чистый scan-matching
плыл на поворотах («карта-облако») → курс якорит **BNO085 через EKF**. Карта и
`map→odom` — на **slam_toolbox** (loop closure «сводит» карту). Езда в точку — **Nav2** с
Regulated Pure Pursuit (умеет разворот на месте, подходит skid-steer).

---

## 0. Установка (на Jetson, один раз)

ROS 2 **Humble** (Ubuntu 22.04). Если ещё не стоит — по
[docs.ros.org Humble (Debian)](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).

```bash
# бинарные пакеты стека
sudo apt update
sudo apt install -y \
  ros-humble-slam-toolbox \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-robot-localization \
  ros-humble-robot-state-publisher ros-humble-xacro \
  ros-humble-teleop-twist-keyboard ros-humble-rviz2 \
  python3-colcon-common-extensions python3-vcstool python3-serial

# IMU-библиотеки в окружение Python (для узла BNO085)
sudo pip3 install adafruit-blinka adafruit-circuitpython-bno08x adafruit-extended-bus
```

Пакеты не из apt (sllidar_ros2, rf2o ros2) — из исходников:

```bash
cd ~/GreenHouse/ros2_ws          # путь к репо на Jetson
vcs import src < rover.repos     # клонирует sllidar_ros2 и rf2o_laser_odometry
rosdep install --from-paths src --ignore-src -r -y   # доустановит зависимости
```

## 1. Сборка

```bash
cd ~/GreenHouse/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash        # добавить в ~/.bashrc для удобства
```

## 2. Доступ к железу

```bash
sudo usermod -aG dialout $USER   # serial (перелогиниться); luki обычно уже в группе
# различать USB по by-id (порядок ttyUSB плавает): CH340=ESP, CP2102/Silabs=RPLIDAR
ls -l /dev/serial/by-id/
# I2C для IMU: /dev/i2c-7 должен быть доступен (гребень pins 3/5)
```

Если порты другие — передать в launch: `lidar_port:=/dev/ttyUSB1 esp_port:=/dev/ttyUSB0`
(или прописать by-id путь).

---

## 3. Bring-up по слоям (проверять каждый)

### 3.1 Драйверы

```bash
ros2 launch rover_bringup drivers.launch.py
```
Проверка в другом терминале (`source install/setup.bash`):
```bash
ros2 topic hz /scan          # ~5–8 Гц от RPLIDAR A1
ros2 topic echo /imu/data --once
ros2 run tf2_tools view_frames   # дерево: base_footprint→base_link→laser/imu_link
# мост ESP32: дать тестовый твист (робот на подставке!)
ros2 topic pub -r 10 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.1}}'   # Ctrl-C → STOP по deadman
```
**Готово, если:** есть `/scan`, `/imu/data`, корректное TF-дерево, по `/cmd_vel` крутятся
колёса, при обрыве — стоп (failsafe).

### 3.2 Карта (SLAM) — как на гифках

```bash
ros2 launch rover_bringup bringup.launch.py use_nav:=false
# отдельный терминал — телеоп (катать вручную, строя карту):
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
В RViz (Fixed Frame = `map`) видно, как `/map` строится по мере езды; при возврате в уже
виденное место карта «схлопывается» loop closure. Катать **медленно**, давать лидару
осматриваться, заезжать в петли.

Сохранить карту:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/GreenHouse/ros2_ws/src/rover_bringup/maps/my_map
# и/или сериализованную позу-граф slam_toolbox (для дозагрузки карты позже):
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '/home/luki/GreenHouse/ros2_ws/src/rover_bringup/maps/my_map'}"
```

### 3.3 Навигация в точку

```bash
ros2 launch rover_bringup bringup.launch.py        # drivers + SLAM + Nav2 + RViz
```
В RViz нажать инструмент **«Nav2 Goal»** и кликнуть точку на карте → робот построит путь
(`/plan`) и поедет, объезжая препятствия по лидару. Цель можно ставить и программно:
```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 1.5, y: 0.0}, orientation: {w: 1.0}}}}"
```
**Готово, если:** робот доезжает в цель с объездом, в допуске `xy_goal_tolerance` (0.25 м).

> Карта строится И навигация работает одновременно (SLAM + Nav2). Чтобы ездить по **готовой**
> карте без достройки — запустить slam_toolbox в режиме `localization` (см. §6).

---

## 4. Калибровка (помечено `# CALIBRATE`)

Порядок — снизу вверх; без этого стек поедет неверно.

| Что | Где | Как |
|---|---|---|
| **Перёд/разворот лидара** | `urdf/rover.urdf.xacro` `lidar_yaw`, `lidar_x` | Поставь препятствие строго по курсу ~1 м. В RViz луч `/scan` должен лечь перед роботом (по +X base_link). Если сзади/сбоку — крути `lidar_yaw` (у нас π = 180°). `lidar_x=−0.13` = лидар позади центра. |
| **Борта приводов** | `config`/launch `swap_sides`, мост | Твист `angular.z:0.5` → робот крутится влево (CCW). Если вправо — `swap_sides:=true`; если едет назад на `linear.x>0` — поменять знак в прошивке/`driveSide`. |
| **Скорость↔ШИМ** | `wheel_base`, `max_linear`, `pwm_max`, `pwm_min_move` | Командуй `linear.x` N сек, померь путь → подгони `max_linear` под `pwm_max`. `pwm_min_move`=ШИМ, ниже которого мотор молчит (~120). |
| **Знак yaw IMU** | `urdf` `imu_link` rpy | Поворот вправо должен уменьшать yaw в EKF (CW = −). Если карта/повороты зеркалятся — `imu_link` `rpy="${pi} 0 0"` (roll=π инвертирует yaw и gyro.z согласованно). |
| **Габарит робота** | `config/nav2.yaml` `robot_radius`, `inflation_radius` | Описанный радиус шасси + зазор. У нас 0.18 м радиус, 0.35 инфляция (тесные ряды теплицы — не задирать). |
| **Скорости Nav2** | `nav2.yaml` `desired_linear_vel`, RPP | Под реальный потолок привода (старт ≥ `pwm_min_move`). |

EKF-нюанс: rf2o даёт `vx, vyaw`, IMU — `yaw, vyaw` (якорь курса). Если IMU нет/глючит —
временно `use_imu:=false`; одометрия деградирует до чистого rf2o (курс поплывёт на поворотах).

---

## 5. Топики/TF и диагностика

| Топик | Кто | Что |
|---|---|---|
| `/scan` | sllidar | LaserScan, frame `laser` |
| `/imu/data` | bno085_imu_node | sensor_msgs/Imu, frame `imu_link` |
| `/odom_rf2o` | rf2o | лазерная одометрия (без TF) |
| `/odometry/filtered` | EKF | слитая поза + TF `odom→base_footprint` |
| `/map` | slam_toolbox | OccupancyGrid + TF `map→odom` |
| `/plan`, `/cmd_vel` | Nav2 | путь и команда скорости → мост ESP32 |

```bash
ros2 run rqt_tf_tree rqt_tf_tree         # TF целостно? нет двух издателей одного ребра?
ros2 topic echo /diagnostics             # EKF print_diagnostics
ros2 node list                           # все узлы поднялись
```
Частые грабли: **нет TF `odom→base_footprint`** → не запущен EKF (или rf2o публикует TF —
должно быть `publish_tf:false`). **Карта дрожит на поворотах** → знак yaw IMU (§4). **Nav2 не
строит путь** → нет `/map` или `map→odom`, либо цель в `unknown`-зоне.

---

## 6. Расширения (когда понадобится)

- **Езда по готовой карте без достройки:** slam_toolbox `mode: localization` + `map_file_name:`
  на сохранённый pose-graph (§3.2). Тогда карта статична, только локализация.
- **Одновременно телеоп и Nav2** (ручной перехват) — добавить `twist_mux`: входы
  `cmd_vel_teleop` (приоритет) и `cmd_vel_nav`, выход `/cmd_vel` на мост.
- **Сглаживание команд** — `nav2_velocity_smoother` (сейчас сглаживание делает сам мост
  через `max_pwm_step`).
- **Запуск автозапуском** — обернуть `bringup.launch.py` в systemd (как `followme.service`),
  `use_rviz:=false`.
