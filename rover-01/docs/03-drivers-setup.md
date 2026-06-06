# 03 — ПО и драйвера: что куда ставить

> Версии ROS2-дистрибутивов меняются. Перед установкой свериться с актуальным LTS
> на docs.ros.org (на момент написания актуальные LTS — Humble и Jazzy).
> Дистрибутив выбирать **под версию Ubuntu**, которую поддерживает образ Jetson.

## Где что живёт

| ПО / пакет              | Где ставится | Зачем                                   |
|-------------------------|--------------|-----------------------------------------|
| Ubuntu + ROS2 (desktop) | Jetson       | База                                    |
| `sllidar_ros2`          | Jetson       | Драйвер RPLIDAR → `/scan`               |
| `slam_toolbox`          | Jetson       | Картирование + локализация              |
| `nav2` (navigation2)    | Jetson       | Планирование + объезд                   |
| `rf2o_laser_odometry`   | Jetson       | Одометрия по лидару (пока нет энкодеров) |
| `robot_localization`    | Jetson       | EKF-фьюжн odom + IMU                     |
| `teleop_twist_keyboard` | Jetson       | Ручное вождение                         |
| `micro-ROS Agent`       | Jetson       | Мост между ESP32 и ROS2                 |
| micro-ROS firmware      | ESP32        | ESP32 как ROS2-нода                     |
| (рабочая станция) RViz2 | ноутбук/Jetson | Визуализация                          |

## 1. Jetson: база

1. Прошить Jetson официальным образом (JetPack), поднять Ubuntu.
2. Установить ROS2 нужного дистрибутива (по официальной инструкции apt).
3. Создать рабочее пространство:
   ```bash
   mkdir -p ~/ros2_ws/src && cd ~/ros2_ws
   colcon build && source install/setup.bash
   ```
4. Добавить `source ~/ros2_ws/install/setup.bash` в `~/.bashrc`.

## 2. Драйвер RPLIDAR

```bash
cd ~/ros2_ws/src
git clone https://github.com/Slamtec/sllidar_ros2.git
cd ~/ros2_ws && colcon build --packages-select sllidar_ros2
# дать права на порт:
sudo chmod 666 /dev/ttyUSB0     # или прописать udev-правило
```
Проверка: запустить launch драйвера, в RViz2 увидеть облако точек `/scan`.
Уточнить **модель** (A1/A2/C1) — у них разный baudrate в launch-файле.

## 3. Навигация и SLAM

```bash
sudo apt install ros-<distro>-slam-toolbox ros-<distro>-navigation2 \
  ros-<distro>-nav2-bringup ros-<distro>-robot-localization \
  ros-<distro>-teleop-twist-keyboard
```
`rf2o_laser_odometry` — собрать из исходников в `~/ros2_ws/src` (apt-пакета может не быть под все дистрибутивы).

## 4. micro-ROS на Jetson (мост)

```bash
# в отдельном ws
git clone -b <distro> https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
colcon build && source install/setup.bash
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
# запуск агента (serial, USB):
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB1
```

## 5. micro-ROS на ESP32 (прошивка)

Среда — PlatformIO или ESP-IDF + компонент `micro_ros_platformio`.
ESP32 становится нодой, которая:
- **подписывается** на `/cmd_vel` (Twist) → крутит ШИМ через L298N;
- **публикует** `/imu` (с BNO055) и обратную связь по моторам;
- держит **watchdog** и аварийный стоп локально.

Детали прошивки и распиновка — в `04-firmware-esp32.md`.

## Карта запретных зон (keep-out) в Nav2

Запреты задаются НЕ сканированием всей территории, а слоем-фильтром costmap:
1. Готовишь «маску» — изображение карты, где запретные области закрашены.
2. Подключаешь `KeepoutFilter` (и/или `SpeedFilter`) в конфиге Nav2 costmap.
3. Планировщик не строит путь через закрашенные зоны.

Это мягкий слой (зависит от локализации) — обязательно дублировать физическим
стопом на ESP32. Конкретный конфиг доведём на этапе 6 плана тестирования.

## Порядок поднятия (smoke-чеклист)

1. `ros2 topic list` — система жива.
2. Лидар: `/scan` идёт, в RViz2 видно облако.
3. micro-ROS agent видит ESP32, `/cmd_vel` доходит до моторов.
4. `/imu` публикуется, значения адекватны при наклоне.
5. SLAM строит карту при ручной езде (teleop).
