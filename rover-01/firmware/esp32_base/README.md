# esp32_base — прошивка моторного слоя (micro-ROS)

ESP32 как ROS2-нода `rover_base`. Принимает `/cmd_vel`, крутит L298N (skid-steer),
держит watchdog и публикует `/rover/heartbeat`. Транспорт — Serial (USB).

## Что делает

- Подписка `/cmd_vel` (`geometry_msgs/Twist`) → скорости бортов → ШИМ на L298N.
- **Watchdog:** нет команды дольше `CMD_TIMEOUT_MS` (300 мс) → моторы в стоп.
- **Slew-rate** (плавный пуск) и **мёртвая зона ШИМ** — бережёт L298N и питание.
- **Heartbeat** `/rover/heartbeat` (`std_msgs/Int32`, растёт) — Jetson видит, что ESP32 жив.
- **Автопереподключение** к агенту; при потере агента — немедленный стоп.

## Разводка (текущая конфигурация)

1× L298N, борт = канал: левые 2 мотора → канал A, правые 2 → канал B.
Пины и параметры — в `include/config.h`. Перед прошивкой проверь там:
GPIO под свою плату, `WHEEL_BASE_M` (измерь колею), `PWM_DEADZONE` (подбери на этапе 0).

## Сборка и прошивка (PlatformIO)

```bash
# из папки firmware/esp32_base
pio run                 # собрать
pio run -t upload       # прошить (ESP32 в USB)
pio device monitor      # лог (115200)
```
Первая сборка скачает micro_ros_platformio и соберёт библиотеку micro-ROS — это долго, это нормально.

## Запуск агента на Jetson

ESP32 не заработает «сам по себе» — на стороне Jetson нужен micro-ROS агент
(установка — см. `../../docs/03-drivers-setup.md`):

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
```
Порт (`/dev/ttyUSB0`) и baudrate (`115200`) должны совпадать с `platformio.ini`.

## Проверка (этап 1 плана тестирования)

**Колёса на подставке, в воздухе.**

```bash
ros2 topic list                      # должны быть /cmd_vel и /rover/heartbeat
ros2 topic echo /rover/heartbeat     # счётчик растёт -> нода жива

# поехали вперёд:
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"
# поворот на месте:
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 1.0}}"
```

Останови публикацию (Ctrl-C) → колёса должны встать через ~300 мс (watchdog).

## Известные ограничения

- **Open-loop:** без энкодеров ШИМ ≠ реальная скорость; `MAX_WHEEL_SPEED` — грубая оценка,
  «прямо» будет приблизительным. Лечится энкодерами + замкнутым контуром (см. план).
- `board_microros_distro` в `platformio.ini` при необходимости задай под дистрибутив Jetson.
- Не повышай `espressif32` до версии с Arduino-core 3.x без правки PWM (изменён LEDC API).

## TODO (следующие итерации)

- IMU BNO055 → публикация `/imu`.
- Энкодеры → `/wheel_ticks` или `/odom`, замкнутый PID по скорости.
- Аппаратная аварийная кнопка/бампер на прерывании → жёсткий стоп.
