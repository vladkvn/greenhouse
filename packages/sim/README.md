# sim

## Purpose

Пакет **`fleet-sim`**: простая **2D-симуляция** нескольких комнат на плоскости (стены как отрезки, связанные коридоры/проёмы в общей стене) и mock-реализации `fleet_contracts` без ROS (`LidarSource`, `MotionController`, `Localizer`, `CameraSource`, `PersonDetector`).

Численный интегратор и рейкаст лидара используются в демо-сценарии: объезд центроидов комнат с ограничением скорости по «ближайшей дистанции вперёд», затем обнаружение цели-силуэта и грубое следование в точку.

## Interface

- Реализации лежат в [`src/fleet_sim/mocks/`](./src/fleet_sim/mocks/).
- Топология мира: [`world.three_rooms_line_world`](./src/fleet_sim/world.py).
- Точка входа: `fleet-sim-demo` (см. `[project.scripts]` в `pyproject.toml`).

## Dependencies

- Python ≥ 3.11
- Пакет **`fleet-contracts`** из [`packages/contracts`](../contracts/) (установить **перед** или вместе с симом, см. ниже)

## Установка и запуск

Из корня монорепозитория:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -e packages/contracts
pip install -e "packages/sim[dev]"
ruff check packages/sim/src && python -m mypy packages/sim/src
fleet-sim-demo
```

## Planned implementations (real / mock / sim)

- Сделано в этом репозитории: много-комнатный мир, лидара рейкаст, truth-localization, геометрический «детектор» цели, демо FSM explore/follow.
- Далее: unit-тесты рейкаста, визуализация (matplotlib), мост к ROS 2 / Gazebo.

## ROS topics / MQTT

Не используется; при интеграции с роботом контракты те же, что и на борту.

## Not done yet

- CI job, сокращение дублирования с будущим Nav2-адаптером.

## Ограничения MVP

- Робот моделируется **точкой** (без габаритов); при выходе за полигон свободного пространства шаг отбрасывается.
- Локализация — **истина симулятора**, не SLAM.
