# sim

## Purpose

Пакет **`fleet-sim`**: простая **2D-симуляция** нескольких комнат на плоскости (стены как отрезки, связанные коридоры/проёмы в общей стене) и mock-реализации `fleet_contracts` без ROS (`LidarSource`, `MotionController`, `Localizer`, `CameraSource`, `PersonDetector`).

Численный интегратор и рейкаст лидара используются в демо-сценарии: объезд центроидов комнат с ограничением скорости по «ближайшей дистанции вперёд», затем обнаружение цели-силуэта и грубое следование в точку.

## Interface

- Реализации лежат в [`src/fleet_sim/mocks/`](./src/fleet_sim/mocks/).
- Топология мира: [`world.three_rooms_line_world`](./src/fleet_sim/world.py).
- Точки входа: `fleet-sim-demo` (консоль), `fleet-sim-viz` ([`demo_visual.py`](./src/fleet_sim/demo_visual.py) — окно matplotlib).

## Dependencies

- Python ≥ 3.11
- Пакет **`fleet-contracts`** из [`packages/contracts`](../contracts/)
- Для окна: **`matplotlib`** (extras `viz` или `dev`, см. ниже)

## Установка и запуск

Из корня монорепозитория:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -e packages/contracts
pip install -e "packages/sim[viz]"     # matplotlib + консольные команды
# или: pip install -e "packages/sim[dev]"  # то же + инструменты ruff/mypy
ruff check packages/sim/src && python -m mypy packages/sim/src   # при [dev]
fleet-sim-demo
```

Только консольное демо (`fleet-sim-demo`) не требует matplotlib, но установка `[viz]` добавляет и его.

**Окно с картой** — стены, редкие лучи лидара (бирюза, вращаются с роботом), зелёный треугольник робота, красная цель следования:

```bash
fleet-sim-viz
```

Нужен доступ к дисплею. При проблемах бэкенда: `export MPLBACKEND=TkAgg` (Linux/macOS часто не требуется).

## Planned implementations (real / mock / sim)

- Сделано в этом репозитории: много-комнатный мир, лидара рейкаст, truth-localization, геометрический «детектор» цели, демо FSM explore/follow, базовая **визуализация matplotlib**.
- Далее: unit-тесты рейкаста, мост к ROS 2 / Gazebo.

## ROS topics / MQTT

Не используется; при интеграции с роботом контракты те же, что и на борту.

## Not done yet

- CI job, сокращение дублирования с будущим Nav2-адаптером.

## Ограничения MVP

- Робот моделируется **точкой** (без габаритов); при выходе за полигон свободного пространства шаг отбрасывается.
- Локализация — **истина симулятора**, не SLAM.
