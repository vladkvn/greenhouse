# GreenHouse

Флот **автономных мобильных роботов** для перевозки грузов в теплице: следование за
человеком, поездки на точку выгрузки и на зарядку, построение карты, объезд препятствий,
локализация, планирование маршрута, закрытые зоны и деление узких проездов между роботами.

**Стек:** Python. Ядро **не зависит от транспорта** — домен, интерфейсы и алгоритмы
ничего не знают про ROS. Симуляция, ROS 2 / Nav2 и драйверы железа подключаются как
адаптеры за одними и теми же `Protocol`-интерфейсами.

## Документы

- [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) — слои, интерфейсы, режимы, keep-out, координация флота
- [docs/ROADMAP.md](docs/ROADMAP.md) — план инкрементов

## Структура

```
src/greenhouse/
├── domain/         # geometry, grid, identifiers, errors — чистый домен
├── sensing/        # интерфейсы датчиков (лидар, камера, одометрия, IMU)
├── control/        # MotionController — приведение в движение
├── navigation/     # mapping, localization, planning, keepout
├── coordination/   # TrafficCoordinator — деление проездов
├── orchestration/  # режимы работы и команды (FSM)
└── runtime/        # Clock — абстракция времени
```

Адаптеры (`adapters/sim`, `adapters/ros2`) добавляются отдельными инкрементами —
см. roadmap. Сейчас зафиксированы **архитектура и интерфейсы** (Инкремент 0).

## Быстрый старт

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -e ".[dev]"

python -c "import greenhouse; print(greenhouse.__version__)"
ruff check src
mypy
```

## Принципы

Contract-first · ядро без ROS · заменяемые реализации за одним интерфейсом ·
сначала симуляция · инкременты вертикальными срезами · типобезопасность на границах.
Подробнее — в [ARCHITECTURE.md](docs/ARCHITECTURE.md).
