# GreenHouse

Монорепозиторий платформы **автономного мобильного робота**: ROS 2 на Raspberry Pi, контракты между onboard-софтом, mocks/симуляцией и cloud backend (FastAPI + MQTT, multi-robot). Сценарий — **работа внутри помещений**, 2D-навигация; прикладной контекст задаётся отдельно.

## Документы

- [Архитектура](docs/ARCHITECTURE.md) — компоненты, MQTT, дорожная карта
- [Принципы и playbook для разработчиков/агентов](AGENTS.md)

## Структура монорепозитория

| Пакет | Описание |
|-------|-----------|
| [packages/contracts](packages/contracts/README.md) | Общие `Protocol` и Pydantic DTO без ROS (`fleet-contracts`) |
| [packages/robot](packages/robot/README.md) | ROS 2 workspace (адаптеры навигационного слоя и будущего кода) |
| [packages/backend](packages/backend/README.md) | Будущий FastAPI-сервис |
| [packages/sim](packages/sim/README.md) | **`fleet-sim`**: несколько комнат, mocks контрактов, демо `fleet-sim-demo` |
| `infra/` | Инфраструктура по мере появления (Compose, MQTT и др.) |

## Быстрый старт (контракты Python)

Из корня репозитория:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -e "packages/contracts[dev]"
ruff check packages/contracts/src
python -m mypy packages/contracts/src
```

## Быстрый старт (симулятор)

После установки контрактов:

```bash
pip install -e "packages/sim[dev]"
fleet-sim-demo
```

Подробнее: [packages/sim/README.md](packages/sim/README.md).

## Намеренно не сделано

Полная интеграция ROS 2/Gazebo Nav2, продакшен backend и infra Compose — развиваются отдельными шагами; в `fleet-sim` уже есть воспроизводимое Python-демо без брокера.