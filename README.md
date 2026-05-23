# GreenHouse Robot

Модульная система автономной тележки для **теплицы** (ROS 2 на Raspberry Pi) с **контрактами** между onboard, симуляцией/cloud backend (FastAPI + MQTT, multi-robot).

## Документы

- [Архитектура](docs/ARCHITECTURE.md) — компоненты, MQTT, дорожная карта
- [Принципы и playbook для разработчиков/агентов](AGENTS.md)

## Структура монорепозитория

| Пакет | Описание |
|-------|-----------|
| [packages/contracts](packages/contracts/README.md) | Общие `Protocol` и Pydantic DTO без ROS |
| [packages/robot](packages/robot/README.md) | ROS 2 workspace (адаптеры нав стек будущего кода) |
| [packages/backend](packages/backend/README.md) | Будущий FastAPI-сервис |
| [packages/sim](packages/sim/README.md) | Mocks и симуляция |
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

## Намеренно не сделано этим шагом

ROS 2 ноды, backend-приложение, Docker Compose для сервисов, симулятор Gazebo — только контракты и документация-план.
