# contracts

## Purpose

Языковые контракты между роботом, симулятором и backend: `typing.Protocol`, Pydantic-модели событий/команд, константы имён MQTT. Без зависимостей от ROS 2 и FastAPI.

## Interface

См. импорт из пакета `greenhouse_contracts` после установки в editable-режиме (`pip install -e packages/contracts`). Модули: `identifiers`, `geometry`, `perception`, `mapping`, `localization`, `navigation`, `follow`, `motion`, `orchestration`, `telemetry_bridge`, `messaging`.
## Dependencies

Python ≥ 3.11, `pydantic>=2`.

## Planned implementations (real / mock / sim)

- **Реально:** любой адаптер в `packages/robot` или backend реализует эти интерфейсы без расширения контрактов «вручную» в потребителях.
- **Mock:** см. [`packages/sim`](../sim/README.md).

## ROS topics / MQTT

Описание топиков MQTT и полей сообщений см. [`docs/ARCHITECTURE.md`](../../docs/ARCHITECTURE.md) и [`greenhouse_contracts.messaging`](./src/greenhouse_contracts/messaging.py).

## Not done yet

- Версионирование схем JSON (`schema_version`).
- Совместное CI с `packages/robot` (установка пакета в Docker build).
