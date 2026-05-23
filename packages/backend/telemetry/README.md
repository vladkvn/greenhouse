# telemetry (backend submodule)

## Purpose

Подписчик MQTT на топики телеметрии роботов, валидация payload против Pydantic, запись временных рядов/последнего состояния.

## Interface

Общая схема: `TelemetryEnvelope` в `fleet_contracts.messaging`.

## Dependencies

`aiomqtt` или asyncio-обёртка; PostgreSQL для хранения.

## Planned implementations (real / mock / sim)

Batch insert по настраиваемой частоте; отдельный consumer worker при масштабировании.

## ROS topics / MQTT

`fleet/robots/{robot_id}/telemetry`.

## Not done yet

Выбор между «сырыми JSON» таблица vs typed columns под основные метрики.
