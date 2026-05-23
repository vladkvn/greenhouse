# telemetry

## Purpose

Мост к backend: публикация телеметрии и приём команд по MQTT (и при необходимости heartbeat по REST из backend).

## Interface

[`TelemetryPublisher`](../../contracts/README.md), [`CommandSubscriber`](../../contracts/README.md) в `fleet_contracts.telemetry_bridge`; DTO из `fleet_contracts.messaging`.

## Dependencies

`paho-mqtt` / `aiomqtt` в backend; на роботе — совместима с тем же клиентским API.

## Planned implementations (real / mock / sim)

Клиент MQTT с reconnect и очередью best-effort.

## ROS topics / MQTT

MQTT префикс `fleet/robots/{robot_id}` см. docs/ARCHITECTURE.md.

## Not done yet

Аутентификация клиента MQTT, backoff, сохранение неотправленного при офлайне.
