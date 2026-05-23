# backend

## Purpose

Центральный сервис: REST для оператора, многопользовательские роботы (`robot_id`), приём телеметрии и маршрутизация команд через MQTT и БД.

## Interface

Разделённые подпакеты; типы сообщений общие через [`packages/contracts`](../contracts/README.md).

## Dependencies

FastAPI, PostgreSQL (async-драйвер), MQTT-клиент, `fleet-contracts`.

## Planned implementations (real / mock / sim)

Сначала ingestion + сохранение; затем авторизация и UI.

## ROS topics / MQTT

Не использует ROS; топики см. submodule `telemetry`/`commands`.

## Not done yet

Весь приложенческий код, миграции БД, docker-compose образ сервиса.
