# commands

## Purpose

Приём команд от REST (или внутренних сервисов) и публикация их в MQTT `.../commands` с подтверждением через `.../commands/ack`.

## Interface

Командный payload: сообщения вида discriminated union в `greenhouse_contracts.messaging`; ack — `CommandAck`.

## Dependencies

MQTT publisher; возможная идемпотентность через `correlation_id`.

## Planned implementations (real / mock / sim)

Отложенное исполнение, retry при временном offline робота (опционально).

## ROS topics / MQTT

`greenhouse/robots/{robot_id}/commands`, `.../commands/ack`.

## Not done yet

RBAC кто может слать команды какому роботу.
