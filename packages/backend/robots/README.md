# robots

## Purpose

Реестр роботов: регистрация при первом подключении, heartbeat, статусы онлайн/офлайн, метаданные (имя зоны и т.д.).

## Interface

DTO робота определяются в `greenhouse_contracts.messaging` (идентификаторы и envelope) расширяются при добавлении API.

## Dependencies

PostgreSQL через backend ORM-слой; REST handlers.

## Planned implementations (real / mock / sim)

CRUD роботов минимально достаточный для управления командой через UI.

## ROS topics / MQTT

Чтение сообщений статусов из топика телеметрии.

## Not done yet

Conflict resolution если два робота с одинаковым `robot_id`.
