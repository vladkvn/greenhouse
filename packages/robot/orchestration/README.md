# orchestration

## Purpose

Машина состояний режимов: Idle, Mapping, Navigating, Following; авторизация и отмена миссий; единственная координация вызовов в другие модули по контрактам.

## Interface

[`RobotBehavior`](../../contracts/README.md), [`MissionHandler`](../../contracts/README.md) в `fleet_contracts.orchestration`.

## Dependencies

Все высокоуровневые контракты; без прямых вызросов между mapping и nav в обход оркестратора.

## Planned implementations (real / mock / sim)

ROS 2 узел состояния с подписками на события/команды.

## ROS topics / MQTT

Приём удалённых команд через [`telemetry`](../telemetry/README.md); возможный latched топик режима робота локально.

## Not done yet

Детальный граф переходов и приоритет `emergency_stop`.
