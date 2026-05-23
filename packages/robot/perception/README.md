# perception

## Purpose

Восприятие: лидар, камера, IMU — адаптеры данных в доменные модели без привязки потребителей к ROS сообщениям.

## Interface

Контракты [`LidarSource`](../../contracts/README.md), [`CameraSource`](../../contracts/README.md), [`ImuSource`](../../contracts/README.md) из `fleet_contracts.perception`.

## Dependencies

ROS 2 ноды/драйверы сенсоров; пакет `fleet-contracts`.

## Planned implementations (real / mock / sim)

- Mock: `packages/sim`; реально: узлы-подписчики на `/scan`, `/camera/...`.

## ROS topics / MQTT

Примеры входов: `/scan`, `/camera/image_raw` (назначение финализируется в launch). MQTT не использует этот модуль напрямую.

## Failure modes / notes

При потере сенсора адаптер возвращает `is_available()==False`; оркестратор переводит систему в безопасное состояние.

## Not done yet

Узлы, калибровка камеры, синхронизация временных меток.
