# mapping

## Purpose

Построение и обслуживание занятой карты (SLAM или загрузка с диска) по данным лидара/одометрии.

## Interface

[`MapBuilder`](../../contracts/README.md), [`MapStore`](../../contracts/README.md) в `fleet_contracts.mapping`.

## Dependencies

Планируемый интеграционный слой около `slam_toolbox` / Nav2 map server; пакет `fleet-contracts`.

## Planned implementations (real / mock / sim)

- Adapter поверх сохранённых карт для тестов; полный SLAM на железе.

## ROS topics / MQTT

Публикации/службы карты в стандарте ROS 2; синхронизация с backend — позже через `packages/backend/maps`.

## Not done yet

Конфиг SLAM, хранилище карт на RPi.
