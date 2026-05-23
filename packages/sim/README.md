# sim

## Purpose

Mock-реализации интерфейсов `greenhouse_contracts` и в перспективе симуляция (Gazebo/другой движок) с теми же adapter-границами.

## Interface

Каждый mock реализует соответствующий `Protocol` из `packages/contracts`.

## Dependencies

Только `greenhouse-contracts` для моков; симуляция добавит ROS 2 и launch-файлы.

## Planned implementations (real / mock / sim)

- Фаза 1: статические mocks в Python.
- Фаза 2: генерация простых синтетических лидар-снимков для unit-тестов.
- Фаза 3: Gazebo интеграция.

## ROS topics / MQTT

Mocks могут использоваться без ROS в чистых тестах orchestration.

## Not done yet

Автоматические тесты и CI job.
