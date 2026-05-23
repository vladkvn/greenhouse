# robot

## Purpose

Onboard программное обеспечение: ROS 2 workspace на Raspberry Pi (Python). Адаптеры сенсоров и стек навигации реализуют контракты из [`packages/contracts`](../contracts/README.md).

## Interface

Подмодули экспортируют ROS 2 узлы или библиотеки, которые через адаптеры удовлетворяют Protocol из `fleet_contracts`.

## Dependencies

ROS 2 (дистро уточнить при сборке образа); Python 3.x дистро; локальная установка `fleet-contracts`.

## Planned implementations (real / mock / sim)

- Поэтапно: mocks из `packages/sim`, затем реальные драйверы и Nav2/`slam_toolbox`.

## ROS topics / MQTT

Планируемый граф — в [`docs/ARCHITECTURE.md`](../../docs/ARCHITECTURE.md). Телеметрия/command bridge — модуль [`telemetry`](./telemetry/README.md).

## Not done yet

- Структура colcon workspace, `package.xml`, launch-файлы.
