# navigation

## Purpose

Глобальное/локальное планирование и отправка робота в точку карты через абстрактный интерфейс.

## Interface

[`GoalNavigator`](../../contracts/README.md), [`PathPlanner`](../../contracts/README.md) в `greenhouse_contracts.navigation`.

## Dependencies

Nav2 (NavigateToPose et al.); `greenhouse-contracts`.

## Planned implementations (real / mock / sim)

Обёртка над action-клиентом Nav2; mock — ручное подтверждение успеха без движения.

## ROS topics / MQTT

Взаимодействие с Nav2 через actions; `cmd_vel` идёт через [`motion`](../motion/README.md).

## Not done yet

Параметры costmap под растения и проходы в теплице.
