# motion

## Purpose

Единственная точка выдачи команд на приводы/шасси (скорость, аварийный стоп).

## Interface

[`MotionController`](../../contracts/README.md) в `greenhouse_contracts.motion`.

## Dependencies

Платформа: дифф. привод, MCU-мост или прямой ROS драйвер (уточняется аппаратно).

## Planned implementations (real / mock / sim)

Mock регистрирует команды для тестов; реально — `/cmd_vel` или serial protocol.

## ROS topics / MQTT

Типично публикация `geometry_msgs/Twist` в адаптере.

## Not done yet

Реализация конкретного привода и лимиты ускорения.
