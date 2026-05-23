# follow

## Purpose

Обнаружение человека на камере, re-ID и следование без смешения с голым `NavigateToPose` (режим включается оркестратором).

## Interface

[`PersonDetector`](../../contracts/README.md), [`PersonReIdentifier`](../../contracts/README.md), [`FollowController`](../../contracts/README.md) в `fleet_contracts.follow`.

## Dependencies

Компьютерное зрение (лёгкие модели на RPi или вынесение на более мощный узел позже).

## Planned implementations (real / mock / sim)

Mock траекторий; затем ONNX/OpenCV пайплайн.

## ROS topics / MQTT

Вход из модуля [`perception`](../perception/README.md); выход управления через [`motion`](../motion/README.md).

## Failure modes / notes

Потеря цели после таймаута — переход в безопасный Idle или останов по политике оркестратора.

## Not done yet

Выбор базовой модели и частот inference.
