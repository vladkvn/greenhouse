# localization

## Purpose

Локализация робота в известной карте (particle filter / одометрия+correction).

## Interface

[`Localizer`](../../contracts/README.md) в `fleet_contracts.localization`.

## Dependencies

Nav2 AMCL или аналог; `fleet-contracts`.

## Planned implementations (real / mock / sim)

Mock фиксированной позы в `packages/sim`; реальный ACML wrapper.

## ROS topics / MQTT

`/tf`, `/particle_cloud` или эквивалент — задаётся адаптером.

## Failure modes / notes

При `lost` статусе локализации оркестратор блокирует `go_to`, требует relocalization.

## Not done yet

Стратегия восстановления позы в условиях помещений.
