# Архитектура ровера: боевое железо за контрактами ядра

Документ описывает, как реальный робот (**Jetson Orin Nano + ESP32-CAM 4WD**, лидар
RPLIDAR A1, USB-камера) получает все режимы и навигацию, уже проверенные в симуляции,
**без переписывания ядра**. Дополняет [ARCHITECTURE.md](ARCHITECTURE.md) (целевая
послойная архитектура) и [ROADMAP.md](ROADMAP.md).

> Главный принцип ARCHITECTURE.md сохраняется: ядро (`domain / sensing / control /
> navigation / coordination / orchestration / runtime`) не зависит от транспорта.
> Железо подключается **адаптерами** за теми же `typing.Protocol`, что и симуляция.

---

## 1. Два мира в репозитории и решение

| Мир | Где | Что умеет | Чего нет |
|---|---|---|---|
| **Ядро + sim** | `src/greenhouse/` | Послойное, contract-first, инкременты 1–13, покрыто тестами: A*-планирование, объезд, scan-match локализация, IMU-фьюжн, картирование, keep-out, следование, координация, зарядка | Запускается только за `adapters/sim` — на железе нет |
| **Боевой код** | `follow-me/follow_me/` | Реальные драйверы на Jetson: YOLO/TensorRT-детектор, RPLIDAR-чтение, фьюжн, UDP-моторы | Плоский цикл, только следование, без режимов/карты/локализации/планировщика |

**Решение.** Не строим второй стек. Заводим железо **новым пакетом
`src/greenhouse/adapters/jetson/`**, зеркалящим `adapters/sim/`, за теми же контрактами.
Тогда «мозг» из симуляции едет на роботе, а меняются только источники данных и привод.
Добавить датчик/режим = подменить адаптер или зарегистрировать поведение — ядро не трогаем.

Несущая логика — гибрид трёх независимо сгенерированных вариантов: **богатые поведения**
(неблокирующий FSM + реестр режимов) в **дисциплине адаптеров** (jetson зеркалит sim,
ядро неизменно) с **порядком ввода в эксплуатацию от телеопа вверх** и деградацией в стоп
как несущей осью.

---

## 2. Раскладка модулей

Статус: **reuse** — переносится дословно, **extend** — дорабатывается, **new** — новый.

### Ядро — переиспользуется как есть (reuse)
| Модуль | Что даёт роверу |
|---|---|
| `orchestration/modes.py` | `RobotMode` — все 4 режима (+`YIELDING`) уже есть |
| `navigation/planning.py` | `AStarPlanner` (инфляция footprint), `ReactiveLocalPlanner` (объезд + габарит) |
| `navigation/localization.py` | `ScanMatchLocalizer` (инкр.5), `ImuFusedLocalizer` (инкр.12) — замена отсутствующей одометрии |
| `navigation/mapping.py` | `EvidenceGridMapper`, `ConfidenceGatedMapper`, `nearest_frontier`, `FileMapStore` |
| `navigation/keepout.py` | `InMemoryKeepoutRegistry`, `Zone`, `ZoneKind` — блокировки оператором |
| `navigation/following.py` | `PersonFollower` — низкоуровневый контроллер фазы TRACKING |
| `coordination/*` | `TrafficCoordinator`, yielding — для флота (на одиночном роботе спит) |

### Ядро — добавляем (new)
| Модуль | Роль |
|---|---|
| `orchestration/orchestrator.py` | `Orchestrator` реализует пустующий Protocol `MissionHandler`: неблокирующий `tick()`, единственная точка переключения режимов |
| `orchestration/registry.py` | `BehaviorRegistry` — реестр `RobotMode → Behavior`; добавить режим = `register(...)` |
| `orchestration/supervisor.py` | `Supervisor` — надрежимная безопасность (E-stop, потеря локализации, низкий заряд, потеря связи) |
| `orchestration/behaviors/*.py` | `Idle/Following/Navigating/Mapping/Charging` Behavior |
| `navigation/navigator.py` | `StepwiseNavigator` реализует `GoalNavigator`: логика `SimGoalNavigator` вынесена в ядро с неблокирующими `begin()/step()`. Один объект обслуживает `GoTo`, `Following.GOTO_LAST_SEEN`, `Charging` |
| `navigation/protocols.py` | `RobotLike` Protocol — поза из `localizer.latest().pose`, а не из истины sim |

### Адаптеры железа — новый пакет (new) `src/greenhouse/adapters/jetson/`
| Класс | Контракт | Внутри |
|---|---|---|
| `JetsonLidar` | `LidarSource` | обёртка над `follow_me.LidarThread`; numpy[360] → `LidarScan` (луч 0 вперёд) |
| `JetsonMotion` | `MotionController` | `Twist2D` → «L R» скид-стир по UDP + `DriveWatchdog` |
| `JetsonTargetDetector` | `TargetDetector` | `follow_me` YOLO+fuse → `TargetObservation` |
| `DeadReckonOdometry` | `OdometrySource` | интеграл поданной `Twist2D` (без энкодеров) |
| `JetsonImu` | `ImuSource` | гироскоп MPU6050/BNO055 (или `ZeroImu` заглушка) |
| `WallClock` | `runtime.Clock` | `time.monotonic()` |
| `build_jetson_robot` | — | композиционный корень; **жёстко запрещает `SimTruthLocalizer`** |

---

## 3. Режимы и под-FSM следования

См. схему в обсуждении. Каждый тик `Orchestrator.tick`: `supervisor.check()` (preempt) →
`behavior.step()` активного режима → применить возвращённый `RobotMode | None`.

**Добавление режима (требование расширяемости):**
```python
registry.register(NewBehavior(robot=r, ...))   # + член RobotMode, + класс Behavior
```

**Под-FSM следования** (`FollowingBehavior`, режим всё время `FOLLOWING`) — закрывает
требование «за угол → точка последнего видения → ожидание»:

```
 TRACKING ──нет цели >0.5с──▶ TARGET_LOST ──есть last-seen──▶ GOTO_LAST_SEEN
    ▲                                                              │ доехал
    │ цель снова видна (с любой фазы)                              ▼
    └──────────────── IDLE_WAIT ◀──таймаут 6с── SEARCH ◀──────────┘
                         │ оператор Stop → режим IDLE
```

- `TRACKING` — `PersonFollower` доворачивает и держит дистанцию; **каждый кадр**
  запоминает `last_seen` как проекцию полярного наблюдения в карту по текущей позе:
  `last_seen = pose ⊕ (range, bearing)` — точка, *куда ушёл человек*.
- `GOTO_LAST_SEEN` — едет в неё **тем же `StepwiseNavigator`, что `GoTo`**: с объездом и
  учётом габарита. Цель показалась по дороге → мгновенно `TRACKING`.
- `SEARCH` — доехал, цели нет: поворот на месте, осмотр сектора.
- `IDLE_WAIT` — не нашёл: стоп, ждём; цель появилась → `TRACKING`.

`PersonFollower` сейчас last-seen не помнит — пробел закрывает обёртка, не трогая сам
контроллер.

---

## 4. Объезд и габариты

Робот = окружность `robot_radius_m`. Габарит учитывается **инфляцией** препятствий:
- глобально — `AStarPlanner._inflate` (круглое ядро `ceil(radius/resolution)`);
- локально — `ReactiveLocalPlanner` (стоп-зазор `radius+0.05`, объезд дугой, гашение
  скорости по клиренсу);
- на ходу — `StepwiseNavigator._fuse_scan` домешивает видимые лидаром препятствия и
  перепланирует каждые `replan_every` тиков.

Keep-out сливается в ту же занятость (`apply_to`: `KEEPOUT → OCCUPIED`) и раздувается
наравне со стенами. Реальная тележка прямоугольная → берём **описанный радиус** (полудиагональ
корпуса) + 5 см; поворот на месте у скид-стира всегда возможен, поэтому это корректно.

---

## 5. Локализация без энкодеров

`Localizer.update(*, scan, odometry)` требует одометрию, а энкодеров нет. Решение:
- `DeadReckonOdometry` интегрирует поданную `Twist2D` по dt — грубый prior;
- **позицию защёлкивает scan-match** (инкр.5);
- **курс перебивает IMU** (инкр.12) — без него курс плывёт при проскальзывании скид-стира.

Предел честный: на пустой местности мало возвратов → `confidence→0` → `is_lost` → стоп.
Митигируется keep-out периметром, обязательным IMU и скоростью ≤0.3–0.4 м/с. На больших
площадях — `slam_toolbox` через адаптер `MapBuilder` без правки локализатора.

---

## 6. Картирование и keep-out

- **MAPPING** — `MappingBehavior` (пошаговая обёртка frontier-исследования:
  `EvidenceGridMapper` + `nearest_frontier`); по исчерпании границ → `MapStore.save` → IDLE.
  Карта переживает перезапуск через `FileMapStore`.
- **Непрерывная актуализация** — `ConfidenceGatedMapper` поглощает скан только при
  `confidence ≥ 0.5`: при дрейфе позы карта не засоряется.
- **Keep-out оператором** — новая команда `EditKeepout(op, zone)` → `InMemoryKeepoutRegistry`;
  `StepwiseNavigator` подхватывает на следующем перепланировании во всех ездовых режимах.
  Так закрывается полуоткрытая местность (открытые края, газон) **без физических стен**.

---

## 7. Зарядка

`ChargingBehavior` (под-FSM `GOTO_DOCK → DOCKED`). Доезд в зону дока — тем же
`StepwiseNavigator`. Триггеры: команда `GoCharge` или `Supervisor` (низкий заряд). Нужен
`BatterySource` (ADC напряжения с ESP32). **Финальная стыковка** (последние 0.5 м) точности
scan-match не хватает — нужен терминальный доводчик (ArUco-маркер / механическая воронка),
вынесен в последний инкремент; до него зарядка полуавтоматическая.

---

## 8. Failsafe — три уровня деградации в стоп

1. **ESP32 watchdog 0.5 с** (аппаратный) — нет команды → моторы стоп.
2. **`DriveWatchdog`** в `JetsonMotion` — heartbeat ~100 мс; зависание процесса >0.3 с → стоп
   до срабатывания грубого ESP32-failsafe.
3. **`Supervisor`** в `Orchestrator` — `EmergencyStop`→IDLE из любого режима; `is_lost`→
   safe-stop; `battery<low`→форс CHARGING; потеря связи с ESP32→стоп.

Плюс независимый аварийный лидарный стоп поверх любого режима (согласует объезд с failsafe).

---

## 9. План инкрементов

| # | Шаг | Что проверяет |
|---|---|---|
| **0** | Телеоп через `MotionController` (`adapters/jetson`: motion + lidar + WallClock) | seam адаптеров работает, угол лидара и команда→скорость откалиброваны |
| 1 | Следование в новой архитектуре (паритет с follow_me) | перенос follow_me в стек ядра без потери функции |
| 2 | `Orchestrator` + `BehaviorRegistry` + `Supervisor` (sim-first) | пустые Protocol стали работающим FSM с реестром |
| 3 | `StepwiseNavigator` + `RobotLike` | неблокирующая навигация, переиспользуемая GoTo/Following/Charging |
| 4 | Локализация лидар+IMU без энкодеров (на железе) | поза стабильна при геометрии, безопасная деградация |
| **5** | `FollowingBehavior` last-seen → search → idle (флагман) | «за угол» полным планировщиком → ожидание |
| 6 | Картирование + сохранение | режим MAPPING |
| 7 | `GoTo` на железе + keep-out оператором | навигация на точку, блокировки, габарит |
| 8 | Зарядка (доезд в зону дока) + все 4 режима + `EmergencyStop` | полный набор режимов |
| 9 | Закрепление failsafe-слоя | все ветки деградации |
| 10 | Автодокинг (`DockApproach`) | автономная стыковка (отдельный риск) |

---

## 10. Открытые вопросы (нужны физические данные)

- **Габарит шасси** (`robot_radius_m`) — реальные д×ш 4WD-кита.
- **Монтаж лидара**: какой raw-бин смотрит вперёд, направление вращения (CW/CCW), видит ли
  лидар собственный корпус (тогда маскируем секторы FOV). Обязателен калибровочный тест
  «препятствие по курсу == луч 0».
- **Калибровка команда→скорость** без энкодеров (стендовый прогон по лидару): `wheel_base_m`,
  коэффициент `(v,w) → PWM`, реальные `MotionLimits`.
- **Датчик заряда**: ESP32-телеметрия напряжения или отдельный INA219?
- **Механизм стыковки** на доке: воронка / ArUco / магнитный разъём?
