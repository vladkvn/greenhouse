# Список покупок на Botland (прототип GreenHouse)

Список под **DIY-прототип**. Учтены правки: контактный бампер заменён на **ультразвуковые
(эхо) датчики**, лидар — **дешёвый 2D**, камера — **обычная, но достаточная**.
**Моторы и корпус уже есть** — драйвер моторов и шасси в список не входят.

Цены/наличие на Botland меняются — проверяй на странице товара. Эхо-датчики выдают сигнал
5 В, а GPIO Jetson — 3.3 В, поэтому в списке есть **конвертер уровней** и **стабилизатор 5 В**.

## Основное

| Что | Зачем (интерфейс ядра) | Кол-во | Ссылка |
|---|---|---|---|
| **Jetson Orin Nano Super Dev Kit, 8GB** | Мозг: ядро + зрение на борту | 1 | [botland.store](https://botland.store/nvidia-modules/22877-nvidia-jetson-orin-nano-super-developer-kit-arm-cortex-a78ae-6x-15ghz-nvidia-ampere-8gb-ram-812674025261.html) |
| **RPLIDAR A1M8-R6 (360°, 12 м)** | 2D-лидар → карта, локализация, объезд (`LidarSource`) | 1 | [botland.store](https://botland.store/laser-scanners-lidar/19625-rplidar-a1m8-r6-360-degree-laser-scanner-kit-12m-range-seeedstudio-114992561-5904422369248.html) |
| **ArduCam IMX708 USB UVC, 12 Mpx** | Камера для следования за человеком (`CameraSource`/`TargetDetector`); USB plug-and-play на Jetson | 1 | [botland.store](https://botland.store/raspberry-pi-cameras/23577-12mpx-imx708-usb-uvc-fixed-focus-camera-module-3-for-raspberry-pi-arducam-b0304-5904422384982.html) |
| **JSN-SR04T (ультразвук, влагозащищённый, 0.2–4.5 м)** | Эхо-датчики вместо бампера — ближний обзор/защита (вход в `LocalPlanner`) | 3–4 | [botland.store](https://botland.store/ultrasonic-distance-sensors/7266-waterproof-ultrasonic-distance-sensor-jsn-sr04t-20-450cm-5904422310066.html) |
| **IMU BNO085 9-DOF (Adafruit)** | Курс/ускорения для устойчивой одометрии (`ImuSource`) | 1 | [botland.store](https://botland.store/9dof-imu-sensors/22113-bno085-9-dof-imu-fusion-breakout-3-axis-accelerometer-magnetometer-and-gyroscope-adafruit-4754.html) |

## Обвязка (для подключения эхо-датчиков)

| Что | Зачем | Кол-во | Ссылка |
|---|---|---|---|
| **Конвертер уровней 3.3↔5 В, 8-канальный** | Сигналы JSN-SR04T (5 В) → GPIO Jetson (3.3 В) | 1 | [botland.store](https://botland.store/voltage-converters/8590-8-channel-bi-directional-logic-level-converter-5904422336660.html) |
| **Step-down 5 В 3.2 А (Pololu D36V28F5)** | Стабильные 5 В для лидара/эхо-датчиков от бортовой шины | 1 | [botland.store](https://botland.store/converters-step-down/17169-step-down-voltage-converter-d36v28f5-5v-32a-pololu-3782-5903351242837.html) |

## Опционально / докупить отдельно

- **Корпус для Jetson Orin с креплением камеры** — защита платы и место под камеру: [botland.store](https://botland.store/nvidia-cases/23879-type-a-housing-for-nvidia-jetson-orin-with-camera-mount-waveshare-25334.html). В тёплой влажной теплице корпус желателен.
- **NVMe SSD (M.2)** — Jetson грузится надёжнее с SSD, чем с microSD (искать в каталоге Botland «SSD M.2»).
- **Аварийная кнопка (E-stop, грибок, NC)** — на Botland явного промышленного варианта не нашёл (есть только узкая модель для 3D-принтера: [пример](https://botland.store/accessories-for-cnc/19797-emergency-button-for-3d-printer-snapmaker-20-5904422347628.html)). Лучше взять обычную промышленную NC-кнопку у поставщика электрики; она должна **аппаратно** рвать силовую линию приводов, не через софт.
- **USB-хаб** — если не хватит портов под лидар + камеру одновременно (у Dev Kit портов обычно достаточно).

## Что НЕ входит (уже есть или вне Botland)

- Моторы и шасси — у тебя уже есть.
- Драйвер моторов — если его ещё нет, нужен контроллер с энкодерным контуром (`MotionController`); подбирается под твои моторы (например RoboClaw/ODrive), но это отдельно.
- Батарея/BMS — зависит от напряжения твоих моторов; подбирается под существующий привод.

## Где Botland завышает и чем заменить

Сравнение цен Botland с производителями/маркетплейсами (цены **ориентировочные**, EUR;
часть собрана из сниппетов и агрегаторов, т.к. Botland отдаёт цену через JS — перепроверь
на странице товара). Курс ~1 € = 4.30 zł.

| Товар | Botland (€) | Дешевле где | Альтернатива | Переплата |
|---|---|---|---|---|
| JSN-SR04T (ультразвук) | ~13.7 | AliExpress ~3 € | [aliexpress](https://www.aliexpress.com/item/32863960886.html) | **высокая (×3–4)** |
| Конвертер уровней 8-кан. (TXS0108E) | ~8–12 | AliExpress ~1–2 € | [aliexpress](https://www.aliexpress.com/item/32994869948.html) | **высокая (в разы)** |
| ArduCam IMX708 USB | ~60–69 | ArduCam офиц. ~46 € | [arducam.com](https://www.arducam.com/12mp-imx708-usb-uvc-fixed-focus-camera-module-3.html) | средняя (~30%) |
| Adafruit BNO085 IMU | ~31 | Adafruit $24.95 / Kiwi ~22–27 € | [adafruit](https://www.adafruit.com/product/4754), [kiwi-electronics](https://www.kiwi-electronics.com/en/adafruit-9-dof-orientation-imu-fusion-breakout-bno085-bno080-stemma-qt-qwiic-11273) | средняя (~15–30%) |
| RPLIDAR A1M8-R6 | ~102 | DFRobot $99 / AliExpress ~$90–99 | [dfrobot](https://www.dfrobot.com/product-1125.html) | низкая |
| Pololu D36V28F5 5V | 18.5 | Opencircuit ~17.4 € | [opencircuit](https://opencircuit.shop/product/5v-3.2a-step-down-voltage-regulator-d36v28f5) | низкая |

**Главная переплата — на «мелочёвке»:**

- **JSN-SR04T и конвертер уровней** — commodity-модули, на AliExpress дешевле в разы; для DIY качество достаточное, бери с запасом (часто продают пачками). Это основной кандидат на заказ из Китая.
- **ArduCam и BNO085** — брендовые: дешевле у самого производителя (ArduCam, Adafruit) или у EU-дистрибьютора без таможни ([Kiwi Electronics](https://www.kiwi-electronics.com)). Экономия ~15–30%.

**Брать на Botland ради удобства/гарантии/быстрой доставки по ЕС:** RPLIDAR A1 и Pololu
D36V28F5 — разница с мировой ценой мала, а доставка из США/Китая её съедает.

> ⚠️ **Jetson Orin Nano Super:** по данным проверки страница Botland могла уйти в
> «снятые с продажи». Проверь наличие; если нет — бери у официальных партнёров
> (Seeed Studio / NVIDIA-партнёры) около MSRP $249.

---

## Привязка к коду

Каждая позиция реализует интерфейс, у которого уже есть sim-двойник: лидар → `SimLidar`
(`LidarSource`), камера → `SimPersonDetector` (`TargetDetector`), IMU → `ImuSource`,
эхо-датчики питают тот же `LocalPlanner`, что и в симуляции. Логику не переписываем —
пишем драйвер под существующий `Protocol`. См. [HARDWARE.md](HARDWARE.md) и [ARCHITECTURE.md](ARCHITECTURE.md).

---
Sources: ссылки на товары Botland приведены в таблицах выше.
