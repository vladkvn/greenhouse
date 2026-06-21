"""Курс с BNO085, подключённого НАПРЯМУЮ к Jetson по I2C (Blinka + Adafruit BNO08x).

ESP32 не тянет протокол SH2 поверх своего I2C (clock stretching) -- датчик переехал на
40-пиновый гребень Jetson, где Linux-I2C держит stretching штатно. ESP остался только на
моторах. Читаем в фоновом потоке, отдаём последний yaw (0..360, по часовой).

Используем GAME rotation vector (гиро+акселерометр, БЕЗ магнитометра): рядом с моторами
магнитное поле искажает компас, а GAME-режим к этому иммунен и даёт чистый отн. курс,
которого SLAM достаточно.

Если библиотека/датчик недоступны -- поток молча выключается, yaw=None (follow_me работает
без курса). Зависимости на Jetson: adafruit-blinka, adafruit-circuitpython-bno08x,
adafruit-extended-bus. Шина: /dev/i2c-7 (40-пиновый гребень, пины 3/5), адрес 0x4A.
"""

from __future__ import annotations

import logging
import math
import threading
import time

log = logging.getLogger("follow_me.imu")


class ImuReader(threading.Thread):
    """Фоновый читатель BNO085 по I2C. get_yaw() -> (yaw_deg|None, monotonic_ts)."""

    def __init__(self, bus: int = 7, address: int = 0x4A):
        super().__init__(daemon=True)
        self._bus = bus
        self._addr = address
        self._yaw: float | None = None
        self._yaw_ts = 0.0
        self._stop = threading.Event()

    def get_yaw(self) -> tuple[float | None, float]:
        return self._yaw, self._yaw_ts

    def stop(self) -> None:
        self._stop.set()

    def run(self) -> None:
        try:
            from adafruit_bno08x import BNO_REPORT_GAME_ROTATION_VECTOR
            from adafruit_bno08x.i2c import BNO08X_I2C
            from adafruit_extended_bus import ExtendedI2C as I2C
        except Exception as exc:  # noqa: BLE001 - библиотека опциональна
            log.warning("IMU библиотека недоступна: %s -- курс выключен", exc)
            return

        while not self._stop.is_set():
            try:
                i2c = I2C(self._bus)  # /dev/i2c-7 (гребень, пины 3/5)
                bno = BNO08X_I2C(i2c, address=self._addr)
                bno.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR)
                log.info("BNO085 подключён по I2C (0x%02X), курс пошёл", self._addr)
                while not self._stop.is_set():
                    qi, qj, qk, qr = bno.game_quaternion  # (i, j, k, real)
                    yaw = math.atan2(2.0 * (qr * qk + qi * qj),
                                     1.0 - 2.0 * (qj * qj + qk * qk))
                    self._yaw = math.degrees(yaw) % 360.0
                    self._yaw_ts = time.monotonic()
                    time.sleep(0.02)  # ~50 Гц
            except Exception as exc:  # noqa: BLE001 - переподключение
                self._yaw = None
                log.warning("IMU ошибка: %s -- переподключение через 2с", exc)
                if self._stop.wait(2.0):
                    break
