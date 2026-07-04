"""IMU-источник для Jetson. Пока физический IMU не подключён — заглушка ZeroImu.

`ImuFusedLocalizer` берёт курс из гироскопа (yaw_rate*dt) — это компенсатор отсутствия
энкодеров при скид-стире. До установки реального IMU (MPU6050/BNO055 по I2C) используем
ZeroImu (yaw_rate=0): тогда курс держит только scan-matching. Реальный драйвер JetsonImu
заведём на железе за тем же контрактом `greenhouse.sensing.ImuSource`.
"""

from __future__ import annotations

from greenhouse.runtime.clock import Clock
from greenhouse.sensing.interfaces import ImuSample


class ZeroImu:
    """Реализует `greenhouse.sensing.ImuSource` нулями (до установки реального IMU)."""

    def __init__(self, *, clock: Clock) -> None:
        self._clock = clock

    def read_imu(self) -> ImuSample:
        return ImuSample(yaw_rate_rad_s=0.0, linear_accel_x_m_s2=0.0, stamp_s=self._clock.now_s())
