"""DeadReckonOdometry — псевдо-одометрия БЕЗ энкодеров (интеграл поданной команды).

На тележке нет колёсных энкодеров, а `Localizer.update(*, scan, odometry)` требует
одометрию. Этот источник интегрирует ПОСЛЕДНЮЮ поданную в привод `Twist2D` по времени и
отдаёт накопленную позу. Это лишь ГРУБЫЙ prior смещения между тиками: реальная скорость
скид-стира без энкодеров != командной (проскальзывание, разный заряд). Реальную позицию
защёлкивает scan-matching, курс перебивает IMU — здесь только начальное приближение.

Кинематика совпадает с SimEngine.step (diff-drive: сначала поворот, затем сдвиг по новому
курсу), чтобы поведение переносилось из симуляции предсказуемо.

Контракт: `greenhouse.sensing.OdometrySource`.
"""

from __future__ import annotations

import math

from greenhouse.domain.geometry import Pose2D, Twist2D
from greenhouse.runtime.clock import Clock
from greenhouse.sensing.interfaces import Odometry


class DeadReckonOdometry:
    """Реализует `greenhouse.sensing.OdometrySource` интегрированием поданных команд."""

    def __init__(self, *, clock: Clock) -> None:
        self._clock = clock
        self._pose = Pose2D(x_m=0.0, y_m=0.0, theta_rad=0.0)
        self._twist = Twist2D.stop()
        self._last_t = clock.now_s()

    def set_command(self, *, twist: Twist2D) -> None:
        """Сообщить о новой команде привода: домотать прошлую по времени, затем сменить её."""
        self._integrate()
        self._twist = twist

    def read_odometry(self) -> Odometry:
        self._integrate()
        return Odometry(pose=self._pose, velocity=self._twist, stamp_s=self._clock.now_s())

    def _integrate(self) -> None:
        now = self._clock.now_s()
        dt = now - self._last_t
        self._last_t = now
        if dt <= 0.0:
            return
        v = self._twist.linear_x_m_s
        w = self._twist.angular_z_rad_s
        theta = self._pose.theta_rad + w * dt
        self._pose = Pose2D(
            x_m=self._pose.x_m + v * math.cos(theta) * dt,
            y_m=self._pose.y_m + v * math.sin(theta) * dt,
            theta_rad=theta,
        )
