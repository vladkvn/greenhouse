"""Тесты адаптеров Jetson (инкремент 0) — чистая логика, без железа.

Проверяют seam привода и лидара через внедрённые заглушки: микшер скид-стира
(Twist2D → L/R), конвенцию углов лидара (луч 0 по курсу, направление вращения,
монтажный сдвиг, маскирование, дальность) и failsafe-deadman привода.
"""

from __future__ import annotations

import math

from greenhouse.adapters.jetson.imu import ZeroImu
from greenhouse.adapters.jetson.lidar import JetsonLidar
from greenhouse.adapters.jetson.motion import Esp32SerialLink, JetsonMotion, twist_to_lr
from greenhouse.adapters.jetson.odometry import DeadReckonOdometry
from greenhouse.adapters.sim.clock import SimClock
from greenhouse.control.interfaces import MotionLimits
from greenhouse.domain.geometry import Twist2D

_LIMITS = MotionLimits(max_linear_m_s=0.4, max_angular_rad_s=1.5, max_linear_accel_m_s2=1.0)


# ----------------------------------------------------------------- twist_to_lr
def _lr(twist: Twist2D) -> tuple[int, int]:
    return twist_to_lr(twist, limits=_LIMITS, wheel_base_m=0.18, pwm_max=220, pwm_min_move=120)


def test_forward_drives_both_sides_equally() -> None:
    left, right = _lr(Twist2D(linear_x_m_s=0.4, angular_z_rad_s=0.0))
    assert left == right == 220  # полный вперёд = потолок ШИМ на оба борта


def test_deadzone_pulls_small_signal_up() -> None:
    left, right = _lr(Twist2D(linear_x_m_s=0.1, angular_z_rad_s=0.0))
    assert left == right == 120  # ниже min_move мотор не крутится -> подтянули


def test_turn_in_place_is_antisymmetric_ccw() -> None:
    left, right = _lr(Twist2D(linear_x_m_s=0.0, angular_z_rad_s=1.0))
    # angular_z > 0 = поворот влево (CCW): правый борт вперёд, левый назад.
    assert right > 0 and left < 0 and left == -right


def test_pwm_is_clamped_to_max() -> None:
    left, right = _lr(Twist2D(linear_x_m_s=100.0, angular_z_rad_s=0.0))
    assert left == right == 220


# -------------------------------------------------------------- JetsonLidar углы
class _FakeScan:
    """Источник сырого оборота: 360 бинов (метры, NaN = нет возврата)."""

    def __init__(self, hits: dict[int, float]) -> None:
        self._bins = [hits.get(i, math.nan) for i in range(360)]

    def get_scan(self) -> list[float]:
        return self._bins


def _scan(hits, **kw) -> tuple[float, ...]:
    lidar = JetsonLidar(clock=SimClock(), scan_source=_FakeScan(hits), **kw)
    return lidar.read_scan().ranges_m


def test_lidar_beam_zero_is_forward() -> None:
    ranges = _scan({0: 1.0}, forward_bin_deg=0.0, clockwise=True)
    assert len(ranges) == 360
    assert math.isclose(ranges[0], 1.0)  # сырой бин 0 -> луч 0 (по курсу)


def test_lidar_clockwise_sign_maps_bin_to_beam() -> None:
    # clockwise: луч робота i соответствует сырому бину (-i) % 360. Бин 90 -> луч 270.
    ranges = _scan({90: 2.5}, forward_bin_deg=0.0, clockwise=True)
    assert math.isclose(ranges[270], 2.5)
    assert math.isnan(ranges[90])


def test_lidar_mount_offset_shifts_forward() -> None:
    # forward_bin_deg=45: сырой бин 45 теперь смотрит по курсу -> луч 0.
    ranges = _scan({45: 3.0}, forward_bin_deg=45.0, clockwise=True)
    assert math.isclose(ranges[0], 3.0)


def test_lidar_ccw_passthrough() -> None:
    ranges = _scan({30: 2.0}, forward_bin_deg=0.0, clockwise=False)
    assert math.isclose(ranges[30], 2.0)  # CCW: луч i = сырой бин i


def test_lidar_masks_self_hit_sector() -> None:
    ranges = _scan({i: 5.0 for i in range(360)}, masked_sectors=[(10, 20)], clockwise=False)
    assert math.isnan(ranges[15])  # сектор корпуса -> NaN
    assert math.isclose(ranges[5], 5.0)


def test_lidar_out_of_range_and_zero_are_no_return() -> None:
    ranges = _scan({1: 20.0, 2: 0.0, 3: 4.0}, max_range_m=12.0, clockwise=False)
    assert math.isnan(ranges[1])  # дальше max_range
    assert math.isnan(ranges[2])  # нулевая дистанция
    assert math.isclose(ranges[3], 4.0)


# ------------------------------------------------------------- failsafe deadman
class _FakeLink:
    def __init__(self) -> None:
        self.drives: list[tuple[int, int]] = []
        self.stops = 0
        self.closed = False

    def drive(self, *, left: int, right: int) -> None:
        self.drives.append((left, right))

    def stop(self) -> None:
        self.stops += 1

    def close(self) -> None:
        self.closed = True


def test_command_sends_lr_and_stop_sends_stop() -> None:
    link = _FakeLink()
    motion = JetsonMotion(link=link, clock=SimClock(), limits=_LIMITS)
    motion.command(twist=Twist2D(linear_x_m_s=0.4, angular_z_rad_s=0.0))
    assert link.drives[-1] == (220, 220)
    motion.stop()
    assert link.stops == 1


def test_deadman_stops_after_silence() -> None:
    clock = SimClock()
    link = _FakeLink()
    motion = JetsonMotion(link=link, clock=clock, limits=_LIMITS, max_silence_s=0.3)
    motion.command(twist=Twist2D(linear_x_m_s=0.4, angular_z_rad_s=0.0))

    clock.advance(dt_s=0.1)  # ещё в пределах тишины -> heartbeat пере-отправляет команду
    motion.heartbeat_tick()
    assert link.drives[-1] == (220, 220)
    assert link.stops == 0

    clock.advance(dt_s=0.4)  # суммарно 0.5с > 0.3с -> deadman глушит привод
    motion.heartbeat_tick()
    assert link.stops == 1


# ------------------------------------------------------- одометрия без энкодеров
def test_dead_reckon_integrates_forward_command() -> None:
    clock = SimClock()
    odom = DeadReckonOdometry(clock=clock)
    odom.set_command(twist=Twist2D(linear_x_m_s=0.5, angular_z_rad_s=0.0))
    clock.advance(dt_s=2.0)  # 0.5 м/с * 2 с = 1.0 м по +X (курс 0)
    o = odom.read_odometry()
    assert math.isclose(o.pose.x_m, 1.0, abs_tol=1e-9)
    assert math.isclose(o.pose.y_m, 0.0, abs_tol=1e-9)
    assert math.isclose(o.pose.theta_rad, 0.0, abs_tol=1e-9)


def test_dead_reckon_integrates_rotation() -> None:
    clock = SimClock()
    odom = DeadReckonOdometry(clock=clock)
    odom.set_command(twist=Twist2D(linear_x_m_s=0.0, angular_z_rad_s=1.0))
    clock.advance(dt_s=1.5)  # 1 рад/с * 1.5 с = 1.5 рад
    o = odom.read_odometry()
    assert math.isclose(o.pose.theta_rad, 1.5, abs_tol=1e-9)
    assert o.velocity.angular_z_rad_s == 1.0  # velocity = последняя команда


def test_dead_reckon_accumulates_across_commands() -> None:
    clock = SimClock()
    odom = DeadReckonOdometry(clock=clock)
    odom.set_command(twist=Twist2D(linear_x_m_s=1.0, angular_z_rad_s=0.0))
    clock.advance(dt_s=1.0)
    odom.set_command(twist=Twist2D(linear_x_m_s=0.0, angular_z_rad_s=0.0))  # домотает первый метр
    o = odom.read_odometry()
    assert math.isclose(o.pose.x_m, 1.0, abs_tol=1e-9)


def test_zero_imu_reads_zeros() -> None:
    sample = ZeroImu(clock=SimClock()).read_imu()
    assert sample.yaw_rate_rad_s == 0.0
    assert sample.linear_accel_x_m_s2 == 0.0


# --------------------------------------------------------- serial-связь с ESP
class _FakePort:
    def __init__(self) -> None:
        self.written: list[bytes] = []

    def write(self, data: bytes) -> int:
        self.written.append(data)
        return len(data)

    def close(self) -> None: ...


def test_serial_link_formats_line_protocol() -> None:
    port = _FakePort()
    link = Esp32SerialLink(port=port)
    link.drive(left=180, right=-140)
    link.stop()
    assert port.written == [b"180 -140\n", b"STOP\n"]  # строки с '\n', как ждёт прошивка

