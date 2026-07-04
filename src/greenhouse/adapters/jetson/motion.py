"""JetsonMotion — реализация MotionController поверх ESP32-привода (скид-стир по UDP).

Принимает желаемую скорость `Twist2D`, обрезает её по `MotionLimits` (как SimMotion),
конвертирует (v, w) в скорости двух бортов (left/right) и шлёт их на ESP32 текстом
"L R" / "STOP" (тот же протокол, что у боевого follow_me, порт 4210).

Два уровня failsafe сверх аппаратного watchdog ESP32 (0.5 с):
  * heartbeat — `DriveWatchdog` каждые ~100 мс пере-отправляет последнюю команду, чтобы
    при нормальной езде грубый ESP32-failsafe не дёргал моторы;
  * deadman — если главный цикл молчит дольше `max_silence_s`, привод сам уходит в стоп
    ДО срабатывания ESP32-watchdog (защита от зависшего процесса Jetson).

Контракт: `greenhouse.control.MotionController`.
"""

from __future__ import annotations

import socket
import threading
import time
from typing import Protocol

from greenhouse.control.interfaces import MotionLimits
from greenhouse.domain.geometry import Twist2D
from greenhouse.runtime.clock import Clock


class MotorLink(Protocol):
    """Низкоуровневая связь с приводом. Реальная — UDP к ESP32; в тестах — заглушка."""

    def drive(self, *, left: int, right: int) -> None: ...

    def stop(self) -> None: ...

    def close(self) -> None: ...


class CommandSink(Protocol):
    """Наблюдатель поданных команд (например, DeadReckonOdometry интегрирует их в позу)."""

    def set_command(self, *, twist: Twist2D) -> None: ...


class Esp32UdpLink:
    """UDP-канал к модулю ESP32. Протокол: 'L R' (-255..255) / 'STOP'."""

    def __init__(self, *, ip: str, port: int) -> None:
        self._addr = (ip, port)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def drive(self, *, left: int, right: int) -> None:
        self._sock.sendto(f"{int(left)} {int(right)}".encode(), self._addr)

    def stop(self) -> None:
        self._sock.sendto(b"STOP", self._addr)

    def close(self) -> None:
        try:
            self._sock.close()
        except OSError:
            pass


class SerialPort(Protocol):
    """Минимальный интерфейс последовательного порта (pyserial.Serial или заглушка в тестах)."""

    def write(self, data: bytes) -> int: ...

    def close(self) -> None: ...


class Esp32SerialLink:
    """USB-serial канал к ESP32. Протокол строками: 'L R\\n' / 'STOP\\n', 115200 бод.

    Открытие порта сбрасывает ESP (CH340 дёргает DTR/RTS) — даём время на загрузку и
    чистим входной буфер от загрузочного баннера. Для тестов можно внедрить свой `port`.
    """

    def __init__(
        self,
        *,
        port_path: str = "/dev/ttyUSB0",
        baud: int = 115200,
        port: SerialPort | None = None,
        settle_s: float = 2.0,
    ) -> None:
        self._port: SerialPort = port if port is not None else _open_serial(port_path, baud, settle_s)

    def drive(self, *, left: int, right: int) -> None:
        self._port.write(f"{int(left)} {int(right)}\n".encode())

    def stop(self) -> None:
        self._port.write(b"STOP\n")

    def close(self) -> None:
        try:
            self._port.close()
        except OSError:
            pass


def _open_serial(port_path: str, baud: int, settle_s: float) -> SerialPort:
    try:
        import serial  # type: ignore[import-untyped]
    except ImportError as exc:  # pragma: no cover - только вне Jetson
        raise ImportError("pyserial не установлен (pip install pyserial)") from exc
    ser = serial.Serial(port_path, baud, timeout=0.1)
    time.sleep(settle_s)      # ESP сбрасывается при открытии порта — ждём загрузку
    ser.reset_input_buffer()  # выкинуть загрузочный баннер
    return ser  # type: ignore[no-any-return]


def twist_to_lr(
    twist: Twist2D,
    *,
    limits: MotionLimits,
    wheel_base_m: float,
    pwm_max: int,
    pwm_min_move: int,
) -> tuple[int, int]:
    """Скид-стир: (v, w) → ШИМ левого/правого бортов.

    v_left = v - w·base/2,  v_right = v + w·base/2 (м/с) → масштаб в ШИМ по limits, с
    deadzone (ниже `pwm_min_move` мотор не крутится — подтягиваем до него). Конвенция
    `angular_z_rad_s` > 0 = поворот влево (CCW): правый борт быстрее левого.
    """
    half = wheel_base_m / 2.0
    v_left = twist.linear_x_m_s - twist.angular_z_rad_s * half
    v_right = twist.linear_x_m_s + twist.angular_z_rad_s * half
    k = pwm_max / limits.max_linear_m_s  # м/с → единицы ШИМ
    return _to_pwm(v_left * k, pwm_max, pwm_min_move), _to_pwm(v_right * k, pwm_max, pwm_min_move)


def _to_pwm(value: float, pwm_max: int, pwm_min_move: int) -> int:
    s = int(round(value))
    if s == 0:
        return 0
    sign = 1 if s > 0 else -1
    return sign * max(pwm_min_move, min(pwm_max, abs(s)))


def _clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


class JetsonMotion:
    """Реализует `greenhouse.control.MotionController` поверх ESP32-привода."""

    def __init__(
        self,
        *,
        link: MotorLink,
        clock: Clock,
        limits: MotionLimits | None = None,
        wheel_base_m: float = 0.18,
        pwm_max: int = 220,
        pwm_min_move: int = 120,
        max_silence_s: float = 0.3,
        command_sink: CommandSink | None = None,
    ) -> None:
        self._link = link
        self._clock = clock
        self._sink = command_sink  # уведомляем об актуальной (обрезанной) команде
        self._limits = limits or MotionLimits(
            max_linear_m_s=0.4, max_angular_rad_s=1.5, max_linear_accel_m_s2=1.0
        )
        self._wheel_base = wheel_base_m
        self._pwm_max = pwm_max
        self._pwm_min = pwm_min_move
        self._max_silence = max_silence_s
        self._last_lr: tuple[int, int] = (0, 0)
        self._last_beat_s = clock.now_s()
        self._watchdog = DriveWatchdog(motion=self)

    # ----------------------------------------------------------- MotionController
    @property
    def limits(self) -> MotionLimits:
        return self._limits

    def command(self, *, twist: Twist2D) -> None:
        v = _clamp(twist.linear_x_m_s, -self._limits.max_linear_m_s, self._limits.max_linear_m_s)
        w = _clamp(twist.angular_z_rad_s, -self._limits.max_angular_rad_s, self._limits.max_angular_rad_s)
        clamped = Twist2D(linear_x_m_s=v, angular_z_rad_s=w)
        left, right = twist_to_lr(
            clamped,
            limits=self._limits,
            wheel_base_m=self._wheel_base,
            pwm_max=self._pwm_max,
            pwm_min_move=self._pwm_min,
        )
        self._last_lr = (left, right)
        self._link.drive(left=left, right=right)
        if self._sink is not None:
            self._sink.set_command(twist=clamped)
        self._beat()

    def stop(self) -> None:
        self._last_lr = (0, 0)
        self._link.stop()
        if self._sink is not None:
            self._sink.set_command(twist=Twist2D.stop())
        self._beat()

    # ------------------------------------------------------------------ failsafe
    def heartbeat_tick(self) -> None:
        """Один тик watchdog-потока: deadman при молчании, иначе пере-отправка команды.

        НЕ обновляет метку beat — её освежают только команды из главного цикла, поэтому
        затянувшаяся тишина гарантированно приводит к стопу.
        """
        if self._clock.now_s() - self._last_beat_s > self._max_silence:
            if self._last_lr != (0, 0):
                self._last_lr = (0, 0)
                self._link.stop()
            return
        left, right = self._last_lr
        if (left, right) == (0, 0):
            self._link.stop()
        else:
            self._link.drive(left=left, right=right)

    def start(self) -> None:
        """Запустить heartbeat-поток (на реальном железе; в юнит-тестах не нужен)."""
        self._watchdog.start()

    def close(self) -> None:
        self._watchdog.stop()
        self.stop()
        self._link.close()

    def _beat(self) -> None:
        self._last_beat_s = self._clock.now_s()


class DriveWatchdog:
    """Фоновый поток heartbeat/deadman для JetsonMotion."""

    def __init__(self, *, motion: JetsonMotion, period_s: float = 0.1) -> None:
        self._motion = motion
        self._period = period_s
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        if self._thread is not None:
            return
        self._thread = threading.Thread(target=self._run, name="drive-watchdog", daemon=True)
        self._thread.start()

    def _run(self) -> None:
        while not self._stop.wait(self._period):
            self._motion.heartbeat_tick()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
