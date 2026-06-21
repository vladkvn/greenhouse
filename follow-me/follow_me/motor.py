"""Motor control over USB-SERIAL to the ESP32 (был UDP). Протокол: 'L R\\n' / 'STOP\\n'.

ESP подключён к Jetson по USB (CH340, /dev/ttyUSB0). Прошивка serial читает строки
"L R\\n"/"STOP\\n" @115200, failsafe 0.5с. compute_drive не менялся — только транспорт.

IMU (BNO085) к ESP НЕ относится — он подключён напрямую к Jetson по I2C, см. imu.py.
ESP теперь чисто контроллер моторов.
"""

from __future__ import annotations

import serial

from .config import FollowMeConfig
from .fusion import PersonTrack


class Esp32Motor:
    """USB-serial линк к ESP32. Пишет 'L R\\n'/'STOP\\n', 115200."""

    def __init__(self, ip=None, port=None, *, serial_port: str = "/dev/ttyUSB0",
                 baud: int = 115200):
        # ip/port оставлены для совместимости со старым UDP-вызовом Esp32Motor(ip, port) — игнор.
        self._s = serial.Serial()
        self._s.port = serial_port
        self._s.baudrate = baud
        self._s.timeout = 0.2
        self._s.dtr = False   # НЕ сбрасывать ESP при открытии порта
        self._s.rts = False
        self._s.open()

    def drive(self, left: int, right: int) -> None:
        self._s.write(f"{int(left)} {int(right)}\n".encode())

    def stop(self) -> None:
        self._s.write(b"STOP\n")

    def close(self) -> None:
        try:
            self._s.close()
        except OSError:
            pass


def _deadzone(speed: float, cfg: FollowMeConfig) -> int:
    """Pull a small non-zero signal up to min_move so the motor actually turns."""
    s = int(round(speed))
    if s == 0:
        return 0
    sign = 1 if s > 0 else -1
    return sign * max(cfg.motor_min_move, min(cfg.motor_max_speed, abs(s)))


def compute_drive(target: PersonTrack | None, cfg: FollowMeConfig):
    """Map the selected target to (left, right, status_text).

    cam_angle_deg: negative = person to the LEFT, positive = RIGHT.
    For skid-steer a right turn means left side faster than right side.
    """
    if target is None:
        return 0, 0, "no target -> stop"

    dist = target.distance_m
    if dist is not None and dist <= cfg.stop_distance_m:
        return 0, 0, f"reached {dist:.2f} m -> stop"

    # steering: normalise bearing to [-1..1] over half the camera FOV
    half_fov = max(1.0, cfg.camera_hfov_deg / 2.0)
    err = max(-1.0, min(1.0, target.cam_angle_deg / half_fov))
    turn = cfg.motor_turn_gain * err * cfg.motor_base_speed

    # forward speed: full cruise, easing off as we close on stop_distance
    speed = cfg.motor_base_speed
    if dist is None:
        # person seen but no LiDAR range -> approach cautiously
        speed = int(cfg.motor_base_speed * 0.6)
    elif dist < cfg.slow_distance_m:
        k = (dist - cfg.stop_distance_m) / max(0.01, cfg.slow_distance_m - cfg.stop_distance_m)
        speed = int(cfg.motor_base_speed * max(0.4, min(1.0, k)))

    left = _deadzone(speed + turn, cfg)
    right = _deadzone(speed - turn, cfg)
    d_txt = "?" if dist is None else f"{dist:.2f}m"
    return left, right, f"follow d={d_txt} ang={target.cam_angle_deg:+.1f} L={left} R={right}"
