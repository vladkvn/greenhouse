"""Motor control: turn a follow-me target into ESP32 drive commands.

The ESP32 (separate WiFi module, static IP) drives 4 DC motors via one L298 in
skid-steer (left pair = channel A, right pair = channel B). It listens on UDP and
speaks a tiny text protocol:

    "L R"   left/right side speed, integers -255..255   (e.g. "180 140")
    "STOP"  immediate stop

The ESP32 has its own failsafe: no command for >0.5 s -> motors off. So we send a
command (drive or stop) every control tick to keep the robot alive only while we
intend it to move.

Control law (open-loop, no encoders):
    target is None .............. stop (nobody to follow)
    distance <= stop_distance ... stop (reached the person / obstacle)
    otherwise ................... drive forward, steering toward the person's bearing,
                                  slowing down as we approach stop_distance.
"""

from __future__ import annotations

import socket

from .config import FollowMeConfig
from .fusion import PersonTrack


class Esp32Motor:
    """UDP link to the ESP32 motor module. Protocol: 'L R' / 'STOP'."""

    def __init__(self, ip: str, port: int):
        self._addr = (ip, port)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def drive(self, left: int, right: int) -> None:
        self._sock.sendto(f"{int(left)} {int(right)}".encode(), self._addr)

    def stop(self) -> None:
        self._sock.sendto(b"STOP", self._addr)

    def close(self) -> None:
        try:
            self._sock.close()
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
