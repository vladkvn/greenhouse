"""Телеоп-каркас (инкремент 0): проверка seam адаптеров до всякой локализации/карты.

Запускает короткую БЕЗОПАСНУЮ последовательность через контракт `MotionController`
(вперёд → стоп → поворот → стоп), печатая дистанцию по курсу с `LidarSource`. Этим
проверяется, что:
  * `Twist2D` доходит до моторов через JetsonMotion ("L R" по USB-serial) и робот едет/крутится;
  * failsafe (heartbeat + deadman) держит привод;
  * RPLIDAR отдаёт `LidarScan` с лучом 0 по курсу (калибровка угла).

Запуск на Jetson:
    python -m greenhouse.adapters.jetson.teleop --motor-port /dev/ttyUSB1 --lidar-port /dev/ttyUSB0
Останов — Ctrl-C (привод гарантированно уходит в стоп в finally).
"""

from __future__ import annotations

import argparse
import math
import time

from greenhouse.adapters.jetson.clock import WallClock
from greenhouse.adapters.jetson.lidar import JetsonLidar
from greenhouse.adapters.jetson.motion import Esp32SerialLink, JetsonMotion
from greenhouse.domain.geometry import Twist2D


def _forward_range_m(scan_ranges: tuple[float, ...]) -> float:
    return scan_ranges[0] if scan_ranges else math.nan


def run_sequence(
    *,
    motion: JetsonMotion,
    lidar: JetsonLidar,
    linear_m_s: float,
    angular_rad_s: float,
    phase_s: float,
    rate_hz: float = 10.0,
) -> None:
    """Безопасная демо-последовательность: вперёд, стоп, поворот, стоп."""
    phases: list[tuple[str, Twist2D]] = [
        ("forward", Twist2D(linear_x_m_s=linear_m_s, angular_z_rad_s=0.0)),
        ("stop", Twist2D.stop()),
        ("turn-left", Twist2D(linear_x_m_s=0.0, angular_z_rad_s=angular_rad_s)),
        ("stop", Twist2D.stop()),
    ]
    dt = 1.0 / rate_hz
    for name, twist in phases:
        ticks = max(1, int(phase_s * rate_hz))
        for _ in range(ticks):
            motion.command(twist=twist)
            fwd = _forward_range_m(lidar.read_scan().ranges_m)
            fwd_txt = f"{fwd:.2f} m" if math.isfinite(fwd) else "--"
            print(f"[{name:9}] L/R sent | fwd={fwd_txt}", flush=True)
            time.sleep(dt)
    motion.stop()


def main() -> None:
    p = argparse.ArgumentParser(description="Jetson teleop seam check (increment 0)")
    p.add_argument("--motor-port", default="/dev/ttyUSB0", help="serial-порт ESP (USB, CH340)")
    p.add_argument("--lidar-port", default="/dev/ttyUSB0")
    p.add_argument("--lidar-baud", type=int, default=115200)
    p.add_argument("--linear", type=float, default=0.15, help="м/с (мало и безопасно)")
    p.add_argument("--angular", type=float, default=0.6, help="рад/с")
    p.add_argument("--phase", type=float, default=1.5, help="длительность фазы, с")
    p.add_argument("--forward-bin-deg", type=float, default=0.0, help="калибровка: бин по курсу")
    p.add_argument("--ccw", action="store_true", help="лидар крутится против часовой")
    args = p.parse_args()

    clock = WallClock()
    motion = JetsonMotion(link=Esp32SerialLink(port_path=args.motor_port), clock=clock)
    lidar = JetsonLidar(
        clock=clock,
        port=args.lidar_port,
        baud=args.lidar_baud,
        forward_bin_deg=args.forward_bin_deg,
        clockwise=not args.ccw,
    )
    lidar.start()
    motion.start()  # heartbeat/deadman поток
    try:
        run_sequence(
            motion=motion,
            lidar=lidar,
            linear_m_s=args.linear,
            angular_rad_s=args.angular,
            phase_s=args.phase,
        )
    finally:
        motion.close()  # гарантированный стоп привода
        print("teleop finished, motors stopped", flush=True)


if __name__ == "__main__":
    main()
