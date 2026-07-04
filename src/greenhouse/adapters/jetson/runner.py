"""Боевой rate-loop: крутит JetsonRobot.tick с реальным временем (замена sim run()/engine).

Поднимает фоновые потоки (лидар, heartbeat привода), крутит цикл на заданной частоте,
гарантированно глушит привод на выходе. Команды режима пока подаются стартовым флагом
(--follow и т.п.); полноценный источник команд оператора (UDP/CLI) — точка расширения.

Запуск на Jetson:
    python -m greenhouse.adapters.jetson.runner --rate 10
"""

from __future__ import annotations

import argparse
import logging
import time

from greenhouse.adapters.jetson.lidar import JetsonLidar
from greenhouse.adapters.jetson.loop import JetsonConfig, JetsonRobot, build_jetson_robot
from greenhouse.adapters.jetson.motion import JetsonMotion
from greenhouse.adapters.operator import (
    CommandSource,
    MultiCommandSource,
    StdinCommandSource,
    UdpCommandSource,
)
from greenhouse.domain.geometry import Point2D
from greenhouse.domain.grid import MapMeta, OccupancyGrid
from greenhouse.navigation.mapping import EvidenceGridMapper, FileMapStore
from greenhouse.orchestration.interfaces import FollowPerson

log = logging.getLogger("rover")


def run_jetson(
    robot: JetsonRobot,
    *,
    rate_hz: float = 10.0,
    max_ticks: int | None = None,
    command_source: CommandSource | None = None,
) -> None:
    """Крутить цикл управления на частоте rate_hz. Привод глушится в finally."""
    dt = 1.0 / rate_hz
    if isinstance(robot.lidar, JetsonLidar):
        robot.lidar.start()
    if isinstance(robot.motion, JetsonMotion):
        robot.motion.start()  # heartbeat/deadman поток
    n = 0
    try:
        while max_ticks is None or n < max_ticks:
            if command_source is not None and robot.orchestrator is not None:
                for cmd in command_source.poll():
                    robot.orchestrator.handle(command=cmd)
                    log.info("команда оператора: %s", cmd.kind)
            status = robot.tick(dt_s=dt)
            if n % int(rate_hz) == 0:
                log.info("mode=%s pose=%s batt=%.2f%s", status.mode,
                         status.pose, status.battery_frac,
                         f" err={status.last_error}" if status.last_error else "")
            time.sleep(dt)
            n += 1
    except KeyboardInterrupt:
        log.info("остановка по Ctrl-C")
    finally:
        robot.motion.stop()
        if isinstance(robot.motion, JetsonMotion):
            robot.motion.close()
        log.info("привод остановлен, выход")


def _load_grid(store: FileMapStore | None, label: str) -> OccupancyGrid | None:
    if store is None:
        return None
    loaded = store.load(label=label)
    return loaded.grid if loaded is not None else None


def _map_meta(cfg: JetsonConfig) -> MapMeta:
    return MapMeta(
        resolution_m=cfg.map_resolution_m,
        width_px=int(cfg.map_width_m / cfg.map_resolution_m),
        height_px=int(cfg.map_height_m / cfg.map_resolution_m),
        origin=Point2D(x_m=cfg.map_origin_x_m, y_m=cfg.map_origin_y_m),
    )


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    p = argparse.ArgumentParser(description="Rover control loop (Jetson)")
    p.add_argument("--rate", type=float, default=10.0, help="частота цикла, Гц")
    p.add_argument("--motor-port", default="/dev/ttyUSB0", help="serial-порт ESP (USB, CH340)")
    p.add_argument("--esp32", default="192.168.1.50", help="IP ESP для transport=udp")
    p.add_argument("--lidar-port", default="/dev/ttyUSB0")
    p.add_argument("--forward-bin-deg", type=float, default=0.0, help="CALIBRATE: бин лидара по курсу")
    p.add_argument("--ccw", action="store_true", help="лидар вращается против часовой")
    p.add_argument("--follow", action="store_true", help="стартовать в режиме следования")
    p.add_argument("--cmd-port", type=int, default=4220, help="UDP-порт команд оператора")
    p.add_argument("--map-dir", default=None, help="каталог карт (FileMapStore); без него карта не грузится")
    p.add_argument("--map-label", default="map")
    args = p.parse_args()

    cfg = JetsonConfig(
        esp32_serial_port=args.motor_port, esp32_ip=args.esp32, lidar_port=args.lidar_port,
        lidar_forward_bin_deg=args.forward_bin_deg, lidar_clockwise=not args.ccw,
    )
    # Mapper для MAPPING с нуля; если задан --map-dir и есть сохранённая карта — грузим её,
    # тогда доступны ездовые режимы (Navigating/Following/Charging).
    store = FileMapStore(directory=args.map_dir) if args.map_dir else None
    robot = build_jetson_robot(
        config=cfg, grid=_load_grid(store, args.map_label),
        mapper=EvidenceGridMapper(meta=_map_meta(cfg)), map_store=store, map_label=args.map_label,
    )
    if args.follow and robot.orchestrator is not None:
        robot.orchestrator.handle(command=FollowPerson())

    # Команды оператора: UDP (с телефона/ноута по сети) + stdin (по SSH).
    sources: list[CommandSource] = [UdpCommandSource(port=args.cmd_port), StdinCommandSource()]
    run_jetson(robot, rate_hz=args.rate, command_source=MultiCommandSource(sources))


if __name__ == "__main__":
    main()
