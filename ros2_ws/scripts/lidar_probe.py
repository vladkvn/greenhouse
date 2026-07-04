#!/usr/bin/env python3
"""Проба лидара боевой библиотекой rplidar (как в follow_me): info→health→2 скана.

Запуск ТОЙ ЖЕ библиотекой/питоном, что в follow_me:
  ~/greenhouse/.venv/bin/python lidar_probe.py /dev/ttyUSB1
Если тут OK, а ROS-узел падает — дело в узле; если тут FAIL — устройство залипло (power-cycle).
"""
import sys

from rplidar import RPLidar

port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB1"
lidar = RPLidar(port, baudrate=115200)
try:
    print("INFO", lidar.get_info())
    print("HEALTH", lidar.get_health())
    n = 0
    for scan in lidar.iter_scans():
        print("SCAN points:", len(scan))
        n += 1
        if n >= 2:
            break
    print("RESULT OK — лидар работает")
except Exception as exc:  # noqa: BLE001
    print("RESULT FAIL", repr(exc))
finally:
    try:
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()
    except Exception:  # noqa: BLE001
        pass
