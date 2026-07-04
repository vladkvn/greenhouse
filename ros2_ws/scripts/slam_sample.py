#!/usr/bin/env python3
"""Один сэмпл качества SLAM для мониторинга во время картирования.

Печатает одной строкой: размер сетки /map, число ячеек-стен и свободных, и трансформации
map->odom (накопленная поправка loop closure) и odom->base_footprint (сырое движение по rf2o).
Рост стен/свободы = карта строится; скачки map->odom = сработал loop closure; аккуратный
рост odom->base = одометрия живая.
"""
import collections
import time

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from tf2_ros import Buffer, TransformListener

rclpy.init()
node = rclpy.create_node("slam_sample")

qos = QoSProfile(depth=1)
qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL   # /map латчится
qos.reliability = QoSReliabilityPolicy.RELIABLE
got: dict = {}


def _cb(m: OccupancyGrid) -> None:
    c = collections.Counter(m.data)
    got["w"] = m.info.width
    got["h"] = m.info.height
    got["walls"] = c[100]
    got["free"] = c[0]


node.create_subscription(OccupancyGrid, "/map", _cb, qos)
buf = Buffer()
TransformListener(buf, node)

t0 = time.time()
while rclpy.ok() and "w" not in got and time.time() - t0 < 5.0:
    rclpy.spin_once(node, timeout_sec=0.3)


def _tf(a: str, b: str) -> str:
    for _ in range(12):
        rclpy.spin_once(node, timeout_sec=0.2)
        try:
            tr = buf.lookup_transform(a, b, rclpy.time.Time()).transform.translation
            return f"({tr.x:+.2f},{tr.y:+.2f})"
        except Exception:  # noqa: BLE001
            continue
    return "n/a"


mo = _tf("map", "odom")
ob = _tf("odom", "base_footprint")
print(
    f"grid={got.get('w', '?')}x{got.get('h', '?')} "
    f"walls={got.get('walls', '?')} free={got.get('free', '?')} "
    f"map->odom={mo} odom->base={ob}"
)
node.destroy_node()
rclpy.shutdown()
