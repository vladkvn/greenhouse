#!/usr/bin/env python3
"""Направление (bearing) ближайшего препятствия в кадре base_link.

Поставьте объект ПРЯМО ПЕРЕД роботом (~0.5 м). Печать каждые 2с:
  ~0°   → перёд лидара верный;
  ~180° → лидар «задом наперёд» (нужен angle_offset_deg=180);
  ~+90° → скан повёрнут, объект уходит влево (offset≈-90/270);
  ~-90° → вправо (offset≈+90).
Учитывает invert и angle_offset_deg узла (тестируем итоговую геометрию) + TF laser→base_link.
"""
import math
import time

import rclpy
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

rclpy.init()
node = rclpy.create_node("scan_bearing")
buf = Buffer()
TransformListener(buf, node)
latest = {"scan": None}
node.create_subscription(LaserScan, "/scan", lambda m: latest.__setitem__("scan", m), 10)

t0 = time.time()
last = 0.0
while rclpy.ok() and time.time() - t0 < 45:
    rclpy.spin_once(node, timeout_sec=0.2)
    s = latest["scan"]
    if s is None or time.time() - last < 2.0:
        continue
    last = time.time()

    best_r, best_a = 1e9, None
    a = s.angle_min
    for r in s.ranges:
        if s.range_min < r < s.range_max and r < best_r:
            best_r, best_a = r, a
        a += s.angle_increment
    if best_a is None:
        print("нет возвратов в скане")
        continue

    lx, ly = best_r * math.cos(best_a), best_r * math.sin(best_a)
    try:
        tf = buf.lookup_transform("base_link", "laser", rclpy.time.Time())
        tx, ty = tf.transform.translation.x, tf.transform.translation.y
        q = tf.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        bx = tx + lx * math.cos(yaw) - ly * math.sin(yaw)
        by = ty + lx * math.sin(yaw) + ly * math.cos(yaw)
        bearing = math.degrees(math.atan2(by, bx))
        print(f"ближайший {best_r:.2f}м  bearing(base_link)={bearing:+.0f}°  "
              f"(0=перёд 180=зад +90=лево -90=право)")
    except Exception as exc:  # noqa: BLE001
        print(f"нет TF laser->base_link: {exc}")
