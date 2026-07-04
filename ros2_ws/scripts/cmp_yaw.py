#!/usr/bin/env python3
"""Сравнение знака поворота: rf2o (лазерная одометрия) vs IMU (BNO085).

Крутите робота ~90° в ОДНУ сторону. Если rf2o и IMU меняют yaw в РАЗНЫЕ стороны — скан
лидара зеркальный (нужен invert) или у IMU не тот знак; они дерутся в EKF → смаз карты.
Печатает raw yaw и delta от первого замера каждую секунду.
"""
import math
import time

import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

rclpy.init()
node = rclpy.create_node("cmp_yaw")
cur = {"rf2o": None, "imu": None}
start = {"rf2o": None, "imu": None}


def _yaw(q) -> float:
    return math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))


def _rcb(m: Odometry) -> None:
    cur["rf2o"] = _yaw(m.pose.pose.orientation)


def _icb(m: Imu) -> None:
    cur["imu"] = _yaw(m.orientation)


node.create_subscription(Odometry, "/odom_rf2o", _rcb, 10)
node.create_subscription(Imu, "/imu/data", _icb, 20)


def _d(k):
    if cur[k] is None or start[k] is None:
        return None
    return (cur[k] - start[k] + 180) % 360 - 180   # wrapped delta


t0 = time.time()
last = 0.0
while rclpy.ok() and time.time() - t0 < 75:
    rclpy.spin_once(node, timeout_sec=0.2)
    for k in ("rf2o", "imu"):
        if start[k] is None and cur[k] is not None:
            start[k] = cur[k]
    if time.time() - last > 2.0:
        last = time.time()
        dr, di = _d("rf2o"), _d("imu")
        if dr is not None and di is not None:
            print(f"rf2o Δ{dr:+.0f}°   imu Δ{di:+.0f}°", flush=True)
        else:
            print("ждём данные rf2o/imu...", flush=True)

dr, di = _d("rf2o"), _d("imu")
if dr is not None and di is not None:
    same = (dr >= 0) == (di >= 0)
    print(f"ИТОГ: rf2o Δ={dr:+.0f}°  imu Δ={di:+.0f}°  — знаки "
          f"{'СОВПАДАЮТ (ok)' if same else 'ПРОТИВОПОЛОЖНЫ (баг конвенции)'}", flush=True)
