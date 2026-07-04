#!/usr/bin/env python3
"""Печатает КАЖДУЮ смену команды /cmd_vel (дедуп) — видно, что долетает от кнопок панели."""
import rclpy
from geometry_msgs.msg import Twist

rclpy.init()
node = rclpy.create_node("cmd_watch")
last = None


def _cb(m: Twist) -> None:
    global last
    cur = (round(m.linear.x, 3), round(m.angular.z, 3))
    if cur != last:
        last = cur
        tag = "СТОП" if cur == (0.0, 0.0) else ""
        print(f"cmd_vel lin={cur[0]:+.2f} ang={cur[1]:+.2f} {tag}", flush=True)


node.create_subscription(Twist, "/cmd_vel", _cb, 10)
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
