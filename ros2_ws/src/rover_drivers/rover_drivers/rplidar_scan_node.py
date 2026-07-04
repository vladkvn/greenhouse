#!/usr/bin/env python3
"""RPLIDAR A1 → sensor_msgs/LaserScan через python-библиотеку `rplidar` (rplidar-roboticia).

Почему не sllidar_ros2: на этом железе (RPLIDAR A1 + CP2102) драйвер Slamtec дёргает DTR при
открытии порта и ресетит лидар прямо во время своего handshake → SL_RESULT_OPERATION_TIMEOUT.
Библиотека `rplidar` (та же, что в боевом follow_me) работает на этом лидаре стабильно.

Читаем сканы в фоновом потоке с авто-переподключением (как follow_me), публикуем LaserScan
в кадре `laser`. Разворот лидара на 180° учтён в URDF (base_link→laser), скан НЕ роллим.
`invert` (CW↔CCW) — калибровка направления: если карта зеркалится, включить.

Зависимость на Jetson (не в системе, ставится без sudo):  pip3 install --user rplidar-roboticia
"""
from __future__ import annotations

import math
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class RPLidarScanNode(Node):
    def __init__(self) -> None:
        super().__init__("rplidar_scan_node")
        self.declare_parameter("serial_port", "/dev/ttyUSB1")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("frame_id", "laser")
        self.declare_parameter("angle_bins", 360)
        self.declare_parameter("range_min", 0.15)
        self.declare_parameter("range_max", 12.0)   # RPLIDAR A1 ~12 м
        self.declare_parameter("scan_time", 0.18)    # ~5.5 Гц
        self.declare_parameter("invert", True)       # RPLIDAR CW → ROS CCW (замерено: скан зеркальный)
        self.declare_parameter("angle_offset_deg", 0.0)  # доворот скана под монтаж (0/90/180/270)

        self.port = self.get_parameter("serial_port").value
        self.baud = int(self.get_parameter("baud").value)
        self.frame_id = self.get_parameter("frame_id").value
        self.bins = int(self.get_parameter("angle_bins").value)
        self.range_min = float(self.get_parameter("range_min").value)
        self.range_max = float(self.get_parameter("range_max").value)
        self.scan_time = float(self.get_parameter("scan_time").value)
        self.invert = bool(self.get_parameter("invert").value)
        self.angle_offset_deg = float(self.get_parameter("angle_offset_deg").value)

        self._pub = self.create_publisher(LaserScan, "scan", 10)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="rplidar-reader", daemon=True)
        self._thread.start()
        self.get_logger().info(
            f"RPLIDAR: port={self.port} baud={self.baud} bins={self.bins} → /scan (frame={self.frame_id})"
        )

    # ------------------------------------------------------------------ reader
    def _run(self) -> None:
        # вендор единственного файла rplidar.py из venv follow_me (см. rplidar_vendor.py)
        from rover_drivers.rplidar_vendor import RPLidar, RPLidarException

        while not self._stop.is_set() and rclpy.ok():
            lidar = None
            try:
                # Точная последовательность из боевого follow_me LidarThread.run() —
                # info ПЕРЕД health синхронизирует протокол, на этом лидаре это критично.
                lidar = RPLidar(self.port, baudrate=self.baud)
                info = lidar.get_info()
                health = lidar.get_health()
                self.get_logger().info(f"RPLIDAR подключён: info={info} health={health}")
                for scan in lidar.iter_scans():
                    if self._stop.is_set() or not rclpy.ok():
                        break
                    self._publish(scan)
            except (RPLidarException, OSError) as exc:
                self.get_logger().warn(f"RPLIDAR ошибка: {exc} — переподключение через 1.5с")
                self._stop.wait(1.5)
            finally:
                if lidar is not None:
                    try:
                        lidar.stop()
                        lidar.stop_motor()
                        lidar.disconnect()
                    except Exception:  # noqa: BLE001
                        pass

    def _publish(self, scan: list) -> None:
        n = self.bins
        ranges = [float("inf")] * n
        inc = 2.0 * math.pi / n
        off = int(round(self.angle_offset_deg / 360.0 * n)) % n   # доворот под монтаж
        for quality, angle, dist in scan:
            if dist <= 0 or quality <= 0:
                continue
            r = dist / 1000.0
            if not (self.range_min <= r <= self.range_max):
                continue
            idx = int(round(angle)) % n
            if self.invert:
                idx = (n - idx) % n
            idx = (idx + off) % n
            ranges[idx] = r

        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.angle_min = 0.0
        msg.angle_max = 2.0 * math.pi - inc
        msg.angle_increment = inc
        msg.range_min = self.range_min
        msg.range_max = self.range_max
        msg.scan_time = self.scan_time
        msg.time_increment = self.scan_time / n
        msg.ranges = ranges
        self._pub.publish(msg)

    def destroy_node(self) -> bool:
        self._stop.set()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RPLidarScanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
