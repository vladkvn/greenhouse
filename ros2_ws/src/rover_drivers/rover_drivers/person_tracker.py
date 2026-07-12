#!/usr/bin/env python3
"""person_tracker — детекция человека камерой и его позиция в кадре КАРТЫ (для follow).

Конвейер (переносит проверенную логику follow-me/detector.py + fusion.py в ROS):
  1. /camera/image/compressed → YOLO11n (класс person) → берём крупнейший bbox (ближайший).
  2. bbox center-x → ПЕЛЕНГ (пинхол-модель, intrinsics из /camera/camera_info).
  3. ДАЛЬНОСТЬ — из /scan: среди возвратов лидара берём ближайший в окне ±window вокруг
     пеленга (лидар видит ноги стоящего человека). Так камера даёт направление, лидар — дистанцию.
  4. Точка цели в base_footprint → через TF map←base_footprint → поза в КАРТЕ.

Публикует:
  /follow/visible      (Bool)          — виден ли человек прямо сейчас
  /follow/bearing      (Float32, рад)  — пеленг (даже если дальность неизвестна — чтобы крутиться к цели)
  /follow/target_base  (PointStamped)  — (x,y) цели в base_footprint (когда есть дальность) — для слежения
  /follow/target_map   (PoseStamped)   — поза цели в map (последняя известная) — для езды за угол через Nav2
"""
from __future__ import annotations

import math

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, CompressedImage, LaserScan
from std_msgs.msg import Bool, Float32
from tf2_ros import Buffer, TransformListener

from .geometry import pixel_to_bearing


def _ang_diff(a: float, b: float) -> float:
    d = a - b
    return (d + math.pi) % (2 * math.pi) - math.pi


def _yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class PersonTracker(Node):
    def __init__(self) -> None:
        super().__init__("person_tracker")
        self.declare_parameter("model_path", "/home/luki/greenhouse/models/yolo11n.engine")
        self.declare_parameter("conf", 0.4)
        self.declare_parameter("rate_hz", 12.0)
        self.declare_parameter("fusion_window_deg", 6.0)   # ±окно поиска дальности в скане
        self.declare_parameter("max_range_m", 8.0)
        self.declare_parameter("person_height_m", 1.7)   # для оценки дистанции по высоте bbox
        self.conf = float(self.get_parameter("conf").value)
        self.person_h = float(self.get_parameter("person_height_m").value)
        self.win = math.radians(float(self.get_parameter("fusion_window_deg").value))
        self.max_range = float(self.get_parameter("max_range_m").value)

        # интринсики (из /camera/camera_info — совпадают с публикуемыми кадрами)
        self.fx: float | None = None
        self.cx: float | None = None
        self.fy: float | None = None
        self._sxy: tuple[float, float] | None = None     # сглаженная поза цели (base)
        self._jpeg: bytes | None = None
        self._scan: LaserScan | None = None

        self.buf = Buffer()
        TransformListener(self.buf, self)

        self.create_subscription(CameraInfo, "/camera/camera_info", self._on_info, 1)
        self.create_subscription(CompressedImage, "/camera/image/compressed", self._on_img, 5)
        self.create_subscription(LaserScan, "/scan", self._on_scan, 10)
        self.create_subscription(Bool, "/follow/enable", self._on_enable, 10)
        self._follow_enabled = False
        self.pub_vis = self.create_publisher(Bool, "/follow/visible", 10)
        self.pub_brg = self.create_publisher(Float32, "/follow/bearing", 10)
        self.pub_base = self.create_publisher(PointStamped, "/follow/target_base", 10)
        self.pub_map = self.create_publisher(PoseStamped, "/follow/target_map", 10)

        self._model = self._load_model(str(self.get_parameter("model_path").value))
        self.create_timer(1.0 / max(float(self.get_parameter("rate_hz").value), 1.0), self._tick)
        self.get_logger().info("person_tracker: YOLO person → /follow/*")

    def _load_model(self, path: str):
        try:
            from ultralytics import YOLO
            m = YOLO(path, task="detect")
            self.get_logger().info(f"YOLO загружен: {path}")
            return m
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"НЕ загрузить YOLO ({path}): {exc} — детекции не будет")
            return None

    def _on_info(self, m: CameraInfo) -> None:
        self.fx, self.cx, self.fy = float(m.k[0]), float(m.k[2]), float(m.k[4])

    def _smooth(self, xy: tuple[float, float]) -> tuple[float, float]:
        """EMA-сглаживание позы цели + отсев прыжков (большой скачок подтягиваем медленно)."""
        if self._sxy is None:
            self._sxy = xy
        else:
            jump = math.hypot(xy[0] - self._sxy[0], xy[1] - self._sxy[1])
            a = 0.5 if jump < 1.0 else 0.15
            self._sxy = (a * xy[0] + (1 - a) * self._sxy[0], a * xy[1] + (1 - a) * self._sxy[1])
        return self._sxy

    def _on_img(self, m: CompressedImage) -> None:
        self._jpeg = bytes(m.data)

    def _on_scan(self, s: LaserScan) -> None:
        self._scan = s

    def _on_enable(self, m: Bool) -> None:
        self._follow_enabled = bool(m.data)

    # ------------------------------------------------------------------
    def _tick(self) -> None:
        if not self._follow_enabled:            # ОПТИМИЗАЦИЯ: YOLO крутим ТОЛЬКО в режиме follow
            self.pub_vis.publish(Bool(data=False))
            self._sxy = None
            return
        if self._model is None or self._jpeg is None or self.fx is None:
            return
        frame = cv2.imdecode(np.frombuffer(self._jpeg, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            return
        try:
            res = self._model(frame, classes=[0], conf=self.conf, verbose=False)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"инференс: {exc}")
            return

        box = self._largest_person(res)
        if box is None:
            self.pub_vis.publish(Bool(data=False))
            self._sxy = None                                 # сброс сглаживания при потере
            return
        x1, y1, x2, y2 = box
        u = 0.5 * (x1 + x2)
        bearing = pixel_to_bearing(u, self.fx, self.cx)     # рад, +влево, в кадре камеры (=перёд робота)
        self.pub_vis.publish(Bool(data=True))
        self.pub_brg.publish(Float32(data=float(bearing)))

        # дистанция: лидар (точно по ногам), НО с проверкой по ВЫСОТЕ bbox (рост≈person_h).
        # Так отсекаем «фон» в окне пеленга и стабилизируем прыгающую позу; если лидар не про
        # человека — берём дистанцию по камере. Затем сглаживаем.
        d_cam = (self.fy * self.person_h / max(1.0, y2 - y1)) if self.fy else None
        base_xy = self._range_from_scan(bearing)
        if base_xy is not None and d_cam is not None:
            d_lidar = math.hypot(base_xy[0], base_xy[1])
            if not (0.5 * d_cam < d_lidar < 2.0 * d_cam):    # лидар явно не про человека → отбрасываем
                base_xy = None
        if base_xy is None and d_cam is not None:            # запасной: дистанция по камере + пеленг
            base_xy = (d_cam * math.cos(bearing), d_cam * math.sin(bearing))
        if base_xy is None:
            return
        bx, by = self._smooth(base_xy)
        ps = PointStamped()
        ps.header.frame_id = "base_footprint"
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.point.x, ps.point.y = float(bx), float(by)
        self.pub_base.publish(ps)

        target_map = self._to_map(bx, by)
        if target_map is not None:
            mx, my = target_map
            pm = PoseStamped()
            pm.header.frame_id = "map"
            pm.header.stamp = ps.header.stamp
            pm.pose.position.x, pm.pose.position.y = float(mx), float(my)
            yaw = math.atan2(by, bx)                          # ориентация «от робота к цели»
            pm.pose.orientation.z = math.sin(yaw / 2.0)
            pm.pose.orientation.w = math.cos(yaw / 2.0)
            self.pub_map.publish(pm)

    def _largest_person(self, res):
        best, best_area = None, 0.0
        for r in res:
            for b in r.boxes:
                if int(b.cls[0]) != 0:
                    continue
                x1, y1, x2, y2 = (float(v) for v in b.xyxy[0])
                area = (x2 - x1) * (y2 - y1)
                if area > best_area:
                    best, best_area = (x1, y1, x2, y2), area
        return best

    def _range_from_scan(self, bearing: float) -> tuple[float, float] | None:
        """Ближайший возврат лидара в окне ±win вокруг пеленга → его точка (x,y) в base_footprint."""
        s = self._scan
        if s is None:
            return None
        try:
            tf = self.buf.lookup_transform("base_footprint", s.header.frame_id or "laser", rclpy.time.Time())
        except Exception:  # noqa: BLE001
            return None
        tx, ty = tf.transform.translation.x, tf.transform.translation.y
        lyaw = _yaw(tf.transform.rotation)
        c, si = math.cos(lyaw), math.sin(lyaw)
        best_r, best_pt = 1e9, None
        a = s.angle_min
        for r in s.ranges:
            if s.range_min < r < s.range_max and r < self.max_range:
                lx, ly = r * math.cos(a), r * math.sin(a)
                bx = tx + lx * c - ly * si
                by = ty + lx * si + ly * c
                if abs(_ang_diff(math.atan2(by, bx), bearing)) < self.win:
                    rng = math.hypot(bx, by)
                    if rng < best_r:
                        best_r, best_pt = rng, (bx, by)
            a += s.angle_increment
        return best_pt

    def _to_map(self, bx: float, by: float) -> tuple[float, float] | None:
        try:
            tf = self.buf.lookup_transform("map", "base_footprint", rclpy.time.Time())
        except Exception:  # noqa: BLE001
            return None
        rx, ry = tf.transform.translation.x, tf.transform.translation.y
        ryaw = _yaw(tf.transform.rotation)
        mx = rx + bx * math.cos(ryaw) - by * math.sin(ryaw)
        my = ry + bx * math.sin(ryaw) + by * math.cos(ryaw)
        return mx, my


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PersonTracker()
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
