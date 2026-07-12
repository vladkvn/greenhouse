#!/usr/bin/env python3
"""camera_node — ЕДИНСТВЕННЫЙ владелец USB-камеры.

/dev/video может открыть лишь один процесс, а кадры нужны панели, детектору person и
монокулярной глубине. Поэтому камеру держит этот узел и публикует кадры (jpeg) +
camera_info (интринсики из robot.yaml) в топики — остальные ПОДПИСЫВАЮТСЯ.

Низкая задержка: MJPG + BUFFERSIZE=1 + grab-луп (как было в панели).
"""
from __future__ import annotations

import threading
import time

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, CompressedImage

from .geometry import camera_intrinsics, load_robot_config


class CameraNode(Node):
    def __init__(self) -> None:
        super().__init__("camera_node")
        self.declare_parameter("cam_index", 0)
        self.declare_parameter("pub_width", 640)      # кадр публикуем в этой ширине (YOLO/панель)
        self.declare_parameter("fps", 20.0)
        self.declare_parameter("jpeg_quality", 60)
        self.declare_parameter("frame_id", "camera_optical_frame")
        self._req_index = int(self.get_parameter("cam_index").value)
        self._pub_w = int(self.get_parameter("pub_width").value)
        self.fps = float(self.get_parameter("fps").value)
        self.q = int(self.get_parameter("jpeg_quality").value)
        self.frame_id = str(self.get_parameter("frame_id").value)

        self.pub = self.create_publisher(CompressedImage, "/camera/image/compressed", 5)
        self.info_pub = self.create_publisher(CameraInfo, "/camera/camera_info", 1)
        try:
            self._cfg = load_robot_config()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"robot.yaml не загружен ({exc}) — без camera_info")
            self._cfg = None
        self._info: CameraInfo | None = None
        self._jpeg: bytes | None = None

        threading.Thread(target=self._grab_loop, daemon=True).start()
        self.create_timer(1.0 / max(self.fps, 1.0), self._publish)
        self.get_logger().info("camera_node: публикую /camera/image/compressed + /camera/camera_info")

    def _grab_loop(self) -> None:
        cap = None
        for idx in [self._req_index, 0, 1, 2, 3]:      # авто-подбор индекса (нумерация плавает)
            c = cv2.VideoCapture(idx)
            if c.isOpened() and c.read()[0]:
                cap = c
                break
            c.release()
        if cap is None:
            self.get_logger().warn("камера не найдена (индексы 0..3)")
            return
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        try:
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:  # noqa: BLE001
            pass
        self.get_logger().info("камера открыта")
        while rclpy.ok():
            if not cap.grab():                         # grab держит очередь пустой → свежий кадр
                time.sleep(0.03)
                continue
            ok, frame = cap.retrieve()
            if not ok:
                continue
            h, w = frame.shape[:2]
            if w != self._pub_w:                       # публикуем в фикс. ширине (интринсики под неё)
                nh = max(1, int(self._pub_w * h / w))
                frame = cv2.resize(frame, (self._pub_w, nh))
                h, w = nh, self._pub_w
            if self._info is None and self._cfg is not None:
                self._info = self._make_info(w, h)
            self._jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, self.q])[1].tobytes()

    def _make_info(self, w: int, h: int) -> CameraInfo:
        fx, fy, cx, cy, _, _ = camera_intrinsics(self._cfg, w, h)
        info = CameraInfo()
        info.width, info.height = int(w), int(h)
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        info.distortion_model = "plumb_bob"
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.header.frame_id = self.frame_id
        return info

    def _publish(self) -> None:
        jpg = self._jpeg
        if jpg is None:
            return
        now = self.get_clock().now().to_msg()
        m = CompressedImage()
        m.header.stamp = now
        m.header.frame_id = self.frame_id
        m.format = "jpeg"
        m.data = jpg
        self.pub.publish(m)
        if self._info is not None:
            self._info.header.stamp = now
            self.info_pub.publish(self._info)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraNode()
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
