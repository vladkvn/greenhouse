#!/usr/bin/env python3
"""depth_obstacles — камерные препятствия монокулярной глубиной (Depth Anything V2 ONNX, GPU).

Ловит то, что лидар НЕ видит (низкое/тонкое/стекло — напр. колонну). Публикует «виртуальный
лидар-скан» препятствий /depth/scan, который едят и follow-объезд, и costmap Nav2.

РЕСУРСО-УМНО (просьба юзера): глубину гоняем ТОЛЬКО когда едем автономно (/follow/active) или по
тест-флагу /perception/depth_enable; модель грузим ЛЕНИВО (по первому запросу); простаиваем — не жжём.

ДИСТАНЦИЯ метрическая, БЕЗ калибровки масштаба сети: выход DA относительный (affine-invariant), но мы
подгоняем аффинно `metric = a·P + b` по ПОЛУ пер-кадр (пиксели пола имеют известную дистанцию из
геометрии `ground_project` по позе камеры из robot.yaml). Препятствие = пиксель, что БЛИЖЕ ожидаемого
пола; его база (контакт с полом) → ground_project → метрическая дистанция → бин скана.

Запуск venv-питоном (onnxruntime в venv/~/.local), `unset PYTHONNOUSERSITE`.
"""
from __future__ import annotations

import math

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, CompressedImage, LaserScan
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener

from .geometry import camera_intrinsics, ground_project, load_robot_config

_MEAN = np.array([0.485, 0.456, 0.406], np.float32)
_STD = np.array([0.229, 0.224, 0.225], np.float32)


class DepthObstacles(Node):
    def __init__(self) -> None:
        super().__init__("depth_obstacles")
        self.declare_parameter("model_path", "/home/luki/greenhouse/models/da_v2_small.onnx")
        self.declare_parameter("input_size", 518)
        self.declare_parameter("rate_hz", 4.0)
        self.declare_parameter("dev_thr", 0.22)         # относит. отклонение P от медианы строки → «ближе пола»
        self.declare_parameter("min_run", 6)            # столько «ближе пола» строк ПОДРЯД = препятствие (не шум)
        self.declare_parameter("max_range_m", 5.0)
        self.declare_parameter("hfov_deg", 70.0)
        self.model_path = str(self.get_parameter("model_path").value)
        self.insize = int(self.get_parameter("input_size").value)
        self.dev_thr = float(self.get_parameter("dev_thr").value)
        self.min_run = int(self.get_parameter("min_run").value)
        self.max_range = float(self.get_parameter("max_range_m").value)
        self.half_fov = math.radians(float(self.get_parameter("hfov_deg").value) / 2.0)

        try:
            self.cfg = load_robot_config()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"нет robot.yaml: {exc}")
            self.cfg = None
        self.fw = self.fh = None
        self._jpeg: bytes | None = None
        self._active = False       # /follow/active
        self._test = False         # /perception/depth_enable (ручной тест)
        self._sess = None          # ленивая загрузка ONNX

        self.buf = Buffer()
        TransformListener(self.buf, self)
        self.create_subscription(CameraInfo, "/camera/camera_info", self._on_info, 1)
        self.create_subscription(CompressedImage, "/camera/image/compressed", self._on_img, 5)
        self.create_subscription(Bool, "/follow/active", self._on_active, 10)
        self.create_subscription(Bool, "/perception/depth_enable", self._on_test, 10)
        self.pub = self.create_publisher(LaserScan, "/depth/scan", 5)
        self.create_timer(1.0 / max(float(self.get_parameter("rate_hz").value), 1.0), self._tick)
        self.get_logger().info("depth_obstacles: жду активации (follow/active или /perception/depth_enable)")

    def _on_info(self, m: CameraInfo) -> None:
        self.fw, self.fh = int(m.width), int(m.height)

    def _on_img(self, m: CompressedImage) -> None:
        self._jpeg = bytes(m.data)

    def _on_active(self, m: Bool) -> None:
        self._active = bool(m.data)

    def _on_test(self, m: Bool) -> None:
        self._test = bool(m.data)

    def _load(self) -> None:
        import onnxruntime as ort
        self._sess = ort.InferenceSession(
            self.model_path, providers=["CUDAExecutionProvider", "CPUExecutionProvider"])
        self._inp = self._sess.get_inputs()[0].name
        self.get_logger().info(f"Depth Anything загружен ({self._sess.get_providers()[0]})")

    # ------------------------------------------------------------------
    def _tick(self) -> None:
        if not (self._active or self._test):        # гейтинг: не едем → GPU не трогаем
            return
        if self._jpeg is None or self.fw is None or self.cfg is None:
            return
        if self._sess is None:
            try:
                self._load()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"не загрузить depth-модель: {exc}")
                return
        frame = cv2.imdecode(np.frombuffer(self._jpeg, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            return
        try:
            depth = self._infer(frame)
            scan = self._to_scan(depth)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"depth: {exc}")
            return
        if scan is not None:
            self.pub.publish(scan)

    def _infer(self, frame) -> np.ndarray:
        s = self.insize
        img = cv2.resize(frame, (s, s))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        img = (img - _MEAN) / _STD
        x = img.transpose(2, 0, 1)[None]
        return self._sess.run(None, {self._inp: x})[0][0]     # SxS относительная глубина

    def _to_scan(self, P: np.ndarray) -> LaserScan | None:
        try:
            tf = self.buf.lookup_transform("base_footprint", "camera_optical_frame", rclpy.time.Time())
        except Exception:  # noqa: BLE001
            return None
        cam_t = (tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z)
        q = tf.transform.rotation
        cam_q = (q.x, q.y, q.z, q.w)
        intr = camera_intrinsics(self.cfg, self.fw, self.fh)
        W, H = self.fw, self.fh
        Ph, Pw = P.shape

        Wf, Hf = float(W), float(H)
        # уровень пола = МЕДИАНА P по строке (в строке пол занимает большинство → медиана=пол).
        # Препятствие = столбец, где P заметно отклоняется от медианы своей строки. Знак-конвенцию
        # (больше P = ближе или дальше?) определяем сами: ближний пол (низ) vs дальний (середина).
        Psm = cv2.blur(P.astype(np.float32), (7, 7))           # сглаживаем текстурный шум глубины
        rowmed = np.median(Psm, axis=1)
        near = float(np.median(rowmed[int(0.82 * Ph):]))
        far = float(np.median(rowmed[int(0.40 * Ph):int(0.55 * Ph)]))
        sgn = 1.0 if near > far else -1.0                      # знак «ближе пола» (конвенцию сети не знаем)

        ainc = math.radians(1.5)
        n = int(2 * self.half_fov / ainc) + 1
        ranges = [float("inf")] * n                            # inf = нет препятствия (НЕ чистим лидар!)
        for pu in range(0, Pw, 2):
            uf = pu / Pw * Wf
            run, base_pv = 0, None
            for pv in range(Ph - 1, int(0.45 * Ph), -2):       # снизу вверх
                med = float(rowmed[pv])
                dev = sgn * (float(Psm[pv, pu]) - med) / (abs(med) + 1e-3)
                if dev > self.dev_thr:                         # ближе пола этой строки
                    if base_pv is None:
                        base_pv = pv                           # низ препятствия = контакт с полом
                    run += 1
                    if run >= self.min_run:                    # подтверждено протяжённостью по высоте
                        gp = ground_project(uf, base_pv / Ph * Hf, intr, cam_t, cam_q,
                                            max_range=self.max_range)
                        if gp is not None:
                            d, brg = math.hypot(gp[0], gp[1]), math.atan2(gp[1], gp[0])
                            i = int((brg + self.half_fov) / ainc)
                            if 0 <= i < n and 0.1 < d < self.max_range:
                                ranges[i] = min(ranges[i], d)
                        break
                else:
                    run, base_pv = 0, None

        sc = LaserScan()
        sc.header.stamp = self.get_clock().now().to_msg()
        sc.header.frame_id = "base_footprint"
        sc.angle_min = -self.half_fov
        sc.angle_max = self.half_fov
        sc.angle_increment = ainc
        sc.range_min = 0.1
        sc.range_max = self.max_range
        sc.ranges = ranges
        return sc


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DepthObstacles()
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
