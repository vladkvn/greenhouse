#!/usr/bin/env python3
"""esp32_sensor_bridge — мост ESP32 сенсор-пульта (esp32-sensor-remote) в ROS 2.

Отдельная ESP32 DevKit («глаза и пульт», НЕ моторная плата) шлёт по USB-serial текстом
(см. esp32-sensor-remote/README.md):
    READY                — старт
    T <dl> <dr> <mode>   — телеметрия ~10Гц: дистанции L/R в СМ (-1 = нет эха), режим 0..9
    K <key>              — кнопка: UP|DOWN|LEFT|RIGHT|OK|0..9
    M <n>                — переключён режим n
    D <lin> <ang>        — движение (ручной режим), семантика как у кнопок веб-панели

Узел ЧИТАЕТ это (на ESP32 ничего не шлём) и публикует:
  * 2× sensor_msgs/Range — HC-SR04 перёд лево/право → в costmap Nav2 через range_sensor_layer
    (ближние/низкие помехи в слепой зоне лидара; заменили камерную глубину-нейросеть);
  * /rover/mode (Int32) — выбранный ИК-пультом режим (трактует follow/оркестратор);
  * /rover/key (String) — нажатая кнопка (для отладки/логики);
  * /cmd_vel (Twist) — ручная езда стрелками пульта. ПУБЛИКУЕМ ТОЛЬКО по строкам `D` (событийно):
    в простое ESP32 не шлёт D → мы молчим → не дерёмся за /cmd_vel с Nav2/панелью (тот же принцип,
    что у rover_webui). Стоп приходит строкой `D 0 0`, дальше — тишина, а deadman узла привода добьёт.

-1 (нет эха) в Range → range=max_range = «чисто» (range_sensor_layer это очистит).
"""
from __future__ import annotations

import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Int32, String


class Esp32SensorBridge(Node):
    def __init__(self) -> None:
        super().__init__("esp32_sensor_bridge")
        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("max_range", 4.0)               # практический потолок HC-SR04, м
        self.declare_parameter("min_range", 0.02)              # м
        self.declare_parameter("fov", 0.26)                    # ~15° конус HC-SR04, рад
        self.declare_parameter("frame_left", "sonar_front_left")   # кадр TF (добавить в URDF)
        self.declare_parameter("frame_right", "sonar_front_right")
        self.declare_parameter("publish_cmd_vel", True)        # ручная езда с пульта в /cmd_vel

        gp = self.get_parameter
        self.port = str(gp("serial_port").value)
        self.baud = int(gp("baud").value)
        self.max_range = float(gp("max_range").value)
        self.min_range = float(gp("min_range").value)
        self.fov = float(gp("fov").value)
        self.frame_l = str(gp("frame_left").value)
        self.frame_r = str(gp("frame_right").value)
        self.pub_cmd = bool(gp("publish_cmd_vel").value)

        self.rng_l = self.create_publisher(Range, "sonar/front_left", 10)
        self.rng_r = self.create_publisher(Range, "sonar/front_right", 10)
        self.mode_pub = self.create_publisher(Int32, "rover/mode", 10)
        self.key_pub = self.create_publisher(String, "rover/key", 10)
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10) if self.pub_cmd else None
        self._mode: int | None = None

        self._ser = self._open()
        self._alive = True
        self._io = threading.Thread(target=self._run, daemon=True)
        self._io.start()
        self.get_logger().info(
            f"esp32_sensor_bridge: port={self.port} → Range(L/R), /rover/mode, "
            f"{'/cmd_vel' if self.pub_cmd else 'без cmd_vel'}"
        )

    # ------------------------------------------------------------------ serial
    def _open(self):
        try:
            import serial  # pyserial
        except ImportError:
            self.get_logger().error("pyserial не установлен — узел вхолостую (apt install python3-serial)")
            return None
        try:
            return serial.Serial(self.port, self.baud, timeout=1.0)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"не открыть {self.port}: {exc} — узел вхолостую")
            return None

    def _run(self) -> None:
        while self._alive:
            if self._ser is None:
                time.sleep(1.0)
                continue
            try:
                line = self._ser.readline()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f"serial read: {exc}", throttle_duration_sec=2.0)
                time.sleep(0.5)
                continue
            if not line:
                continue
            try:
                self._parse(line.decode(errors="ignore").strip())
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f"parse {line!r}: {exc}", throttle_duration_sec=2.0)

    # ------------------------------------------------------------------ parse
    def _parse(self, s: str) -> None:
        if not s:
            return
        p = s.split()
        tag = p[0]
        if tag == "T" and len(p) >= 4:
            self.rng_l.publish(self._range(self.frame_l, self._i(p[1])))
            self.rng_r.publish(self._range(self.frame_r, self._i(p[2])))
            self._emit_mode(self._i(p[3]))
        elif tag == "M" and len(p) >= 2:
            self._emit_mode(self._i(p[1]))
        elif tag == "K" and len(p) >= 2:
            self.key_pub.publish(String(data=p[1]))
        elif tag == "D" and len(p) >= 3 and self.cmd_pub is not None:
            t = Twist()
            t.linear.x = self._f(p[1])
            t.angular.z = self._f(p[2])
            self.cmd_pub.publish(t)               # событийно: только пока пульт шлёт D
        elif tag == "READY":
            self.get_logger().info("ESP32 sensor-remote: READY")

    def _range(self, frame: str, dist_cm: int | None) -> Range:
        m = Range()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = frame
        m.radiation_type = Range.ULTRASOUND
        m.field_of_view = self.fov
        m.min_range = self.min_range
        m.max_range = self.max_range
        if dist_cm is None or dist_cm < 0:        # нет эха → «чисто»
            m.range = self.max_range
        else:
            m.range = max(self.min_range, min(self.max_range, dist_cm / 100.0))
        return m

    def _emit_mode(self, mode: int | None) -> None:
        if mode is None or mode == self._mode:
            return
        self._mode = mode
        self.mode_pub.publish(Int32(data=int(mode)))
        self.get_logger().info(f"режим → {mode}")

    @staticmethod
    def _i(s: str) -> int | None:
        try:
            return int(s)
        except ValueError:
            return None

    @staticmethod
    def _f(s: str) -> float:
        try:
            return float(s)
        except ValueError:
            return 0.0

    def destroy_node(self) -> bool:
        self._alive = False
        if self._io.is_alive():
            self._io.join(timeout=0.5)
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:  # noqa: BLE001
                pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Esp32SensorBridge()
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
