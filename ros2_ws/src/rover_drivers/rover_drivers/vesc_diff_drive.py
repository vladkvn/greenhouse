#!/usr/bin/env python3
"""cmd_vel → Dual VESC (Flipsky 6.7) diff-drive + колёсная одометрия.

Заменяет мост ESP32. Два мотора-гироскутера (BLDC, FOC, датчики Холла), одна плата Dual VESC,
один USB в Jetson. Локальную половину (та, что на USB) командуем НАПРЯМУЮ бинарным VESC-протоколом;
вторую половину — тем же протоколом, обёрнутым в COMM_FORWARD_CAN по её CAN-id (внутренний CAN дуала).

Что даёт (чего не было на ESP32):
  * ЗАМКНУТАЯ скорость — команда в ERPM, VESC сам держит обороты под нагрузкой (не разомкнутый ШИМ);
  * НАСТОЯЩАЯ одометрия — тахометр каждой половины → путь колеса → /odom (раньше крутились на rf2o+IMU).

Геометрия — из ЕДИНОГО robot.yaml (wheel_radius, wheel_base=колея); электрика привода — из секции
`drivetrain` (порт, пары полюсов, CAN-id половин, инверсия бортов, лимиты). Пересчёт:

    ERPM = v[м/с] / (2π·r) · 60 · pole_pairs · gear_ratio          (gear_ratio = обороты мотора на 1 оборот колеса)
    метр/тик_тахометра = 2π·r / (6 · pole_pairs · gear_ratio)      (VESC считает 6 шагов на электрический оборот)

Конвенция REP-103: linear.x>0 = вперёд, angular.z>0 = поворот влево (CCW) → правый борт быстрее.
Инверсию бортов (левый/правый мотор зеркальны) задают флаги invert в конфиге — калибруются на первом пуске.

TF odom→base_footprint по умолчанию НЕ публикуем (его строит EKF robot_localization). Топик /odom
скармливается в EKF как источник колёсной скорости. Для езды без EKF — publish_tf:=true.
"""
from __future__ import annotations

import math
import struct
import threading
import time

import rclpy
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from .geometry import load_robot_config

# --- VESC binary UART protocol (packet framing + нужные команды) ---------------
COMM_GET_VALUES = 4
COMM_SET_CURRENT = 6
COMM_SET_RPM = 8
COMM_FORWARD_CAN = 34


def _crc16(data: bytes) -> int:
    """CRC16-CCITT (XMODEM), init 0x0000 — как в прошивке VESC (по payload)."""
    crc = 0
    for b in data:
        crc ^= (b << 8) & 0xFFFF
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc & 0xFFFF


def _frame(payload: bytes) -> bytes:
    """Обернуть payload в кадр VESC: [start][len][payload][crc16][0x03]."""
    n = len(payload)
    head = bytes([2, n]) if n <= 255 else bytes([3, (n >> 8) & 0xFF, n & 0xFF])
    crc = _crc16(payload)
    return head + payload + bytes([(crc >> 8) & 0xFF, crc & 0xFF, 3])


def _wrap_can(can_id: int, inner: bytes) -> bytes:
    """Обёртка forward-CAN: локальная половина ретранслирует inner на CAN-id по внутреннему CAN."""
    return bytes([COMM_FORWARD_CAN, can_id & 0xFF]) + inner


class _VescLink:
    """Сериал-канал к локальной половине VESC (USB-CDC). Команды/опрос обеих половин через неё."""

    def __init__(self, port: str, baud: int = 115200):
        import serial  # pyserial
        self.ser = serial.Serial(port, baud, timeout=0.02)
        self._buf = bytearray()

    def close(self) -> None:
        try:
            self.ser.close()
        except Exception:  # noqa: BLE001
            pass

    def _send(self, inner: bytes, can_id: int | None) -> None:
        self.ser.write(_frame(inner if can_id is None else _wrap_can(can_id, inner)))

    def set_rpm(self, erpm: int, can_id: int | None) -> None:
        self._send(bytes([COMM_SET_RPM]) + struct.pack(">i", int(erpm)), can_id)

    def set_current(self, amps: float, can_id: int | None) -> None:
        # SET_CURRENT: int32 миллиампер. amps=0 → мотор свободен (выбег).
        self._send(bytes([COMM_SET_CURRENT]) + struct.pack(">i", int(amps * 1000.0)), can_id)

    def request_values(self, can_id: int | None) -> None:
        self._send(bytes([COMM_GET_VALUES]), can_id)

    def _extract(self) -> bytes | None:
        """Вытащить один валидный кадр из буфера (payload) либо None, если данных ещё мало."""
        buf = self._buf
        while buf:
            s = buf[0]
            if s == 2:
                if len(buf) < 2:
                    return None
                hdr, plen = 2, buf[1]
            elif s == 3:
                if len(buf) < 3:
                    return None
                hdr, plen = 3, (buf[1] << 8) | buf[2]
            else:
                buf.pop(0)               # мусор/рассинхрон — ищем стартовый байт дальше
                continue
            total = hdr + plen + 3
            if len(buf) < total:
                return None              # кадр ещё не пришёл целиком
            payload = bytes(buf[hdr:hdr + plen])
            crc_rx = (buf[hdr + plen] << 8) | buf[hdr + plen + 1]
            end = buf[hdr + plen + 2]
            del buf[:total]
            if end == 3 and _crc16(payload) == crc_rx:
                return payload
            # битый кадр — отбросили, пробуем со следующего байта
        return None

    def read_values(self, timeout: float) -> tuple[float, int] | None:
        """Прочитать ответ GET_VALUES → (erpm, tachometer). None при таймауте.

        Опрос половин последовательный (запросил → дождался ответа), поэтому кадры не путаются:
        SET_* ответа не дают, единственные входящие кадры — ответы GET_VALUES по порядку запросов.
        """
        deadline = time.monotonic() + timeout
        while True:
            payload = self._extract()
            if payload is not None and payload and payload[0] == COMM_GET_VALUES and len(payload) >= 49:
                erpm = struct.unpack_from(">i", payload, 23)[0]      # data[22:26] (после id-байта)
                tach = struct.unpack_from(">i", payload, 45)[0]      # data[44:48]
                return float(erpm), int(tach)
            if time.monotonic() >= deadline:
                return None
            chunk = self.ser.read(128)
            if chunk:
                self._buf.extend(chunk)


class VescDiffDrive(Node):
    def __init__(self) -> None:
        super().__init__("vesc_diff_drive")

        # --- конфиг: геометрия из robot.yaml (единый источник), электрика из секции drivetrain ---
        try:
            cfg = load_robot_config()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"нет robot.yaml ({exc}) — дефолты")
            cfg = {}
        dt = cfg.get("drivetrain", {}) if cfg else {}
        left = dt.get("left", {})
        right = dt.get("right", {})

        self.declare_parameter("port", dt.get("port", "/dev/ttyACM0"))
        self.declare_parameter("baud", 115200)
        self.declare_parameter("wheel_radius", float(cfg.get("wheel_radius", 0.0825)))  # м
        self.declare_parameter("track_width", float(cfg.get("wheel_base", 0.50)))       # колея, м
        self.declare_parameter("pole_pairs", int(dt.get("pole_pairs", 15)))
        self.declare_parameter("gear_ratio", float(dt.get("gear_ratio", 1.0)))
        self.declare_parameter("max_speed", float(dt.get("max_speed", 1.0)))            # м/с потолок
        self.declare_parameter("accel_limit", float(dt.get("accel_limit", 1.2)))       # м/с²
        self.declare_parameter("cmd_timeout", float(dt.get("cmd_timeout", 0.4)))       # с → стоп
        self.declare_parameter("rate_hz", float(dt.get("rate_hz", 50.0)))
        self.declare_parameter("local_id", int(dt.get("local_id", 52)))                # id половины на USB
        self.declare_parameter("left_can_id", int(left.get("can_id", 52)))
        self.declare_parameter("right_can_id", int(right.get("can_id", 4)))
        self.declare_parameter("left_invert", bool(left.get("invert", False)))
        self.declare_parameter("right_invert", bool(right.get("invert", True)))
        self.declare_parameter("publish_tf", False)                                    # EKF даёт свой TF
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")

        gp = self.get_parameter
        self.port = str(gp("port").value)
        self.baud = int(gp("baud").value)
        self.r = float(gp("wheel_radius").value)
        self.track = float(gp("track_width").value)
        self.pp = int(gp("pole_pairs").value)
        self.gear = float(gp("gear_ratio").value)
        self.max_speed = float(gp("max_speed").value)
        self.accel = float(gp("accel_limit").value)
        self.cmd_timeout = float(gp("cmd_timeout").value)
        self.rate = max(float(gp("rate_hz").value), 1.0)
        local_id = int(gp("local_id").value)
        l_id = int(gp("left_can_id").value)
        r_id = int(gp("right_can_id").value)
        self.sign_l = -1.0 if bool(gp("left_invert").value) else 1.0
        self.sign_r = -1.0 if bool(gp("right_invert").value) else 1.0
        self.publish_tf = bool(gp("publish_tf").value)
        self.odom_frame = str(gp("odom_frame").value)
        self.base_frame = str(gp("base_frame").value)

        # локальную половину командуем напрямую (can_id=None), вторую — forward-CAN по её id
        self.id_l = None if l_id == local_id else l_id
        self.id_r = None if r_id == local_id else r_id
        self._k_erpm = 60.0 * self.pp * self.gear / (2.0 * math.pi * self.r)   # м/с → ERPM
        self.max_erpm = self._k_erpm * self.max_speed
        self.m_per_count = 2.0 * math.pi * self.r / (6.0 * self.pp * self.gear)  # тик тахометра → м
        self._read_to = min(1.0 / self.rate, 0.02)   # таймаут ответа GET_VALUES каждой половины

        # --- состояние ---
        self._lock = threading.Lock()
        self._tv = 0.0            # целевая линейная, м/с
        self._tw = 0.0            # целевая угловая, рад/с
        self._last_cmd = 0.0      # monotonic последней cmd_vel
        self._cvl = 0.0           # текущая (после рампы) скорость левого колеса, м/с
        self._cvr = 0.0
        self.x = self.y = self.th = 0.0
        self._tach_l: int | None = None
        self._tach_r: int | None = None

        # --- ROS ---
        self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        self.tf_bc = TransformBroadcaster(self) if self.publish_tf else None
        self.create_subscription(Twist, "cmd_vel", self._on_cmd, 10)

        self._link: _VescLink | None = None
        try:
            self._link = _VescLink(self.port, self.baud)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"не открыть VESC {self.port}: {exc} — узел вхолостую")

        self._alive = True
        self._io = threading.Thread(target=self._run, daemon=True)
        self._io.start()
        self.get_logger().info(
            f"vesc_diff_drive: port={self.port} r={self.r:.3f}м колея={self.track:.3f}м "
            f"pp={self.pp} L=id{l_id}{'(local)' if self.id_l is None else ''}/inv{self.sign_l<0} "
            f"R=id{r_id}{'(local)' if self.id_r is None else ''}/inv{self.sign_r<0} "
            f"max={self.max_speed}м/с(~{self.max_erpm:.0f}ERPM) tf={self.publish_tf}"
        )

    # ------------------------------------------------------------------ ROS cb
    def _on_cmd(self, msg: Twist) -> None:
        with self._lock:
            self._tv = max(-self.max_speed, min(self.max_speed, msg.linear.x))
            self._tw = msg.angular.z
            self._last_cmd = time.monotonic()

    # ------------------------------------------------------------------ math
    def _erpm(self, v: float) -> float:
        return self._k_erpm * v

    def _speed(self, erpm: float) -> float:
        return erpm / self._k_erpm

    @staticmethod
    def _ramp(cur: float, tgt: float, max_delta: float) -> float:
        d = tgt - cur
        if abs(d) <= max_delta:
            return tgt
        return cur + math.copysign(max_delta, d)

    # ------------------------------------------------------------------ IO loop
    def _run(self) -> None:
        period = 1.0 / self.rate
        prev = time.monotonic()
        while self._alive:
            t0 = time.monotonic()
            dt = t0 - prev
            prev = t0

            with self._lock:
                stale = (t0 - self._last_cmd) > self.cmd_timeout
                tv = 0.0 if stale else self._tv
                tw = 0.0 if stale else self._tw

            # цель по бортам → рампа ускорения (плавный старт/стоп)
            tvl, tvr = tv - tw * self.track / 2.0, tv + tw * self.track / 2.0
            step = self.accel * dt
            self._cvl = self._ramp(self._cvl, tvl, step)
            self._cvr = self._ramp(self._cvr, tvr, step)

            if self._link is not None:
                self._drive(tv, tw)
                self._odom_step(dt)

            sleep = period - (time.monotonic() - t0)
            if sleep > 0:
                time.sleep(sleep)

    def _drive(self, tv: float, tw: float) -> None:
        idle = abs(tv) < 1e-3 and abs(tw) < 1e-3 and abs(self._cvl) < 1e-3 and abs(self._cvr) < 1e-3
        try:
            if idle:
                self._link.set_current(0.0, self.id_l)   # выбег: не греем моторы удержанием 0 об/мин
                self._link.set_current(0.0, self.id_r)
            else:
                el = max(-self.max_erpm, min(self.max_erpm, self._erpm(self._cvl) * self.sign_l))
                er = max(-self.max_erpm, min(self.max_erpm, self._erpm(self._cvr) * self.sign_r))
                self._link.set_rpm(el, self.id_l)
                self._link.set_rpm(er, self.id_r)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"VESC write: {exc}", throttle_duration_sec=2.0)

    def _odom_step(self, dt: float) -> None:
        # последовательный опрос: запрос → ответ каждой половины (кадры не путаются)
        try:
            self._link.request_values(self.id_l)
            vl = self._link.read_values(self._read_to)
            self._link.request_values(self.id_r)
            vr = self._link.read_values(self._read_to)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"VESC read: {exc}", throttle_duration_sec=2.0)
            return
        if vl is None or vr is None:
            return

        erpm_l, tach_l = vl
        erpm_r, tach_r = vr
        # измеренная скорость колеса в кадре робота (снимаем инверсию борта)
        mvl = self._speed(erpm_l) * self.sign_l
        mvr = self._speed(erpm_r) * self.sign_r
        vx = (mvl + mvr) / 2.0
        wz = (mvr - mvl) / self.track

        # положение — по приращению тахометра (точнее интегрирования ERPM, без дрейфа)
        if self._tach_l is not None:
            dl = (tach_l - self._tach_l) * self.m_per_count * self.sign_l
            dr = (tach_r - self._tach_r) * self.m_per_count * self.sign_r
            dc = (dl + dr) / 2.0
            dth = (dr - dl) / self.track
            self.x += dc * math.cos(self.th + dth / 2.0)
            self.y += dc * math.sin(self.th + dth / 2.0)
            self.th = math.atan2(math.sin(self.th + dth), math.cos(self.th + dth))
        self._tach_l, self._tach_r = tach_l, tach_r

        self._publish(vx, wz)

    def _publish(self, vx: float, wz: float) -> None:
        now = self.get_clock().now().to_msg()
        qz, qw = math.sin(self.th / 2.0), math.cos(self.th / 2.0)

        od = Odometry()
        od.header.stamp = now
        od.header.frame_id = self.odom_frame
        od.child_frame_id = self.base_frame
        od.pose.pose.position.x = self.x
        od.pose.pose.position.y = self.y
        od.pose.pose.orientation.z = qz
        od.pose.pose.orientation.w = qw
        od.twist.twist.linear.x = vx
        od.twist.twist.angular.z = wz
        od.pose.covariance[0] = od.pose.covariance[7] = 0.002       # x,y
        od.pose.covariance[35] = 0.01                                # yaw
        od.twist.covariance[0] = 0.002
        od.twist.covariance[35] = 0.01
        self.odom_pub.publish(od)

        if self.tf_bc is not None:
            tf = TransformStamped()
            tf.header.stamp = now
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self.tf_bc.sendTransform(tf)

    def destroy_node(self) -> bool:
        self._alive = False
        if self._io.is_alive():
            self._io.join(timeout=0.5)
        if self._link is not None:
            try:
                self._link.set_current(0.0, self.id_l)   # отпустить моторы
                self._link.set_current(0.0, self.id_r)
            except Exception:  # noqa: BLE001
                pass
            self._link.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VescDiffDrive()
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
