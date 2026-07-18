#!/usr/bin/env python3
"""cmd_vel → две половины VESC (Flipsky 6.7) diff-drive + колёсная одометрия.

Заменяет мост ESP32. Два мотора-гироскутера (BLDC, FOC, датчики Холла), КАЖДАЯ половина VESC — на
СВОЁМ USB (по /dev/ttyACM*). CAN между половинами не используем (не завёлся) — обе командуем и читаем
напрямую бинарным VESC-протоколом. Узел сам определяет, где какая половина, по VESC-id из ответа платы,
поэтому порядок портов (кто ACM0, кто ACM1) неважен.

Что даёт (чего не было на ESP32):
  * ЗАМКНУТАЯ скорость — команда в ERPM, VESC сам держит обороты под нагрузкой (не разомкнутый ШИМ);
  * НАСТОЯЩАЯ одометрия — тахометр каждой половины → путь колеса → /odom.

Геометрия — из ЕДИНОГО robot.yaml (wheel_radius, wheel_base=колея); привод — из секции `drivetrain`
(ports, pole_pairs, left/right{vesc_id, invert}, лимиты). Пересчёт:

    ERPM = v[м/с] / (2π·r) · 60 · pole_pairs · gear_ratio
    метр/тик_тахометра = 2π·r / (6 · pole_pairs · gear_ratio)   (VESC: 6 шагов на электрический оборот)

Конвенция REP-103: linear.x>0 = вперёд, angular.z>0 = поворот влево (CCW) → правый борт быстрее.
Инверсию бортов задают флаги invert (калибруются на первом пуске). TF odom→base_footprint по умолчанию
НЕ публикуем — его строит EKF; /odom скармливается в EKF как источник колёсной скорости.
"""
from __future__ import annotations

import glob
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

# --- VESC binary UART protocol -------------------------------------------------
COMM_GET_VALUES = 4
COMM_SET_DUTY = 5
COMM_SET_CURRENT = 6
COMM_SET_RPM = 8


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


class _VescLink:
    """Сериал-канал к ОДНОЙ половине VESC (USB-CDC). Прямые команды/опрос, без forward-CAN."""

    def __init__(self, port: str, baud: int = 115200):
        import serial  # pyserial
        self.ser = serial.Serial(port, baud, timeout=0.02)
        self.port = port
        self._buf = bytearray()

    def close(self) -> None:
        try:
            self.ser.close()
        except Exception:  # noqa: BLE001
            pass

    def set_rpm(self, erpm: float) -> None:
        self.ser.write(_frame(bytes([COMM_SET_RPM]) + struct.pack(">i", int(erpm))))

    def set_duty(self, duty: float) -> None:
        self.ser.write(_frame(bytes([COMM_SET_DUTY]) + struct.pack(">i", int(duty * 100000.0))))

    def set_current(self, amps: float) -> None:
        # amps=0 → мотор свободен (выбег)
        self.ser.write(_frame(bytes([COMM_SET_CURRENT]) + struct.pack(">i", int(amps * 1000.0))))

    def request_values(self) -> None:
        self.ser.write(_frame(bytes([COMM_GET_VALUES])))

    def _extract(self) -> bytes | None:
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
                buf.pop(0)
                continue
            total = hdr + plen + 3
            if len(buf) < total:
                return None
            payload = bytes(buf[hdr:hdr + plen])
            crc_rx = (buf[hdr + plen] << 8) | buf[hdr + plen + 1]
            end = buf[hdr + plen + 2]
            del buf[:total]
            if end == 3 and _crc16(payload) == crc_rx:
                return payload
        return None

    def _recv(self, timeout: float) -> bytes | None:
        """Прочитать один валидный кадр GET_VALUES-ответа (payload)."""
        deadline = time.monotonic() + timeout
        while True:
            payload = self._extract()
            if payload is not None and payload and payload[0] == COMM_GET_VALUES:
                return payload
            if time.monotonic() >= deadline:
                return None
            chunk = self.ser.read(128)
            if chunk:
                self._buf.extend(chunk)

    def probe_id(self, timeout: float = 0.5) -> int | None:
        """VESC-id (controller_id) этой половины — чтобы понять, где какая. None если не ответила."""
        self.request_values()
        payload = self._recv(timeout)
        if payload is not None and len(payload) > 58:
            return int(payload[58])            # controller_id: data[57] = payload[58]
        return None

    def read_values(self, timeout: float) -> tuple[float, int] | None:
        """(erpm, tachometer) из ответа GET_VALUES. None при таймауте."""
        self.request_values()
        payload = self._recv(timeout)
        if payload is not None and len(payload) >= 49:
            erpm = struct.unpack_from(">i", payload, 23)[0]      # data[22:26]
            tach = struct.unpack_from(">i", payload, 45)[0]      # data[44:48]
            return float(erpm), int(tach)
        return None


class VescDiffDrive(Node):
    def __init__(self) -> None:
        super().__init__("vesc_diff_drive")

        try:
            cfg = load_robot_config()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"нет robot.yaml ({exc}) — дефолты")
            cfg = {}
        dt = cfg.get("drivetrain", {}) if cfg else {}
        left = dt.get("left", {})
        right = dt.get("right", {})

        self.declare_parameter("ports", list(dt.get("ports", ["/dev/ttyACM0", "/dev/ttyACM1"])))
        self.declare_parameter("baud", 115200)
        self.declare_parameter("wheel_radius", float(cfg.get("wheel_radius", 0.0825)))
        self.declare_parameter("track_width", float(cfg.get("wheel_base", 0.50)))
        self.declare_parameter("pole_pairs", int(dt.get("pole_pairs", 15)))
        self.declare_parameter("gear_ratio", float(dt.get("gear_ratio", 1.0)))
        self.declare_parameter("max_speed", float(dt.get("max_speed", 1.0)))
        self.declare_parameter("accel_limit", float(dt.get("accel_limit", 1.2)))
        # DUTY-управление: SET_RPM не стартует ниже ~1000 eRPM (стикция), а duty трогается с низов.
        # duty ≈ duty_per_ms · скорость[м/с] (из замера: duty 0.08 → 501 eRPM ≈ 0.29 м/с). Под нагрузкой
        # скорость плывёт, но одометрия по тахометру точная. max_duty — потолок.
        self.declare_parameter("duty_per_ms", float(dt.get("duty_per_ms", 0.28)))
        self.declare_parameter("max_duty", float(dt.get("max_duty", 0.85)))
        # Замкнутый контур скорости (ток): держит ОБЕ половины на заданной скорости независимо от
        # разницы моторов/нагрузки → едет прямо и с любой малой скорости. ток = kff·v + kp·err + ki·∫err.
        self.declare_parameter("speed_kp", float(dt.get("speed_kp", 12.0)))
        self.declare_parameter("speed_ki", float(dt.get("speed_ki", 20.0)))
        self.declare_parameter("speed_kff", float(dt.get("speed_kff", 4.0)))
        self.declare_parameter("max_current", float(dt.get("max_current", 15.0)))
        self.declare_parameter("cmd_timeout", float(dt.get("cmd_timeout", 0.4)))
        self.declare_parameter("rate_hz", float(dt.get("rate_hz", 50.0)))
        self.declare_parameter("left_vesc_id", int(left.get("vesc_id", 52)))
        self.declare_parameter("right_vesc_id", int(right.get("vesc_id", 4)))
        self.declare_parameter("left_invert", bool(left.get("invert", False)))
        self.declare_parameter("right_invert", bool(right.get("invert", True)))
        self.declare_parameter("publish_tf", False)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")

        gp = self.get_parameter
        ports = list(gp("ports").value)
        self.baud = int(gp("baud").value)
        self.r = float(gp("wheel_radius").value)
        self.track = float(gp("track_width").value)
        self.pp = int(gp("pole_pairs").value)
        self.gear = float(gp("gear_ratio").value)
        self.max_speed = float(gp("max_speed").value)
        self.accel = float(gp("accel_limit").value)
        self.duty_per_ms = float(gp("duty_per_ms").value)
        self.max_duty = float(gp("max_duty").value)
        self.kp = float(gp("speed_kp").value)
        self.ki = float(gp("speed_ki").value)
        self.kff = float(gp("speed_kff").value)
        self.max_current = float(gp("max_current").value)
        self._imax = self.max_current / self.ki if self.ki > 0 else 0.0   # анти-виндап интеграла
        self.cmd_timeout = float(gp("cmd_timeout").value)
        self.rate = max(float(gp("rate_hz").value), 1.0)
        left_id = int(gp("left_vesc_id").value)
        right_id = int(gp("right_vesc_id").value)
        self.sign_l = -1.0 if bool(gp("left_invert").value) else 1.0
        self.sign_r = -1.0 if bool(gp("right_invert").value) else 1.0
        self.publish_tf = bool(gp("publish_tf").value)
        self.odom_frame = str(gp("odom_frame").value)
        self.base_frame = str(gp("base_frame").value)

        self._k_erpm = 60.0 * self.pp * self.gear / (2.0 * math.pi * self.r)
        self.max_erpm = self._k_erpm * self.max_speed
        self.m_per_count = 2.0 * math.pi * self.r / (6.0 * self.pp * self.gear)
        self._read_to = 0.03

        # --- открыть все порты, определить где какая половина по VESC-id ---
        if not ports:
            ports = sorted(glob.glob("/dev/ttyACM*"))
        found: dict[int, _VescLink] = {}
        for port in ports:
            try:
                lk = _VescLink(port, self.baud)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f"{port}: не открыть ({exc})")
                continue
            cid = lk.probe_id()
            if cid is None:
                self.get_logger().warn(f"{port}: VESC не ответил — пропуск")
                lk.close()
                continue
            found[cid] = lk
            self.get_logger().info(f"{port}: VESC id={cid}")
        self.link_l = found.pop(left_id, None)
        self.link_r = found.pop(right_id, None)
        for lk in found.values():        # лишние (не левый/правый) — закрыть
            lk.close()
        if self.link_l is None:
            self.get_logger().error(f"левая половина (id={left_id}) не найдена!")
        if self.link_r is None:
            self.get_logger().error(f"правая половина (id={right_id}) не найдена!")

        # --- состояние ---
        self._lock = threading.Lock()
        self._tv = self._tw = 0.0
        self._last_cmd = 0.0
        self._cvl = self._cvr = 0.0
        self._mvl = self._mvr = 0.0        # измеренная скорость колёс, м/с (для PI)
        self._int_l = self._int_r = 0.0    # интегралы тока
        self.x = self.y = self.th = 0.0
        self._tach_l: int | None = None
        self._tach_r: int | None = None

        # --- ROS ---
        self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        self.tf_bc = TransformBroadcaster(self) if self.publish_tf else None
        self.create_subscription(Twist, "cmd_vel", self._on_cmd, 10)

        self._alive = True
        self._io = threading.Thread(target=self._run, daemon=True)
        self._io.start()
        self.get_logger().info(
            f"vesc_diff_drive: r={self.r:.3f}м колея={self.track:.3f}м pp={self.pp} "
            f"L=id{left_id}/inv{self.sign_l<0}{'' if self.link_l else '(НЕТ)'} "
            f"R=id{right_id}/inv{self.sign_r<0}{'' if self.link_r else '(НЕТ)'} "
            f"max={self.max_speed}м/с(~{self.max_erpm:.0f}ERPM) tf={self.publish_tf}"
        )

    # ------------------------------------------------------------------ ROS cb
    def _on_cmd(self, msg: Twist) -> None:
        with self._lock:
            self._tv = max(-self.max_speed, min(self.max_speed, msg.linear.x))
            self._tw = msg.angular.z
            self._last_cmd = time.monotonic()
        self.get_logger().debug(
            f"RX cmd_vel v={msg.linear.x:.2f} w={msg.angular.z:.2f}", throttle_duration_sec=1.0)

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

            tvl, tvr = tv - tw * self.track / 2.0, tv + tw * self.track / 2.0
            step = self.accel * dt
            self._cvl = self._ramp(self._cvl, tvl, step)
            self._cvr = self._ramp(self._cvr, tvr, step)

            self._odom_step(dt)      # сначала читаем скорость колёс...
            self._drive(dt)          # ...потом PI по току на её основе

            sleep = period - (time.monotonic() - t0)
            if sleep > 0:
                time.sleep(sleep)

    def _pi(self, target: float, measured: float, integ: float, dt: float) -> tuple[float, float]:
        """PI по скорости колеса → ток (в кадре робота, + = вперёд). Возвращает (ток, новый интеграл)."""
        err = target - measured
        integ = max(-self._imax, min(self._imax, integ + err * dt))
        cur = self.kff * target + self.kp * err + self.ki * integ
        return max(-self.max_current, min(self.max_current, cur)), integ

    def _drive(self, dt: float) -> None:
        # Цель ~0 (стоп/таймаут, после рампы) → ОТПУСКАЕМ ток = выбег, не тормозим PI.
        # Тормозить свободное колесо в ноль нельзя: высоко-Kv мотор без нагрузки уходит в
        # автоколебания (PI качает ±ток). На полу трение само гасит; на весу — просто катится.
        idle = abs(self._cvl) < 1e-3 and abs(self._cvr) < 1e-3
        try:
            if idle:
                self._int_l = self._int_r = 0.0
                if self.link_l:
                    self.link_l.set_current(0.0)
                if self.link_r:
                    self.link_r.set_current(0.0)
                return
            cl, self._int_l = self._pi(self._cvl, self._mvl, self._int_l, dt)
            cr, self._int_r = self._pi(self._cvr, self._mvr, self._int_r, dt)
            if self.link_l:
                self.link_l.set_current(cl * self.sign_l)     # ток в кадр мотора
            if self.link_r:
                self.link_r.set_current(cr * self.sign_r)
            self.get_logger().debug(
                f"DRIVE cl={cl:.1f} cr={cr:.1f}A tgt={self._cvl:.2f}/{self._cvr:.2f} meas={self._mvl:.2f}/{self._mvr:.2f}",
                throttle_duration_sec=1.0)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"VESC write: {exc}", throttle_duration_sec=2.0)

    def _odom_step(self, dt: float) -> None:
        if self.link_l is None or self.link_r is None:
            return
        try:
            vl = self.link_l.read_values(self._read_to)
            vr = self.link_r.read_values(self._read_to)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"VESC read: {exc}", throttle_duration_sec=2.0)
            return
        if vl is None or vr is None:
            return

        erpm_l, tach_l = vl
        erpm_r, tach_r = vr
        mvl = self._speed(erpm_l) * self.sign_l
        mvr = self._speed(erpm_r) * self.sign_r
        a = 0.35                               # ФНЧ на измеренную скорость → глаже PI (меньше рывков от шума eRPM)
        self._mvl = a * mvl + (1.0 - a) * self._mvl
        self._mvr = a * mvr + (1.0 - a) * self._mvr
        vx = (mvl + mvr) / 2.0                 # /odom — по СЫРОЙ скорости (точность одометрии)
        wz = (mvr - mvl) / self.track
        self.get_logger().debug(
            f"READ Lerpm={erpm_l:.0f} Rerpm={erpm_r:.0f} vx={vx:.2f} wz={wz:.2f}",
            throttle_duration_sec=1.0)

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
        od.pose.covariance[0] = od.pose.covariance[7] = 0.002
        od.pose.covariance[35] = 0.01
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
        for lk in (self.link_l, self.link_r):
            if lk is not None:
                try:
                    lk.set_current(0.0)      # отпустить моторы
                except Exception:  # noqa: BLE001
                    pass
                lk.close()
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
