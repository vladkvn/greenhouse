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
COMM_SET_CURRENT_BRAKE = 7
COMM_SET_RPM = 8
COMM_FORWARD_CAN = 34  # [34, can_id, <inner comm...>] → команда/опрос половины по CAN (X5100: один USB)


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


class _SerialBus:
    """Один USB-CDC порт. Поддерживает и старую схему (одна половина на порт), и X5100
    (ОДИН USB на обе половины: local напрямую + вторая через forward-CAN). Кадры GET_VALUES
    раскладываем по controller_id (payload[58]) → каждая половина забирает свой."""

    def __init__(self, port: str, baud: int = 115200):
        import serial  # pyserial
        self.ser = serial.Serial(port, baud, timeout=0.02)
        self.port = port
        self._buf = bytearray()
        self.latest: dict[int, tuple[float, int]] = {}   # cid -> (erpm, tach), свежий кадр

    def close(self) -> None:
        try:
            self.ser.close()
        except Exception:  # noqa: BLE001
            pass

    def write(self, payload: bytes) -> None:
        self.ser.write(_frame(payload))

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

    def pump(self) -> None:
        """Дочитать байты и разложить все свежие GET_VALUES-кадры по controller_id."""
        n = self.ser.in_waiting
        if n:
            self._buf.extend(self.ser.read(n))
        while True:
            p = self._extract()
            if p is None:
                break
            if p and p[0] == COMM_GET_VALUES and len(p) >= 59:
                cid = p[58]                                       # controller_id
                erpm = struct.unpack_from(">i", p, 23)[0]         # data[22:26]
                tach = struct.unpack_from(">i", p, 45)[0]         # data[44:48]
                self.latest[cid] = (float(erpm), int(tach))

    def probe_local(self, timeout: float = 0.5) -> int | None:
        """controller_id половины, что отвечает НАПРЯМУЮ (local) на этом порту."""
        self.latest.clear()
        self.write(bytes([COMM_GET_VALUES]))
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.pump()
            if self.latest:
                return next(iter(self.latest))
            time.sleep(0.01)
        return None

    def probe_forward(self, can_id: int, timeout: float = 0.5) -> bool:
        """Достаётся ли половина can_id через forward-CAN на этом порту."""
        self.write(bytes([COMM_FORWARD_CAN, can_id, COMM_GET_VALUES]))
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.pump()
            if can_id in self.latest:
                return True
            time.sleep(0.01)
        return False


class _VescLink:
    """Логическая половина на общем _SerialBus. can_id=None → local (прямые команды),
    иначе — обёртка COMM_FORWARD_CAN. Ответы берём из bus.latest по своему cid."""

    def __init__(self, bus: _SerialBus, cid: int, can_id: int | None = None):
        self.bus = bus
        self.cid = cid
        self.can_id = can_id

    def _wrap(self, inner: bytes) -> bytes:
        return inner if self.can_id is None else bytes([COMM_FORWARD_CAN, self.can_id]) + inner

    def set_rpm(self, erpm: float) -> None:
        self.bus.write(self._wrap(bytes([COMM_SET_RPM]) + struct.pack(">i", int(erpm))))

    def set_duty(self, duty: float) -> None:
        self.bus.write(self._wrap(bytes([COMM_SET_DUTY]) + struct.pack(">i", int(duty * 100000.0))))

    def set_current(self, amps: float) -> None:
        # amps=0 → мотор свободен (выбег)
        self.bus.write(self._wrap(bytes([COMM_SET_CURRENT]) + struct.pack(">i", int(amps * 1000.0))))

    def set_current_brake(self, amps: float) -> None:
        # штатный тормоз VESC: тормозит мотор к нулю и держит (знак не важен). amps=0 → отпустить.
        self.bus.write(self._wrap(bytes([COMM_SET_CURRENT_BRAKE]) + struct.pack(">i", int(abs(amps) * 1000.0))))

    def request_values(self) -> None:
        self.bus.write(self._wrap(bytes([COMM_GET_VALUES])))

    def poll(self) -> tuple[float, int] | None:
        """НЕблокирующе: подтянуть кадры и вернуть СВЕЖИЙ (erpm,tach) своей половины. pop — только
        новый кадр, иначе None и вызывающий держит прежнее. Контур не ждёт serial → 50 Гц."""
        self.bus.pump()
        return self.bus.latest.pop(self.cid, None)

    def close(self) -> None:
        pass   # порт закрывает _SerialBus (общий на обе половины)


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
        # Активный тормоз на стоп: ток = -kbrake·измеренная скорость (демпфер) → встаёт где сказано,
        # не докатывается по инерции. Слю-лимит: выходной ток меняется не быстрее max_current за slew_s
        # секунд → нет рывка/слема на срыве стикции.
        self.declare_parameter("brake_a", float(dt.get("brake_a", 4.0)))
        self.declare_parameter("current_slew_s", float(dt.get("current_slew_s", 0.12)))
        # Срыв стикции: пока колесо СТОИТ (|скорость|<stiction_v), а команда есть — форсировать
        # ток stiction_a в сторону цели (скид-стир: старт против скраб-трения требует толчка, PI
        # копит его слишком долго → «мёртвый» старт). Как только поехало — обычный PI.
        self.declare_parameter("stiction_a", float(dt.get("stiction_a", 3.5)))
        self.declare_parameter("stiction_v", float(dt.get("stiction_v", 0.05)))
        # Связка бортов по курсу: держим заданную РАЗНИЦУ скоростей (mvr-mvl). Правый тяжелее срывается
        # из покоя → без связки борта стартуют вразнобой и робот уводит. Связка добавляет ток отстающему
        # и придерживает опережающий → едет ровно (и прямая, и заданный поворот). А·с/м на ошибку курса.
        self.declare_parameter("heading_ksync", float(dt.get("heading_ksync", 8.0)))
        self.declare_parameter("cmd_timeout", float(dt.get("cmd_timeout", 0.4)))
        self.declare_parameter("rate_hz", float(dt.get("rate_hz", 50.0)))
        # Режим управления: "rpm" — замкнутый контур скорости ВНУТРИ VESC (SET_RPM): прошивка держит
        # обороты своим быстрым локальным контуром → нагрузко-инвариантно и не зависит от лага forward-CAN.
        # "current" — наш PI по току (старый). rpm_min_erpm: пол |eRPM| в rpm-режиме (0=выкл), если
        # прошивка не держит совсем низкие обороты.
        self.declare_parameter("control_mode", str(dt.get("control_mode", "rpm")))
        self.declare_parameter("rpm_min_erpm", float(dt.get("rpm_min_erpm", 0.0)))
        # Детект застревания: колесо под командой (|цель|>0.03 м/с), но НЕ крутится (|изм.eRPM|<stall_erpm)
        # дольше stall_time_s → снять тягу на stall_hold_s (не качать ток, не сорваться рывком). rpm-режим:
        # прошивка иначе льёт ток до своего лимита — это единственная защита. Порог по времени > времени
        # нормального срыва старта, чтобы не ловить на трогании.
        self.declare_parameter("stall_detect", bool(dt.get("stall_detect", True)))
        self.declare_parameter("stall_erpm", float(dt.get("stall_erpm", 40.0)))
        self.declare_parameter("stall_time_s", float(dt.get("stall_time_s", 0.7)))
        self.declare_parameter("stall_hold_s", float(dt.get("stall_hold_s", 1.0)))
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
        self.use_rpm = str(gp("control_mode").value).strip().lower() == "rpm"
        self.rpm_min = float(gp("rpm_min_erpm").value)
        self.stall_detect = bool(gp("stall_detect").value)
        self.stall_erpm = float(gp("stall_erpm").value)
        self.stall_time_s = float(gp("stall_time_s").value)
        self.stall_hold_s = float(gp("stall_hold_s").value)
        self.duty_per_ms = float(gp("duty_per_ms").value)
        self.max_duty = float(gp("max_duty").value)
        self.kp = float(gp("speed_kp").value)
        self.ki = float(gp("speed_ki").value)
        self.kff = float(gp("speed_kff").value)
        self.max_current = float(gp("max_current").value)
        self._imax = self.max_current / self.ki if self.ki > 0 else 0.0   # анти-виндап интеграла
        self.brake_a = float(gp("brake_a").value)
        self.slew_s = max(float(gp("current_slew_s").value), 1e-3)
        self.stiction_a = float(gp("stiction_a").value)
        self.stiction_v = float(gp("stiction_v").value)
        self.ksync = float(gp("heading_ksync").value)
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

        # --- открыть порт(ы) и найти половины ---
        # X5100: ОДИН USB на обе половины (local напрямую + вторая через forward-CAN). Также
        # работает старая схема (две USB, по половине на порт). Половину находим по её VESC-id:
        # если id отвечает напрямую (local) — прямой канал, иначе — обёртка forward-CAN.
        if not ports:
            ports = sorted(glob.glob("/dev/ttyACM*"))
        self._buses: list[_SerialBus] = []
        local_of: dict[int, _SerialBus] = {}
        for port in ports:
            try:
                bus = _SerialBus(port, self.baud)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f"{port}: не открыть ({exc})")
                continue
            self._buses.append(bus)
            cid = bus.probe_local()
            if cid is None:
                self.get_logger().warn(f"{port}: VESC не ответил")
                continue
            local_of[cid] = bus
            self.get_logger().info(f"{port}: local VESC id={cid}")

        def _mk(target_id: int) -> _VescLink | None:
            bus = local_of.get(target_id)
            if bus is not None:
                self.get_logger().info(f"id={target_id}: прямой (local) на {bus.port}")
                return _VescLink(bus, target_id, can_id=None)
            for b in self._buses:                       # не local → искать через forward-CAN
                if b.probe_forward(target_id):
                    self.get_logger().info(f"id={target_id}: через forward-CAN на {b.port}")
                    return _VescLink(b, target_id, can_id=target_id)
            return None

        self.link_l = _mk(left_id)
        self.link_r = _mk(right_id)
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
        self._cl_prev = self._cr_prev = 0.0  # прошлый выходной ток (кадр робота) для слю-лимита
        self._moving_l = self._moving_r = False  # защёлка «колесо катится» → срыв-стикции только со старта
        self.x = self.y = self.th = 0.0
        self._tach_l: int | None = None
        self._tach_r: int | None = None
        self._stall_t0: float | None = None   # когда началось состояние «стоит под командой»
        self._stall_hold_until = 0.0          # до этого времени держим тягу снятой (кулдаун после stall)

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

    @staticmethod
    def _slew(prev: float, target: float, dcap: float) -> float:
        """Ограничить скорость изменения выходного тока: не больше dcap за цикл."""
        d = target - prev
        if abs(d) <= dcap:
            return target
        return prev + math.copysign(dcap, d)

    # ------------------------------------------------------------------ IO loop
    def _run(self) -> None:
        period = 1.0 / self.rate
        prev = time.monotonic()
        for lk in (self.link_l, self.link_r):     # прайм: запросить значения к 1-му циклу
            if lk:
                try:
                    lk.request_values()
                except Exception:  # noqa: BLE001
                    pass
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

            self._odom_poll()        # неблокирующе подхватить свежий eRPM/tach (ответ на прошлый запрос)
            self._drive(dt)          # PI/тормоз + слю-лимит → ток
            for lk in (self.link_l, self.link_r):   # запросить значения к следующему циклу
                if lk:
                    try:
                        lk.request_values()
                    except Exception:  # noqa: BLE001
                        pass

            sleep = period - (time.monotonic() - t0)
            if sleep > 0:
                time.sleep(sleep)

    def _pi(self, target: float, measured: float, integ: float, dt: float) -> tuple[float, float]:
        """PI по скорости колеса → ток (в кадре робота, + = вперёд). Анти-виндап: не копим интеграл,
        когда выход уже в насыщении, а ошибка тянет ГЛУБЖЕ в насыщение (иначе перекрут на срыве стикции)."""
        err = target - measured
        base = self.kff * target + self.kp * err
        cur_unsat = base + self.ki * integ
        saturated_deeper = abs(cur_unsat) >= self.max_current and (cur_unsat > 0) == (err > 0)
        if not saturated_deeper:
            integ = max(-self._imax, min(self._imax, integ + err * dt))
        cur = base + self.ki * integ
        return max(-self.max_current, min(self.max_current, cur)), integ

    @staticmethod
    def _latch_moving(moving: bool, measured: float) -> bool:
        """«Колесо поехало»: раскрутилось >0.08 → True и ДЕРЖИМ до полной остановки (её сбросит только
        тормозная ветка). В езде не расцепляем — иначе провал лимит-цикла к нулю перезапускает срыв-
        стикции (пинок 3.5А посреди хода) → дёрганье. Срыв нужен РОВНО один раз со старта."""
        return moving or abs(measured) > 0.08

    def _stiction(self, target: float, measured: float, cur: float, moving: bool) -> float:
        """Колесо СТОИТ (не катится) и команда есть → толчок stiction_a в сторону цели (срыв трения
        старта). Уже катится (защёлка moving) — не вмешиваемся, чтобы на прямой не было дёрганья.
        stiction_a<=0 → выключено (на этом шасси срыв колёса рвал вразнобой → увод на прямой)."""
        if self.stiction_a <= 0.0:
            return cur
        if not moving and abs(target) > 0.03 and abs(measured) < self.stiction_v:
            floor = math.copysign(self.stiction_a, target)
            if abs(cur) < self.stiction_a or (cur > 0) != (target > 0):
                return floor
        return cur

    def _brake(self, link, measured) -> None:
        """Штатный тормоз VESC к нулю; почти стоит → отпустить (не держать, не греть)."""
        if link is None:
            return
        if abs(measured) < 0.03:
            link.set_current(0.0)
        else:
            link.set_current_brake(self.brake_a)

    def _drive_rpm(self) -> None:
        """RPM-режим: замкнутый контур скорости ВНУТРИ VESC. Шлём целевой eRPM (из рампленной цели
        _cvl/_cvr), прошивка держит обороты быстрым локальным контуром → нагрузко-инвариантно и без
        опоры на лаговый forward-CAN фидбэк (телеметрию читаем только для одометрии). Стоп → отпустить."""
        braking = abs(self._cvl) < 1e-3 and abs(self._cvr) < 1e-3
        try:
            if braking:
                self._brake(self.link_l, self._mvl)
                self._brake(self.link_r, self._mvr)
                return
            erpml = self._erpm(self._cvl) * self.sign_l
            erpmr = self._erpm(self._cvr) * self.sign_r
            if self.rpm_min > 0.0:                       # пол оборотов, если прошивка не держит низы
                if 0.0 < abs(erpml) < self.rpm_min:
                    erpml = math.copysign(self.rpm_min, erpml)
                if 0.0 < abs(erpmr) < self.rpm_min:
                    erpmr = math.copysign(self.rpm_min, erpmr)
            if self.link_l:
                self.link_l.set_rpm(erpml)
            if self.link_r:
                self.link_r.set_rpm(erpmr)
            self.get_logger().debug(
                f"DRIVE RPM L={erpml:.0f} R={erpmr:.0f} meas={self._mvl:.2f}/{self._mvr:.2f}",
                throttle_duration_sec=0.5)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"VESC set_rpm: {exc}", throttle_duration_sec=2.0)

    def _stall_guard(self) -> bool:
        """True → колесо застряло под командой: снять тягу (не качать ток, не сорваться рывком).
        Работает в ОБОИХ режимах (проверяется до диспатча). В rpm-режиме это единственная защита —
        прошивка VESC иначе льёт ток в стоящее колесо до своего лимита. Кулдаун stall_hold_s, потом
        повторная попытка (за это время Nav2 recovery/backup успеет вмешаться и сдёрнуть с места)."""
        if not self.stall_detect:
            return False
        now = time.monotonic()
        if now < self._stall_hold_until:            # кулдаун — держим отпущенным
            self._brake(self.link_l, self._mvl)
            self._brake(self.link_r, self._mvr)
            return True
        stall_v = self.stall_erpm / self._k_erpm    # м/с, эквивалент порога eRPM «не крутится»
        stuck_l = abs(self._cvl) > 0.03 and abs(self._mvl) < stall_v
        stuck_r = abs(self._cvr) > 0.03 and abs(self._mvr) < stall_v
        if stuck_l or stuck_r:
            if self._stall_t0 is None:
                self._stall_t0 = now
            elif now - self._stall_t0 > self.stall_time_s:
                self.get_logger().warn(
                    f"STALL: колесо стоит под командой (L={stuck_l}/R={stuck_r}) → снимаю тягу на "
                    f"{self.stall_hold_s:.1f}с", throttle_duration_sec=2.0)
                self._stall_hold_until = now + self.stall_hold_s
                self._stall_t0 = None
                self._brake(self.link_l, self._mvl)
                self._brake(self.link_r, self._mvr)
                return True
        else:
            self._stall_t0 = None
        return False

    def _drive(self, dt: float) -> None:
        if self._stall_guard():          # застревание → снять тягу, не качать ток (см. _stall_guard)
            return
        if self.use_rpm:                 # замкнутый контур внутри VESC (см. _drive_rpm)
            self._drive_rpm()
            return
        # Цель ~0 (стоп/таймаут после рампы) → ШТАТНЫЙ ТОРМОЗ VESC к нулю: встаёт где сказано, не
        # докатывается по инерции. Иначе — PI по току + срыв-стикции + слю-лимит (плавный разгон).
        braking = abs(self._cvl) < 1e-3 and abs(self._cvr) < 1e-3
        try:
            if braking:
                self._int_l = self._int_r = 0.0
                self._cl_prev = self._cr_prev = 0.0    # слю-состояние сброшено к следующему разгону
                self._moving_l = self._moving_r = False   # встали → срыв снова вооружён
                self._brake(self.link_l, self._mvl)
                self._brake(self.link_r, self._mvr)
                self.get_logger().debug(
                    f"DRIVE BRK meas={self._mvl:.2f}/{self._mvr:.2f}", throttle_duration_sec=0.5)
                return
            dcap = self.max_current * dt / self.slew_s    # потолок изменения тока за цикл (слю-лимит)
            self._moving_l = self._latch_moving(self._moving_l, self._mvl)
            self._moving_r = self._latch_moving(self._moving_r, self._mvr)
            cl, self._int_l = self._pi(self._cvl, self._mvl, self._int_l, dt)
            cr, self._int_r = self._pi(self._cvr, self._mvr, self._int_r, dt)
            cl = self._stiction(self._cvl, self._mvl, cl, self._moving_l)   # толчок только со старта
            cr = self._stiction(self._cvr, self._mvr, cr, self._moving_r)
            # связка бортов по курсу: держать заданную разницу скоростей → не уводит от асимметрии старта
            sync = self.ksync * ((self._cvr - self._cvl) - (self._mvr - self._mvl))
            cl = max(-self.max_current, min(self.max_current, cl - sync))
            cr = max(-self.max_current, min(self.max_current, cr + sync))
            cl = self._slew(self._cl_prev, cl, dcap)      # без скачка тока → без рывка/слема
            cr = self._slew(self._cr_prev, cr, dcap)
            self._cl_prev, self._cr_prev = cl, cr
            if self.link_l:
                self.link_l.set_current(cl * self.sign_l)     # ток в кадр мотора
            if self.link_r:
                self.link_r.set_current(cr * self.sign_r)
            self.get_logger().debug(
                f"DRIVE PI  cl={cl:.1f} cr={cr:.1f}A "
                f"tgt={self._cvl:.2f}/{self._cvr:.2f} meas={self._mvl:.2f}/{self._mvr:.2f}",
                throttle_duration_sec=0.5)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"VESC write: {exc}", throttle_duration_sec=2.0)

    def _odom_poll(self) -> None:
        """Неблокирующе: обновить скорость по борту, где пришёл свежий кадр; одометрию/публикацию —
        когда пришли ОБА (иначе пропуск цикла, не критично). Контур не ждёт serial."""
        if self.link_l is None or self.link_r is None:
            return
        try:
            vl = self.link_l.poll()
            vr = self.link_r.poll()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"VESC read: {exc}", throttle_duration_sec=2.0)
            return

        a = 0.25                               # ФНЧ на измеренную скорость → глаже PI (меньше шума eRPM)
        mvl_raw = mvr_raw = None
        tach_l = tach_r = None
        if vl is not None:
            erpm_l, tach_l = vl
            mvl_raw = self._speed(erpm_l) * self.sign_l
            self._mvl = a * mvl_raw + (1.0 - a) * self._mvl
        if vr is not None:
            erpm_r, tach_r = vr
            mvr_raw = self._speed(erpm_r) * self.sign_r
            self._mvr = a * mvr_raw + (1.0 - a) * self._mvr
        if tach_l is None or tach_r is None:
            return                             # нет свежей пары — одометрию не двигаем

        vx = (mvl_raw + mvr_raw) / 2.0         # /odom twist — по СЫРОЙ скорости (точность)
        wz = (mvr_raw - mvl_raw) / self.track
        self.get_logger().debug(
            f"READ vx={vx:.2f} wz={wz:.2f} mL={self._mvl:.2f} mR={self._mvr:.2f}",
            throttle_duration_sec=0.5)

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
        for bus in getattr(self, "_buses", []):   # порт(ы) закрываем на уровне шины
            bus.close()
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
