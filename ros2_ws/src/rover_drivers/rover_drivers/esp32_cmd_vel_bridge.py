#!/usr/bin/env python3
"""cmd_vel → ESP32 skid-steer bridge.

Подписывается на geometry_msgs/Twist (`cmd_vel`), переводит (vx, wz) в ШИМ
двух бортов skid-steer 4WD-шасси и шлёт "L R\\n" / "STOP\\n" на ESP32 по
USB-serial (115200, CH340) — тот же протокол, что у боевой прошивки follow_me.

Failsafe повторяет не-ROS стек:
  * heartbeat — пере-отправка последнего ШИМ с частотой `heartbeat_hz`, чтобы
    собственный watchdog ESP32 (0.5 с) не дёргал моторы при нормальной езде;
  * deadman — нет cmd_vel дольше `cmd_timeout` секунд → STOP;
  * slew-rate — ограничение приращения ШИМ за тик (плавный старт/стоп для
    разомкнутого привода), зеркалит motor_ramp_step боевого кода.

Конвенция (REP-103): linear.x > 0 = вперёд, angular.z > 0 = поворот влево (CCW),
тогда правый борт быстрее левого.
"""
from __future__ import annotations

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


def twist_to_lr(
    vx: float,
    wz: float,
    *,
    max_linear: float,
    max_angular: float,
    pwm_max: int,
    pwm_min_move: int,
    turn_gain: float = 1.0,
) -> tuple[int, int]:
    """(v, w) → ШИМ левого/правого бортов (skid-steer), РАЗДЕЛЬНЫЙ масштаб linear/angular.

    Прежняя kinematic-модель через wheel_base давала на поворот почти нулевой ШИМ (робот
    жужжал и не проворачивался — skid-steer требует высокого ШИМ для проворота на месте).
    Здесь angular масштабируется независимо: при |w|=max_angular борт получает pwm_max·turn_gain.
    Конвенция: w>0 = поворот влево (CCW) → правый борт вперёд.
    """
    k_lin = pwm_max / max_linear if max_linear > 0 else 0.0
    k_ang = pwm_max / max_angular if max_angular > 0 else 0.0
    lin = vx * k_lin
    ang = wz * k_ang * turn_gain
    return _to_pwm(lin - ang, pwm_max, pwm_min_move), _to_pwm(lin + ang, pwm_max, pwm_min_move)


def _to_pwm(value: float, pwm_max: int, pwm_min_move: int) -> int:
    s = int(round(value))
    if s == 0:
        return 0
    sign = 1 if s > 0 else -1
    return sign * max(pwm_min_move, min(pwm_max, abs(s)))


def _ramp(current: int, target: int, step: int) -> int:
    if step <= 0 or abs(target - current) <= step:
        return target
    return current + step if target > current else current - step


class Esp32CmdVelBridge(Node):
    def __init__(self) -> None:
        super().__init__("esp32_cmd_vel_bridge")
        # --- параметры (калибруются под шасси; дефолты из follow_me/config.py) ---
        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("wheel_base", 0.18)        # колея, м
        self.declare_parameter("max_linear", 0.4)          # м/с при pwm_max
        self.declare_parameter("max_angular", 1.5)         # рад/с потолок
        self.declare_parameter("turn_gain", 1.0)           # усиление поворота (skid-steer)
        self.declare_parameter("pwm_max", 220)             # потолок ШИМ на борт
        self.declare_parameter("pwm_min_move", 120)        # ниже — мотор не крутится
        self.declare_parameter("cmd_timeout", 0.4)         # тишина → STOP, с
        self.declare_parameter("heartbeat_hz", 15.0)       # частота пере-отправки
        self.declare_parameter("max_pwm_step", 20)         # slew-rate за тик (0=выкл)
        self.declare_parameter("settle_s", 2.0)            # пауза после open (сброс ESP)
        self.declare_parameter("swap_sides", False)        # поменять L↔R при ошибке монтажа

        self.port = self.get_parameter("serial_port").value
        self.baud = int(self.get_parameter("baud").value)
        self.wheel_base = float(self.get_parameter("wheel_base").value)
        self.max_linear = float(self.get_parameter("max_linear").value)
        self.max_angular = float(self.get_parameter("max_angular").value)
        self.turn_gain = float(self.get_parameter("turn_gain").value)
        self.pwm_max = int(self.get_parameter("pwm_max").value)
        self.pwm_min_move = int(self.get_parameter("pwm_min_move").value)
        self.cmd_timeout = float(self.get_parameter("cmd_timeout").value)
        hz = float(self.get_parameter("heartbeat_hz").value)
        self.max_pwm_step = int(self.get_parameter("max_pwm_step").value)
        self.swap_sides = bool(self.get_parameter("swap_sides").value)

        self._ser = self._open_serial()
        self._target_lr: tuple[int, int] = (0, 0)
        self._sent_lr: tuple[int, int] = (0, 0)
        self._last_cmd_t = self.get_clock().now()
        self._stopped = True

        self.create_subscription(Twist, "cmd_vel", self._on_cmd, 10)
        self.create_timer(1.0 / max(hz, 1.0), self._heartbeat)
        self.get_logger().info(
            f"ESP32 bridge: port={self.port} base={self.wheel_base}m "
            f"pwm[{self.pwm_min_move}..{self.pwm_max}] max_v={self.max_linear}m/s"
        )

    # ------------------------------------------------------------------ serial
    def _open_serial(self):
        try:
            import serial  # pyserial
        except ImportError:
            self.get_logger().error("pyserial не установлен — узел работает вхолостую (sudo apt install python3-serial)")
            return None
        try:
            ser = serial.Serial(self.port, self.baud, timeout=0.1)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"не открыть {self.port}: {exc} — узел работает вхолостую")
            return None
        import time

        # КРИТИЧНО: перевести ESP32 в RUN. Иначе DTR/RTS при открытии оставляют его в
        # загрузчике — прошивка не стартует, и strapping-пины моторов (12/14/15) крутят
        # робота «на максимуме» при подаче питания. Проверено esp_probe.py: default-open →
        # нет boot; reset-into-run → SPI_FAST_FLASH_BOOT. Полярность как в пробнике.
        try:
            ser.dtr = False    # GPIO0 = high (RUN, не bootloader)
            ser.rts = True     # EN = low (сброс)
            time.sleep(0.1)
            ser.rts = False    # EN = high → загрузка прошивки
        except Exception:  # noqa: BLE001
            pass
        time.sleep(float(self.get_parameter("settle_s").value))  # время на загрузку прошивки
        try:
            banner = ser.read(400)          # ловим баннер — подтверждение старта прошивки
            if b"READY" in banner:
                self.get_logger().info("ESP32 прошивка стартовала (READY) — моторы управляемы")
            else:
                self.get_logger().warn(
                    f"ESP32: READY не пойман, прошивка могла не стартовать: {banner[:48]!r}"
                )
            ser.reset_input_buffer()
        except Exception:  # noqa: BLE001
            pass
        return ser

    def _write(self, data: bytes) -> None:
        if self._ser is None:
            return
        try:
            self._ser.write(data)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"ошибка записи в serial: {exc}")

    def _send_drive(self, left: int, right: int) -> None:
        if self.swap_sides:
            left, right = right, left
        self._write(f"{int(left)} {int(right)}\n".encode())

    def _send_stop(self) -> None:
        self._write(b"STOP\n")

    # ------------------------------------------------------------------ ROS
    def _on_cmd(self, msg: Twist) -> None:
        vx = max(-self.max_linear, min(self.max_linear, msg.linear.x))
        wz = max(-self.max_angular, min(self.max_angular, msg.angular.z))
        self._target_lr = twist_to_lr(
            vx,
            wz,
            max_linear=self.max_linear,
            max_angular=self.max_angular,
            pwm_max=self.pwm_max,
            pwm_min_move=self.pwm_min_move,
            turn_gain=self.turn_gain,
        )
        self._last_cmd_t = self.get_clock().now()

    def _heartbeat(self) -> None:
        dt = (self.get_clock().now() - self._last_cmd_t).nanoseconds * 1e-9
        if dt > self.cmd_timeout:
            # deadman — команд нет, гарантированный стоп
            self._target_lr = (0, 0)
            if not self._stopped:
                self._send_stop()
                self._sent_lr = (0, 0)
                self._stopped = True
            return

        # slew-rate к целевому ШИМ (плавность для разомкнутого привода)
        tl, tr = self._target_lr
        cl, cr = self._sent_lr
        nl = _ramp(cl, tl, self.max_pwm_step)
        nr = _ramp(cr, tr, self.max_pwm_step)
        self._sent_lr = (nl, nr)
        if (nl, nr) == (0, 0):
            self._send_stop()
            self._stopped = True
        else:
            self._send_drive(nl, nr)
            self._stopped = False

    def destroy_node(self) -> bool:
        self._send_stop()
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:  # noqa: BLE001
                pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Esp32CmdVelBridge()
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
