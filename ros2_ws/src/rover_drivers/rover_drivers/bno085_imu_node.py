#!/usr/bin/env python3
"""BNO085 → sensor_msgs/Imu (I2C, Blinka + Adafruit BNO08x).

Читает GAME rotation vector (гиро+акселерометр, БЕЗ магнитометра → иммунитет к
магнитным помехам моторов), гироскоп и акселерометр с BNO085 на 40-пиновом гребне
Jetson (шина /dev/i2c-7, адрес 0x4A) и публикует sensor_msgs/Imu в `imu/data` для
robot_localization. При ошибке I2C переподключается.

Конвенция кадра: публикуем СЫРОЙ кватернион/гиро в кадре `imu_link`. Физический
монтаж (в т.ч. знак yaw) калибруется ориентацией joint `imu_link` в URDF — EKF
трансформирует данные IMU через TF. Если карта/повороты зеркалятся — перевернуть
imu_link на roll=π в URDF (инвертирует yaw и gyro.z согласованно).

Зависимости на Jetson (в окружении ROS-Python):
  sudo pip3 install adafruit-blinka adafruit-circuitpython-bno08x adafruit-extended-bus
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class Bno085ImuNode(Node):
    def __init__(self) -> None:
        super().__init__("bno085_imu_node")
        self.declare_parameter("i2c_bus", 7)        # /dev/i2c-7 (гребень, пины 3/5)
        self.declare_parameter("address", 0x4A)
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("publish_accel", True)
        # дисперсии (вариансы) для EKF: yaw и vyaw малые (доверяем), остальное большое
        self.declare_parameter("yaw_variance", 0.02)
        self.declare_parameter("gyro_variance", 0.001)
        self.declare_parameter("accel_variance", 0.2)

        self.bus = int(self.get_parameter("i2c_bus").value)
        self.addr = int(self.get_parameter("address").value)
        self.frame_id = self.get_parameter("frame_id").value
        self.publish_accel = bool(self.get_parameter("publish_accel").value)
        self.yaw_var = float(self.get_parameter("yaw_variance").value)
        self.gyro_var = float(self.get_parameter("gyro_variance").value)
        self.accel_var = float(self.get_parameter("accel_variance").value)

        self._bno = None
        self._warned = False
        self._pub = self.create_publisher(Imu, "imu/data", 20)
        self.create_timer(1.0 / max(float(self.get_parameter("rate_hz").value), 1.0), self._tick)
        self.get_logger().info(f"BNO085 IMU: bus=/dev/i2c-{self.bus} addr=0x{self.addr:02X} → imu/data")

    # ------------------------------------------------------------------ I2C
    def _connect(self):
        from adafruit_bno08x import (  # type: ignore[import-untyped]
            BNO_REPORT_ACCELEROMETER,
            BNO_REPORT_GAME_ROTATION_VECTOR,
            BNO_REPORT_GYROSCOPE,
        )
        from adafruit_bno08x.i2c import BNO08X_I2C  # type: ignore[import-untyped]
        from adafruit_extended_bus import ExtendedI2C as I2C  # type: ignore[import-untyped]

        i2c = I2C(self.bus)
        bno = BNO08X_I2C(i2c, address=self.addr)
        bno.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR)
        bno.enable_feature(BNO_REPORT_GYROSCOPE)
        if self.publish_accel:
            bno.enable_feature(BNO_REPORT_ACCELEROMETER)
        return bno

    def _tick(self) -> None:
        if self._bno is None:
            try:
                self._bno = self._connect()
                self.get_logger().info("BNO085 подключён, поток пошёл")
                self._warned = False
            except Exception as exc:  # noqa: BLE001
                if not self._warned:
                    self.get_logger().warn(f"IMU недоступна: {exc} — повтор каждый тик")
                    self._warned = True
                return

        try:
            qi, qj, qk, qr = self._bno.game_quaternion  # (i, j, k, real)
            gx, gy, gz = self._bno.gyro                  # рад/с
            ax = ay = az = 0.0
            if self.publish_accel:
                ax, ay, az = self._bno.acceleration      # м/с²
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"IMU ошибка чтения: {exc} — переподключение")
            self._bno = None
            return

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.orientation.x = float(qi)
        msg.orientation.y = float(qj)
        msg.orientation.z = float(qk)
        msg.orientation.w = float(qr)
        # ковариация ориентации: roll/pitch неизвестны (большая), yaw — доверяем
        big = 1e6
        msg.orientation_covariance = [big, 0.0, 0.0, 0.0, big, 0.0, 0.0, 0.0, self.yaw_var]

        msg.angular_velocity.x = float(gx)
        msg.angular_velocity.y = float(gy)
        msg.angular_velocity.z = float(gz)
        msg.angular_velocity_covariance = [
            self.gyro_var, 0.0, 0.0, 0.0, self.gyro_var, 0.0, 0.0, 0.0, self.gyro_var,
        ]

        if self.publish_accel:
            msg.linear_acceleration.x = float(ax)
            msg.linear_acceleration.y = float(ay)
            msg.linear_acceleration.z = float(az)
            msg.linear_acceleration_covariance = [
                self.accel_var, 0.0, 0.0, 0.0, self.accel_var, 0.0, 0.0, 0.0, self.accel_var,
            ]
        else:
            msg.linear_acceleration_covariance = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self._pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Bno085ImuNode()
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
