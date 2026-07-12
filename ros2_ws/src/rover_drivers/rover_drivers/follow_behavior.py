#!/usr/bin/env python3
"""follow_behavior — режим следования за человеком (конечный автомат).

Потребляет /follow/* от person_tracker и управляет роботом. Порт логики
greenhouse/orchestration/behaviors/following.py на ROS.

Состояния:
  TRACKING  — видим человека → едем к нему, держим дистанцию (реактивно, БЕЗ карты)
  LOST      — потеряли дольше grace → решаем, что делать
  GOTO      — едем в ПОСЛЕДНЮЮ известную точку через Nav2 (заезд за угол)
  SEARCH    — крутимся на месте к последней стороне, ищем узким углом камеры
  WAIT      — не нашли / ещё не видели → стоим, ждём появления

Владение /cmd_vel: в TRACKING/SEARCH публикуем сами; в GOTO ведёт Nav2 (мы молчим).
Панель гейтит свой телеоп по /follow/active; кнопка Follow шлёт /follow/enable.
follow_behavior — чистый rclpy (без torch), запускается обычным `ros2 run`.
"""
from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, String
from tf2_ros import Buffer, TransformListener

try:
    from action_msgs.msg import GoalStatus  # noqa: F401
    from nav2_msgs.action import NavigateToPose
    _NAV = True
except Exception:  # noqa: BLE001
    _NAV = False


class FollowBehavior(Node):
    def __init__(self) -> None:
        super().__init__("follow_behavior")
        self.declare_parameter("standoff_m", 1.5)      # дистанция, которую держим до человека
        self.declare_parameter("k_ang", 1.6)           # усиление доворота на пеленг
        self.declare_parameter("k_lin", 0.6)           # усиление подъезда по ошибке дистанции
        self.declare_parameter("max_lin", 0.20)        # медленно (rf2o не теряется)
        self.declare_parameter("max_ang", 1.2)
        self.declare_parameter("lost_grace_s", 1.5)    # сколько «терпим» пропажу до LOST
        self.declare_parameter("search_timeout_s", 16.0)
        self.declare_parameter("search_ang", 0.9)      # скорость вращения в поиске
        self.declare_parameter("clearance_m", 0.35)    # полу-габарит + запас: ближе → STOP (не таранить)
        self.declare_parameter("brake_dist_m", 0.9)    # с этой дистанции тормозим и объезжаем (раньше)
        self.declare_parameter("avoid_gain", 2.0)      # сила доворота ОТ препятствия
        self.declare_parameter("rate_hz", 10.0)
        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        self.standoff = float(gp("standoff_m"))
        self.k_ang, self.k_lin = float(gp("k_ang")), float(gp("k_lin"))
        self.max_lin, self.max_ang = float(gp("max_lin")), float(gp("max_ang"))
        self.lost_grace = float(gp("lost_grace_s"))
        self.search_timeout = float(gp("search_timeout_s"))
        self.search_ang = float(gp("search_ang"))
        self.clearance = float(gp("clearance_m"))
        self.brake = float(gp("brake_dist_m"))
        self.avoid_gain = float(gp("avoid_gain"))

        # восприятие
        self.visible = False
        self.bearing = 0.0
        self.tb: tuple[float, float] | None = None
        self.tm: tuple[float, float] | None = None
        self.last_vis_t = self.get_clock().now()
        self.last_seen_map: tuple[float, float] | None = None
        self.last_side = 1.0                            # знак последнего пеленга (куда крутиться в поиске)
        # FSM
        self.enabled = False
        self.state = "IDLE"
        self._search_t = self.get_clock().now()
        self.goal_handle = None
        self.nav_done = False

        self.create_subscription(Bool, "/follow/visible", self._on_vis, 10)
        self.create_subscription(Float32, "/follow/bearing", self._on_brg, 10)
        self.create_subscription(PointStamped, "/follow/target_base", self._on_tb, 10)
        self.create_subscription(PoseStamped, "/follow/target_map", self._on_tm, 10)
        self.create_subscription(Bool, "/follow/enable", self._on_enable, 10)
        self.create_subscription(LaserScan, "/scan", self._on_scan, 10)
        self.buf = Buffer()
        TransformListener(self.buf, self)
        self._scan: LaserScan | None = None
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pub_active = self.create_publisher(Bool, "/follow/active", 10)
        self.pub_state = self.create_publisher(String, "/follow/state", 10)
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose") if _NAV else None
        self.create_timer(1.0 / max(float(gp("rate_hz")), 1.0), self._tick)
        self.get_logger().info("follow_behavior: жду /follow/enable")

    # ---------------------------------------------------------------- callbacks
    def _on_vis(self, m: Bool) -> None:
        self.visible = bool(m.data)
        if self.visible:
            self.last_vis_t = self.get_clock().now()

    def _on_brg(self, m: Float32) -> None:
        self.bearing = float(m.data)
        if abs(self.bearing) > 0.05:
            self.last_side = 1.0 if self.bearing > 0 else -1.0

    def _on_tb(self, m: PointStamped) -> None:
        self.tb = (m.point.x, m.point.y)

    def _on_scan(self, m: LaserScan) -> None:
        self._scan = m

    def _on_tm(self, m: PoseStamped) -> None:
        self.tm = (m.pose.position.x, m.pose.position.y)
        self.last_seen_map = self.tm

    def _on_enable(self, m: Bool) -> None:
        en = bool(m.data)
        if en != self.enabled:
            self.enabled = en
            # старт в WAIT: не спиним при включении, если человека нет в кадре — ждём появления
            self._set_state("WAIT" if en else "IDLE")
            if not en:
                self._cancel_nav()
                self._stop()

    # ---------------------------------------------------------------- helpers
    def _set_state(self, s: str) -> None:
        if s != self.state:
            self.state = s
            if s == "SEARCH":
                self._search_t = self.get_clock().now()
            self.get_logger().info(f"follow → {s}")

    def _age(self, t) -> float:
        return (self.get_clock().now() - t).nanoseconds * 1e-9

    def _stop(self) -> None:
        self.pub.publish(Twist())

    def _drive(self, lin: float, ang: float) -> None:
        t = Twist()
        t.linear.x = float(max(-self.max_lin, min(self.max_lin, lin)))
        t.angular.z = float(max(-self.max_ang, min(self.max_ang, ang)))
        self.pub.publish(t)

    def _avoid(self, lin: float, ang: float) -> tuple[float, float]:
        """Реактивный объезд по лидару БЕЗ карты: тормозим и доворачиваем ОТ ближайшего
        препятствия в переднем секторе, держим габарит (clearance). Человек на дистанции
        стендоффа (~1.5 м) дальше brake — не мешает; срабатывает только на близкое (колонна/стена)."""
        s = self._scan
        if s is None or lin <= 1e-3:
            return lin, ang
        try:
            tf = self.buf.lookup_transform("base_footprint", s.header.frame_id or "laser", rclpy.time.Time())
        except Exception:  # noqa: BLE001
            return lin, ang
        tx, ty = tf.transform.translation.x, tf.transform.translation.y
        q = tf.transform.rotation
        lyaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        c, si = math.cos(lyaw), math.sin(lyaw)
        min_d, min_b = 1e9, 0.0
        a = s.angle_min
        for r in s.ranges:
            if s.range_min < r < s.range_max:
                lx, ly = r * math.cos(a), r * math.sin(a)
                bx, by = tx + lx * c - ly * si, ty + lx * si + ly * c
                if bx > 0.0 and abs(math.atan2(by, bx)) < 1.2:      # передний сектор ±~70°
                    d = math.hypot(bx, by)
                    if d < min_d:
                        min_d, min_b = d, math.atan2(by, bx)
            a += s.angle_increment
        if min_d < self.brake:
            f = (min_d - self.clearance) / max(1e-3, self.brake - self.clearance)
            lin *= max(0.0, min(1.0, f))                            # 0 у clearance, полный у brake
            side = -1.0 if (min_b >= 0.0) else 1.0                  # препятствие слева → вправо, и наоборот
            ang += side * self.avoid_gain * (self.brake - min_d)
        return lin, ang

    # ---------------------------------------------------------------- FSM
    def _tick(self) -> None:
        self.pub_active.publish(Bool(data=self.enabled))
        self.pub_state.publish(String(data=self.state if self.enabled else "off"))
        if not self.enabled:
            return
        vis = self.visible and self._age(self.last_vis_t) < 0.5
        if self.state == "TRACKING":
            self._tracking(vis)
        elif self.state == "LOST":
            self._lost()
        elif self.state == "GOTO":
            self._goto(vis)
        elif self.state == "SEARCH":
            self._search(vis)
        elif self.state == "WAIT":
            self._wait(vis)
        else:
            self._set_state("TRACKING")

    def _tracking(self, vis: bool) -> None:
        if vis and self.tb is not None:
            x, y = self.tb
            rng, brg = math.hypot(x, y), math.atan2(y, x)
            ang = self.k_ang * brg
            lin = self.k_lin * (rng - self.standoff) if rng > self.standoff else 0.0
            if abs(brg) > 0.6:                 # сильно вбок → сперва довернуть, потом ехать
                lin *= 0.3
            lin, ang = self._avoid(lin, ang)   # объезд препятствий (напр. колонны)
            self._drive(lin, ang)
        elif self._age(self.last_vis_t) > self.lost_grace:
            self._set_state("LOST")
        else:
            # краткий «коаст» за угол (БЕЗ карты): едем вперёд + доворот в сторону, куда ушёл человек
            lin, ang = self._avoid(self.max_lin * 0.5, self.k_ang * 0.4 * self.last_side)
            self._drive(lin, ang)

    def _lost(self) -> None:
        if self.last_seen_map is not None and self.nav is not None and self.nav.server_is_ready():
            self._send_nav(self.last_seen_map)
            self._set_state("GOTO")
        else:
            self._set_state("SEARCH")           # некуда ехать (не видели/нет Nav2) → искать на месте

    def _goto(self, vis: bool) -> None:
        if vis:                                 # снова увидели по пути → бросаем Nav2, следуем
            self._cancel_nav()
            self._set_state("TRACKING")
        elif self.nav_done:                     # доехали до последней точки → крутимся, ищем
            self._set_state("SEARCH")
        # иначе Nav2 ведёт — /cmd_vel НЕ трогаем

    def _search(self, vis: bool) -> None:
        if vis:
            self._set_state("TRACKING")
        elif self._age(self._search_t) > self.search_timeout:
            self._stop()
            self._set_state("WAIT")
        else:
            self._drive(0.0, self.search_ang * self.last_side)   # к последней замеченной стороне

    def _wait(self, vis: bool) -> None:
        self._stop()
        if vis:
            self._set_state("TRACKING")

    # ---------------------------------------------------------------- Nav2
    def _send_nav(self, xy: tuple[float, float]) -> None:
        goal = NavigateToPose.Goal()
        ps = PoseStamped()
        ps.header.frame_id = "map"
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x, ps.pose.position.y = float(xy[0]), float(xy[1])
        ps.pose.orientation.w = 1.0
        goal.pose = ps
        self.nav_done = False
        self.nav.send_goal_async(goal).add_done_callback(self._nav_resp)

    def _nav_resp(self, fut) -> None:
        try:
            gh = fut.result()
        except Exception:  # noqa: BLE001
            self.nav_done = True
            return
        if not gh.accepted:
            self.nav_done = True
            return
        self.goal_handle = gh
        gh.get_result_async().add_done_callback(lambda _f: setattr(self, "nav_done", True))

    def _cancel_nav(self) -> None:
        if self.goal_handle is not None:
            try:
                self.goal_handle.cancel_goal_async()
            except Exception:  # noqa: BLE001
                pass
            self.goal_handle = None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FollowBehavior()
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
