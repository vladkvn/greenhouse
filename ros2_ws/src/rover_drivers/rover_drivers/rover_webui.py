#!/usr/bin/env python3
"""rover_webui — браузерная панель ровера: живой лидар-скан + карта slam_toolbox + поза
робота + ПОСТАНОВКА ЦЕЛИ Nav2 кликом по карте, плюс кнопки телеопа. Открыть
http://<jetson>:8091.

Зачем: видеть, что строит SLAM, ставить точку назначения и рулить БЕЗ RViz и без sudo
(как веб-панель follow_me). Подписки: /map (латч), /scan, /plan (плановый маршрут Nav2);
поза — из TF map→base_footprint; скан — в кадре map через TF map→laser.

Два режима вождения на одном /cmd_vel (важно — иначе дерутся):
  * ТЕЛЕОП: кнопки/WASD шлют /cmd_vel (hold-to-drive, deadman 0.4с). Пока едет Nav2,
    панель МОЛЧИТ (не публикует нули), а любое нажатие WASD ОТМЕНЯЕТ цель Nav2 и
    забирает управление.
  * НАВИГАЦИЯ: клик по карте → action navigate_to_pose (Nav2 сам публикует /cmd_vel).
Рендер cv2, сервер на stdlib http.server (потоки). Кросс-потоковые вызовы rclpy
(отправка/отмена цели) сериализованы в spin-поток через _nav_tick — HTTP-обработчики лишь
выставляют «pending».
"""
from __future__ import annotations

import math
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

try:  # nav2 может отсутствовать на «сухом» стенде — панель всё равно должна работать
    from action_msgs.msg import GoalStatus
    from nav2_msgs.action import NavigateToPose
    _NAV_OK = True
except Exception:  # noqa: BLE001
    _NAV_OK = False

PAGE = """<!doctype html><html><head><meta charset=utf-8>
<meta name=viewport content="width=device-width,initial-scale=1"><title>Rover SLAM</title><style>
body{font-family:sans-serif;background:#111;color:#ddd;text-align:center;margin:0;padding:8px}
img{max-width:100%;border:1px solid #333;background:#000}
#v{image-rendering:pixelated;cursor:crosshair}
.cap{color:#888;font-size:12px;margin:8px 0 2px}
.pad{display:grid;grid-template-columns:repeat(3,72px);gap:8px;justify-content:center;margin-top:12px}
button{height:64px;font-size:22px;background:#2a2a2a;color:#fff;border:1px solid #555;border-radius:10px;user-select:none;touch-action:none}
button:active{background:#0a6}.stop{background:#833}.sp{visibility:hidden}
#navbtn{height:40px;width:auto;padding:0 18px;font-size:15px;margin-top:10px}
#st{color:#8b8;font-size:13px}#nav{color:#8bd;font-size:14px;min-height:18px;font-weight:bold}
</style></head><body>
<h3>Rover · SLAM live</h3>
<div class=cap>камера робота</div><img id=cam src=/cam.jpg>
<div class=cap>карта + лидар-скан &mdash; <b>клик = ехать в точку (Nav2)</b></div><img id=v src=/view.jpg>
<p id=nav>&mdash;</p>
<button id=navbtn class=stop>&#10006; отмена навигации</button>
<div class=pad>
<button class="sp"></button><button id=fwd>&#9650;</button><button class="sp"></button>
<button id=left>&#9664;</button><button id=stop class=stop>&#9632;</button><button id=right>&#9654;</button>
<button class="sp"></button><button id=back>&#9660;</button><button class="sp"></button>
</div>
<p class=cap>WASD/стрелки (удерживать) или кнопки &mdash; ручной режим отменяет Nav2 · красная точка = робот · &#10006; фиолетовый = цель</p>
<p id=st>&mdash;</p>
<script>
const LIN=0.16, ANG=1.4; let iv=null;
function drive(l,a){fetch(`/drive?lin=${l}&ang=${a}`).then(r=>r.text()).then(t=>document.getElementById('st').textContent=t);}
function stop(){if(iv){clearInterval(iv);iv=null;}fetch('/stop');}
function hold(el,l,a){
 const s=e=>{e.preventDefault();drive(l,a);if(iv)clearInterval(iv);iv=setInterval(()=>drive(l,a),150);};
 const e2=e=>{e.preventDefault();stop();};
 el.addEventListener('mousedown',s);el.addEventListener('mouseup',e2);el.addEventListener('mouseleave',e2);
 el.addEventListener('touchstart',s);el.addEventListener('touchend',e2);}
hold(document.getElementById('fwd'),LIN,0);hold(document.getElementById('back'),-LIN,0);
hold(document.getElementById('left'),0,ANG);hold(document.getElementById('right'),0,-ANG);
document.getElementById('stop').addEventListener('click',stop);
const km={w:[LIN,0],s:[-LIN,0],a:[0,ANG],d:[0,-ANG],ArrowUp:[LIN,0],ArrowDown:[-LIN,0],ArrowLeft:[0,ANG],ArrowRight:[0,-ANG]};
let pk=null;
document.addEventListener('keydown',e=>{const k=km[e.key];if(!k)return;e.preventDefault();if(pk===e.key)return;pk=e.key;drive(k[0],k[1]);if(iv)clearInterval(iv);iv=setInterval(()=>drive(k[0],k[1]),150);});
document.addEventListener('keyup',e=>{if(!km[e.key])return;e.preventDefault();if(pk===e.key){pk=null;stop();}});
// клик по карте → цель Nav2 (доля ширины/высоты картинки, сервер сам переведёт в кадр map)
const V=document.getElementById('v');
V.addEventListener('click',e=>{const r=V.getBoundingClientRect();
 const fx=(e.clientX-r.left)/r.width, fy=(e.clientY-r.top)/r.height;
 if(fx<0||fx>1||fy<0||fy>1)return;
 fetch(`/goal?fx=${fx.toFixed(4)}&fy=${fy.toFixed(4)}`).then(r=>r.text()).then(t=>document.getElementById('nav').textContent=t);});
document.getElementById('navbtn').addEventListener('click',()=>fetch('/cancel_goal').then(r=>r.text()).then(t=>document.getElementById('nav').textContent=t));
setInterval(()=>{document.getElementById('cam').src='/cam.jpg?'+Date.now();},200);
setInterval(()=>{document.getElementById('v').src='/view.jpg?'+Date.now();},500);
setInterval(()=>fetch('/nav').then(r=>r.text()).then(t=>{document.getElementById('nav').textContent=t;}),700);
</script></body></html>"""


class RoverWebUI(Node):
    def __init__(self) -> None:
        super().__init__("rover_webui")
        self.declare_parameter("port", 8091)
        self.declare_parameter("max_lin", 0.20)
        self.declare_parameter("max_ang", 1.6)
        self.port = int(self.get_parameter("port").value)
        self.max_lin = float(self.get_parameter("max_lin").value)
        self.max_ang = float(self.get_parameter("max_ang").value)

        self._lock = threading.Lock()
        self._map: OccupancyGrid | None = None
        self._scan: LaserScan | None = None
        self._plan: Path | None = None
        self._target = (0.0, 0.0)
        self._last_cmd = self.get_clock().now()

        # --- состояние навигации Nav2 ---
        self._nav_active = False
        self._nav_status = "—"
        self._nav_dist: float | None = None
        self._goal_xy: tuple[float, float] | None = None
        self._goal_handle = None
        self._pending_goal: tuple[float, float] | None = None   # (fx, fy) из клика
        self._pending_cancel = False

        mapqos = QoSProfile(depth=1)
        mapqos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        mapqos.reliability = QoSReliabilityPolicy.RELIABLE
        self.create_subscription(OccupancyGrid, "/map", self._on_map, mapqos)
        self.create_subscription(LaserScan, "/scan", self._on_scan, 10)
        self.create_subscription(Path, "/plan", self._on_plan, 10)
        self._pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.buf = Buffer()
        TransformListener(self.buf, self)
        self.create_timer(1.0 / 15.0, self._drive_tick)

        self._nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose") if _NAV_OK else None
        if _NAV_OK:
            self.create_timer(0.2, self._nav_tick)   # сериализация action-вызовов в spin-поток

        self.declare_parameter("cam_index", 0)
        self.declare_parameter("cam_enable", True)
        self._cam_jpeg: bytes | None = None
        if bool(self.get_parameter("cam_enable").value):
            threading.Thread(
                target=self._cam_loop,
                args=(int(self.get_parameter("cam_index").value),),
                daemon=True,
            ).start()

        srv = ThreadingHTTPServer(("0.0.0.0", self.port), self._handler())
        threading.Thread(target=srv.serve_forever, daemon=True).start()
        self.get_logger().info(f"веб-панель: http://0.0.0.0:{self.port}  (клик по карте = цель Nav2)")

    def _on_map(self, m: OccupancyGrid) -> None:
        with self._lock:
            self._map = m

    def _on_scan(self, s: LaserScan) -> None:
        with self._lock:
            self._scan = s

    def _on_plan(self, p: Path) -> None:
        with self._lock:
            self._plan = p

    def _cam_loop(self, index: int) -> None:
        # авто-подбор индекса: /dev/video* плавает, USB-камеры дают 2 ноды (capture+metadata)
        cap = None
        for idx in [index, 0, 1, 2, 3]:
            c = cv2.VideoCapture(idx)
            if c.isOpened() and c.read()[0]:
                cap, index = c, idx
                break
            c.release()
        if cap is None:
            self.get_logger().warn("камера не найдена (индексы 0..3) — панель без видео")
            return
        # НИЗКАЯ ЗАДЕРЖКА: MJPG + маленький кадр + буфер=1. Иначе read() отдаёт старые кадры
        # из внутренней очереди и видео в браузере отстаёт на секунды.
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        try:
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:  # noqa: BLE001
            pass
        self.get_logger().info(f"камера открыта: index {index}")
        while rclpy.ok():
            if not cap.grab():          # grab() держит очередь пустой → всегда свежий кадр
                time.sleep(0.03)
                continue
            ok, frame = cap.retrieve()
            if not ok:
                continue
            h, w = frame.shape[:2]
            if w > 640:                              # даунскейл для веба
                frame = cv2.resize(frame, (640, max(1, int(640 * h / w))))
            # присваивание ссылки атомарно под GIL — без лока, чтобы луп был быстрым
            self._cam_jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 55])[1].tobytes()

    def set_target(self, lin: float, ang: float) -> None:
        with self._lock:
            if (abs(lin) > 1e-3 or abs(ang) > 1e-3) and self._nav_active:
                self._pending_cancel = True   # взяли ручное управление → отменяем Nav2
            self._target = (
                max(-self.max_lin, min(self.max_lin, lin)),
                max(-self.max_ang, min(self.max_ang, ang)),
            )
            self._last_cmd = self.get_clock().now()

    def stop(self) -> None:
        with self._lock:
            self._target = (0.0, 0.0)
            self._last_cmd = self.get_clock().now()

    def _drive_tick(self) -> None:
        with self._lock:
            if self._nav_active:
                return                          # /cmd_vel во власти Nav2 — панель молчит
            lin, ang = self._target
            dt = (self.get_clock().now() - self._last_cmd).nanoseconds * 1e-9
        if dt > 0.4:                            # deadman
            lin, ang = 0.0, 0.0
        t = Twist()
        t.linear.x = float(lin)
        t.angular.z = float(ang)
        self._pub.publish(t)

    # ------------------------------------------------------------------ Nav2
    def request_goal(self, fx: float, fy: float) -> str:
        if not _NAV_OK:
            return "nav2_msgs недоступен на роботе"
        with self._lock:
            if self._map is None:
                return "нет карты — цель поставить некуда"
            self._pending_goal = (fx, fy)
        return "цель принята, планирую…"

    def request_cancel(self) -> str:
        with self._lock:
            if not self._nav_active:
                return "нет активной цели"
            self._pending_cancel = True
        return "отмена…"

    def _nav_tick(self) -> None:
        """Выполняется в spin-потоке — здесь безопасно дёргать action-клиент."""
        with self._lock:
            pend = self._pending_goal
            self._pending_goal = None
            cancel = self._pending_cancel
            self._pending_cancel = False
            gh = self._goal_handle
        if cancel and gh is not None:
            try:
                gh.cancel_goal_async()
            except Exception:  # noqa: BLE001
                pass
            with self._lock:
                self._nav_status = "отмена…"
        if pend is not None:
            self._do_send_goal(pend)

    def _do_send_goal(self, fxfy: tuple[float, float]) -> None:
        fx, fy = fxfy
        with self._lock:
            m = self._map
        if m is None:
            with self._lock:
                self._nav_status = "нет карты"
            return
        w, h, res = m.info.width, m.info.height, m.info.resolution
        ox, oy = m.info.origin.position.x, m.info.origin.position.y
        mx = ox + fx * w * res
        my = oy + (1.0 - fy) * h * res           # клик fy сверху-вниз → карта y снизу-вверх
        yaw = 0.0
        try:                                     # финальный курс = «носом на цель» от текущей позы
            tb = self.buf.lookup_transform("map", "base_footprint", rclpy.time.Time())
            rx, ry = tb.transform.translation.x, tb.transform.translation.y
            yaw = math.atan2(my - ry, mx - rx)
        except Exception:  # noqa: BLE001
            pass
        if self._nav_client is None or not self._nav_client.server_is_ready():
            with self._lock:
                self._nav_status = "Nav2 не запущен (navigate_to_pose нет)"
                self._nav_active = False
            return
        goal = NavigateToPose.Goal()
        ps = PoseStamped()
        ps.header.frame_id = "map"
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = float(mx)
        ps.pose.position.y = float(my)
        ps.pose.orientation.z = math.sin(yaw / 2.0)
        ps.pose.orientation.w = math.cos(yaw / 2.0)
        goal.pose = ps
        with self._lock:
            self._goal_xy = (mx, my)
            self._nav_active = True
            self._nav_status = f"планирую → ({mx:.2f}, {my:.2f})"
            self._nav_dist = None
        fut = self._nav_client.send_goal_async(goal, feedback_callback=self._nav_feedback)
        fut.add_done_callback(self._nav_goal_response)

    def _nav_feedback(self, fb) -> None:
        try:
            d = float(fb.feedback.distance_remaining)
        except Exception:  # noqa: BLE001
            return
        with self._lock:
            self._nav_dist = d
            if self._nav_active:
                self._nav_status = "🧭 еду к цели"

    def _nav_goal_response(self, future) -> None:
        try:
            gh = future.result()
        except Exception as exc:  # noqa: BLE001
            with self._lock:
                self._nav_status = f"ошибка отправки: {exc}"
                self._nav_active = False
            return
        if not gh.accepted:
            with self._lock:
                self._nav_status = "цель отклонена Nav2"
                self._nav_active = False
                self._goal_xy = None
            return
        with self._lock:
            self._goal_handle = gh
        gh.get_result_async().add_done_callback(self._nav_result)

    def _nav_result(self, future) -> None:
        status = None
        try:
            status = future.result().status
        except Exception:  # noqa: BLE001
            pass
        if status == GoalStatus.STATUS_SUCCEEDED:
            txt = "✓ цель достигнута"
        elif status == GoalStatus.STATUS_CANCELED:
            txt = "отменено"
        elif status == GoalStatus.STATUS_ABORTED:
            txt = "✗ прервано (не доехал — см. costmap/локализацию)"
        else:
            txt = f"навигация завершена (status={status})"
        with self._lock:
            self._nav_status = txt
            self._nav_active = False
            self._goal_handle = None
            self._goal_xy = None
            self._plan = None
        # гарантированный стоп после навигации
        t = Twist()
        self._pub.publish(t)

    def _nav_text(self) -> str:
        with self._lock:
            s = self._nav_status
            d = self._nav_dist
            active = self._nav_active
        if active and d is not None:
            return f"{s} · осталось {d:.2f} м"
        return s

    # ------------------------------------------------------------------ render
    def _yaw(self, q) -> float:
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def render(self) -> bytes:
        with self._lock:
            m = self._map
            s = self._scan
            plan = self._plan
            goal = self._goal_xy
        if m is None:
            img = np.full((280, 280, 3), 55, np.uint8)
            cv2.putText(img, "no /map yet", (40, 145), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)
            return cv2.imencode(".jpg", img)[1].tobytes()

        w, h, res = m.info.width, m.info.height, m.info.resolution
        ox, oy = m.info.origin.position.x, m.info.origin.position.y
        data = np.array(m.data, dtype=np.int16).reshape(h, w)
        img = np.full((h, w, 3), 128, np.uint8)      # unknown
        img[data == 0] = (240, 240, 240)             # free
        img[data >= 50] = (40, 40, 40)               # occupied
        img = np.flipud(img)                         # ROS y-up → image y-down

        if s is not None:
            try:
                tf = self.buf.lookup_transform("map", "laser", rclpy.time.Time())
                tx, ty = tf.transform.translation.x, tf.transform.translation.y
                yaw = self._yaw(tf.transform.rotation)
                ca, sa = math.cos(yaw), math.sin(yaw)
                ang = s.angle_min
                for r in s.ranges:
                    if math.isfinite(r) and s.range_min < r < s.range_max:
                        lx, ly = r * math.cos(ang), r * math.sin(ang)
                        mx = tx + lx * ca - ly * sa
                        my = ty + lx * sa + ly * ca
                        col = int((mx - ox) / res)
                        py = h - 1 - int((my - oy) / res)
                        if 0 <= col < w and 0 <= py < h:
                            img[py, col] = (0, 220, 0)
                    ang += s.angle_increment
            except Exception:  # noqa: BLE001
                pass

        scale = max(1, int(640 / max(w, h)))
        img = cv2.resize(img, (w * scale, h * scale), interpolation=cv2.INTER_NEAREST)

        def to_px(mx: float, my: float) -> tuple[int, int]:
            return int((mx - ox) / res) * scale, (h - 1 - int((my - oy) / res)) * scale

        # плановый маршрут Nav2 (синяя линия)
        if plan is not None and len(plan.poses) > 1:
            pts = [to_px(p.pose.position.x, p.pose.position.y) for p in plan.poses]
            for i in range(1, len(pts)):
                cv2.line(img, pts[i - 1], pts[i], (255, 130, 0), 2)

        # цель (фиолетовый крест)
        if goal is not None:
            gx, gy = to_px(goal[0], goal[1])
            cv2.drawMarker(img, (gx, gy), (255, 0, 255), cv2.MARKER_TILTED_CROSS, 20, 2)
            cv2.circle(img, (gx, gy), 12, (255, 0, 255), 2)

        # робот — ПОВЕРХ увеличенной карты, фикс. размер → всегда крупно и заметно:
        # красный кружок + белая обводка + толстый луч курса.
        try:
            tb = self.buf.lookup_transform("map", "base_footprint", rclpy.time.Time())
            rx, ry = tb.transform.translation.x, tb.transform.translation.y
            ryaw = self._yaw(tb.transform.rotation)
            cx, cy = to_px(rx, ry)
            ex = int(cx + 26 * math.cos(ryaw))
            ey = int(cy - 26 * math.sin(ryaw))
            cv2.line(img, (cx, cy), (ex, ey), (0, 0, 255), 3)
            cv2.circle(img, (cx, cy), 9, (0, 0, 255), -1)
            cv2.circle(img, (cx, cy), 9, (255, 255, 255), 2)
        except Exception:  # noqa: BLE001
            cv2.putText(img, "no robot pose (TF)", (10, 26),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        return cv2.imencode(".jpg", img)[1].tobytes()

    # ------------------------------------------------------------------ HTTP
    def _handler(self):
        ui = self

        class H(BaseHTTPRequestHandler):
            def log_message(self, *a):  # тихо
                pass

            def do_GET(self):
                p = urlparse(self.path)
                q = parse_qs(p.query)
                if p.path == "/view.jpg":
                    jpg = ui.render()
                    self.send_response(200)
                    self.send_header("Content-Type", "image/jpeg")
                    self.send_header("Content-Length", str(len(jpg)))
                    self.end_headers()
                    self.wfile.write(jpg)
                elif p.path == "/cam.jpg":
                    with ui._lock:
                        jpg = ui._cam_jpeg
                    if jpg is None:
                        img = np.full((240, 320, 3), 45, np.uint8)
                        cv2.putText(img, "no camera", (95, 128),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (180, 180, 180), 1)
                        jpg = cv2.imencode(".jpg", img)[1].tobytes()
                    self.send_response(200)
                    self.send_header("Content-Type", "image/jpeg")
                    self.send_header("Content-Length", str(len(jpg)))
                    self.end_headers()
                    self.wfile.write(jpg)
                elif p.path == "/drive":
                    lin = float(q.get("lin", ["0"])[0])
                    ang = float(q.get("ang", ["0"])[0])
                    ui.set_target(lin, ang)
                    self._txt(f"drive {lin:+.2f} {ang:+.2f}")
                elif p.path == "/stop":
                    ui.stop()
                    self._txt("stop")
                elif p.path == "/goal":
                    fx = float(q.get("fx", ["0"])[0])
                    fy = float(q.get("fy", ["0"])[0])
                    self._txt(ui.request_goal(fx, fy))
                elif p.path == "/cancel_goal":
                    self._txt(ui.request_cancel())
                elif p.path == "/nav":
                    self._txt(ui._nav_text())
                else:
                    body = PAGE.encode()
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html; charset=utf-8")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)

            def _txt(self, s: str):
                b = s.encode()
                self.send_response(200)
                self.send_header("Content-Type", "text/plain; charset=utf-8")
                self.send_header("Content-Length", str(len(b)))
                self.end_headers()
                self.wfile.write(b)

        return H


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RoverWebUI()
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
