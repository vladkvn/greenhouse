"""Веб-интерфейс ровера-фолловера.

Без сторонних зависимостей: stdlib http.server + cv2 для JPEG. Сервер крутится в
фоновом потоке параллельно перцепции. Отдаёт:
  * GET /              -- HTML-панель (видео + кнопки режимов + статус);
  * GET /stream.mjpg   -- MJPEG-поток размеченного кадра камеры;
  * GET /state         -- JSON текущего состояния (режим, цель, fps, команда моторам);
  * POST /mode?m=...   -- переключение режима.

Режимы: idle (безопасный, моторы стоят), follow (следование за человеком),
goto (езда в точку), mapping (построение карты). goto/mapping пока заглушки --
кнопки и маршрутизация есть, поведение углубляем позже.
"""

from __future__ import annotations

import json
import logging
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np

log = logging.getLogger("follow_me.web")

MODES = ("idle", "follow", "goto", "mapping")
MODE_LABELS = {
    "idle": "Ожидание",
    "follow": "Следование",
    "goto": "Езда в точку",
    "mapping": "Построение карты",
}


class SharedState:
    """Потокобезопасный обмен между циклом перцепции и веб-сервером."""

    def __init__(self, mode: str = "idle") -> None:
        self._lock = threading.Lock()
        self._jpeg = b""
        self._status: dict = {"mode": mode}
        self._scan: list = []
        self._teleop: tuple[int, int, float] = (0, 0, 0.0)  # l, r, monotonic ts
        self._map_png: bytes = b""
        self._goal_req: tuple[int, int] | None = None  # клик цели (col,row), одноразово
        self._mode = mode if mode in MODES else "idle"

    def publish(self, jpeg: bytes | None, status: dict, scan: list | None = None) -> None:
        with self._lock:
            if jpeg:
                self._jpeg = jpeg
            self._status = status
            if scan is not None:
                self._scan = scan

    def get_scan(self) -> list:
        with self._lock:
            return list(self._scan)

    def set_teleop(self, left: int, right: int) -> None:
        with self._lock:
            self._teleop = (int(left), int(right), time.monotonic())

    def get_teleop(self) -> tuple[int, int, float]:
        with self._lock:
            return self._teleop

    def set_map_png(self, png: bytes) -> None:
        with self._lock:
            self._map_png = png

    def get_map_png(self) -> bytes:
        with self._lock:
            return self._map_png

    def set_goal_request(self, col: int, row: int) -> None:
        with self._lock:
            self._goal_req = (int(col), int(row))

    def take_goal_request(self) -> tuple[int, int] | None:
        """Вернуть и сбросить запрос цели (main потребляет один раз)."""
        with self._lock:
            g = self._goal_req
            self._goal_req = None
            return g

    def get_jpeg(self) -> bytes:
        with self._lock:
            return self._jpeg

    def get_status(self) -> dict:
        with self._lock:
            return dict(self._status)

    @property
    def mode(self) -> str:
        with self._lock:
            return self._mode

    def set_mode(self, mode: str) -> bool:
        if mode not in MODES:
            return False
        with self._lock:
            self._mode = mode
        log.info("Режим переключён -> %s", mode)
        return True


def annotate(frame: np.ndarray, tracks, target, status: dict) -> np.ndarray:
    """Нарисовать боксы людей, выделить цель и вывести HUD. Работает на копии кадра."""
    img = frame
    h = img.shape[0]
    # Вертикальная линия центра кадра -- ось камеры (0 град).
    cx0 = img.shape[1] // 2
    cv2.line(img, (cx0, 0), (cx0, h), (60, 60, 60), 1)

    for t in tracks:
        d = t.det
        is_target = target is not None and t is target
        color = (0, 230, 0) if is_target else (170, 170, 170)
        thick = 3 if is_target else 1
        cv2.rectangle(img, (int(d.x1), int(d.y1)), (int(d.x2), int(d.y2)), color, thick)
        label = f"{t.cam_angle_deg:+.0f}\xb0"
        if t.distance_m is not None:
            label += f" {t.distance_m:.2f}m"
        cv2.putText(img, label, (int(d.x1), max(18, int(d.y1) - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

    mode = status.get("mode", "idle")
    hud1 = f"{MODE_LABELS.get(mode, mode).upper()}   {status.get('fps', 0):.0f} fps   L={status.get('left', 0)} R={status.get('right', 0)}"
    cv2.putText(img, hud1, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
    cv2.putText(img, status.get("drive_status", ""), (10, 56),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    return img


def encode_jpeg(frame: np.ndarray, quality: int = 70) -> bytes | None:
    ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
    return buf.tobytes() if ok else None


def _make_handler(state: SharedState):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *args) -> None:  # тишина в логах follow_me
            pass

        # --------------------------------------------------------------- GET
        def do_GET(self) -> None:
            path = urlparse(self.path).path
            if path in ("/", "/index.html"):
                self._send_bytes(PAGE.encode("utf-8"), "text/html; charset=utf-8")
            elif path == "/state":
                body = json.dumps(state.get_status()).encode()
                self._send_bytes(body, "application/json", cache=False)
            elif path == "/scan":
                body = json.dumps(state.get_scan()).encode()
                self._send_bytes(body, "application/json", cache=False)
            elif path == "/map.png":
                png = state.get_map_png()
                if png:
                    self._send_bytes(png, "image/png", cache=False)
                else:
                    self.send_error(404)
            elif path == "/stream.mjpg":
                self._send_stream()
            else:
                self.send_error(404)

        # -------------------------------------------------------------- POST
        def do_POST(self) -> None:
            parsed = urlparse(self.path)
            if parsed.path == "/mode":
                m = (parse_qs(parsed.query).get("m") or [""])[0]
                ok = state.set_mode(m)
                body = json.dumps({"ok": ok, "mode": state.mode}).encode()
                self._send_bytes(body, "application/json", code=200 if ok else 400,
                                 cache=False)
            elif parsed.path == "/drive":
                q = parse_qs(parsed.query)
                try:
                    left = int((q.get("l") or ["0"])[0])
                    right = int((q.get("r") or ["0"])[0])
                except ValueError:
                    self.send_error(400)
                    return
                state.set_teleop(left, right)
                self._send_bytes(b'{"ok":true}', "application/json", cache=False)
            elif parsed.path == "/goal":
                q = parse_qs(parsed.query)
                try:
                    col = int((q.get("col") or ["0"])[0])
                    row = int((q.get("row") or ["0"])[0])
                except ValueError:
                    self.send_error(400)
                    return
                state.set_goal_request(col, row)
                self._send_bytes(b'{"ok":true}', "application/json", cache=False)
            else:
                self.send_error(404)

        # ------------------------------------------------------------ helpers
        def _send_bytes(self, body: bytes, ctype: str, code: int = 200,
                        cache: bool = True) -> None:
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            if not cache:
                self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _send_stream(self) -> None:
            self.send_response(200)
            self.send_header("Age", "0")
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Pragma", "no-cache")
            self.send_header("Content-Type",
                             "multipart/x-mixed-replace; boundary=FRAME")
            self.end_headers()
            try:
                while True:
                    jpeg = state.get_jpeg()
                    if jpeg:
                        self.wfile.write(b"--FRAME\r\n")
                        self.wfile.write(b"Content-Type: image/jpeg\r\n")
                        self.wfile.write(
                            f"Content-Length: {len(jpeg)}\r\n\r\n".encode())
                        self.wfile.write(jpeg)
                        self.wfile.write(b"\r\n")
                    time.sleep(0.05)
            except (BrokenPipeError, ConnectionResetError):
                pass

    return Handler


class WebServer:
    """Фоновый HTTP-сервер. Жизненный цикл: start() ... stop()."""

    def __init__(self, state: SharedState, host: str = "0.0.0.0",
                 port: int = 8080) -> None:
        self._state = state
        self._httpd = ThreadingHTTPServer((host, port), _make_handler(state))
        self._httpd.daemon_threads = True
        self._thread = threading.Thread(target=self._httpd.serve_forever,
                                        daemon=True)
        self._port = port

    def start(self) -> None:
        self._thread.start()
        log.info("Веб-интерфейс на http://0.0.0.0:%d", self._port)

    def stop(self) -> None:
        try:
            self._httpd.shutdown()
            self._httpd.server_close()
        except OSError:
            pass


PAGE = """<!doctype html>
<html lang="ru">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Ровер · фолловер</title>
<style>
  :root { color-scheme: dark; }
  * { box-sizing: border-box; }
  body { margin: 0; font-family: system-ui, sans-serif; background: #111; color: #eee; }
  header { padding: 12px 16px; background: #181818; border-bottom: 1px solid #2a2a2a;
           display: flex; align-items: center; gap: 12px; }
  header h1 { font-size: 16px; margin: 0; font-weight: 600; }
  #dot { width: 10px; height: 10px; border-radius: 50%; background: #c33; }
  #dot.ok { background: #3c3; }
  main { max-width: 960px; margin: 0 auto; padding: 16px; }
  .video { position: relative; background: #000; border-radius: 8px; overflow: hidden;
           aspect-ratio: 16 / 9; }
  .video img { width: 100%; height: 100%; object-fit: contain; display: block; }
  .modes { display: grid; grid-template-columns: repeat(2, 1fr); gap: 10px;
           margin: 16px 0; }
  @media (min-width: 640px) { .modes { grid-template-columns: repeat(4, 1fr); } }
  .modes button { padding: 14px; font-size: 15px; border: 1px solid #333; border-radius: 8px;
                  background: #1d1d1d; color: #ddd; cursor: pointer; transition: .15s; }
  .modes button:hover { background: #262626; }
  .modes button.active { background: #1f6feb; border-color: #1f6feb; color: #fff;
                         font-weight: 600; }
  .grid { display: grid; grid-template-columns: repeat(2, 1fr); gap: 8px 16px; }
  .grid div { background: #181818; padding: 10px 12px; border-radius: 6px; }
  .grid .k { color: #888; font-size: 12px; }
  .grid .v { font-size: 18px; font-variant-numeric: tabular-nums; }
  #pad { display: none; margin: 16px 0; }
  #pad.on { display: block; }
  #pad .row { display: flex; justify-content: center; gap: 10px; margin: 8px 0; }
  #pad button { width: 96px; height: 64px; font-size: 22px; border: 1px solid #333;
                border-radius: 10px; background: #1d1d1d; color: #eee; cursor: pointer;
                user-select: none; touch-action: none; }
  #pad button:active { background: #1f6feb; }
  #pad .stop { background: #5a1d1d; }
  #pad .hint { text-align: center; color: #888; font-size: 12px; margin-top: 6px; }
  .zbtn { width: 34px; height: 28px; border: 1px solid #333; border-radius: 6px;
          background: #1d1d1d; color: #ddd; cursor: pointer; font-size: 15px; }
  .zbtn:hover { background: #262626; }
</style>
</head>
<body>
<header>
  <span id="dot"></span>
  <h1>Ровер · фолловер</h1>
  <span id="modeName" style="margin-left:auto; color:#9cf;"></span>
</header>
<main>
  <div class="video"><img src="/stream.mjpg" alt="камера"></div>
  <div id="mapbox" style="display:none; margin-top:16px;">
    <div style="color:#888; font-size:12px; margin-bottom:6px; display:flex;
                align-items:center; gap:8px; flex-wrap:wrap;">
      <span>Карта (SLAM). В «Езда в точку» — клик по карте.</span>
      <span style="margin-left:auto;">зум:</span>
      <button id="zout" class="zbtn">−</button>
      <button id="zfit" class="zbtn">1:1</button>
      <button id="zin" class="zbtn">+</button>
      <button id="zcenter" class="zbtn" style="width:auto; padding:0 8px;">⌖ робот</button>
    </div>
    <div id="mapwrap" style="overflow:auto; width:min(100%,520px);
         height:min(70vh,520px); background:#000; border-radius:8px;
         touch-action:none;">
      <img id="map" src="/map.png" alt="карта"
           style="display:block; width:500px; image-rendering:pixelated;
                  cursor:grab; user-select:none;">
    </div>
  </div>
  <div class="modes" id="modes">
    <button data-m="idle">Ожидание</button>
    <button data-m="follow">Следование</button>
    <button data-m="goto">Езда в точку</button>
    <button data-m="mapping">Карта</button>
  </div>
  <div id="pad">
    <div class="row"><button data-l="160" data-r="160">▲</button></div>
    <div class="row">
      <button data-l="-210" data-r="210">◄</button>
      <button class="stop" data-l="0" data-r="0">СТОП</button>
      <button data-l="210" data-r="-210">►</button>
    </div>
    <div class="row"><button data-l="-160" data-r="-160">▼</button></div>
    <div class="hint">кнопки или клавиши W/A/S/D — удерживай, едет; отпустил — стоп (deadman 0.6с)</div>
  </div>
  <div class="grid">
    <div><div class="k">Людей в кадре</div><div class="v" id="people">--</div></div>
    <div><div class="k">FPS</div><div class="v" id="fps">--</div></div>
    <div><div class="k">Угол цели</div><div class="v" id="angle">--</div></div>
    <div><div class="k">Дистанция</div><div class="v" id="dist">--</div></div>
    <div><div class="k">Команда L / R</div><div class="v" id="lr">--</div></div>
    <div><div class="k">Лидар</div><div class="v" id="lidar">--</div></div>
    <div><div class="k">Точек лидара</div><div class="v" id="lpts">--</div></div>
    <div><div class="k">Ближайшая точка</div><div class="v" id="lnear">--</div></div>
    <div><div class="k">Курс IMU</div><div class="v" id="yaw">--</div></div>
    <div><div class="k">Поза (x, y, θ)</div><div class="v" id="pose">--</div></div>
  </div>
  <p id="status" style="color:#9cf; min-height:1.2em;"></p>
</main>
<script>
async function setMode(m) {
  await fetch('/mode?m=' + m, { method: 'POST' });
  poll();
}
document.querySelectorAll('#modes button').forEach(b =>
  b.onclick = () => setMode(b.dataset.m));

function drive(l, r) { fetch('/drive?l=' + l + '&r=' + r, { method: 'POST' }); }
let driveTimer = null;
function startDrive(l, r) {
  drive(l, r);
  clearInterval(driveTimer);
  driveTimer = setInterval(() => drive(l, r), 200);  // обновляем, иначе deadman остановит
}
function stopDrive() { clearInterval(driveTimer); driveTimer = null; drive(0, 0); }
document.querySelectorAll('#pad button').forEach(b => {
  const l = +b.dataset.l, r = +b.dataset.r;
  const start = e => { e.preventDefault(); (l || r) ? startDrive(l, r) : stopDrive(); };
  b.addEventListener('pointerdown', start);
  b.addEventListener('pointerup', stopDrive);
  b.addEventListener('pointerleave', stopDrive);
  b.addEventListener('pointercancel', stopDrive);
});

// WASD-телеоп: несколько клавиш сразу -> смешиваем (W+D = дуга вправо и т.п.).
const keys = { w: false, a: false, s: false, d: false };
let keyTimer = null;
const SPD = 160, ARC = 95, SPIN = 210;   // ход / доворот по дуге / разворот на месте
function keyCmd() {
  const f = (keys.w ? 1 : 0) - (keys.s ? 1 : 0);   // вперёд/назад
  const t = (keys.d ? 1 : 0) - (keys.a ? 1 : 0);   // +вправо / -влево
  if (!f && !t) return null;
  let l, r;
  if (!f) { l = t * SPIN; r = -t * SPIN; }          // разворот на месте
  else { l = f * SPD + t * ARC; r = f * SPD - t * ARC; }  // дуга
  const cl = v => Math.max(-255, Math.min(255, Math.round(v)));
  return [cl(l), cl(r)];
}
function keyTick() {
  const c = keyCmd();
  if (c) drive(c[0], c[1]); else stopKeys();
}
function startKeys() { if (!keyTimer) { keyTick(); keyTimer = setInterval(keyTick, 150); } }
function stopKeys() { clearInterval(keyTimer); keyTimer = null; drive(0, 0); }
document.addEventListener('keydown', e => {
  if (curMode !== 'mapping') return;
  const k = e.key.toLowerCase();
  if (!(k in keys)) return;
  e.preventDefault();
  if (!keys[k]) { keys[k] = true; startKeys(); }
});
document.addEventListener('keyup', e => {
  const k = e.key.toLowerCase();
  if (!(k in keys)) return;
  keys[k] = false;
  if (!keys.w && !keys.a && !keys.s && !keys.d) stopKeys();
});

// Зум карты + центрирование на роботе.
let mapZoom = 2, mapPx = 500, lastRobotPx = null, followRobot = true;
const mapWrap = document.getElementById('mapwrap');
function applyZoom() { mapImg.style.width = Math.round(mapPx * mapZoom) + 'px'; centerRobot(); }
function centerRobot() {
  if (!lastRobotPx) return;
  mapWrap.scrollLeft = lastRobotPx[0] * mapZoom - mapWrap.clientWidth / 2;
  mapWrap.scrollTop = lastRobotPx[1] * mapZoom - mapWrap.clientHeight / 2;
}
document.getElementById('zin').onclick = () => { mapZoom = Math.min(8, mapZoom * 1.4); applyZoom(); };
document.getElementById('zout').onclick = () => { mapZoom = Math.max(0.5, mapZoom / 1.4); applyZoom(); };
document.getElementById('zfit').onclick = () => { mapZoom = 1; applyZoom(); };
document.getElementById('zcenter').onclick = () => { followRobot = true; centerRobot(); };

// Перетаскивание карты мышью (pan) + клик-цель в goto (если не тащили).
let curMode = 'idle';
const mapImg = document.getElementById('map');
let dragging = false, dragMoved = 0, lastX = 0, lastY = 0;
mapImg.addEventListener('pointerdown', e => {
  dragging = true; dragMoved = 0; lastX = e.clientX; lastY = e.clientY;
  mapImg.setPointerCapture(e.pointerId); mapImg.style.cursor = 'grabbing';
});
mapImg.addEventListener('pointermove', e => {
  if (!dragging) return;
  const dx = e.clientX - lastX, dy = e.clientY - lastY;
  lastX = e.clientX; lastY = e.clientY; dragMoved += Math.abs(dx) + Math.abs(dy);
  if (dragMoved > 6) followRobot = false;   // ручная панорама -> перестать следить
  mapWrap.scrollLeft -= dx; mapWrap.scrollTop -= dy;
});
mapImg.addEventListener('pointerup', e => {
  dragging = false; mapImg.style.cursor = 'grab';
  if (dragMoved < 6 && curMode === 'goto') {   // это клик, не drag -> цель
    const rect = mapImg.getBoundingClientRect();
    const col = Math.round((e.clientX - rect.left) / rect.width * mapImg.naturalWidth);
    const row = Math.round((e.clientY - rect.top) / rect.height * mapImg.naturalHeight);
    fetch('/goal?col=' + col + '&row=' + row, { method: 'POST' });
  }
});
// Обновляем карту, пока виден её блок.
setInterval(() => {
  if (document.getElementById('mapbox').style.display !== 'none')
    mapImg.src = '/map.png?t=' + Date.now();
}, 500);

async function poll() {
  try {
    const s = await (await fetch('/state', { cache: 'no-store' })).json();
    document.getElementById('dot').classList.add('ok');
    document.getElementById('modeName').textContent = (s.mode_label || s.mode || '');
    curMode = s.mode;
    document.querySelectorAll('#modes button').forEach(b =>
      b.classList.toggle('active', b.dataset.m === s.mode));
    document.getElementById('pad').classList.toggle('on', s.mode === 'mapping');
    const mapOn = (s.mode === 'mapping' || s.mode === 'goto');
    const mapBox = document.getElementById('mapbox');
    const wasOff = mapBox.style.display === 'none';
    mapBox.style.display = mapOn ? 'block' : 'none';
    if (s.map_px) mapPx = s.map_px;
    if (s.robot_px) lastRobotPx = s.robot_px;
    if (mapOn && wasOff) { followRobot = true; setTimeout(applyZoom, 150); }
    else if (mapOn && followRobot) centerRobot();  // непрерывно следим за роботом
    document.getElementById('pose').textContent = s.pose
      ? s.pose.x.toFixed(2) + ', ' + s.pose.y.toFixed(2) + ', ' + s.pose.theta.toFixed(0) + '\\u00b0'
      : '--';
    document.getElementById('people').textContent = s.n_people ?? '--';
    document.getElementById('fps').textContent = (s.fps ?? 0).toFixed(0);
    document.getElementById('angle').textContent =
      s.target ? (s.target.angle >= 0 ? '+' : '') + s.target.angle.toFixed(0) + '\\u00b0' : '--';
    document.getElementById('dist').textContent =
      s.target && s.target.dist != null ? s.target.dist.toFixed(2) + ' м' : '--';
    document.getElementById('lr').textContent = (s.left ?? 0) + ' / ' + (s.right ?? 0);
    document.getElementById('lidar').textContent = s.lidar_ready ? 'готов' : 'нет';
    document.getElementById('lpts').textContent = s.lidar_returns ?? '--';
    document.getElementById('lnear').textContent =
      s.lidar_nearest ? s.lidar_nearest.dist.toFixed(2) + ' м @ ' + s.lidar_nearest.angle + '\\u00b0' : '--';
    document.getElementById('yaw').textContent =
      s.imu_yaw != null ? s.imu_yaw.toFixed(1) + '\\u00b0' : '--';
    document.getElementById('status').textContent = s.drive_status || '';
  } catch (e) {
    document.getElementById('dot').classList.remove('ok');
  }
}
poll();
setInterval(poll, 500);
</script>
</body>
</html>
"""
