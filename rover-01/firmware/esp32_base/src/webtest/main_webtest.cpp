// rover-01 — сборка webtest: управление роботом из браузера (без Jetson/ROS).
// ------------------------------------------------------------
// ESP32 поднимает свою Wi-Fi-сеть (AP) и веб-страницу-пульт.
// Кнопки/слайдеры шлют HTTP-команды, которые дёргают ТУ ЖЕ моторную логику,
// что и /cmd_vel в ROS-сборке (motor_control.*). Поэтому для робота это
// неотличимо от команд по ROS — включая watchdog.
//
// Назначение: тестовый стенд этапов 0–2 (см. docs/05-test-plan.md) с ноутбука,
// до того как поднят Jetson и micro-ROS агент.
// ------------------------------------------------------------

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>

#include "config.h"
#include "motor_control.h"

WebServer server(WEB_PORT);
unsigned long last_tick_ms = 0;

// Браузер шлёт команды потоком, как ROS. Если поток прервётся (закрыли вкладку,
// ушёл Wi-Fi) — watchdog в motorsTick() сам остановит моторы через CMD_TIMEOUT_MS.

// ───────────────────────── Веб-страница (пульт) ─────────────────────────
const char INDEX_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html><html lang="ru"><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>rover-01 — пульт</title>
<style>
  :root{color-scheme:dark}
  body{font-family:system-ui,sans-serif;background:#111;color:#eee;margin:0;padding:16px;text-align:center}
  h1{font-size:18px;font-weight:600;margin:4px 0 12px}
  .grid{display:grid;grid-template-columns:repeat(3,90px);grid-gap:10px;justify-content:center;margin:14px 0}
  button{font-size:22px;padding:18px 0;border:0;border-radius:12px;background:#2a2a2a;color:#eee;user-select:none;touch-action:none}
  button:active{background:#3a6df0}
  .stop{background:#b22; grid-column:2}
  .row{margin:14px auto;max-width:320px;text-align:left;font-size:14px}
  input[type=range]{width:100%}
  .val{float:right;color:#9ad}
  #st{margin-top:12px;font-size:13px;color:#9a9}
  .hint{color:#888;font-size:12px;margin-top:10px;line-height:1.4}
</style></head><body>
<h1>rover-01 — веб-пульт</h1>

<div class="row">Скорость <span class="val" id="lv">0.20</span> м/с
  <input type="range" id="lin" min="0" max="__LMAX__" step="0.01" value="0.2"></div>
<div class="row">Поворот <span class="val" id="av">1.00</span> рад/с
  <input type="range" id="ang" min="0" max="__AMAX__" step="0.05" value="1.0"></div>

<div class="grid">
  <div></div>
  <button id="fwd">▲</button>
  <div></div>
  <button id="left">◀</button>
  <button class="stop" id="stop">■</button>
  <button id="right">▶</button>
  <div></div>
  <button id="back">▼</button>
  <div></div>
</div>

<div id="st">состояние: —</div>
<div class="hint">Удерживай кнопку — робот едет. Отпустил — стоп.<br>
Закрытие вкладки = автостоп (watchdog).</div>

<script>
const lin=document.getElementById('lin'), ang=document.getElementById('ang');
const lv=document.getElementById('lv'), av=document.getElementById('av'), st=document.getElementById('st');
lin.oninput=()=>lv.textContent=(+lin.value).toFixed(2);
ang.oninput=()=>av.textContent=(+ang.value).toFixed(2);

let cur=null, timer=null;
function send(lx,az){
  fetch(`/cmd?lx=${lx}&az=${az}`).then(r=>r.text()).then(t=>st.textContent='состояние: '+t).catch(()=>st.textContent='нет связи');
}
function start(lx,az){ cur={lx,az}; send(lx,az); if(!timer) timer=setInterval(()=>{ if(cur) send(cur.lx,cur.az); },100); }
function stop(){ cur=null; if(timer){clearInterval(timer);timer=null;} send(0,0); }

function bind(id, fn){
  const b=document.getElementById(id);
  const on=e=>{e.preventDefault(); fn();};
  b.addEventListener('pointerdown',on);
  b.addEventListener('pointerup',stop);
  b.addEventListener('pointerleave',stop);
  b.addEventListener('pointercancel',stop);
}
bind('fwd', ()=>start(+lin.value, 0));
bind('back',()=>start(-lin.value, 0));
bind('left',()=>start(0, +ang.value));
bind('right',()=>start(0, -ang.value));
document.getElementById('stop').addEventListener('pointerdown',e=>{e.preventDefault();stop();});
</script>
</body></html>
)HTML";

// ───────────────────────── Обработчики ─────────────────────────
void handleRoot() {
  String page = FPSTR(INDEX_HTML);
  page.replace("__LMAX__", String(WEB_MAX_LINEAR, 2));
  page.replace("__AMAX__", String(WEB_MAX_ANGULAR, 2));
  server.send(200, "text/html", page);
}

// /cmd?lx=<linear>&az=<angular> — действует как пришедший /cmd_vel.
void handleCmd() {
  float lx = server.hasArg("lx") ? server.arg("lx").toFloat() : 0.0f;
  float az = server.hasArg("az") ? server.arg("az").toFloat() : 0.0f;

  // подстраховка: ограничим веб-команды настроенными пределами
  if (lx >  WEB_MAX_LINEAR)  lx =  WEB_MAX_LINEAR;
  if (lx < -WEB_MAX_LINEAR)  lx = -WEB_MAX_LINEAR;
  if (az >  WEB_MAX_ANGULAR) az =  WEB_MAX_ANGULAR;
  if (az < -WEB_MAX_ANGULAR) az = -WEB_MAX_ANGULAR;

  motorsSetTarget(lx, az);
  server.send(200, "text/plain", "lx=" + String(lx, 2) + " az=" + String(az, 2));
}

void handleStop() {
  motorsStopHard();
  server.send(200, "text/plain", "STOP");
}

// ───────────────────────── setup / loop ─────────────────────────
void setup() {
  motorsBegin();   // пины, ШИМ, безопасный стоп

  Serial.begin(115200);
  delay(200);

  // Поднимаем Access Point.
  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASSWORD, WIFI_AP_CHANNEL);
  IPAddress ip = WiFi.softAPIP();   // обычно 192.168.4.1

  Serial.println();
  Serial.print("AP SSID: "); Serial.println(WIFI_AP_SSID);
  Serial.print("Open:    http://"); Serial.println(ip);

  server.on("/", handleRoot);
  server.on("/cmd", handleCmd);
  server.on("/stop", handleStop);
  server.begin();

  last_tick_ms = millis();
}

void loop() {
  server.handleClient();

  // Управляющий тик с фиксированным периодом (watchdog + slew + моторы).
  if (millis() - last_tick_ms >= CONTROL_PERIOD_MS) {
    last_tick_ms = millis();
    motorsTick();
  }
}
