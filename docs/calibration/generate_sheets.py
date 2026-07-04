"""Генератор печатного калибровочного комплекта ровера (A4, точный масштаб 1:1).

3 листа:
  1. Угломер-центровка — куда ставить робота (центр = ось лидара), ось ВПЕРЁД 0°,
     лучи углов (+вправо / −влево), линейка проверки масштаба печати.
  2. Лист замеров — процедуры: угол лидара, скорость↔ШИМ, поворот/колея, габарит.
  3. JetsonConfig — таблица значений «вписать» + чек-лист ввода в эксплуатацию.

Запуск:  uv run --no-project --with reportlab python docs/calibration/generate_sheets.py
Печать:  100% (БЕЗ «вписать в страницу»); проверь по линейке 100 мм на листе 1.
"""

from __future__ import annotations

import math
import os

from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.pdfgen import canvas

OUT = os.path.join(os.path.dirname(__file__), "rover_calibration_sheets.pdf")
W, H = A4  # точки (210×297 мм)

_REG = "/System/Library/Fonts/Supplemental/Arial.ttf"
_BOLD = "/System/Library/Fonts/Supplemental/Arial Bold.ttf"
pdfmetrics.registerFont(TTFont("Cal", _REG))
pdfmetrics.registerFont(TTFont("CalB", _BOLD if os.path.exists(_BOLD) else _REG))

INK = (0.12, 0.12, 0.12)
GREY = (0.55, 0.55, 0.55)
FAINT = (0.78, 0.78, 0.78)
ACCENT = (0.10, 0.42, 0.74)


def _ink(c: canvas.Canvas, rgb: tuple[float, float, float]) -> None:
    c.setStrokeColorRGB(*rgb)
    c.setFillColorRGB(*rgb)


def text(c, x, y, s, size=10, font="Cal", rgb=INK, align="left"):
    c.setFillColorRGB(*rgb)
    c.setFont(font, size)
    if align == "center":
        c.drawCentredString(x, y, s)
    elif align == "right":
        c.drawRightString(x, y, s)
    else:
        c.drawString(x, y, s)


# ============================================================ ЛИСТ 1 — УГЛОМЕР
def page_protractor(c: canvas.Canvas) -> None:
    cx, cy, R = 105 * mm, 174 * mm, 84 * mm

    text(c, cx, 285 * mm, "КАЛИБРОВКА РОВЕРА · ЛИСТ 1 — ЦЕНТРОВКА И УГЛОМЕР", 15, "CalB", align="center")
    text(c, cx, 278 * mm, "Печать 100% (без «вписать в страницу»). Проверь по линейке 100 мм внизу.",
         9, "Cal", GREY, align="center")

    # Оси через центр до краёв листа — продлить скотчем на полу, чтобы выставить лист квадратно.
    _ink(c, FAINT)
    c.setLineWidth(0.5)
    c.setDash([2, 3])
    c.line(8 * mm, cy, (210 - 8) * mm, cy)
    c.line(cx, 12 * mm, cx, 284 * mm)
    c.setDash([])

    # Направляющие окружности (визуальные, без метрики).
    _ink(c, FAINT)
    c.setLineWidth(0.6)
    for k in (0.33, 0.66):
        c.circle(cx, cy, R * k, stroke=1, fill=0)

    # Лучи углов: тонкие каждые 10°, толще каждые 30°.
    for d in range(0, 360, 10):
        rad = math.radians(d)
        x2, y2 = cx + R * math.sin(rad), cy + R * math.cos(rad)
        if d % 30 == 0:
            _ink(c, GREY)
            c.setLineWidth(0.9)
            c.line(cx, cy, x2, y2)
        else:
            _ink(c, FAINT)
            c.setLineWidth(0.5)
            xi, yi = cx + 0.93 * R * math.sin(rad), cy + 0.93 * R * math.cos(rad)
            c.line(xi, yi, x2, y2)

    # Внешняя окружность.
    _ink(c, INK)
    c.setLineWidth(1.2)
    c.circle(cx, cy, R, stroke=1, fill=0)

    # Числа углов каждые 30° (0/180 — словами выше/ниже). + вправо, − влево.
    labels = {30: "+30", 60: "+60", 90: "+90", 120: "+120", 150: "+150",
              210: "−150", 240: "−120", 270: "−90", 300: "−60", 330: "−30"}
    for d, lab in labels.items():
        rad = math.radians(d)
        lx, ly = cx + 1.05 * R * math.sin(rad), cy + 1.05 * R * math.cos(rad)
        text(c, lx, ly - 1.3 * mm, lab, 9, "CalB", align="center")

    # Подписи сторон (слова сверху/снизу, где места достаточно) + легенда знака.
    text(c, cx, cy + R + 10 * mm, "ВПЕРЁД (0°) — нос робота сюда", 11, "CalB", ACCENT, align="center")
    text(c, cx, cy + R + 4 * mm, "+ вправо   ·   − влево", 8.5, "Cal", GREY, align="center")
    text(c, cx, cy - R - 7 * mm, "НАЗАД 180°", 9, "CalB", align="center")

    # Жирная ось ВПЕРЁД со стрелкой.
    _ink(c, ACCENT)
    c.setLineWidth(2.0)
    c.line(cx, cy, cx, cy + R)
    c.line(cx, cy + R, cx - 2.5 * mm, cy + R - 4 * mm)
    c.line(cx, cy + R, cx + 2.5 * mm, cy + R - 4 * mm)

    # Центр: крест + кольцо + подпись.
    _ink(c, INK)
    c.setLineWidth(1.2)
    c.circle(cx, cy, 3 * mm, stroke=1, fill=0)
    c.line(cx - 6 * mm, cy, cx + 6 * mm, cy)
    c.line(cx, cy - 6 * mm, cx, cy + 6 * mm)
    text(c, cx + 5 * mm, cy + 5 * mm, "ЦЕНТР = ось вращения лидара", 9, "CalB", INK)

    # Линейка проверки масштаба печати (100 мм).
    ry = 74 * mm
    rx0 = (105 - 50) * mm
    _ink(c, INK)
    c.setLineWidth(1.0)
    c.line(rx0, ry, rx0 + 100 * mm, ry)
    for i in range(0, 101, 10):
        x = rx0 + i * mm
        h = 3.5 * mm if i % 50 == 0 else 2.2 * mm
        c.line(x, ry, x, ry + h)
    text(c, 105 * mm, ry - 4 * mm, "100 мм — проверь линейкой, что напечатано 1:1", 9, "Cal", GREY, align="center")

    # Инструкция.
    _ink(c, FAINT)
    c.setLineWidth(0.8)
    c.rect(12 * mm, 11 * mm, 186 * mm, 53 * mm, stroke=1, fill=0)
    text(c, 16 * mm, 57 * mm, "Как пользоваться", 11, "CalB", ACCENT)
    lines = [
        "1. Положи лист на ровный пол, закрепи. Поставь робот так, чтобы ось вращения лидара",
        "    совпала с ЦЕНТРОМ, а нос смотрел строго вдоль стрелки ВПЕРЁД (0°).",
        "2. Калибровка угла лидара: положи узкий предмет на луч 0° в ~1 м. В визуализации/логе",
        "    лидара посмотри, какой бин даёт дистанцию → это lidar_forward_bin_deg.",
        "3. Направление вращения: передвинь предмет к лучу +90° (вправо). Бин вырос → лидар CCW",
        "    (lidar_clockwise=False); бин уменьшился → CW (True). Записать на лист 2.",
        "4. Этот же угломер — опора для проверки поворота (лист 2, поворот 360°).",
    ]
    yy = 50 * mm
    for ln in lines:
        text(c, 16 * mm, yy, ln, 9, "Cal", INK)
        yy -= 5.4 * mm

    c.showPage()


# ====================================================== ЛИСТ 2 — ПРОЦЕДУРЫ
def _box(c, x, y, w, h, title):
    _ink(c, FAINT)
    c.setLineWidth(0.8)
    c.rect(x, y, w, h, stroke=1, fill=0)
    text(c, x + 4 * mm, y + h - 6.5 * mm, title, 11, "CalB", ACCENT)


def _field(c, x, y, label, unit=""):
    text(c, x, y, label, 9.5, "Cal", INK)
    _ink(c, GREY)
    c.setLineWidth(0.7)
    c.line(x + 52 * mm, y - 1 * mm, x + 78 * mm, y - 1 * mm)
    if unit:
        text(c, x + 80 * mm, y, unit, 9, "Cal", GREY)


def page_procedures(c: canvas.Canvas) -> None:
    text(c, 105 * mm, 285 * mm, "ЛИСТ 2 — ЗАМЕРЫ И ПРОЦЕДУРЫ", 15, "CalB", align="center")
    text(c, 105 * mm, 279 * mm, "Робот / дата: ___________________________      см. docs/ROVER_BRINGUP.md",
         9, "Cal", GREY, align="center")

    # A. Угол лидара.
    _box(c, 12 * mm, 232 * mm, 186 * mm, 40 * mm, "A. Угол и направление лидара (лист 1)")
    text(c, 16 * mm, 256 * mm, "Поставь робота на угломер. Найди бин по курсу (0°) и проверь сторону +90°.",
         9, "Cal", INK)
    _field(c, 16 * mm, 246 * mm, "lidar_forward_bin_deg", "°")
    _field(c, 110 * mm, 246 * mm, "lidar_clockwise (CW/CCW)")
    _field(c, 16 * mm, 238 * mm, "masked_sectors (self-hit)", "° напр. 170–190")

    # B. Привод: скорость ↔ ШИМ.
    _box(c, 12 * mm, 150 * mm, 186 * mm, 76 * mm, "B. Привод: скорость ↔ ШИМ (без энкодеров)")
    text(c, 16 * mm, 210 * mm, "Прогон по прямой: дай команду, замерь время и путь (рулетка/лидар), вычисли v=s/t.",
         9, "Cal", INK)
    # Таблица.
    tx, ty = 16 * mm, 204 * mm
    cols = [("команда v, м/с", 30), ("ШИМ pwm", 28), ("время t, с", 26), ("путь s, м", 26), ("v_изм = s/t", 30)]
    _ink(c, GREY)
    c.setLineWidth(0.6)
    x = tx
    for name, wmm in cols:
        text(c, x + 1.5 * mm, ty, name, 8.5, "CalB", INK)
        x += wmm * mm
    for r in range(3):
        yrow = ty - (r + 1) * 8 * mm
        c.line(tx, yrow, tx + 140 * mm, yrow)
    x = tx
    for _, wmm in cols:
        c.line(x, ty + 5 * mm, x, ty - 24 * mm)
        x += wmm * mm
    c.line(x, ty + 5 * mm, x, ty - 24 * mm)
    c.line(tx, ty + 5 * mm, tx + 140 * mm, ty + 5 * mm)

    text(c, 16 * mm, 168 * mm, "Порог трогания: уменьшай ШИМ, пока мотор не перестаёт крутиться → pwm_min_move.",
         9, "Cal", INK)
    text(c, 16 * mm, 162 * mm, "Поворот: дай угловую скорость, замерь время полного оборота 360° (по угломеру).",
         9, "Cal", INK)
    _field(c, 16 * mm, 154 * mm, "max_linear_m_s", "м/с")
    _field(c, 110 * mm, 154 * mm, "pwm_max")

    # C. Габарит.
    _box(c, 12 * mm, 92 * mm, 186 * mm, 52 * mm, "C. Габарит робота (footprint)")
    text(c, 16 * mm, 128 * mm, "Измерь корпус. robot_radius = полудиагональ + запас 0.05 м.", 9, "Cal", INK)
    # Эскиз прямоугольника с диагональю.
    rx, ry, rw, rh = 150 * mm, 100 * mm, 34 * mm, 22 * mm
    _ink(c, GREY)
    c.setLineWidth(0.9)
    c.rect(rx, ry, rw, rh, stroke=1, fill=0)
    c.line(rx, ry, rx + rw, ry + rh)
    text(c, rx + rw / 2, ry - 4 * mm, "длина", 8, "Cal", GREY, align="center")
    text(c, rx + rw + 2 * mm, ry + rh / 2, "ширина", 8, "Cal", GREY)
    _field(c, 16 * mm, 118 * mm, "длина корпуса", "м")
    _field(c, 16 * mm, 110 * mm, "ширина корпуса", "м")
    _field(c, 16 * mm, 102 * mm, "robot_radius_m", "м")
    _field(c, 16 * mm, 96 * mm, "wheel_base_m (колея)", "м")

    # D. Камера/прочее.
    _box(c, 12 * mm, 52 * mm, 186 * mm, 34 * mm, "D. Камера и питание")
    _field(c, 16 * mm, 72 * mm, "hfov_rad (гориз. FOV)", "рад")
    _field(c, 110 * mm, 72 * mm, "camera_index")
    _field(c, 16 * mm, 62 * mm, "датчик заряда?", "ADC ESP / INA219 / нет")
    _field(c, 16 * mm, 56 * mm, "pwm_min_move")

    text(c, 105 * mm, 44 * mm, "Перенеси значения на лист 3 и в JetsonConfig.", 9, "Cal", GREY, align="center")
    c.showPage()


# ===================================================== ЛИСТ 3 — JetsonConfig
def page_config(c: canvas.Canvas) -> None:
    text(c, 105 * mm, 285 * mm, "ЛИСТ 3 — JetsonConfig: ВПИСАТЬ ЗНАЧЕНИЯ", 15, "CalB", align="center")
    text(c, 105 * mm, 279 * mm, "Из листа 2 → в src/greenhouse/adapters/jetson/loop.py (JetsonConfig)",
         9, "Cal", GREY, align="center")

    rows = [
        ("lidar_forward_bin_deg", "бин лидара, смотрящий вперёд (°)"),
        ("lidar_clockwise", "направление вращения (True=CW)"),
        ("lidar_masked_sectors", "секторы self-hit, напр. ((170,190),)"),
        ("wheel_base_m", "колея, м"),
        ("max_linear_m_s", "безопасная макс. скорость, м/с"),
        ("max_angular_rad_s", "макс. угловая, рад/с"),
        ("pwm_max", "ШИМ при max_linear"),
        ("pwm_min_move", "порог трогания мотора"),
        ("robot_radius_m", "описанный радиус + 0.05 м"),
        ("hfov_rad", "горизонтальный FOV камеры, рад"),
        ("use_imu", "True, когда заведён реальный IMU"),
        ("камера / порт мотора", "camera_index, порт ESP (USB-serial)"),
        ("датчик заряда", "BatterySource (ADC/INA219) или нет"),
        ("док (dock Pose2D)", "поза док-станции на карте"),
    ]
    top = 268 * mm
    rowh = 9.5 * mm
    x0, wlab, wval = 14 * mm, 56 * mm, 38 * mm
    _ink(c, GREY)
    c.setLineWidth(0.6)
    for i, (key, desc) in enumerate(rows):
        y = top - i * rowh
        c.line(x0, y - rowh + 2 * mm, x0 + 182 * mm, y - rowh + 2 * mm)
        text(c, x0 + 1 * mm, y - 5 * mm, key, 9.5, "CalB", INK)
        # поле значения
        c.setLineWidth(0.7)
        c.line(x0 + wlab, y - 5.5 * mm, x0 + wlab + wval, y - 5.5 * mm)
        c.setLineWidth(0.6)
        text(c, x0 + wlab + wval + 3 * mm, y - 5 * mm, desc, 8.5, "Cal", GREY)
    c.line(x0, top + 2 * mm, x0 + 182 * mm, top + 2 * mm)

    # Чек-лист ввода в эксплуатацию.
    cy = top - len(rows) * rowh - 6 * mm
    _ink(c, FAINT)
    c.setLineWidth(0.8)
    c.rect(12 * mm, cy - 60 * mm, 186 * mm, 60 * mm, stroke=1, fill=0)
    text(c, 16 * mm, cy - 7 * mm, "Чек-лист ввода (docs/ROVER_BRINGUP.md)", 11, "CalB", ACCENT)
    steps = [
        "Прошита ESP, питание, лидар/камера видны (check_devices).",
        "Шаг 0–1: телеоп — едет/крутится; failsafe держит; угол лидара откалиброван.",
        "Шаг 2: скорость↔ШИМ и колея сняты на стенде.",
        "Шаг 4: локализация (scan-match + IMU) держит позу; теряет → стоп.",
        "Шаг 5: следование «за угол» (last-seen → поиск → ожидание).",
        "Шаг 6–8: карта (save/load), GoTo, keep-out, зарядка.",
        "Шаг 9–10: failsafe-ветки проверены; докинг.",
    ]
    yy = cy - 15 * mm
    for s in steps:
        _ink(c, GREY)
        c.setLineWidth(0.9)
        c.rect(16 * mm, yy - 3 * mm, 4 * mm, 4 * mm, stroke=1, fill=0)
        text(c, 22 * mm, yy, s, 9, "Cal", INK)
        yy -= 7 * mm

    text(c, 105 * mm, 14 * mm, "Команды у робота: UDP/stdin — go_to X Y · follow · map · charge · stop · keepout add/remove",
         8.5, "Cal", GREY, align="center")
    c.showPage()


def main() -> None:
    c = canvas.Canvas(OUT, pagesize=A4)
    c.setTitle("Калибровочный комплект ровера")
    page_protractor(c)
    page_procedures(c)
    page_config(c)
    c.save()
    print("written:", OUT)


if __name__ == "__main__":
    main()
