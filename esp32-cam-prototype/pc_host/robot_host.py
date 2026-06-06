#!/usr/bin/env python3
"""
GreenHouse — ПК-хост для прототипа робота на ESP32-CAM
------------------------------------------------------
Что делает:
  1. Ловит MJPEG-видеопоток с ESP32-CAM через OpenCV.
  2. Прогоняет каждый кадр через функцию process_frame() (сейчас — заглушка,
     сюда подключаешь свою обработку: детекция, навигация и т.д.).
  3. По результату обработки шлёт команды управления моторами на ESP32 по UDP.
  4. Показывает окно с видео; можно рулить вручную с клавиатуры.

Управление с клавиатуры (когда окно видео в фокусе):
  W/S — вперёд/назад,  A/D — поворот,  ПРОБЕЛ — стоп,
  M   — переключить ручной / авто (заглушка) режим,
  Q или ESC — выход (моторы глушатся).

Запуск:
  pip install opencv-python numpy
  python robot_host.py --host 192.168.1.50      # IP твоей ESP32-CAM

Можно работать без робота, для проверки логики:
  python robot_host.py --host 0.0.0.0 --source webcam   # вебка ноута вместо ESP32
"""

import argparse
import socket
import sys
import time

import cv2
import numpy as np


# ----------------------- Связь с роботом -----------------------
class RobotLink:
    """Отправка команд моторам по UDP. Шлёт строку 'L R' (-255..255)."""

    def __init__(self, host: str, port: int = 4210):
        self.addr = (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._last = (None, None)

    def drive(self, left: int, right: int):
        left = int(max(-255, min(255, left)))
        right = int(max(-255, min(255, right)))
        if (left, right) == self._last:
            return  # не спамим одинаковыми пакетами
        self._last = (left, right)
        try:
            self.sock.sendto(f"{left} {right}".encode(), self.addr)
        except OSError as e:
            print(f"[warn] не удалось отправить команду: {e}", file=sys.stderr)

    def stop(self):
        self._last = (None, None)
        try:
            self.sock.sendto(b"STOP", self.addr)
        except OSError:
            pass


# ----------------------- Обработка кадра -----------------------
def process_frame(frame, state):
    """
    СЮДА подключаешь свою логику. На вход — кадр BGR (numpy-массив).
    Должна вернуть (left, right) — скорости моторов -255..255 — и (опц.)
    кадр для отрисовки.

    Заглушка ниже: ищет самый яркий регион в кадре и "поворачивается" к нему
    (демонстрация замкнутого контура камера->обработка->моторы). Замени на своё.
    """
    h, w = frame.shape[:2]
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (21, 21), 0)
    _, _, _, max_loc = cv2.minMaxLoc(gray)
    cx = max_loc[0]

    cv2.circle(frame, max_loc, 12, (0, 0, 255), 2)

    if not state["auto"]:
        return None, frame  # в ручном режиме обработка не управляет

    # Пропорциональный поворот к яркой точке.
    err = (cx - w / 2) / (w / 2)        # -1..1
    base = 120
    turn = int(err * 120)
    left = base + turn
    right = base - turn
    return (left, right), frame


# ----------------------- Источник видео -----------------------
def open_source(args):
    if args.source == "webcam":
        cap = cv2.VideoCapture(0)
    else:
        url = f"http://{args.host}:{args.stream_port}/stream"
        print(f"[info] открываю поток: {url}")
        cap = cv2.VideoCapture(url)
    if not cap.isOpened():
        print("[error] не удалось открыть источник видео", file=sys.stderr)
        sys.exit(1)
    return cap


# ----------------------- Ручное управление -----------------------
def manual_command(key, base=200):
    if key in (ord('w'), ord('W')):
        return (base, base)
    if key in (ord('s'), ord('S')):
        return (-base, -base)
    if key in (ord('a'), ord('A')):
        return (-base, base)
    if key in (ord('d'), ord('D')):
        return (base, -base)
    if key == ord(' '):
        return (0, 0)
    return None


def main():
    ap = argparse.ArgumentParser(description="ПК-хост для ESP32-CAM робота")
    ap.add_argument("--host", required=True, help="IP ESP32-CAM (для UDP-команд)")
    ap.add_argument("--port", type=int, default=4210, help="UDP-порт команд")
    ap.add_argument("--stream-port", type=int, default=81, help="порт MJPEG-стрима")
    ap.add_argument("--source", choices=["esp32", "webcam"], default="esp32",
                    help="источник видео (webcam — для отладки без робота)")
    args = ap.parse_args()

    link = RobotLink(args.host, args.port)
    cap = open_source(args)
    state = {"auto": False}

    print("[info] W/S/A/D — движение, ПРОБЕЛ — стоп, M — авто/ручной, Q — выход")
    fps_t, fps_n, fps = time.time(), 0, 0.0

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("[warn] кадр не получен, повтор...", file=sys.stderr)
                time.sleep(0.1)
                continue

            cmd, vis = process_frame(frame, state)
            vis = vis if vis is not None else frame

            # FPS
            fps_n += 1
            if time.time() - fps_t >= 1.0:
                fps = fps_n / (time.time() - fps_t)
                fps_t, fps_n = time.time(), 0
            mode = "AUTO" if state["auto"] else "MANUAL"
            cv2.putText(vis, f"{mode}  {fps:4.1f} fps", (8, 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imshow("GreenHouse ESP32-CAM", vis)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), ord('Q'), 27):
                break
            if key in (ord('m'), ord('M')):
                state["auto"] = not state["auto"]
                link.stop()
                continue

            mcmd = manual_command(key)
            if mcmd is not None:
                link.drive(*mcmd)
            elif state["auto"] and cmd is not None:
                link.drive(*cmd)

    finally:
        link.stop()
        cap.release()
        cv2.destroyAllWindows()
        print("[info] остановлено, моторы заглушены")


if __name__ == "__main__":
    main()
