#!/usr/bin/env python3
"""Восстановление RPLIDAR A1 из залипшего режима сканирования.

follow_me умер, не послав STOP → лидар гонит скан бесконечно и игнорирует мягкий STOP.
Сначала пробуем STOP (0xA5 0x25); если поток не прекратился — RESET (0xA5 0x40, перезагрузка
устройства) с полным сливом загрузочного баннера, затем проверяем ответ GET_HEALTH (a5 5a...).

Использование: python3 lidar_reset.py [/dev/ttyUSB1]
"""
import sys
import time

import serial

port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB1"
s = serial.Serial(port, 115200, timeout=1)
time.sleep(0.3)
print("streaming_before =", len(s.read(400)))

s.write(b"\xA5\x25")            # STOP scan
time.sleep(0.3)
s.reset_input_buffer()
after_stop = len(s.read(400))
print("streaming_after_stop =", after_stop)

if after_stop > 50:
    print("STOP не сработал -> RESET (перезагрузка лидара)")
    s.write(b"\xA5\x40")        # RESET core
    time.sleep(2.0)
    print("banner_bytes =", len(s.read(4000)))   # слить загрузочный баннер
    s.reset_input_buffer()
    time.sleep(0.3)
    print("streaming_after_reset =", len(s.read(400)), "  (≈0 = лидар в покое)")

s.reset_input_buffer()
s.write(b"\xA5\x52")            # GET_HEALTH
time.sleep(0.3)
print("health =", s.read(20).hex(), "  (a55a... = устройство отвечает на команды)")
s.close()
print("done")
