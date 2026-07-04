#!/usr/bin/env python3
"""Диагностика ESP32: стартует ли прошивка (баннер 'READY') при разных сбросах.

Гипотеза спина «на максимуме при подаче питания»: при открытии порта DTR/RTS оставляют ESP
в загрузчике → firmware не исполняется → моторные strapping-пины (12/14/15) держат драйвер.
Проверяем, появляется ли 'READY' (a) при обычном открытии, (b) после явного reset-into-RUN.
"""
import sys
import time

import serial

port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"


def read_boot(s, label):
    time.sleep(2.0)
    data = s.read(2000)
    ready = b"READY" in data
    print(f"[{label}] READY={ready} out={data[:160]!r}")
    return ready


s = serial.Serial(port, 115200, timeout=0.3)

# (a) как открывает мост (DTR/RTS asserted по умолчанию)
read_boot(s, "open-default")

# (b) reset-into-RUN: GPIO0=high (DTR False), пульс EN (RTS True->False)
s.dtr = False
s.rts = True
time.sleep(0.1)
s.rts = False
read_boot(s, "reset-into-run")

# канал жив? шлём STOP (прошивка не эхоит, но проверим отсутствие ошибок)
s.write(b"STOP\n")
time.sleep(0.3)
print("after STOP tail:", s.read(200)[:80])
s.close()
