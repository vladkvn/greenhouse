"""Проверка направлений привода по USB-serial: не перепутаны ли право/лево и вперёд/назад.

Гоняет ИЗОЛИРОВАННЫЕ движения с подписями ожидаемого поведения. Между фазами — стоп и
пауза, чтобы ты успел посмотреть и записать, что реально произошло. По расхождениям правится
прошивка (маппинг бортов / направление колеса) — см. комментарии в firmware_esp32_serial.ino.

ВНИМАНИЕ: ПРИПОДНИМИ КОЛЁСА (поставь на подставку) — робот будет дёргаться.

Запуск на Jetson (порт ESP — см. `ls -l /dev/serial/by-id/`, CH340):
    python -m greenhouse.adapters.jetson.direction_check --port /dev/ttyUSB1
"""

from __future__ import annotations

import argparse
import time

from greenhouse.adapters.jetson.motion import Esp32SerialLink, MotorLink

# (подпись, скорость левого борта, скорость правого борта)
_PHASES: list[tuple[str, int, int]] = [
    ("1) ОБА борта ВПЕРЁД      -> робот должен ехать ВПЕРЁД", +1, +1),
    ("2) ОБА борта НАЗАД       -> робот должен ехать НАЗАД", -1, -1),
    ("3) ТОЛЬКО ЛЕВЫЙ вперёд   -> крутится ЛЕВОЕ колесо; робот доворачивает ВПРАВО", +1, 0),
    ("4) ТОЛЬКО ПРАВЫЙ вперёд  -> крутится ПРАВОЕ колесо; робот доворачивает ВЛЕВО", 0, +1),
    ("5) ПОВОРОТ на месте ВЛЕВО  (L назад, R вперёд)", -1, +1),
    ("6) ПОВОРОТ на месте ВПРАВО (L вперёд, R назад)", +1, -1),
]


def run_direction_check(
    link: MotorLink,
    *,
    speed: int = 150,
    phase_s: float = 2.0,
    rate_hz: float = 10.0,
    interactive: bool = True,
) -> None:
    """Прогнать фазы проверки направлений. Между фазами стоп и (опц.) ожидание Enter."""
    dt = 1.0 / rate_hz
    print("Приподними колёса! Поехали. Ctrl-C — стоп.\n")
    try:
        for label, ls, rs in _PHASES:
            left, right = ls * speed, rs * speed
            print(f"{label}    [L={left:+4d} R={right:+4d}]")
            t_end = time.monotonic() + phase_s
            while time.monotonic() < t_end:
                link.drive(left=left, right=right)  # пере-отправка: failsafe ESP = 0.5с
                time.sleep(dt)
            link.stop()
            if interactive:
                input("   запиши, что реально произошло, и нажми Enter для следующей фазы...")
            else:
                time.sleep(0.8)
    finally:
        link.stop()
        print("\nГотово. Сверь увиденное с подписями:")
        print(" - едет НАЗАД на фазе 1  -> ОБА борта инвертированы (поменяй M0<->M1 в обоих driveSide)")
        print(" - на фазе 3 крутится ПРАВОЕ колесо -> борта перепутаны (поменяй строки в setMotors)")
        print(" - одно колесо крутит не туда -> поменяй M0<->M1 в его driveSide")


def main() -> None:
    p = argparse.ArgumentParser(description="ESP32 motor direction check (USB serial)")
    p.add_argument("--port", default="/dev/ttyUSB0", help="serial-порт ESP (CH340)")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--speed", type=int, default=150, help="ШИМ 0..255 для теста")
    p.add_argument("--phase", type=float, default=2.0, help="длительность фазы, с")
    p.add_argument("--auto", action="store_true", help="без ожидания Enter между фазами")
    args = p.parse_args()

    link = Esp32SerialLink(port_path=args.port, baud=args.baud)
    try:
        run_direction_check(link, speed=args.speed, phase_s=args.phase, interactive=not args.auto)
    finally:
        link.close()


if __name__ == "__main__":
    main()
