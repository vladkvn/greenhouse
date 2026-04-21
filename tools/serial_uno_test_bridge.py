#!/usr/bin/env python3
"""
Bridge between the Arduino Uno (USB serial) and the rest of the stack.

- Reads newline-delimited JSON from the serial port (telemetry from the firmware).
- Prints each line to stdout.
- Sends a JSON command back on the serial port so the LCD can show `l1` / `l2`.

Later you can extend this script to publish telemetry to MQTT and subscribe for real server commands.
"""

from __future__ import annotations

import argparse
import json
import sys

import serial


def main() -> int:
    parser = argparse.ArgumentParser(description="USB serial test bridge for arduino-uno-node")
    parser.add_argument(
        "--port",
        required=True,
        help="Serial device, e.g. /dev/tty.usbmodem1101 (macOS) or COM5 (Windows)",
    )
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args()

    seq = 0

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.2)
    except serial.SerialException as exc:
        print(f"Could not open serial port: {exc}", file=sys.stderr)
        return 2

    print(f"Opened {args.port} @ {args.baud}. Waiting for lines from Arduino...", flush=True)

    while True:
        try:
            raw = ser.readline()
        except serial.SerialException as exc:
            print(f"Serial read failed: {exc}", file=sys.stderr)
            return 3

        if not raw:
            continue

        line = raw.decode("utf-8", errors="replace").strip()
        if not line:
            continue

        print(line, flush=True)

        try:
            msg = json.loads(line)
        except json.JSONDecodeError:
            continue

        if msg.get("msgType") != "telemetry":
            continue

        seq += 1
        cmd = {
            "schemaVersion": 1,
            "msgType": "command",
            "l1": f"USB tick {seq}",
            "l2": "host bridge",
        }
        out = json.dumps(cmd, separators=(",", ":")) + "\n"
        ser.write(out.encode("utf-8"))
        ser.flush()
        print(f">>> sent: {out.strip()}", flush=True)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
