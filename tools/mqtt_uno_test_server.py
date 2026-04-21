#!/usr/bin/env python3
"""
Minimal 'server' for the Arduino Uno test node: listens for telemetry and publishes
a JSON command payload the firmware shows on the LCD (fields l1, l2).

Environment:
  MQTT_HOST (default: 127.0.0.1)
  MQTT_PORT (default: 1883)
"""

from __future__ import annotations

import json
import os
import re
import sys

import paho.mqtt.client as mqtt

TOPIC_TELEMETRY_RE = re.compile(r"^greenhouse/nodes/([^/]+)/telemetry$")


def _extract_node_id(topic: str) -> str | None:
    match = TOPIC_TELEMETRY_RE.match(topic)
    if match is None:
        return None
    return match.group(1)


def main() -> int:
    host = os.environ.get("MQTT_HOST", "127.0.0.1")
    port = int(os.environ.get("MQTT_PORT", "1883"))

    seq_holder = {"seq": 0}

    def on_connect(client: mqtt.Client, _userdata: object, _flags: dict[str, int], rc: int) -> None:
        if rc != 0:
            print(f"MQTT connect failed: rc={rc}", file=sys.stderr)
            return
        client.subscribe("greenhouse/nodes/+/telemetry", qos=0)
        print("Connected; subscribed to greenhouse/nodes/+/telemetry")

    def on_message(client: mqtt.Client, _userdata: object, msg: mqtt.MQTTMessage) -> None:
        topic = msg.topic or ""
        node_id = _extract_node_id(topic)
        if node_id is None:
            return

        seq_holder["seq"] += 1
        seq = seq_holder["seq"]
        cmd_topic = f"greenhouse/nodes/{node_id}/commands"
        payload = {
            "schemaVersion": 1,
            "msgType": "command",
            "l1": f"tick {seq}",
            "l2": "from test srv",
        }
        body = json.dumps(payload, separators=(",", ":"))
        client.publish(cmd_topic, body, qos=0, retain=False)
        print(f"Published command to {cmd_topic}: {body}")

    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message

    print(f"Connecting to {host}:{port} ...")
    client.connect(host, port, keepalive=30)
    client.loop_forever()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
