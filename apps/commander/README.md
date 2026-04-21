# GreenHouse Commander

Small NestJS service that accepts **commands for a device by `deviceId`**, looks up **`lastKnownIp`** in the GreenHouse API, and forwards the command to the ESP8266 HTTP endpoint (`GET /command?cmd=...`).

## Configuration (environment)

| Variable | Description |
|----------|-------------|
| `PORT` / `COMMANDER_PORT` | Listen port (default **3001**) |
| `GREENHOUSE_API_URL` | Base URL of the API (e.g. `http://localhost:3000` or `http://api:3000` in Docker) |
| `GREENHOUSE_API_KEY` | Same secret as **`API_KEY`** on the API (used for `GET /devices/by-id/...`) |
| `COMMANDER_API_KEY` | Secret required from clients of this service (`X-API-Key` header) |
| `COMMAND_TOKEN` | Optional; if set, appended as `&token=` when calling the device (must match `COMMAND_TOKEN` on the ESP8266 sketch) |
| `DEVICE_HTTP_PORT` | Port on the device (default **80**) |
| `DEVICE_HTTP_TIMEOUT_MS` | HTTP timeout when calling the device (default **10000**) |

## Endpoint

`POST /commands`

**Headers:** `X-API-Key: <COMMANDER_API_KEY>`, `Content-Type: application/json`

**Body**

```json
{
  "deviceId": "gh-node-1",
  "cmd": "PING"
}
```

Allowed `cmd` values: `PING`, `RELAY_ON`, `RELAY_OFF` (same whitelist as the firmware).

**Response** (example): `deviceHttpStatus` is the HTTP status from the ESP8266.

```json
{
  "ok": true,
  "deviceId": "gh-node-1",
  "cmd": "PING",
  "deviceHttpStatus": 200,
  "deviceResponse": "{\"ok\":true,\"cmd\":\"PING\"}"
}
```

## Docker

Use the root [docker-compose.yml](../../docker-compose.yml); Commander is exposed on port **3001**.

## LAN routing

The Commander container must be able to open **TCP to the device’s `lastKnownIp` on your LAN**. If that fails from Docker, try host networking on Linux, or run the stack so the container shares the host network path to IoT devices (Docker Desktop on macOS/Windows often allows this; if not, document your network setup).

## Local development

```bash
npm install
cp .env.example .env
npm run start:dev
```

`.env` should set `GREENHOUSE_API_URL`, `GREENHOUSE_API_KEY`, and `COMMANDER_API_KEY`.
