# GreenHouse API (NestJS + PostgreSQL)

HTTP API for registering Arduino-class devices and ingesting sensor readings. Devices authenticate with a shared secret via the `X-API-Key` header.

## Prerequisites

- Node.js 20+
- Docker (recommended: full stack — [../../docker-compose.yml](../../docker-compose.yml) at repo root)

## Docker (API + Postgres + Mosquitto + Commander)

From the **repository root**:

```bash
cp .env.example .env
# set API_KEY and COMMANDER_API_KEY in .env
docker compose up --build -d
```

- API: `http://localhost:3000`
- Commander (send commands to devices by `deviceId`): `http://localhost:3001`
- Postgres on the host: port **5433** (maps to 5432 in the container) — use this in `DATABASE_URL` when connecting tools from your machine.

## Setup (local Node, without Docker for the API)

1. Copy environment file:

   ```bash
   cp .env.example .env
   ```

2. Edit `.env`: set `API_KEY` to a strong secret and ensure `DATABASE_URL` matches your Postgres instance.

3. Start PostgreSQL (e.g. root `docker compose up -d postgres` only, or use the full stack above).

4. Install dependencies and apply migrations:

   ```bash
   npm install
   npx prisma migrate deploy
   ```

5. Run the API:

   ```bash
   npm run start:dev
   ```

Default URL: `http://localhost:3000` (override with `PORT` in `.env`).

## Endpoints

### `POST /devices/register`

Registers or updates a device (upsert by `deviceId`). Call this when the device boots so the backend knows it exists.

**Headers**

- `X-API-Key`: same value as `API_KEY` on the server
- `Content-Type`: `application/json`

**Body**

```json
{
  "deviceId": "gh-node-1",
  "name": "greenhouse-main",
  "firmwareVersion": "0.1.0",
  "lastKnownIp": "192.168.1.50"
}
```

`deviceId` must match `^[a-zA-Z0-9_-]+$` (your hardcoded ID in firmware). `name`, `firmwareVersion`, and **`lastKnownIp`** (IPv4 of the ESP8266 module on the LAN) are optional but **recommended** so the Commander service can reach `http://<ip>/command`.

**Response**

```json
{
  "deviceId": "gh-node-1",
  "registeredAt": "2025-04-21T12:00:00.000Z",
  "isNew": true
}
```

### `GET /devices/by-id/:deviceId`

Returns the stored device record (including `lastKnownIp`) for use by the Commander app. Requires `X-API-Key`.

**Response** (example)

```json
{
  "deviceId": "gh-node-1",
  "lastKnownIp": "192.168.1.50",
  "name": null,
  "firmwareVersion": "esp8266-web-bridge",
  "registeredAt": "2025-04-21T12:00:00.000Z",
  "lastSeenAt": "2025-04-21T12:05:00.000Z"
}
```

### `POST /readings`

Stores one telemetry sample. Intended to be called about once per minute from the device (e.g. ESP8266 HTTP client).

**Headers**

- `X-API-Key`: same as `API_KEY`
- `Content-Type`: `application/json`

**Body**

```json
{
  "deviceId": "gh-node-1",
  "payload": { "moisture": 512 },
  "takenAt": "2025-04-21T12:00:00.000Z"
}
```

- `payload` is a JSON object; values may be strings, numbers, booleans, `null`, or nested objects/arrays of those types.
- `takenAt` is optional ISO-8601 timestamp from the device clock.

If `deviceId` was never registered, the API responds with **404** (`Device not found`).

**Response**

```json
{
  "id": "42",
  "receivedAt": "2025-04-21T12:00:00.000Z"
}
```

`id` is the reading row id (stringified `BIGINT`).

## Example: `curl`

Replace `API_KEY` and IP/host as needed.

```bash
export API_KEY='change-me'
export BASE='http://localhost:3000'

curl -sS -X POST "$BASE/devices/register" \
  -H "Content-Type: application/json" \
  -H "X-API-Key: $API_KEY" \
  -d '{"deviceId":"gh-node-1","firmwareVersion":"0.1.0"}'

curl -sS -X POST "$BASE/readings" \
  -H "Content-Type: application/json" \
  -H "X-API-Key: $API_KEY" \
  -d '{"deviceId":"gh-node-1","payload":{"moisture":512}}'
```

## Firmware (`arduino-test/`)

The Petoi ESP8266 bridge sends `POST /devices/register` (with `lastKnownIp` from `WiFi.localIP()`) when the Uno requests `N,REG`, and `POST /readings` on Uno’s schedule. See [../../arduino-test/README.md](../../arduino-test/README.md). The **Commander** app (`apps/commander`) sends commands to the module using the stored `lastKnownIp`.

## Scripts

| Script            | Description                          |
| ----------------- | ------------------------------------ |
| `npm run build`   | Compile to `dist/`                   |
| `npm run start:dev` | Run with file watch                |
| `npm run lint`    | ESLint                               |
| `npm run typecheck` | `tsc --noEmit`                    |
| `npx prisma migrate deploy` | Apply migrations in production |
| `npx prisma migrate dev`      | Dev migrations (needs DB)      |
