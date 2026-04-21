# GreenHouse API (NestJS + PostgreSQL)

HTTP API for registering Arduino-class devices and ingesting sensor readings. Devices authenticate with a shared secret via the `X-API-Key` header.

## Prerequisites

- Node.js 20+
- Docker (optional, for local PostgreSQL — see [../infra/docker-compose.yml](../infra/docker-compose.yml))

## Setup

1. Copy environment file:

   ```bash
   cp .env.example .env
   ```

2. Edit `.env`: set `API_KEY` to a strong secret and ensure `DATABASE_URL` matches your Postgres instance.

3. Start PostgreSQL (example using repo compose from `infra/`):

   ```bash
   cd ../infra && docker compose up -d postgres
   ```

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
  "firmwareVersion": "0.1.0"
}
```

`deviceId` must match `^[a-zA-Z0-9_-]+$` (your hardcoded ID in firmware). `name` and `firmwareVersion` are optional.

**Response**

```json
{
  "deviceId": "gh-node-1",
  "registeredAt": "2025-04-21T12:00:00.000Z",
  "isNew": true
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

## Firmware follow-up (not implemented in this package)

To use this from the current Arduino + Petoi ESP8266 bridge:

1. On boot (after Wi-Fi is up), `POST /devices/register` with the hardcoded `deviceId`.
2. Every 60 seconds, `POST /readings` with `deviceId` and sensor values in `payload`.

The ESP8266 sketch would need HTTP client calls added (or a small gateway); the sketches under `arduino-test/` are unchanged by this work.

## Scripts

| Script            | Description                          |
| ----------------- | ------------------------------------ |
| `npm run build`   | Compile to `dist/`                   |
| `npm run start:dev` | Run with file watch                |
| `npm run lint`    | ESLint                               |
| `npm run typecheck` | `tsc --noEmit`                    |
| `npx prisma migrate deploy` | Apply migrations in production |
| `npx prisma migrate dev`      | Dev migrations (needs DB)      |
