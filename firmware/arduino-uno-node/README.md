# Arduino Uno — test telemetry + command display

Test build for the first field module: **read water level (analog)**, **send telemetry every 15 s**, **receive a command** and show `l1` / `l2` on a **16×2 I2C LCD**.

## Default transport: USB cable only (`env:uno`)

The default firmware (`transport_serial.cpp`) does **not** use Wi‑Fi or Ethernet. It sends **one JSON line per telemetry** on `Serial` (USB), and accepts **commands as JSON lines** on the same serial port (see `tools/serial_uno_test_bridge.py` on your PC).

**Important:** only **one** program can open the serial port at a time. Close **PlatformIO Serial Monitor** / Arduino Serial Monitor before running the Python bridge.

### Hardware

- **Arduino Uno** + USB cable to the PC.
- **Water level sensor** (analog output) → `A0` (`APP_WATER_SENSOR_ANALOG_PIN` in `include/AppConfig.h`).
- **LCD 1602 + I2C backpack** → Uno `A4` (SDA), `A5` (SCL), `5V`, `GND`. If the display stays blank, try I2C address `0x3F` instead of `0x27`.

### Serial line protocol (test contract)

- **Arduino → PC:** newline-terminated JSON, e.g. `{"msgType":"telemetry","transport":"serial",...}`.
- **PC → Arduino:** newline-terminated JSON command with string fields `l1`, `l2` (up to 16 visible characters per LCD line). Non‑JSON lines show raw text on line 1.

`nodeId` defaults to `uno-test-1` (`APP_NODE_ID`).

### Configure

Edit `include/AppConfig.h` if needed:

- `APP_SERIAL_BAUD` (default `115200`).
- Sensor pin, LCD I2C address, telemetry interval.

### Build / flash (PlatformIO)

```bash
cd firmware/arduino-uno-node
pio run -e uno
pio run -e uno -t upload
```

### End-to-end test over USB (from repository root)

Run the bridge on the **same computer** that has the Arduino USB cable (the bridge opens the serial port locally).

#### macOS / Linux

1. Create a venv and install deps:

   ```bash
   python3 -m venv .venv-serial
   source .venv-serial/bin/activate
   pip install -r tools/requirements-serial-bridge.txt
   ```

2. Find the serial device (examples: `/dev/tty.usbmodem*`, `/dev/tty.usbserial*`).

3. Close **Arduino IDE Serial Monitor** / **PlatformIO Device Monitor** (only one app may open the port).

4. Run the bridge:

   ```bash
   python3 tools/serial_uno_test_bridge.py --port /dev/tty.usbmodem1101
   ```

5. Reset the Uno if needed.

#### Windows (host PC)

Use **Python installed on Windows** (not inside WSL) unless you have USB serial forwarding set up for WSL2.

1. Find the COM port: **Device Manager → Ports (COM & LPT)** → look for **Arduino Uno**, **CH340**, **CP210x**, etc. The name will be like **COM5** (the number can change if you use another USB socket).

2. From a terminal **in the repository root** (PowerShell):

   ```powershell
   py -3 -m venv .venv-serial
   .\.venv-serial\Scripts\Activate.ps1
   pip install -r tools\requirements-serial-bridge.txt
   py tools\serial_uno_test_bridge.py --port COM5
   ```

   If `py` is not available, try `python -m venv ...` and `python tools\serial_uno_test_bridge.py --port COM5`.

   If script activation is blocked by execution policy, use **cmd.exe**:

   ```bat
   py -3 -m venv .venv-serial
   .venv-serial\Scripts\activate.bat
   pip install -r tools\requirements-serial-bridge.txt
   py tools\serial_uno_test_bridge.py --port COM5
   ```

3. Close **Arduino IDE Serial Monitor** / **PlatformIO Device Monitor** before starting the bridge (only one app may open the COM port).

4. Replace `COM5` with your actual port. Reset the Uno if needed.

#### What you should see

About every **15 s**, a telemetry JSON line appears in the terminal; the script sends a command line back; the **LCD** updates (`USB tick N` / `host bridge`).

---

## Optional transport: Ethernet + MQTT (`env:uno-mqtt-eth`)

When you add an **Ethernet Shield** (W5100 / compatible), build:

```bash
pio run -e uno-mqtt-eth
pio run -e uno-mqtt-eth -t upload
```

Edit `include/AppConfig.h` for `APP_MQTT_BROKER_IP_*` and `APP_ETH_MAC`.

### MQTT topics (same logical contract as the server)

- Publish: `greenhouse/nodes/<nodeId>/telemetry`
- Subscribe: `greenhouse/nodes/<nodeId>/commands` — JSON with `l1`, `l2`

### End-to-end test with Mosquitto + stub server

From repository root (Docker Desktop on Windows is fine):

```bash
docker compose -f infra/docker-compose.yml up -d
python3 -m venv .venv-mqtt
source .venv-mqtt/bin/activate
pip install -r tools/requirements-mqtt-test.txt
python3 tools/mqtt_uno_test_server.py
```

Windows (PowerShell) equivalent:

```powershell
docker compose -f infra/docker-compose.yml up -d
py -3 -m venv .venv-mqtt
.\.venv-mqtt\Scripts\Activate.ps1
pip install -r tools\requirements-mqtt-test.txt
py tools\mqtt_uno_test_server.py
```

---

## Later upgrades

Components will change; keep **topic prefix** and **JSON field names** aligned with `packages/shared-types` when those schemas land. The USB bridge is a deliberate stand-in until network hardware is wired.
