/*
 * ESP8266 (Petoi WiFi module): HTTP API + UART bridge to Arduino Uno.
 *
 * Wiring to Uno:
 *   ESP TX  -> Uno D2 (SoftwareSerial RX)
 *   ESP RX  <- Uno D3 via 5V->3.3V divider (e.g. 10k + 20k to GND)
 *   GND     -- GND
 *
 * UART protocol (newline-terminated lines):
 *   Uno -> ESP: telemetry "T,<0..1023>"
 *   ESP -> Uno: "I,<IPv4>" repeated for ~2 min every 4 s (Uno may miss first line; LCD line 2 until first command)
 *   ESP -> Uno: command "C,<NAME>" (same whitelist as in arduino-test.ino)
 *
 * WiFi: set STASSID / STAPSK below (or -DSTASSID / -DSTAPSK at build time).
 * Optional: COMMAND_TOKEN — if non-empty, require ?token=... on /command
 *
 * Onboard LED (usually GPIO2, active LOW): steady = Wi-Fi connected, blink = not connected.
 * A separate power LED on the board may not be controlled by this sketch.
 *
 * Debug: connect USB to this module, Serial Monitor @ 9600 — lines prefixed [wifi].
 */
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif

#ifndef STASSID
#define STASSID "Autasnap"
#endif
#ifndef STAPSK
#define STAPSK "pryF6vy9"
#endif

#ifndef COMMAND_TOKEN
#define COMMAND_TOKEN ""
#endif

// Same baud as arduino-test.ino espLink (9600 is reliable for Uno SoftwareSerial).
static const int UART_BAUD = 9600;

ESP8266WebServer server(80);

int lastSensorValue = -1;
unsigned long lastSensorMillis = 0;

unsigned long wifiConnectedAtMs = 0;
unsigned long lastIpToUnoMs = 0;

unsigned long wifiLedBlinkMs = 0;
bool wifiLedBlinkPhase = false;

String rxLine;
const size_t kMaxLine = 96;

static void printWlStatus(wl_status_t s) {
  switch (s) {
    case WL_IDLE_STATUS:
      Serial.print(F("IDLE"));
      break;
    case WL_NO_SSID_AVAIL:
      Serial.print(F("NO_SSID"));
      break;
    case WL_SCAN_COMPLETED:
      Serial.print(F("SCAN_DONE"));
      break;
    case WL_CONNECTED:
      Serial.print(F("CONNECTED"));
      break;
    case WL_CONNECT_FAILED:
      Serial.print(F("CONNECT_FAILED"));
      break;
    case WL_CONNECTION_LOST:
      Serial.print(F("CONNECTION_LOST"));
      break;
    case WL_DISCONNECTED:
      Serial.print(F("DISCONNECTED"));
      break;
    default:
      Serial.print(F("code "));
      Serial.print(static_cast<int>(s));
      break;
  }
}

static void setupWifiSerialLogging() {
  WiFi.onStationModeConnected([](const WiFiEventStationModeConnected &evt) {
    Serial.print(F("[wifi] associated BSSID channel="));
    Serial.println(evt.channel);
  });
  WiFi.onStationModeGotIP([](const WiFiEventStationModeGotIP &evt) {
    Serial.print(F("[wifi] DHCP ok ip="));
    Serial.print(evt.ip);
    Serial.print(F(" mask="));
    Serial.print(evt.mask);
    Serial.print(F(" gw="));
    Serial.println(evt.gw);
  });
  WiFi.onStationModeDisconnected([](const WiFiEventStationModeDisconnected &evt) {
    Serial.print(F("[wifi] disconnected reason="));
    Serial.println(evt.reason);
  });
}

static void sendCorsJson(int status, const String &body) {
  server.sendHeader(F("Access-Control-Allow-Origin"), F("*"));
  server.sendHeader(F("Access-Control-Allow-Methods"), F("GET, POST, OPTIONS"));
  server.sendHeader(F("Access-Control-Allow-Headers"), F("Content-Type"));
  server.send(status, F("application/json"), body);
}

static bool tokenOk() {
  const char *tok = COMMAND_TOKEN;
  if (tok == nullptr || tok[0] == '\0') {
    return true;
  }
  return server.hasArg(F("token")) && server.arg(F("token")) == String(tok);
}

static bool allowedCommand(const String &cmd) {
  return cmd == F("PING") || cmd == F("RELAY_ON") || cmd == F("RELAY_OFF");
}

static void forwardCommandToUno(const String &cmd) {
  Serial.print(F("C,"));
  Serial.print(cmd);
  Serial.print(F("\n"));
}

static void sendLocalIpToUno() {
  const IPAddress lip = WiFi.localIP();
  Serial.print(F("I,"));
  Serial.print(lip.toString());
  Serial.print(F("\n"));
}

void handleSensor() {
  if (lastSensorValue < 0) {
    sendCorsJson(503, F("{\"error\":\"no_data_yet\"}"));
    return;
  }
  String body;
  body.reserve(64);
  body += F("{\"value\":");
  body += lastSensorValue;
  body += F(",\"ageMs\":");
  body += (millis() - lastSensorMillis);
  body += F("}");
  sendCorsJson(200, body);
}

void handleCommand() {
  if (!tokenOk()) {
    sendCorsJson(401, F("{\"error\":\"unauthorized\"}"));
    return;
  }

  String cmd = server.arg(F("cmd"));
  if (cmd.length() == 0) {
    sendCorsJson(400, F("{\"error\":\"missing_cmd\"}"));
    return;
  }
  if (!allowedCommand(cmd)) {
    sendCorsJson(400, F("{\"error\":\"unknown_cmd\"}"));
    return;
  }

  forwardCommandToUno(cmd);
  String body = F("{\"ok\":true,\"cmd\":\"");
  body += cmd;
  body += F("\"}");
  sendCorsJson(200, body);
}

void handleCommandOptions() {
  server.sendHeader(F("Access-Control-Allow-Origin"), F("*"));
  server.sendHeader(F("Access-Control-Allow-Methods"), F("GET, POST, OPTIONS"));
  server.sendHeader(F("Access-Control-Allow-Headers"), F("Content-Type"));
  server.send(204);
}

void handleRoot() {
  server.sendHeader(F("Location"), F("/sensor"));
  server.send(302, F("text/plain"), F(""));
}

void ingestSerialLine(const String &line) {
  if (line.length() == 0) {
    return;
  }
  if (!line.startsWith(F("T,"))) {
    return;
  }
  const String rest = line.substring(2);
  if (rest.length() == 0) {
    return;
  }
  for (unsigned int i = 0; i < rest.length(); i++) {
    const char c = rest[i];
    if (c < '0' || c > '9') {
      return;
    }
  }
  const int v = rest.toInt();
  lastSensorValue = v;
  lastSensorMillis = millis();
}

static void ledInit() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
}

static void ledSetWifiConnected(bool connected) {
  if (connected) {
    digitalWrite(LED_BUILTIN, LOW);
  }
}

static void ledUpdateDisconnectedBlink() {
  const unsigned long t = millis();
  if (t - wifiLedBlinkMs < 400) {
    return;
  }
  wifiLedBlinkMs = t;
  wifiLedBlinkPhase = !wifiLedBlinkPhase;
  digitalWrite(LED_BUILTIN, wifiLedBlinkPhase ? LOW : HIGH);
}

static void updateWifiStatusLed() {
  if (WiFi.status() == WL_CONNECTED) {
    ledSetWifiConnected(true);
    return;
  }
  ledUpdateDisconnectedBlink();
}

void pollUartFromUno() {
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\r') {
      continue;
    }
    if (c == '\n') {
      ingestSerialLine(rxLine);
      rxLine = "";
      continue;
    }
    if (rxLine.length() < kMaxLine) {
      rxLine += c;
    } else {
      rxLine = "";
    }
  }
}

void setup() {
  Serial.begin(UART_BAUD);
  Serial.setTimeout(10);

  Serial.println();
  Serial.println(F("[wifi] esp8266-web-bridge boot"));
  Serial.print(F("[wifi] target SSID="));
  Serial.println(STASSID);

  ledInit();

  setupWifiSerialLogging();

  WiFi.mode(WIFI_STA);
  Serial.println(F("[wifi] WiFi.mode(STA), beginning..."));
  WiFi.begin(STASSID, STAPSK);

  unsigned long wifiStarted = millis();
  unsigned long lastStatusLog = 0;
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - wifiStarted > 60000UL) {
      Serial.println(F("[wifi] timeout 60s, restart"));
      WiFi.disconnect();
      delay(1000);
      ESP.restart();
    }
    if (millis() - lastStatusLog >= 3000UL) {
      lastStatusLog = millis();
      Serial.print(F("[wifi] waiting "));
      Serial.print((millis() - wifiStarted) / 1000UL);
      Serial.print(F("s status="));
      printWlStatus(WiFi.status());
      Serial.println();
    }
    updateWifiStatusLed();
    delay(250);
  }

  ledSetWifiConnected(true);
  wifiConnectedAtMs = millis();
  Serial.print(F("[wifi] linked. RSSI="));
  Serial.print(WiFi.RSSI());
  Serial.print(F(" dBm localIP="));
  Serial.println(WiFi.localIP());

  delay(200);
  sendLocalIpToUno();
  lastIpToUnoMs = millis();

  server.on(F("/"), handleRoot);
  server.on(F("/sensor"), handleSensor);
  server.on(F("/command"), HTTP_GET, handleCommand);
  server.on(F("/command"), HTTP_POST, handleCommand);
  server.on(F("/command"), HTTP_OPTIONS, handleCommandOptions);
  server.begin();
  Serial.println(F("[wifi] HTTP server :80"));
}

void loop() {
  server.handleClient();
  pollUartFromUno();
  updateWifiStatusLed();

  const unsigned long now = millis();
  if (now - wifiConnectedAtMs < 120000UL && now - lastIpToUnoMs >= 4000UL) {
    lastIpToUnoMs = now;
    sendLocalIpToUno();
  }
}
