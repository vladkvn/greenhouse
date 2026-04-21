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
 *   ESP -> Uno: after Wi-Fi connect, once: "I,<IPv4>" (shown on Uno LCD line 2 until first command)
 *   ESP -> Uno: command "C,<NAME>" (same whitelist as in arduino-test.ino)
 *
 * WiFi: set STASSID / STAPSK below (or -DSTASSID / -DSTAPSK at build time).
 * Optional: COMMAND_TOKEN — if non-empty, require ?token=... on /command
 */
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

#ifndef STASSID
#define STASSID "Autosnap"
#endif
#ifndef STAPSK
#define STAPSK "pryF6vy9"
#endif

#ifndef COMMAND_TOKEN
#define COMMAND_TOKEN ""
#endif

static const int UART_BAUD = 57600;

ESP8266WebServer server(80);

int lastSensorValue = -1;
unsigned long lastSensorMillis = 0;

String rxLine;
const size_t kMaxLine = 96;

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

  WiFi.mode(WIFI_STA);
  WiFi.begin(STASSID, STAPSK);
  unsigned long wifiStarted = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - wifiStarted > 60000UL) {
      WiFi.disconnect();
      delay(1000);
      ESP.restart();
    }
    delay(250);
  }

  delay(100);
  sendLocalIpToUno();

  server.on(F("/"), handleRoot);
  server.on(F("/sensor"), handleSensor);
  server.on(F("/command"), HTTP_GET, handleCommand);
  server.on(F("/command"), HTTP_POST, handleCommand);
  server.on(F("/command"), HTTP_OPTIONS, handleCommandOptions);
  server.begin();
}

void loop() {
  server.handleClient();
  pollUartFromUno();
}
