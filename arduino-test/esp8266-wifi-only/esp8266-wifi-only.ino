/*
 * ESP8266 only: Wi-Fi + HTTP, no Arduino / no UART bridge.
 * Use while debugging power (disconnect LCD Uno stack): fake sensor every 2 s.
 *
 * API matches esp8266-web-bridge for curl tests: GET /sensor, GET|POST /command?cmd=...
 */
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

#ifndef STASSID
#define STASSID "YOUR_WIFI_SSID"
#endif
#ifndef STAPSK
#define STAPSK "YOUR_WIFI_PASSWORD"
#endif

#ifndef COMMAND_TOKEN
#define COMMAND_TOKEN ""
#endif

static const unsigned long FAKE_INTERVAL_MS = 2000;

ESP8266WebServer server(80);

int fakeSensorValue = 512;
unsigned long lastFakeMs = 0;
unsigned long lastSensorMillis = 0;

bool relayDemo = false;

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

void handleSensor() {
  String body;
  body.reserve(96);
  body += F("{\"value\":");
  body += fakeSensorValue;
  body += F(",\"ageMs\":");
  body += (millis() - lastSensorMillis);
  body += F(",\"relay\":");
  body += relayDemo ? F("true") : F("false");
  body += F(",\"mode\":\"wifi_only_fake\"}");
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

  if (cmd == F("RELAY_ON")) {
    relayDemo = true;
  } else if (cmd == F("RELAY_OFF")) {
    relayDemo = false;
  }

  Serial.print(F("cmd "));
  Serial.println(cmd);

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

static void tickFakeSensor() {
  const unsigned long now = millis();
  if (lastFakeMs != 0 && (now - lastFakeMs) < FAKE_INTERVAL_MS) {
    return;
  }
  lastFakeMs = now;
  fakeSensorValue = 200 + random(0, 824);
  lastSensorMillis = now;
  Serial.print(F("fake "));
  Serial.println(fakeSensorValue);
}

void setup() {
  Serial.begin(115200);
  delay(100);
  randomSeed(ESP.random());

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

  Serial.println(WiFi.localIP());

  server.on(F("/"), handleRoot);
  server.on(F("/sensor"), handleSensor);
  server.on(F("/command"), HTTP_GET, handleCommand);
  server.on(F("/command"), HTTP_POST, handleCommand);
  server.on(F("/command"), HTTP_OPTIONS, handleCommandOptions);
  server.begin();
}

void loop() {
  server.handleClient();
  tickFakeSensor();
}
