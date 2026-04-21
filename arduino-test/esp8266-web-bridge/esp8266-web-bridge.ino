/*
 * ESP8266 (Petoi WiFi module): HTTP API + UART from Arduino Uno.
 *
 * Wiring to Uno:
 *   ESP TX  -> Uno D2 (SoftwareSerial RX)
 *   ESP RX  <- Uno D3 via 5V->3.3V divider (e.g. 10k + 20k to GND)
 *   GND     -- GND
 *
 * Uno sends one line per reading: decimal value + newline (e.g. "512\n").
 *
 * WiFi: set STASSID / STAPSK below (or build with -DSTASSID=... -DSTAPSK=...).
 */
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

#ifndef STASSID
#define STASSID "Autosnap"
#endif
#ifndef STAPSK
#define STAPSK "pryF6vy9"
#endif

static const int UART_BAUD = 57600;

ESP8266WebServer server(80);

int lastSensorValue = -1;
unsigned long lastSensorMillis = 0;

String rxLine;
const size_t kMaxLine = 48;

void handleSensor() {
  server.sendHeader(F("Access-Control-Allow-Origin"), F("*"));
  if (lastSensorValue < 0) {
    server.send(503, F("application/json"), F("{\"error\":\"no_data_yet\"}"));
    return;
  }
  String body;
  body.reserve(48);
  body += F("{\"value\":");
  body += lastSensorValue;
  body += F(",\"ageMs\":");
  body += (millis() - lastSensorMillis);
  body += F("}");
  server.send(200, F("application/json"), body);
}

void handleRoot() {
  server.sendHeader(F("Location"), F("/sensor"));
  server.send(302, F("text/plain"), F(""));
}

void ingestSerialLine(const String &line) {
  if (line.length() == 0) {
    return;
  }
  bool ok = true;
  for (unsigned int i = 0; i < line.length(); i++) {
    const char c = line[i];
    if (c < '0' || c > '9') {
      ok = false;
      break;
    }
  }
  if (!ok) {
    return;
  }
  const int v = line.toInt();
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

  Serial.println();
  Serial.println(WiFi.localIP());

  server.on(F("/"), handleRoot);
  server.on(F("/sensor"), handleSensor);
  server.begin();
}

void loop() {
  server.handleClient();
  pollUartFromUno();
}
