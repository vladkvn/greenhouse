/*
 * ESP8266 (Petoi WiFi module): MQTT (Mosquitto) + UART bridge to Arduino Uno.
 * Локальный HTTP-сервер отключён — команды и телеметрия идут через брокер.
 *
 * Wiring to Uno:
 *   ESP TX  -> Uno D2 (SoftwareSerial RX)
 *   ESP RX  <- Uno D3 via 5V->3.3V divider (e.g. 10k + 20k to GND)
 *   GND     -- GND
 *
 * UART protocol (newline-terminated lines):
 *   Uno -> ESP: telemetry "T,<0..1023>"
 *   Uno -> ESP: "N,REG" — publish register JSON to MQTT greenhouse/<id>/register
 *   Uno -> ESP: "N,S,<0..1023>" — publish readings JSON to greenhouse/<id>/readings
 *   ESP -> Uno: "I,<IPv4>" repeated for ~2 min every 4 s
 *   ESP -> Uno: "C,<NAME>" — команда из MQTT topic greenhouse/<id>/cmd (payload = имя команды)
 *   ESP -> Uno: "K,REG,<code>" / "K,S,<code>" — результат публикации в MQTT (200=ok)
 *
 * MQTT topics (prefix greenhouse/<GH_DEVICE_ID>/):
 *   .../cmd     — subscribe, payload = PING, RELAY_ON, VENT_OPEN, ...
 *   .../register — publish JSON для apps/api (MqttIngestService)
 *   .../readings — publish JSON для apps/api
 *
 * Задайте MQTT_BROKER_HOST = IP ПК с Docker (Mosquitto порт 1883), GH_DEVICE_ID как в API.
 *
 * Library: PubSubClient (Arduino Library Manager).
 */
#include <ESP8266WiFi.h>
#define MQTT_MAX_PACKET_SIZE 512
#include <PubSubClient.h>

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

// Nest API (see apps/api): host = PC or server IP in LAN, same API_KEY as in apps/api/.env
#ifndef GH_API_HOST
#define GH_API_HOST "192.168.1.34"
#endif

#ifndef MQTT_BROKER_HOST
#define MQTT_BROKER_HOST "192.168.1.34"
#endif
#ifndef MQTT_BROKER_PORT
#define MQTT_BROKER_PORT 1883
#endif
#ifndef GH_DEVICE_ID
#define GH_DEVICE_ID "gh-node-1"
#endif

static const int UART_BAUD = 9600;

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

int lastSensorValue = -1;
unsigned long lastSensorMillis = 0;

unsigned long wifiConnectedAtMs = 0;
unsigned long lastIpToUnoMs = 0;

unsigned long wifiLedBlinkMs = 0;
bool wifiLedBlinkPhase = false;

unsigned long lastMqttReconnectMs = 0;

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

static bool allowedCommand(const String &cmd) {
  return cmd == F("PING") || cmd == F("RELAY_ON") || cmd == F("RELAY_OFF") ||
         cmd == F("VENT_OPEN") || cmd == F("VENT_CLOSE");
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

static void mqttCallback(char *topic, byte *payload, unsigned int length) {
  (void)topic;
  String cmd;
  for (unsigned int i = 0; i < length; i++) {
    cmd += static_cast<char>(payload[i]);
  }
  cmd.trim();
  if (cmd.length() == 0) {
    return;
  }
  if (!allowedCommand(cmd)) {
    Serial.print(F("[mqtt] ignored cmd: "));
    Serial.println(cmd);
    return;
  }
  forwardCommandToUno(cmd);
}

static bool mqttBrokerConfigured() {
  return MQTT_BROKER_HOST[0] != '\0';
}

static String mqttClientId() {
  return String(F("gh-")) + String(GH_DEVICE_ID) + F("-") +
         String(ESP.getChipId(), HEX);
}

static void sendBackendAckToUno(const __FlashStringHelper *op, int code) {
  Serial.print(F("K,"));
  Serial.print(op);
  Serial.print(F(","));
  Serial.println(code);
}

static bool connectMqttBroker() {
  if (!mqttBrokerConfigured()) {
    return false;
  }
  mqttClient.setServer(MQTT_BROKER_HOST, MQTT_BROKER_PORT);
  mqttClient.setCallback(mqttCallback);
  Serial.print(F("[mqtt] connecting to "));
  Serial.print(MQTT_BROKER_HOST);
  Serial.print(F(":"));
  Serial.println(MQTT_BROKER_PORT);

  if (!mqttClient.connect(mqttClientId().c_str())) {
    Serial.print(F("[mqtt] failed state="));
    Serial.println(mqttClient.state());
    return false;
  }

  const String cmdTopic =
      String(F("greenhouse/")) + String(GH_DEVICE_ID) + F("/cmd");
  if (mqttClient.subscribe(cmdTopic.c_str())) {
    Serial.print(F("[mqtt] subscribed "));
    Serial.println(cmdTopic);
  } else {
    Serial.println(F("[mqtt] subscribe failed"));
    return false;
  }
  return true;
}

static void mqttMaintain() {
  if (!mqttBrokerConfigured() || WiFi.status() != WL_CONNECTED) {
    return;
  }
  if (mqttClient.connected()) {
    mqttClient.loop();
    return;
  }
  const unsigned long now = millis();
  if (now - lastMqttReconnectMs < 3000UL) {
    return;
  }
  lastMqttReconnectMs = now;
  connectMqttBroker();
}

static bool mqttPublishJson(const char *suffix, const String &jsonBody) {
  if (!mqttBrokerConfigured()) {
    return false;
  }
  if (!mqttClient.connected()) {
    if (!connectMqttBroker()) {
      return false;
    }
  }
  const String topic =
      String(F("greenhouse/")) + String(GH_DEVICE_ID) + String(suffix);
  const bool ok = mqttClient.publish(topic.c_str(), jsonBody.c_str(), false);
  mqttClient.loop();
  return ok;
}

static int mqttPublishRegister() {
  if (!mqttBrokerConfigured()) {
    Serial.println(F("[mqtt] register skipped (set MQTT_BROKER_HOST)"));
    sendBackendAckToUno(F("REG"), -1);
    return -1;
  }
  String body = String(F("{\"deviceId\":\"")) + String(GH_DEVICE_ID) +
                  F("\",\"firmwareVersion\":\"esp8266-mqtt\",\"lastKnownIp\":\"") +
                  WiFi.localIP().toString() + F("\"}");
  const bool ok = mqttPublishJson("/register", body);
  Serial.print(F("[mqtt] publish register ok="));
  Serial.println(ok ? F("1") : F("0"));
  const int code = ok ? 200 : -3;
  sendBackendAckToUno(F("REG"), code);
  return code;
}

static int mqttPublishReading(int moisture) {
  if (!mqttBrokerConfigured()) {
    Serial.println(F("[mqtt] readings skipped (set MQTT_BROKER_HOST)"));
    sendBackendAckToUno(F("S"), -1);
    return -1;
  }
  String body = String(F("{\"deviceId\":\"")) + String(GH_DEVICE_ID) +
                  F("\",\"payload\":{\"moisture\":") + String(moisture) + F("}}");
  const bool ok = mqttPublishJson("/readings", body);
  Serial.print(F("[mqtt] publish readings ok="));
  Serial.println(ok ? F("1") : F("0"));
  const int code = ok ? 200 : -3;
  sendBackendAckToUno(F("S"), code);
  return code;
}

static bool allDigits(const String &s) {
  if (s.length() == 0) {
    return false;
  }
  for (unsigned int i = 0; i < s.length(); i++) {
    const char c = s[i];
    if (c < '0' || c > '9') {
      return false;
    }
  }
  return true;
}

static void ingestTelemetryFromUno(const String &line) {
  if (line.length() == 0) {
    return;
  }
  if (!line.startsWith(F("T,"))) {
    return;
  }
  const String rest = line.substring(2);
  if (rest.length() == 0 || !allDigits(rest)) {
    return;
  }
  const int v = rest.toInt();
  lastSensorValue = v;
  lastSensorMillis = millis();
}

static void ingestNetworkRequestFromUno(const String &lineRaw) {
  String line = lineRaw;
  line.trim();
  if (line == F("N,REG")) {
    mqttPublishRegister();
    return;
  }
  if (!line.startsWith(F("N,S,"))) {
    return;
  }
  const String rest = line.substring(4);
  if (rest.length() == 0 || !allDigits(rest)) {
    return;
  }
  const int v = rest.toInt();
  if (v < 0 || v > 1023) {
    return;
  }
  mqttPublishReading(v);
}

static void ingestSerialLine(const String &line) {
  if (line.length() == 0) {
    return;
  }
  if (line.startsWith(F("T,"))) {
    ingestTelemetryFromUno(line);
    return;
  }
  if (line.startsWith(F("N,"))) {
    ingestNetworkRequestFromUno(line);
  }
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

void setup() {
  Serial.begin(UART_BAUD);
  Serial.setTimeout(10);

  Serial.println();
  Serial.println(F("[wifi] esp8266-mqtt-bridge boot"));
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

  if (mqttBrokerConfigured()) {
    connectMqttBroker();
    Serial.println(F("[mqtt] broker configured; UART N,REG / N,S publish to greenhouse/..."));
  } else {
    Serial.println(F("[mqtt] disabled — set MQTT_BROKER_HOST to your broker IP (e.g. PC running Docker)"));
  }
}

void loop() {
  mqttMaintain();
  pollUartFromUno();
  updateWifiStatusLed();

  const unsigned long now = millis();
  if (now - wifiConnectedAtMs < 120000UL && now - lastIpToUnoMs >= 4000UL) {
    lastIpToUnoMs = now;
    sendLocalIpToUno();
  }
}
