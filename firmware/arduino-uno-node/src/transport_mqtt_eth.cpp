#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <string.h>
#include <Ethernet.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <LiquidCrystal_I2C.h>

#include "AppConfig.h"

namespace {

constexpr char kTopicPrefix[] = "greenhouse/nodes/";
constexpr size_t kTopicBufferLen = 96;
constexpr size_t kMqttPayloadLen = 384;
constexpr size_t kLcdLineLen = 17;

LiquidCrystal_I2C lcd(APP_LCD_I2C_ADDR, 16, 2);
EthernetClient ethernetClient;
PubSubClient mqtt(ethernetClient);

char gTopicTelemetry[kTopicBufferLen] = {0};
char gTopicCommands[kTopicBufferLen] = {0};

unsigned long gLastTelemetryMs = 0;
char gCmdLine1[kLcdLineLen] = "Waiting cmd...";
char gCmdLine2[kLcdLineLen] = "";

void buildTopics() {
  snprintf(gTopicTelemetry, sizeof(gTopicTelemetry), "%s%s/telemetry", kTopicPrefix, APP_NODE_ID);
  snprintf(gTopicCommands, sizeof(gTopicCommands), "%s%s/commands", kTopicPrefix, APP_NODE_ID);
}

void lcdWriteLine(uint8_t row, const char *text) {
  lcd.setCursor(0, row);
  for (uint8_t col = 0; col < 16; col++) {
    const char ch = (text != nullptr && text[col] != '\0') ? static_cast<unsigned char>(text[col]) : ' ';
    lcd.write(ch);
  }
}

void showCommandOnLcd() {
  lcdWriteLine(0, gCmdLine1);
  lcdWriteLine(1, gCmdLine2);
}

void onMqttMessage(char *topic, byte *payload, unsigned int length) {
  (void)topic;

  if (length >= kMqttPayloadLen) {
    length = kMqttPayloadLen - 1;
  }

  char textBuf[kMqttPayloadLen] = {0};
  memcpy(textBuf, payload, length);
  textBuf[length] = '\0';

  JsonDocument doc(256);
  const DeserializationError err = deserializeJson(doc, textBuf);
  if (err) {
    strncpy(gCmdLine1, textBuf, 16);
    gCmdLine1[16] = '\0';
    gCmdLine2[0] = '\0';
    showCommandOnLcd();
    return;
  }

  const char *l1 = doc["l1"].as<const char *>();
  const char *l2 = doc["l2"].as<const char *>();
  if (l1 == nullptr) {
    l1 = "(no l1)";
  }
  if (l2 == nullptr) {
    l2 = "";
  }

  strncpy(gCmdLine1, l1, 16);
  gCmdLine1[16] = '\0';
  strncpy(gCmdLine2, l2, 16);
  gCmdLine2[16] = '\0';
  showCommandOnLcd();
}

bool connectMqtt() {
  mqtt.setServer(
      IPAddress(APP_MQTT_BROKER_IP_1, APP_MQTT_BROKER_IP_2, APP_MQTT_BROKER_IP_3, APP_MQTT_BROKER_IP_4),
      APP_MQTT_PORT);
  mqtt.setCallback(onMqttMessage);

  if (mqtt.connected()) {
    return true;
  }

  if (mqtt.connect(APP_NODE_ID)) {
    const bool subOk = mqtt.subscribe(gTopicCommands);
    if (!subOk) {
      return false;
    }
    return true;
  }
  return false;
}

void publishTelemetry() {
  if (!mqtt.connected()) {
    return;
  }

  const int waterAdc = analogRead(APP_WATER_SENSOR_ANALOG_PIN);

  JsonDocument doc(256);
  doc["schemaVersion"] = 1;
  doc["nodeId"] = APP_NODE_ID;
  doc["msgType"] = "telemetry";
  doc["uptimeMs"] = static_cast<uint32_t>(millis());
  doc["waterLevel"]["adcRaw"] = waterAdc;

  char payload[kMqttPayloadLen] = {0};
  const size_t n = serializeJson(doc, payload, sizeof(payload) - 1);
  if (n == 0) {
    return;
  }

  mqtt.publish(gTopicTelemetry, payload, n);
}

}  // namespace

void setup() {
  Serial.begin(115200);

  Wire.begin();
  lcd.init();
  lcd.backlight();
  lcdWriteLine(0, "GreenHouse");
  lcdWriteLine(1, APP_NODE_ID);

  buildTopics();

  Serial.println(F("Starting Ethernet (DHCP)..."));
  uint8_t mac[6];
  memcpy(mac, APP_ETH_MAC, sizeof(mac));
  if (Ethernet.begin(mac) == 0) {
    Serial.println(F("Ethernet DHCP failed"));
    lcdWriteLine(0, "ETH DHCP FAIL");
    lcdWriteLine(1, "Check cable");
    while (true) {
      delay(1000);
    }
  }

  Serial.print(F("IP: "));
  Serial.println(Ethernet.localIP());

  mqtt.setBufferSize(512);
  gLastTelemetryMs = millis() - APP_TELEMETRY_INTERVAL_MS;

  lcdWriteLine(0, "MQTT connect..");
  lcdWriteLine(1, "");

  unsigned long start = millis();
  while (!connectMqtt()) {
    Serial.println(F("MQTT connect failed, retrying..."));
    if (millis() - start > 60000UL) {
      lcdWriteLine(0, "MQTT FAIL");
      lcdWriteLine(1, "reboot?");
      while (true) {
        delay(1000);
      }
    }
    delay(1000);
  }

  Serial.println(F("MQTT connected"));
  lcdWriteLine(0, "MQTT OK");
  lcdWriteLine(1, APP_NODE_ID);
  delay(800);
  showCommandOnLcd();
}

void loop() {
  Ethernet.maintain();
  if (!mqtt.connected()) {
    Serial.println(F("MQTT disconnected, reconnecting"));
    if (connectMqtt()) {
      Serial.println(F("MQTT reconnected"));
    } else {
      delay(500);
      return;
    }
  }

  mqtt.loop();

  const unsigned long now = millis();
  if (now - gLastTelemetryMs >= APP_TELEMETRY_INTERVAL_MS) {
    gLastTelemetryMs = now;
    publishTelemetry();
    Serial.print(F("Telemetry published, water adc="));
    Serial.println(analogRead(APP_WATER_SENSOR_ANALOG_PIN));
  }
}
