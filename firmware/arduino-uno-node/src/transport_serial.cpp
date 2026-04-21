#include <Arduino.h>
#include <Wire.h>
#include <string.h>
#include <ArduinoJson.h>
#include <LiquidCrystal_I2C.h>

#include "AppConfig.h"

namespace {

constexpr size_t kSerialLineMax = 384;
constexpr size_t kLcdLineLen = 17;

LiquidCrystal_I2C lcd(APP_LCD_I2C_ADDR, 16, 2);

unsigned long gLastTelemetryMs = 0;

char gSerialLine[kSerialLineMax] = {0};
size_t gSerialLineLen = 0;

char gCmdLine1[kLcdLineLen] = "USB: wait...";
char gCmdLine2[kLcdLineLen] = "";

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

void applyCommandText(const char *textBuf) {
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

void pollSerialCommands() {
  while (Serial.available() > 0) {
    const int raw = Serial.read();
    if (raw < 0) {
      break;
    }
    const char ch = static_cast<char>(raw);
    if (ch == '\r') {
      continue;
    }
    if (ch == '\n') {
      gSerialLine[gSerialLineLen] = '\0';
      if (gSerialLineLen > 0) {
        applyCommandText(gSerialLine);
      }
      gSerialLineLen = 0;
      continue;
    }
    if (gSerialLineLen < kSerialLineMax - 1) {
      gSerialLine[gSerialLineLen] = ch;
      gSerialLineLen++;
    } else {
      gSerialLineLen = 0;
    }
  }
}

void publishTelemetrySerial() {
  const int waterAdc = analogRead(APP_WATER_SENSOR_ANALOG_PIN);

  JsonDocument doc(256);
  doc["schemaVersion"] = 1;
  doc["nodeId"] = APP_NODE_ID;
  doc["msgType"] = "telemetry";
  doc["transport"] = "serial";
  doc["uptimeMs"] = static_cast<uint32_t>(millis());
  doc["waterLevel"]["adcRaw"] = waterAdc;

  char payload[kSerialLineMax] = {0};
  const size_t n = serializeJson(doc, payload, sizeof(payload) - 1);
  if (n == 0) {
    return;
  }
  Serial.println(payload);
}

}  // namespace

void setup() {
  Serial.begin(APP_SERIAL_BAUD);

  Wire.begin();
  lcd.init();
  lcd.backlight();
  lcdWriteLine(0, "GreenHouse");
  lcdWriteLine(1, "USB serial");

  gLastTelemetryMs = millis() - APP_TELEMETRY_INTERVAL_MS;
  showCommandOnLcd();
}

void loop() {
  pollSerialCommands();

  const unsigned long now = millis();
  if (now - gLastTelemetryMs >= APP_TELEMETRY_INTERVAL_MS) {
    gLastTelemetryMs = now;
    publishTelemetrySerial();
  }
}
