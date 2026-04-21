#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>

// UART to ESP8266 (Petoi): Uno D2 = RX <- ESP TX, Uno D3 = TX -> ESP RX (use 5V to 3.3V level shift on D3)
SoftwareSerial espLink(2, 3);

LiquidCrystal_I2C lcd(0x27, 16, 2);

const int sensorPin = A0;

static const long ESP_BAUD = 57600;
static const unsigned long SENSOR_INTERVAL_MS = 500;

// Virtual relay for demo (replace with digitalWrite to a pin when hardware is wired)
bool relayOn = false;

String rxLine;
const size_t kMaxRxLine = 64;

unsigned long lastSensorMs = 0;

static bool isAllowedCommand(const String &cmd) {
  return cmd == F("PING") || cmd == F("RELAY_ON") || cmd == F("RELAY_OFF");
}

static void handleCommand(const String &cmd) {
  if (!isAllowedCommand(cmd)) {
    return;
  }
  if (cmd == F("PING")) {
    Serial.println(F("cmd:PING"));
    return;
  }
  if (cmd == F("RELAY_ON")) {
    relayOn = true;
  } else if (cmd == F("RELAY_OFF")) {
    relayOn = false;
  }
}

static void processIncomingFromEsp() {
  while (espLink.available() > 0) {
    const char c = static_cast<char>(espLink.read());
    if (c == '\r') {
      continue;
    }
    if (c == '\n') {
      if (rxLine.startsWith(F("C,"))) {
        handleCommand(rxLine.substring(2));
      }
      rxLine = "";
      continue;
    }
    if (rxLine.length() < kMaxRxLine) {
      rxLine += c;
    } else {
      rxLine = "";
    }
  }
}

static void drawUi(int value) {
  lcd.setCursor(0, 0);
  lcd.print(F("S:"));
  lcd.print(value);
  lcd.print(F("        "));
  lcd.setCursor(0, 1);
  if (value > 500) {
    lcd.print(F("WET"));
  } else {
    lcd.print(F("DRY"));
  }
  lcd.print(F(" R:"));
  lcd.print(relayOn ? F("ON ") : F("OFF"));
  lcd.print(F("    "));
}

static void readSensorAndPublish() {
  const int value = analogRead(sensorPin);
  drawUi(value);

  Serial.print(F("T,"));
  Serial.println(value);

  espLink.print(F("T,"));
  espLink.println(value);
}

void setup() {
  Serial.begin(9600);
  espLink.begin(ESP_BAUD);

  lcd.init();
  lcd.backlight();

  lcd.setCursor(0, 0);
  lcd.print(F("Starting..."));
  delay(1000);
  lcd.clear();

  lastSensorMs = millis();
}

void loop() {
  processIncomingFromEsp();

  const unsigned long now = millis();
  if (now - lastSensorMs >= SENSOR_INTERVAL_MS) {
    lastSensorMs = now;
    readSensorAndPublish();
  }
}
