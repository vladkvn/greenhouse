#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>

// UART to ESP8266 (Petoi): Uno D2 = RX <- ESP TX, Uno D3 = TX -> ESP RX (use 5V to 3.3V level shift on D3)
// Speed 9600: stable for SoftwareSerial on Uno; must match esp8266-web-bridge UART_BAUD.
// USB Serial Monitor on THIS board: 9600 baud (not the ESP8266's rate if different).
//
// Bump when changing this sketch (printed on boot at 9600 baud).
static const char FIRMWARE_VERSION[] = "arduino-test/0.1.0";

SoftwareSerial espLink(2, 3);

LiquidCrystal_I2C lcd(0x27, 16, 2);

const int sensorPin = A0;

static const long ESP_BAUD = 9600;
static const unsigned long SENSOR_INTERVAL_MS = 500;
// Отправка в Nest API (строки N,* уходит на ESP по UART; инициатор — только Uno)
static const unsigned long NEST_SEND_INTERVAL_MS = 60000;

// Virtual relay for demo (replace with digitalWrite to a pin when hardware is wired)
bool relayOn = false;

bool nestRegisterSent = false;
unsigned long lastNestPostMs = 0;

String rxLine;
const size_t kMaxRxLine = 64;

unsigned long lastSensorMs = 0;

int lastSensorValue = 0;

bool firstCommandHandled = false;
String espAnnounceIp;

String pendingCmdDisplay;
unsigned long cmdDisplayUntilMs = 0;
static const unsigned long CMD_LCD_DURATION_MS = 2500;

static void padLineTo16(const String &text) {
  String line = text;
  if (line.length() > 16) {
    line = line.substring(0, 16);
  }
  lcd.print(line);
  for (unsigned int i = line.length(); i < 16; i++) {
    lcd.print(' ');
  }
}

static void drawUi(int value);

static bool isAllowedCommand(const String &cmd) {
  return cmd == F("PING") || cmd == F("RELAY_ON") || cmd == F("RELAY_OFF");
}

static bool looksLikeIpv4Text(const String &s) {
  if (s.length() < 7 || s.length() > 15) {
    return false;
  }
  for (unsigned int i = 0; i < s.length(); i++) {
    const char c = s[i];
    if ((c >= '0' && c <= '9') || c == '.') {
      continue;
    }
    return false;
  }
  unsigned int dots = 0;
  for (unsigned int i = 0; i < s.length(); i++) {
    if (s[i] == '.') {
      dots++;
    }
  }
  return dots >= 3;
}

static void handleIpAnnounce(const String &ipText) {
  if (firstCommandHandled) {
    return;
  }
  String t = ipText;
  t.trim();
  if (!looksLikeIpv4Text(t)) {
    return;
  }
  espAnnounceIp = t;
  Serial.print(F("ip:"));
  Serial.println(t);
  if (!nestRegisterSent) {
    nestRegisterSent = true;
    espLink.println(F("N,REG"));
    lastNestPostMs = millis();
  }
  drawUi(lastSensorValue);
}

static void handleCommand(const String &cmd) {
  if (!isAllowedCommand(cmd)) {
    return;
  }
  firstCommandHandled = true;
  pendingCmdDisplay = cmd;
  cmdDisplayUntilMs = millis() + CMD_LCD_DURATION_MS;

  if (cmd == F("PING")) {
    Serial.println(F("cmd:PING"));
  } else if (cmd == F("RELAY_ON")) {
    relayOn = true;
  } else if (cmd == F("RELAY_OFF")) {
    relayOn = false;
  }

  drawUi(lastSensorValue);
}

static void drawLine1Status(int value) {
  lcd.setCursor(0, 1);
  const unsigned long now = millis();
  if (now < cmdDisplayUntilMs && pendingCmdDisplay.length() > 0) {
    String line = F("Cmd ");
    line += pendingCmdDisplay;
    padLineTo16(line);
    return;
  }
  if (!firstCommandHandled) {
    if (espAnnounceIp.length() == 0) {
      padLineTo16(String(F("WiFi...")));
    } else {
      padLineTo16(espAnnounceIp);
    }
    return;
  }
  if (value > 500) {
    lcd.print(F("WET"));
  } else {
    lcd.print(F("DRY"));
  }
  lcd.print(F(" R:"));
  lcd.print(relayOn ? F("ON ") : F("OFF"));
  lcd.print(F("    "));
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
      } else if (rxLine.startsWith(F("I,"))) {
        handleIpAnnounce(rxLine.substring(2));
      } else if (rxLine.startsWith(F("K,"))) {
        Serial.print(F("nest:"));
        Serial.println(rxLine);
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
  drawLine1Status(value);
}

static void readSensorAndPublish() {
  const int value = analogRead(sensorPin);
  lastSensorValue = value;
  drawUi(value);

  Serial.print(F("T,"));
  Serial.println(value);

  espLink.print(F("T,"));
  espLink.println(value);
}

void setup() {
  Serial.begin(9600);
  Serial.print(F("fw:"));
  Serial.println(FIRMWARE_VERSION);

  espLink.begin(ESP_BAUD);

  lcd.init();
  lcd.backlight();

  lcd.setCursor(0, 0);
  lcd.print(F("Starting..."));
  delay(1000);
  lcd.clear();

  lastSensorMs = millis();
  lastSensorValue = analogRead(sensorPin);
  drawUi(lastSensorValue);
}

void loop() {
  processIncomingFromEsp();

  const unsigned long now = millis();

  if (nestRegisterSent && (now - lastNestPostMs >= NEST_SEND_INTERVAL_MS)) {
    lastNestPostMs = now;
    espLink.print(F("N,S,"));
    espLink.println(lastSensorValue);
  }

  if (now - lastSensorMs >= SENSOR_INTERVAL_MS) {
    lastSensorMs = now;
    readSensorAndPublish();
  } else if (cmdDisplayUntilMs != 0UL && now >= cmdDisplayUntilMs) {
    cmdDisplayUntilMs = 0UL;
    pendingCmdDisplay = "";
    drawUi(lastSensorValue);
  }
}
