/*
 * Uno + ESP8266 (UART D2/D3 @ 9600): проверка, что модуль подключился к Wi‑Fi.
 *
 * Сам Uno к Wi‑Fi не подключается — это делает ESP. После Wi‑Fi прошивка ESP
 * шлёт строку "I,<IPv4>\\n" (как в esp8266-web-bridge). Этот скетч ждёт её
 * и сообщает в Serial (USB) об успехе или таймауте.
 *
 * Проводка: ESP TX -> D2, D3 -> ESP RX (5V->3.3V на D3), GND общий.
 * На ESP должна быть прошита esp8266-web-bridge (или совместимая логика I,).
 */
#include <SoftwareSerial.h>

SoftwareSerial espLink(2, 3);

static const long ESP_BAUD = 9600;
static const unsigned long WAIT_IP_TIMEOUT_MS = 120000UL;
static const unsigned long STATUS_EVERY_MS = 10000UL;

String rxLine;
const size_t kMaxRxLine = 96;

bool wifiOk = false;
String reportedIp;

unsigned long startedMs;
unsigned long lastStatusMs;

static bool looksLikeIpv4(const String &s) {
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

static void tryParseLine(const String &line) {
  if (!line.startsWith(F("I,"))) {
    return;
  }
  String rest = line.substring(2);
  rest.trim();
  if (!looksLikeIpv4(rest)) {
    return;
  }
  wifiOk = true;
  reportedIp = rest;
  Serial.println();
  Serial.print(F("[OK] Wi-Fi на модуле, IP: "));
  Serial.println(reportedIp);
  digitalWrite(LED_BUILTIN, HIGH);
}

static void pollEsp() {
  while (espLink.available() > 0) {
    const char c = static_cast<char>(espLink.read());
    if (c == '\r') {
      continue;
    }
    if (c == '\n') {
      tryParseLine(rxLine);
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

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(9600);
  espLink.begin(ESP_BAUD);

  Serial.println(F("arduino-wifi-test: жду строку I,<IP> от ESP8266..."));
  startedMs = millis();
  lastStatusMs = startedMs;
}

void loop() {
  if (wifiOk) {
    delay(200);
    return;
  }

  pollEsp();

  const unsigned long now = millis();
  if (now - startedMs >= WAIT_IP_TIMEOUT_MS) {
    Serial.println(F("[FAIL] Таймаут: нет строки I,<IP>. Проверьте ESP, Wi-Fi, D2/D3, 9600 бод."));
    digitalWrite(LED_BUILTIN, LOW);
    while (true) {
      delay(1000);
    }
  }

  if (now - lastStatusMs >= STATUS_EVERY_MS) {
    lastStatusMs = now;
    Serial.print(F("... жду I,<IP>, прошло с "));
    Serial.print((now - startedMs) / 1000UL);
    Serial.println(F(" с"));
  }
}
