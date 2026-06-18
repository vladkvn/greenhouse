/*
 * GreenHouse — follower: прошивка модуля управления (обычный ESP32)
 * ----------------------------------------------------------------
 * Роль ESP32 в этой схеме — ТОЛЬКО моторы и безопасность.
 * Камера и лидар висят на Jetson; Jetson считает, куда ехать, и шлёт
 * сюда команды по WiFi (UDP). Никаких вычислений на ESP32 нет.
 *
 *   [Jetson: камера+лидар+логика]  --UDP "L R"-->  [ESP32]  -->  L298  -->  4 мотора
 *
 * Привод: skid-steer (танковый). 4 DC-мотора, ОДНА плата L298:
 *   - левые 2 мотора подключены параллельно к каналу A (OUT1/OUT2),
 *   - правые 2 мотора параллельно к каналу B (OUT3/OUT4).
 *   Итого 6 сигналов на ESP32: IN1,IN2 (лево), IN3,IN4 (право), ENA,ENB (ШИМ).
 *
 * Протокол команд (UDP, обычный текст, одна строка), порт 4210:
 *      "L R"   L,R — скорости левого/правого бортов, целые -255..255.
 *              "200 200" вперёд, "-200 -200" назад, "200 -200" поворот, "0 0" стоп.
 *      "STOP"  — немедленная остановка.
 *
 * Безопасность (слой реального времени, не зависит от Jetson):
 *   Watchdog: нет команд дольше CMD_TIMEOUT_MS — моторы глушатся.
 *   Это ловит зависший Jetson, обрыв WiFi, упавший скрипт.
 *
 * Плата: любой обычный ESP32 dev board (DOIT DevKit v1, WROOM-32 и т.п.).
 */

#include <WiFi.h>
#include <WiFiUdp.h>

// ================== НАСТРОЙКИ — ОТРЕДАКТИРУЙ ПОД СЕБЯ ==================
const char* WIFI_SSID = "Autasnap";     // <-- впиши имя своей сети
const char* WIFI_PASS = "pryF6vy9";   // <-- впиши пароль

// Статический IP, чтобы Jetson всегда знал, куда слать команды.
// Должен быть в той же подсети, что и Jetson (192.168.1.x), и свободен.
IPAddress STATIC_IP (192, 168, 1, 50);
IPAddress GATEWAY   (192, 168, 1, 1);
IPAddress SUBNET    (255, 255, 255, 0);
IPAddress DNS       (192, 168, 1, 1);

const uint16_t UDP_PORT       = 4210;   // порт приёма команд
const uint32_t CMD_TIMEOUT_MS = 500;    // failsafe: нет команд дольше — стоп

// Пины драйвера моторов L298 (обычный ESP32 dev board).
// Выбраны безопасные выходные GPIO (не strapping, не input-only).
const int PIN_L_IN1 = 27;   // L298 IN1  (левый борт, направление)
const int PIN_L_IN2 = 26;   // L298 IN2
const int PIN_R_IN1 = 25;   // L298 IN3  (правый борт, направление)
const int PIN_R_IN2 = 33;   // L298 IN4
const int PIN_L_EN  = 14;   // L298 ENA  (ШИМ скорости левого борта) — снять джампер ENA!
const int PIN_R_EN  = 32;   // L298 ENB  (ШИМ скорости правого борта) — снять джампер ENB!
// ======================================================================

// Параметры ШИМ (LEDC). В ESP32 Arduino core 3.x API пин-ориентированный:
// ledcAttach(pin, freq, res) + ledcWrite(pin, duty).
const int PWM_FREQ = 1000;   // 1 кГц — спокойно для L298
const int PWM_RES  = 8;      // 8 бит -> 0..255

WiFiUDP udp;
char udpBuf[64];
volatile uint32_t lastCmdMs = 0;

// -------------------- МОТОРЫ --------------------
void setupMotors() {
  pinMode(PIN_L_IN1, OUTPUT);
  pinMode(PIN_L_IN2, OUTPUT);
  pinMode(PIN_R_IN1, OUTPUT);
  pinMode(PIN_R_IN2, OUTPUT);
  ledcAttach(PIN_L_EN, PWM_FREQ, PWM_RES);
  ledcAttach(PIN_R_EN, PWM_FREQ, PWM_RES);
}

// speed: -255..255 (знак — направление, модуль — ШИМ-скорость)
void driveSide(int in1, int in2, int enPin, int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    ledcWrite(enPin, speed);
  } else if (speed < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    ledcWrite(enPin, -speed);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    ledcWrite(enPin, 0);
  }
}

void setMotors(int left, int right) {
  driveSide(PIN_L_IN1, PIN_L_IN2, PIN_L_EN, left);
  driveSide(PIN_R_IN1, PIN_R_IN2, PIN_R_EN, right);
}

void stopMotors() { setMotors(0, 0); }

// -------------------- UDP-КОМАНДЫ --------------------
void pollUdp() {
  int sz = udp.parsePacket();
  if (sz <= 0) return;
  int len = udp.read(udpBuf, sizeof(udpBuf) - 1);
  if (len <= 0) return;
  udpBuf[len] = '\0';

  if (strncmp(udpBuf, "STOP", 4) == 0) {
    stopMotors();
    lastCmdMs = millis();
    return;
  }

  int l = 0, r = 0;
  if (sscanf(udpBuf, "%d %d", &l, &r) == 2) {
    setMotors(l, r);
    lastCmdMs = millis();
  }
}

void failsafeCheck() {
  if (millis() - lastCmdMs > CMD_TIMEOUT_MS) {
    stopMotors();
  }
}

// -------------------- SETUP / LOOP --------------------
void setup() {
  Serial.begin(115200);
  delay(200);

  setupMotors();
  stopMotors();

  WiFi.mode(WIFI_STA);
  if (!WiFi.config(STATIC_IP, GATEWAY, SUBNET, DNS)) {
    Serial.println("Не удалось задать статический IP — будет DHCP");
  }
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Подключение к WiFi");
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(400);
    Serial.print(".");
    if (millis() - t0 > 20000) {   // 20с не вышло — ребут
      Serial.println("\nWiFi не поднялся, перезагрузка");
      ESP.restart();
    }
  }
  Serial.println();
  Serial.print("IP-адрес: ");
  Serial.println(WiFi.localIP());
  Serial.printf("Команды:  UDP %s:%u  (формат \"L R\", -255..255)\n",
                WiFi.localIP().toString().c_str(), UDP_PORT);

  udp.begin(UDP_PORT);
  lastCmdMs = millis();
}

void loop() {
  pollUdp();
  failsafeCheck();
}
