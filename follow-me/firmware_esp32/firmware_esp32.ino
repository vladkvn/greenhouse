/*
 * GreenHouse — follower: прошивка модуля управления
 * Плата: ESP32-CAM 4WD Robot Car (AI-Thinker, OV3660) — используется как тележка.
 * ------------------------------------------------------------------------------
 * Роль платы здесь — ТОЛЬКО моторы. Камера и лидар на Jetson; Jetson считает,
 * куда ехать, и шлёт сюда команды по WiFi (UDP). Камеру НЕ инициализируем.
 *
 *   [Jetson: камера+лидар+логика]  --UDP "L R"-->  [ESP32-CAM]  -->  4 мотора
 *
 * Привод этого кита — дольный ШИМ: на каждый борт ДВА пина (M0/M1), без отдельного
 * EN. Направление задаётся тем, на какой из двух пинов подаётся ШИМ:
 *   вперёд  -> ШИМ на M1, M0=0
 *   назад   -> ШИМ на M0, M1=0
 * Пины моторов (из оригинальной прошивки кита):
 *   LEFT_M0=13  LEFT_M1=12   RIGHT_M0=14  RIGHT_M1=15
 * ВНИМАНИЕ: 12/14/15 — strapping-пины ESP32, при загрузке могут кратко дёрнуть
 * мотор (это особенность кита; оригинальная прошивка живёт с этим же). После
 * setup() всё под контролем, watchdog держит моторы выключенными без команд.
 *
 * Протокол команд (UDP, текст, одна строка), порт 4210 — ТОТ ЖЕ, что у Jetson:
 *   "L R"   скорости левого/правого бортов, -255..255 ("180 140" вперёд,
 *           "-180 -180" назад, "180 -180" поворот, "0 0" стоп).
 *   "STOP"  немедленная остановка.
 * Failsafe: нет команд дольше CMD_TIMEOUT_MS — моторы глушатся.
 */

#include <WiFi.h>
#include <WiFiUdp.h>

// ================== НАСТРОЙКИ — ОТРЕДАКТИРУЙ ПОД СЕБЯ ==================
const char* WIFI_SSID = "Autasnap";
const char* WIFI_PASS = "pryF6vy9";

IPAddress STATIC_IP (192, 168, 1, 50);
IPAddress GATEWAY   (192, 168, 1, 1);
IPAddress SUBNET    (255, 255, 255, 0);
IPAddress DNS       (192, 168, 1, 1);

const uint16_t UDP_PORT       = 4210;
const uint32_t CMD_TIMEOUT_MS = 500;    // failsafe: нет команд дольше — стоп

// Пины моторов ESP32-CAM 4WD (НЕ менять — это разводка платы).
const int LEFT_M0  = 13;
const int LEFT_M1  = 12;
const int RIGHT_M0 = 14;
const int RIGHT_M1 = 15;
const int STATUS_LED = 33;   // встроенный светодиод ESP32-CAM (активный LOW)
// ======================================================================

// ШИМ (LEDC), core 3.x: ledcAttach(pin, freq, res) + ledcWrite(pin, duty)
const int PWM_FREQ = 2000;   // как в оригинальной прошивке кита
const int PWM_RES  = 8;      // 8 бит -> 0..255

WiFiUDP udp;
char udpBuf[64];
volatile uint32_t lastCmdMs = 0;

// -------------------- МОТОРЫ --------------------
void setupMotors() {
  ledcAttach(LEFT_M0,  PWM_FREQ, PWM_RES);
  ledcAttach(LEFT_M1,  PWM_FREQ, PWM_RES);
  ledcAttach(RIGHT_M0, PWM_FREQ, PWM_RES);
  ledcAttach(RIGHT_M1, PWM_FREQ, PWM_RES);
}

// speed: -255..255. Дольный ШИМ: вперёд -> M1, назад -> M0.
void driveSide(int m0, int m1, int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    ledcWrite(m0, 0);
    ledcWrite(m1, speed);
  } else if (speed < 0) {
    ledcWrite(m0, -speed);
    ledcWrite(m1, 0);
  } else {
    ledcWrite(m0, 0);
    ledcWrite(m1, 0);
  }
}

void setMotors(int left, int right) {
  driveSide(LEFT_M0,  LEFT_M1,  left);
  driveSide(RIGHT_M0, RIGHT_M1, right);
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
  stopMotors();                 // моторы выключены сразу
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, HIGH);   // LED выключен (активный LOW)

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
    stopMotors();               // на всякий случай держим стоп, пока не подняли WiFi
    if (millis() - t0 > 20000) {
      Serial.println("\nWiFi не поднялся, перезагрузка");
      ESP.restart();
    }
  }
  digitalWrite(STATUS_LED, LOW);    // LED горит = WiFi есть
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
