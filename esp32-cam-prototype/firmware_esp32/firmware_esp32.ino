/*
 * GreenHouse — прототип робота на ESP32-CAM
 * ------------------------------------------
 * Что делает прошивка:
 *   1. Поднимает WiFi (режим станции — подключается к твоему роутеру).
 *   2. Стримит видео с камеры по HTTP (MJPEG)  ->  http://<IP>:81/stream
 *      и отдаёт одиночный кадр                  ->  http://<IP>/jpg
 *   3. Слушает UDP-порт 4210 и принимает команды управления моторами.
 *
 * Протокол команд (UDP, обычный текст, одна строка):
 *      "L R"   где L и R — скорости левого и правого моторов, от -255 до 255.
 *      Примеры:  "200 200"  — вперёд
 *                "-200 -200" — назад
 *                "200 -200"  — поворот на месте
 *                "0 0"       — стоп
 *      Спец-команда: "STOP" — немедленная остановка.
 *
 * Безопасность: если команд нет дольше CMD_TIMEOUT_MS — моторы глушатся
 * (failsafe на случай потери связи).
 *
 * Плата: AI-Thinker ESP32-CAM.
 * Драйвер моторов: L298N (или совместимый), дифференциальный привод (2 мотора).
 *
 * ВАЖНО про пины: у ESP32-CAM почти все ноги заняты камерой и PSRAM.
 * Свободны и безопасны для вывода: GPIO 12, 13, 14, 15, 2, 4 (4 — это
 * встроенный фонарик-LED), 16. Ниже выбраны 12,13,14,15. GPIO0 не трогаем
 * (нужен для прошивки). Если будешь использовать SD-карту — пины пересекутся,
 * в прототипе SD не используем.
 */

#include "esp_camera.h"
#include <WiFi.h>
#include <WiFiUdp.h>

// ================== НАСТРОЙКИ — ОТРЕДАКТИРУЙ ПОД СЕБЯ ==================
const char* WIFI_SSID = "ТВОЙ_WIFI";
const char* WIFI_PASS = "ПАРОЛЬ_WIFI";

const uint16_t UDP_PORT      = 4210;   // порт приёма команд
const uint32_t CMD_TIMEOUT_MS = 500;   // failsafe: нет команд дольше — стоп

// Пины драйвера моторов L298N (IN1..IN4 + ENA/ENB для ШИМ-скорости)
const int PIN_L_IN1 = 12;
const int PIN_L_IN2 = 13;
const int PIN_R_IN1 = 14;
const int PIN_R_IN2 = 15;
const int PIN_L_EN  = 2;    // ENA — ШИМ скорости левого
const int PIN_R_EN  = 16;   // ENB — ШИМ скорости правого
// ======================================================================

// --- Распиновка камеры AI-Thinker ESP32-CAM ---
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// ШИМ-каналы для скорости моторов
const int PWM_CH_L = 3;
const int PWM_CH_R = 4;
const int PWM_FREQ = 5000;
const int PWM_RES  = 8;    // 8 бит -> 0..255

WiFiUDP udp;
char udpBuf[64];
volatile uint32_t lastCmdMs = 0;

WiFiServer streamServer(81);   // MJPEG-стрим

// -------------------- МОТОРЫ --------------------
void setupMotors() {
  pinMode(PIN_L_IN1, OUTPUT);
  pinMode(PIN_L_IN2, OUTPUT);
  pinMode(PIN_R_IN1, OUTPUT);
  pinMode(PIN_R_IN2, OUTPUT);
  ledcSetup(PWM_CH_L, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CH_R, PWM_FREQ, PWM_RES);
  ledcAttachPin(PIN_L_EN, PWM_CH_L);
  ledcAttachPin(PIN_R_EN, PWM_CH_R);
}

// speed: -255..255
void driveMotor(int in1, int in2, int pwmCh, int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    ledcWrite(pwmCh, speed);
  } else if (speed < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    ledcWrite(pwmCh, -speed);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    ledcWrite(pwmCh, 0);
  }
}

void setMotors(int left, int right) {
  driveMotor(PIN_L_IN1, PIN_L_IN2, PWM_CH_L, left);
  driveMotor(PIN_R_IN1, PIN_R_IN2, PWM_CH_R, right);
}

void stopMotors() { setMotors(0, 0); }

// -------------------- КАМЕРА --------------------
bool setupCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  // Качество/размер кадра. Для низкой задержки держим небольшое разрешение.
  if (psramFound()) {
    config.frame_size = FRAMESIZE_VGA;   // 640x480
    config.jpeg_quality = 12;            // меньше = лучше качество, больше трафик
    config.fb_count = 2;
    config.grab_mode = CAMERA_GRAB_LATEST;
    config.fb_location = CAMERA_FB_IN_PSRAM;
  } else {
    config.frame_size = FRAMESIZE_QVGA;  // 320x240
    config.jpeg_quality = 15;
    config.fb_count = 1;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    config.fb_location = CAMERA_FB_IN_DRAM;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Ошибка инициализации камеры: 0x%x\n", err);
    return false;
  }
  return true;
}

// -------------------- MJPEG-СТРИМ --------------------
// Простой блокирующий обработчик одного клиента на порту 81.
void handleStreamClient(WiFiClient &client) {
  client.print("HTTP/1.1 200 OK\r\n");
  client.print("Content-Type: multipart/x-mixed-replace; boundary=frame\r\n");
  client.print("Cache-Control: no-cache\r\n\r\n");

  while (client.connected()) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) break;
    client.print("--frame\r\n");
    client.print("Content-Type: image/jpeg\r\n");
    client.printf("Content-Length: %u\r\n\r\n", fb->len);
    client.write(fb->buf, fb->len);
    client.print("\r\n");
    esp_camera_fb_return(fb);

    // Параллельно обрабатываем команды и failsafe, чтобы не зависнуть.
    pollUdp();
    failsafeCheck();
    if (!client.connected()) break;
  }
}

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
  Serial.setDebugOutput(false);

  setupMotors();
  stopMotors();

  if (!setupCamera()) {
    Serial.println("Камера не запустилась — перезагрузка через 3с");
    delay(3000);
    ESP.restart();
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Подключение к WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(400);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("IP-адрес: ");
  Serial.println(WiFi.localIP());
  Serial.printf("Стрим:    http://%s:81/stream\n", WiFi.localIP().toString().c_str());
  Serial.printf("Команды:  UDP %s:%u  (формат \"L R\", -255..255)\n",
                WiFi.localIP().toString().c_str(), UDP_PORT);

  udp.begin(UDP_PORT);
  streamServer.begin();
  lastCmdMs = millis();
}

void loop() {
  pollUdp();
  failsafeCheck();

  WiFiClient client = streamServer.available();
  if (client) {
    // Считываем строку запроса (нам не важно её содержимое — отдаём стрим).
    while (client.connected() && client.available()) client.read();
    handleStreamClient(client);
    client.stop();
  }
}
