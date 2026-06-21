/*
 * GreenHouse — follower: прошивка модуля управления (USB-SERIAL вариант)
 * Плата: ESP32-CAM 4WD Robot Car (AI-Thinker, OV3660) — используется как тележка.
 * ------------------------------------------------------------------------------
 * ОТЛИЧИЕ ОТ WiFi-версии: команды приходят НЕ по WiFi/UDP, а по USB-SERIAL (UART0,
 * тот же кабель, что и для прошивки). Jetson подключён к ESP по USB и шлёт строки.
 *
 *   [Jetson: камера+лидар+IMU+логика]  --USB serial "L R\n"-->  [ESP32-CAM]  -->  4 мотора
 *
 * IMU (BNO085) к ESP НЕ подключается — он сидит напрямую на I2C Jetson (ESP32 не тянет
 * протокол SH2 поверх своего I2C из-за clock stretching). ESP — чисто моторы.
 *
 * Привод этого кита — дольный ШИМ: на каждый борт ДВА пина (M0/M1), без отдельного EN.
 * Направление задаётся тем, на какой из двух пинов подаётся ШИМ:
 *   вперёд  -> ШИМ на M1, M0=0
 *   назад   -> ШИМ на M0, M1=0
 * Пины моторов (из оригинальной прошивки кита):
 *   LEFT_M0=13  LEFT_M1=12   RIGHT_M0=14  RIGHT_M1=15
 * ВНИМАНИЕ: 12/14/15 — strapping-пины ESP32, при загрузке могут кратко дёрнуть мотор.
 *
 * Протокол команд (USB serial, текст, строки оканчиваются '\n'), 115200 бод:
 *   "L R\n"  скорости левого/правого бортов, -255..255 ("180 140\n" вперёд,
 *            "-180 -180\n" назад, "180 -180\n" поворот, "0 0\n" стоп).
 *   "STOP\n" немедленная остановка.
 * Failsafe: нет команд дольше CMD_TIMEOUT_MS — моторы глушатся.
 *
 * ЕСЛИ ПУТАЕТСЯ ПРАВО/ЛЕВО или ВПЕРЁД/НАЗАД — это правится здесь, в маппинге пинов
 * (поменять LEFT_* <-> RIGHT_*) или в driveSide (поменять M0 <-> M1), см. комментарии ниже.
 */

#include <Arduino.h>

// ================== ПИНЫ МОТОРОВ (разводка платы, не выдумывать) ==================
const int LEFT_M0  = 13;
const int LEFT_M1  = 12;
const int RIGHT_M0 = 14;
const int RIGHT_M1 = 15;
const int STATUS_LED = 33;   // встроенный светодиод ESP32-CAM (активный LOW)
// =================================================================================

// ШИМ (LEDC), core 3.x: ledcAttach(pin, freq, res) + ledcWrite(pin, duty)
const int PWM_FREQ = 2000;   // как в оригинальной прошивке кита
const int PWM_RES  = 8;      // 8 бит -> 0..255

const uint32_t CMD_TIMEOUT_MS = 500;  // failsafe: нет команд дольше — стоп

char lineBuf[48];
uint8_t lineLen = 0;
uint32_t lastCmdMs = 0;

// -------------------- МОТОРЫ --------------------
void setupMotors() {
  ledcAttach(LEFT_M0,  PWM_FREQ, PWM_RES);
  ledcAttach(LEFT_M1,  PWM_FREQ, PWM_RES);
  ledcAttach(RIGHT_M0, PWM_FREQ, PWM_RES);
  ledcAttach(RIGHT_M1, PWM_FREQ, PWM_RES);
}

// speed: -255..255. Дольный ШИМ: вперёд -> M1, назад -> M0.
// Если КОНКРЕТНОЕ колесо крутится не в ту сторону — поменяй местами m0 и m1 в его вызове.
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
  driveSide(LEFT_M0,  LEFT_M1,  left);   // если ПРАВО/ЛЕВО перепутаны — поменяй эти две строки
  driveSide(RIGHT_M0, RIGHT_M1, right);
}

void stopMotors() { setMotors(0, 0); }

// -------------------- РАЗБОР КОМАНД --------------------
void handleLine(char* line) {
  if (strncmp(line, "STOP", 4) == 0) {
    stopMotors();
    lastCmdMs = millis();
    return;
  }
  int l = 0, r = 0;
  if (sscanf(line, "%d %d", &l, &r) == 2) {
    setMotors(l, r);
    lastCmdMs = millis();
    digitalWrite(STATUS_LED, LOW);   // мигнём — команда принята (active LOW = горит)
  }
}

void pollSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineLen > 0) {
        lineBuf[lineLen] = '\0';
        handleLine(lineBuf);
        lineLen = 0;
      }
    } else if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    } else {
      lineLen = 0;  // переполнение — сбросить строку
    }
  }
}

void failsafeCheck() {
  if (millis() - lastCmdMs > CMD_TIMEOUT_MS) {
    stopMotors();
    digitalWrite(STATUS_LED, HIGH);  // нет команд — LED выключен
  }
}

// -------------------- SETUP / LOOP --------------------
void setup() {
  Serial.begin(115200);
  delay(200);
  setupMotors();
  stopMotors();                  // моторы выключены сразу
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, HIGH);
  lastCmdMs = millis();
  Serial.println("READY esp32-serial L R / STOP @115200");
}

void loop() {
  pollSerial();
  failsafeCheck();
}
