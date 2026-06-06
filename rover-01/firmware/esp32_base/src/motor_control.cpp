// rover-01 — реализация общего моторного модуля. См. motor_control.h.

#include <Arduino.h>
#include "config.h"
#include "motor_control.h"

// ───────────────────────── Внутреннее состояние ─────────────────────────
static volatile unsigned long s_last_cmd_ms = 0;     // время последней команды
static float s_target_left = 0.0f, s_target_right = 0.0f;  // цель, м/с
static float s_cur_left = 0.0f, s_cur_right = 0.0f;        // текущие, м/с

// ───────────────────────── Низкий уровень ─────────────────────────
// speed_mps: скорость борта в м/с (знак = направление).
static void applyMotor(int in_a, int in_b, int ledc_ch, float speed_mps) {
  float norm = speed_mps / MAX_WHEEL_SPEED;     // нормируем в [-1, 1]
  if (norm > 1.0f)  norm = 1.0f;
  if (norm < -1.0f) norm = -1.0f;

  float mag = fabsf(norm);

  // Около нуля — полная остановка (coast).
  if (mag < 0.02f) {
    digitalWrite(in_a, LOW);
    digitalWrite(in_b, LOW);
    ledcWrite(ledc_ch, 0);
    return;
  }

  // Масштаб за мёртвой зоной: [DEADZONE .. PWM_MAX].
  int pwm = PWM_DEADZONE + (int)((PWM_MAX - PWM_DEADZONE) * mag);
  if (pwm > PWM_MAX) pwm = PWM_MAX;

  if (norm >= 0.0f) {            // вперёд
    digitalWrite(in_a, HIGH);
    digitalWrite(in_b, LOW);
  } else {                       // назад
    digitalWrite(in_a, LOW);
    digitalWrite(in_b, HIGH);
  }
  ledcWrite(ledc_ch, pwm);
}

// Плавно приближаем cur к target (ограничение ускорения).
static float slew(float cur, float target) {
  float max_step = SLEW_PER_TICK * MAX_WHEEL_SPEED;  // м/с за тик
  float d = target - cur;
  if (d >  max_step) d =  max_step;
  if (d < -max_step) d = -max_step;
  return cur + d;
}

// ───────────────────────── Публичный API ─────────────────────────
void motorsBegin() {
  pinMode(PIN_IN1, OUTPUT); pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT); pinMode(PIN_IN4, OUTPUT);

  ledcSetup(LEDC_CH_LEFT,  PWM_FREQ_HZ, PWM_RES_BITS);
  ledcSetup(LEDC_CH_RIGHT, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcAttachPin(PIN_ENA, LEDC_CH_LEFT);
  ledcAttachPin(PIN_ENB, LEDC_CH_RIGHT);

  motorsStopHard();
  s_last_cmd_ms = millis();
}

void motorsSetTarget(float linear_x, float angular_z) {
  // skid-steer: скорости бортов
  s_target_left  = linear_x - angular_z * (WHEEL_BASE_M / 2.0f);
  s_target_right = linear_x + angular_z * (WHEEL_BASE_M / 2.0f);
  s_last_cmd_ms = millis();   // пнули watchdog
}

void motorsTick() {
  // Watchdog: давно не было команды -> цель = стоп.
  if (millis() - s_last_cmd_ms > CMD_TIMEOUT_MS) {
    s_target_left = 0.0f;
    s_target_right = 0.0f;
  }

  // Плавный выход на цель.
  s_cur_left  = slew(s_cur_left,  s_target_left);
  s_cur_right = slew(s_cur_right, s_target_right);

  // Применяем.
  applyMotor(PIN_IN1, PIN_IN2, LEDC_CH_LEFT,  s_cur_left);
  applyMotor(PIN_IN3, PIN_IN4, LEDC_CH_RIGHT, s_cur_right);
}

void motorsStopHard() {
  s_cur_left = s_cur_right = 0.0f;
  s_target_left = s_target_right = 0.0f;
  digitalWrite(PIN_IN1, LOW); digitalWrite(PIN_IN2, LOW); ledcWrite(LEDC_CH_LEFT, 0);
  digitalWrite(PIN_IN3, LOW); digitalWrite(PIN_IN4, LOW); ledcWrite(LEDC_CH_RIGHT, 0);
}

unsigned long motorsMsSinceLastCmd() {
  return millis() - s_last_cmd_ms;
}
