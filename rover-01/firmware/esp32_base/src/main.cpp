// rover-01 — прошивка ESP32 как micro-ROS нода
// ------------------------------------------------------------
// Роль: низкоуровневый слой реального времени и безопасности.
//   - подписка на /cmd_vel (geometry_msgs/Twist)
//   - перевод в skid-steer и управление L298N (борт = канал)
//   - watchdog: нет команды дольше CMD_TIMEOUT_MS -> стоп
//   - slew-rate (плавный пуск), мёртвая зона ШИМ
//   - публикация /rover/heartbeat (Int32-счётчик) — признак "жив"
//   - автопереподключение к micro-ROS агенту (state machine)
//
// ВАЖНО: этот слой должен уметь остановить робота САМ, без Jetson.
// Watchdog ловит зависший Jetson, обрыв USB и упавшую ноду.
// ------------------------------------------------------------

#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int32.h>

#include "config.h"

// ───────────────────────── micro-ROS сущности ─────────────────────────
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;

rcl_subscription_t cmd_vel_sub;
geometry_msgs__msg__Twist cmd_vel_msg;

rcl_publisher_t heartbeat_pub;
std_msgs__msg__Int32 heartbeat_msg;

rcl_timer_t control_timer;

// ───────────────────────── Состояние управления ─────────────────────────
volatile unsigned long last_cmd_ms = 0;  // время последнего /cmd_vel
float target_left = 0.0f, target_right = 0.0f;  // целевые скорости бортов, м/с
float cur_left = 0.0f, cur_right = 0.0f;        // текущие (после slew), м/с
int32_t heartbeat_count = 0;
uint8_t tick_count = 0;

// Машина состояний подключения к агенту — стандартный надёжный паттерн micro-ROS.
enum AgentState { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED };
AgentState agent_state = WAITING_AGENT;

// ───────────────────────── Макросы проверки ─────────────────────────
// "Мягкий" вариант: при ошибке не зависаем в loop'е, а возвращаем false выше,
// чтобы машина состояний пересоздала сущности.
#define RCCHECK(fn)        { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) { return false; } }
#define RCSOFTCHECK(fn)    { rcl_ret_t rc = fn; (void)rc; }
#define EXECUTE_EVERY_N_MS(MS, X) do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis(); } \
  if (uxr_millis() - init > (MS)) { X; init = uxr_millis(); } \
} while (0)

// ───────────────────────── Управление мотором ─────────────────────────
// speed_mps: скорость борта в м/с (знак = направление).
void setMotor(int in_a, int in_b, int ledc_ch, float speed_mps) {
  float norm = speed_mps / MAX_WHEEL_SPEED;     // нормируем в [-1, 1]
  if (norm > 1.0f)  norm = 1.0f;
  if (norm < -1.0f) norm = -1.0f;

  float mag = fabsf(norm);

  // Около нуля — полная остановка (coast): оба входа LOW, ШИМ 0.
  if (mag < 0.02f) {
    digitalWrite(in_a, LOW);
    digitalWrite(in_b, LOW);
    ledcWrite(ledc_ch, 0);
    return;
  }

  // Масштабируем за мёртвой зоной: [DEADZONE .. PWM_MAX].
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

void stopMotorsHard() {
  cur_left = cur_right = 0.0f;
  target_left = target_right = 0.0f;
  digitalWrite(PIN_IN1, LOW); digitalWrite(PIN_IN2, LOW); ledcWrite(LEDC_CH_LEFT, 0);
  digitalWrite(PIN_IN3, LOW); digitalWrite(PIN_IN4, LOW); ledcWrite(LEDC_CH_RIGHT, 0);
}

// Плавно приближаем cur к target (ограничение ускорения).
static float slew(float cur, float target) {
  float max_step = SLEW_PER_TICK * MAX_WHEEL_SPEED;  // шаг в м/с за тик
  float d = target - cur;
  if (d >  max_step) d =  max_step;
  if (d < -max_step) d = -max_step;
  return cur + d;
}

// ───────────────────────── Колбэк /cmd_vel ─────────────────────────
void cmd_vel_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *m = (const geometry_msgs__msg__Twist *)msgin;
  float x = (float)m->linear.x;    // вперёд, м/с
  float z = (float)m->angular.z;   // поворот, рад/с

  // skid-steer: скорости бортов
  target_left  = x - z * (WHEEL_BASE_M / 2.0f);
  target_right = x + z * (WHEEL_BASE_M / 2.0f);

  last_cmd_ms = millis();          // пнули watchdog
}

// ───────────────────────── Управляющий таймер ─────────────────────────
void control_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  (void)last_call_time;
  if (timer == NULL) return;

  // Watchdog: давно не было команды -> цель = стоп.
  if (millis() - last_cmd_ms > CMD_TIMEOUT_MS) {
    target_left = 0.0f;
    target_right = 0.0f;
  }

  // Плавный выход на целевую скорость.
  cur_left  = slew(cur_left,  target_left);
  cur_right = slew(cur_right, target_right);

  // Применяем к моторам.
  setMotor(PIN_IN1, PIN_IN2, LEDC_CH_LEFT,  cur_left);
  setMotor(PIN_IN3, PIN_IN4, LEDC_CH_RIGHT, cur_right);

  // Heartbeat раз в HEARTBEAT_EVERY_TICKS тиков.
  if (++tick_count >= HEARTBEAT_EVERY_TICKS) {
    tick_count = 0;
    heartbeat_msg.data = ++heartbeat_count;
    RCSOFTCHECK(rcl_publish(&heartbeat_pub, &heartbeat_msg, NULL));
  }
}

// ───────────────────────── Создание / удаление сущностей ─────────────────────────
bool create_entities() {
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, NODE_NAME, "", &support));

  RCCHECK(rclc_subscription_init_default(
      &cmd_vel_sub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      TOPIC_CMD_VEL));

  RCCHECK(rclc_publisher_init_default(
      &heartbeat_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      TOPIC_HEARTBEAT));

  RCCHECK(rclc_timer_init_default(
      &control_timer, &support,
      RCL_MS_TO_NS(CONTROL_PERIOD_MS),
      control_timer_callback));

  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(
      &executor, &cmd_vel_sub, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &control_timer));

  return true;
}

void destroy_entities() {
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_subscription_fini(&cmd_vel_sub, &node);
  rcl_publisher_fini(&heartbeat_pub, &node);
  rcl_timer_fini(&control_timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

// ───────────────────────── setup / loop ─────────────────────────
void setup() {
  // Пины направления
  pinMode(PIN_IN1, OUTPUT); pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT); pinMode(PIN_IN4, OUTPUT);

  // ШИМ на ENA/ENB через LEDC (Arduino-core 2.x API)
  ledcSetup(LEDC_CH_LEFT,  PWM_FREQ_HZ, PWM_RES_BITS);
  ledcSetup(LEDC_CH_RIGHT, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcAttachPin(PIN_ENA, LEDC_CH_LEFT);
  ledcAttachPin(PIN_ENB, LEDC_CH_RIGHT);

  stopMotorsHard();  // безопасное стартовое состояние — стоим

  // Транспорт micro-ROS: Serial (USB). Совпадает с monitor_speed и baudrate агента.
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);

  last_cmd_ms = millis();
  agent_state = WAITING_AGENT;
}

void loop() {
  switch (agent_state) {
    case WAITING_AGENT:
      // пингуем агента раз в 500 мс
      EXECUTE_EVERY_N_MS(500,
        agent_state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;

    case AGENT_AVAILABLE:
      agent_state = create_entities() ? AGENT_CONNECTED : WAITING_AGENT;
      if (agent_state == WAITING_AGENT) destroy_entities();
      break;

    case AGENT_CONNECTED:
      // следим, что агент жив
      EXECUTE_EVERY_N_MS(200,
        agent_state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (agent_state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(20));
      }
      break;

    case AGENT_DISCONNECTED:
      // потеряли агента — НЕМЕДЛЕННО стоп (страховка поверх watchdog), затем чистимся
      stopMotorsHard();
      destroy_entities();
      agent_state = WAITING_AGENT;
      break;
  }

  // Дополнительная страховка: если по любой причине executor не крутится,
  // а команд нет — моторы всё равно должны стоять.
  if (agent_state != AGENT_CONNECTED && (millis() - last_cmd_ms > CMD_TIMEOUT_MS)) {
    stopMotorsHard();
  }
}
