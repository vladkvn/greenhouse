// rover-01 — сборка microros: ESP32 как ROS2-нода (приём /cmd_vel по ROS2).
// ------------------------------------------------------------
// Роль: низкоуровневый слой реального времени и безопасности.
//   - подписка на /cmd_vel (geometry_msgs/Twist) -> motorsSetTarget()
//   - управляющий тик (watchdog + slew + моторы) -> motorsTick()
//   - публикация /rover/heartbeat (Int32-счётчик) — признак "жив"
//   - автопереподключение к micro-ROS агенту (state machine)
//
// Вся моторная логика — в общем модуле motor_control.* (см. motor_control.h).
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
#include "motor_control.h"

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

int32_t heartbeat_count = 0;
uint8_t tick_count = 0;

enum AgentState { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED };
AgentState agent_state = WAITING_AGENT;

#define RCCHECK(fn)     { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) { return false; } }
#define RCSOFTCHECK(fn) { rcl_ret_t rc = fn; (void)rc; }
#define EXECUTE_EVERY_N_MS(MS, X) do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis(); } \
  if (uxr_millis() - init > (MS)) { X; init = uxr_millis(); } \
} while (0)

// ───────────────────────── Колбэки ─────────────────────────
void cmd_vel_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *m = (const geometry_msgs__msg__Twist *)msgin;
  motorsSetTarget((float)m->linear.x, (float)m->angular.z);
}

void control_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  (void)last_call_time;
  if (timer == NULL) return;

  motorsTick();   // watchdog + slew + моторы

  if (++tick_count >= HEARTBEAT_EVERY_TICKS) {
    tick_count = 0;
    heartbeat_msg.data = ++heartbeat_count;
    RCSOFTCHECK(rcl_publish(&heartbeat_pub, &heartbeat_msg, NULL));
  }
}

// ───────────────────────── Сущности ─────────────────────────
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
  motorsBegin();   // пины, ШИМ, безопасный стоп

  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);

  agent_state = WAITING_AGENT;
}

void loop() {
  switch (agent_state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500,
        agent_state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;

    case AGENT_AVAILABLE:
      agent_state = create_entities() ? AGENT_CONNECTED : WAITING_AGENT;
      if (agent_state == WAITING_AGENT) destroy_entities();
      break;

    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200,
        agent_state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (agent_state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(20));
      }
      break;

    case AGENT_DISCONNECTED:
      motorsStopHard();        // потеряли агента -> немедленный стоп
      destroy_entities();
      agent_state = WAITING_AGENT;
      break;
  }

  // Страховка: если executor не крутится и команд нет — моторы стоят.
  if (agent_state != AGENT_CONNECTED && motorsMsSinceLastCmd() > CMD_TIMEOUT_MS) {
    motorsStopHard();
  }
}
