// ESP32 + L298N + micro-ROS (Wi-Fi UDP) + PlatformIO
// - Subscribes to /cmd_vel (geometry_msgs/Twist)
// - Pure in-place spin when linear.x == 0 and angular.z != 0
// - Uses your wiring & network settings
// - No movement on boot; safe stop on timeout

#include <Arduino.h>
#include <WiFi.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/joint_state.h>
#include <rosidl_runtime_c/string_functions.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// =================== Wi-Fi + micro-ROS agent ===================
static char WIFI_SSID[] = "iPhone";          // your Wi-Fi SSID (mutable char[])
static char WIFI_PSK[]  = "12345678";     // your Wi-Fi password (mutable char[])
IPAddress AGENT_IP(172,20,10,3);         // micro-ROS agent IP
uint16_t AGENT_PORT = 8888;                // micro-ROS agent port

// =================== Motor pins (your wiring) ==================
#define ENA 25   // Right motor PWM
#define IN1 26   // Right motor dir A
#define IN2 27   // Right motor dir B
#define IN3 32   // Left  motor dir A
#define IN4 33   // Left  motor dir B
#define ENB 14   // Left  motor PWM

// =================== Encoder pins =================================
#define ENC_RIGHT_A  18  // Right encoder channel A
#define ENC_RIGHT_B  19  // Right encoder channel B
#define ENC_LEFT_A   21  // Left encoder channel A
#define ENC_LEFT_B   22  // Left encoder channel B

// Encoder specs (adjust for your TT motor: typically 11 PPR * gear_ratio)
static const int32_t TICKS_PER_REV = 528;  // 11 PPR * 48:1 gearbox

// Software direction flips (toggle if a wheel turns the wrong way)
static const bool LEFT_DIR_INVERT  = true;
static const bool RIGHT_DIR_INVERT = true;

// =================== PWM (LEDC) ================================
static const int PWM_CH_RIGHT = 0;
static const int PWM_CH_LEFT  = 1;
static const int PWM_FREQ     = 20000;  // 20 kHz (inaudible)
static const int PWM_RES      = 8;      // 0..255 duty

// Duty floors so small non-zero commands actually move the wheels
static const uint8_t MIN_DUTY      = 30;
// ↓ Gentler spin: lower PWM floor so it doesn't jump too hard
static const uint8_t MIN_DUTY_SPIN = 80;   // was 100

// =================== Kinematics / Scaling ======================
static const float BASE_WIDTH_M    = 0.12f;  // your track width (12 cm)
static const float WHEEL_RADIUS_M  = 0.033f; // wheel radius
static const float MAX_RPM         = 70.0f;
static const float MAX_WHEEL_MS    = (2.0f * M_PI * WHEEL_RADIUS_M) * (MAX_RPM / 60.0f);

static const float LIN_DEADBAND    = 0.02f;  // m/s
static const float ANG_DEADBAND    = 0.05f;  // rad/s
// ↓ Gentler spin: lower per-wheel floor so small angular commands work
static const float SPIN_FLOOR_MS   = 0.08f;  // was 0.10

// ---- Angular shaping ----
static const float WZ_SIGN         = -1.0f;  // flip angular direction
static const float WZ_GAIN         = 3.0f;   // gentler spin gain (was 5.0f)

static const uint32_t CMD_TIMEOUT_MS = 500;  // stop if cmd stream stale

// =================== micro-ROS handles =========================
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_subscription_t cmd_sub;
rcl_publisher_t joint_state_pub;
rclc_executor_t executor;
geometry_msgs__msg__Twist cmd_msg;
sensor_msgs__msg__JointState joint_state_msg;

// =================== State ====================================
volatile float v_left_ms  = 0.0f;
volatile float v_right_ms = 0.0f;
bool spin_mode = false;
uint32_t last_cmd_ms = 0;

// Encoder state
volatile int32_t enc_left_ticks  = 0;
volatile int32_t enc_right_ticks = 0;
uint32_t last_encoder_pub_ms = 0;
static const uint32_t ENCODER_PUB_INTERVAL_MS = 50; // 20 Hz

// ↓ Small smoothing so spins are more delicate
float vL_filt = 0.0f, vR_filt = 0.0f;
static const float ALPHA = 0.25f;  // 0..1 (lower = smoother)

// =================== Encoder ISRs =============================
void IRAM_ATTR enc_right_isr() {
  if (digitalRead(ENC_RIGHT_B) == digitalRead(ENC_RIGHT_A)) {
    enc_right_ticks++;
  } else {
    enc_right_ticks--;
  }
}

void IRAM_ATTR enc_left_isr() {
  if (digitalRead(ENC_LEFT_B) == digitalRead(ENC_LEFT_A)) {
    enc_left_ticks++;
  } else {
    enc_left_ticks--;
  }
}

// =================== Helpers ==================================
static bool connectWiFi(unsigned long timeout_ms = 30000) {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PSK);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - t0 > timeout_ms) return false;
    delay(250);
  }
  return true;
}

static void waitForAgent() {
  while (rmw_uros_ping_agent(500, 1) != RMW_RET_OK) {
    delay(500);
  }
}

static inline uint8_t duty_from_speed(float v_ms, bool is_spin) {
  float m = fabsf(v_ms) / MAX_WHEEL_MS;
  if (m > 1.0f) m = 1.0f;
  uint8_t d = (uint8_t)lrintf(m * 255.0f);
  uint8_t min_duty = is_spin ? MIN_DUTY_SPIN : MIN_DUTY;
  if (d > 0 && d < min_duty) d = min_duty;
  return d;
}

static void set_motor_left(float v_ms) {
  float cmd = LEFT_DIR_INVERT ? -v_ms : v_ms;
  if (cmd >= 0) { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); }
  else          { digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); }
  ledcWrite(PWM_CH_LEFT, duty_from_speed(cmd, spin_mode));
}

static void set_motor_right(float v_ms) {
  float cmd = RIGHT_DIR_INVERT ? -v_ms : v_ms;
  if (cmd >= 0) { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); }
  else          { digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); }
  ledcWrite(PWM_CH_RIGHT, duty_from_speed(cmd, spin_mode));
}

static void stop_motors() {
  ledcWrite(PWM_CH_RIGHT, 0);
  ledcWrite(PWM_CH_LEFT,  0);
}

// =================== /cmd_vel callback ========================
static void cmd_vel_cb(const void * msgin) {
  const auto *m = (const geometry_msgs__msg__Twist *)msgin;

  float v  = (float)m->linear.x;
  float wz = (float)m->angular.z;

  if (!isfinite(v))  v  = 0.0f;
  if (!isfinite(wz)) wz = 0.0f;

  if (fabsf(v)  < LIN_DEADBAND) v  = 0.0f;
  if (fabsf(wz) < ANG_DEADBAND) wz = 0.0f;

  // ----- flip & gently amplify angular so normal values work -----
  wz = WZ_SIGN * WZ_GAIN * wz;

  // Default differential mapping (unicycle → wheels)
  float vL = v - 0.5f * wz * BASE_WIDTH_M;
  float vR = v + 0.5f * wz * BASE_WIDTH_M;

  // Force in-place spin when only angular is commanded
  if (v == 0.0f && wz != 0.0f) {
    float target = 0.5f * fabsf(wz) * BASE_WIDTH_M;  // ideal |vL|=|vR|
    if (target < SPIN_FLOOR_MS) target = SPIN_FLOOR_MS;
    // wz>0 (CCW): left backward, right forward
    vL = -copysignf(target, wz);
    vR =  copysignf(target, wz);
    spin_mode = true;
  } else {
    spin_mode = false;
  }

  // Clamp
  vL = fmaxf(fminf(vL,  MAX_WHEEL_MS), -MAX_WHEEL_MS);
  vR = fmaxf(fminf(vR,  MAX_WHEEL_MS), -MAX_WHEEL_MS);

  // ---- Smooth the commands for a delicate feel ----
  vL_filt = vL_filt + ALPHA * (vL - vL_filt);
  vR_filt = vR_filt + ALPHA * (vR - vR_filt);

  // Command hardware (inversion is applied inside setters)
  v_left_ms  = vL_filt;
  v_right_ms = vR_filt;
  set_motor_left(v_left_ms);
  set_motor_right(v_right_ms);

  last_cmd_ms = millis();
}

// =================== Arduino setup/loop =======================
void setup() {
  Serial.begin(115200);
  delay(200);

  // Direction pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Encoder pins
  pinMode(ENC_RIGHT_A, INPUT_PULLUP);
  pinMode(ENC_RIGHT_B, INPUT_PULLUP);
  pinMode(ENC_LEFT_A, INPUT_PULLUP);
  pinMode(ENC_LEFT_B, INPUT_PULLUP);
  
  // Attach interrupts for encoders
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_A), enc_right_isr, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A), enc_left_isr, RISING);

  // PWM setup (LEDC)
  ledcSetup(PWM_CH_RIGHT, PWM_FREQ, PWM_RES);
  ledcAttachPin(ENA, PWM_CH_RIGHT);
  ledcSetup(PWM_CH_LEFT,  PWM_FREQ, PWM_RES);
  ledcAttachPin(ENB, PWM_CH_LEFT);

  stop_motors();  // ensure no movement at boot

  // Wi-Fi
  if (!connectWiFi()) {
    Serial.println("[WiFi] connect failed; halting");
    while (true) { delay(1000); }
  }
  Serial.print("[WiFi] IP: "); Serial.println(WiFi.localIP());

  // micro-ROS transport (Wi-Fi UDP)
  set_microros_wifi_transports(WIFI_SSID, WIFI_PSK, AGENT_IP, AGENT_PORT);
  Serial.println("[micro-ROS] waiting for agent...");
  waitForAgent();
  Serial.println("[micro-ROS] agent found");

  // micro-ROS init
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_diffdrive_node", "", &support);

  geometry_msgs__msg__Twist__init(&cmd_msg);
  rclc_subscription_init_default(
    &cmd_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/cmd_vel");

  // Initialize joint_state publisher
  sensor_msgs__msg__JointState__init(&joint_state_msg);
  
  // Allocate arrays for 2 joints
  joint_state_msg.name.capacity = 2;
  joint_state_msg.name.size = 2;
  joint_state_msg.name.data = (rosidl_runtime_c__String*)malloc(2 * sizeof(rosidl_runtime_c__String));
  rosidl_runtime_c__String__init(&joint_state_msg.name.data[0]);
  rosidl_runtime_c__String__init(&joint_state_msg.name.data[1]);
  rosidl_runtime_c__String__assign(&joint_state_msg.name.data[0], "left_wheel_joint");
  rosidl_runtime_c__String__assign(&joint_state_msg.name.data[1], "right_wheel_joint");
  
  joint_state_msg.position.capacity = 2;
  joint_state_msg.position.size = 2;
  joint_state_msg.position.data = (double*)malloc(2 * sizeof(double));
  
  joint_state_msg.velocity.capacity = 2;
  joint_state_msg.velocity.size = 2;
  joint_state_msg.velocity.data = (double*)malloc(2 * sizeof(double));
  
  joint_state_msg.effort.capacity = 0;
  joint_state_msg.effort.size = 0;
  joint_state_msg.effort.data = NULL;
  
  rclc_publisher_init_default(
    &joint_state_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "/joint_states");

  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &cmd_sub, &cmd_msg, &cmd_vel_cb, ON_NEW_DATA);

  last_cmd_ms = millis();
  last_encoder_pub_ms = millis();
  Serial.println("[Setup] Ready: sub /cmd_vel, pub /joint_states");
}

void loop() {
  // Process incoming ROS messages
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));

  // Publish joint states at regular intervals
  if (millis() - last_encoder_pub_ms >= ENCODER_PUB_INTERVAL_MS) {
    // Convert ticks to radians
    joint_state_msg.position.data[0] = (double)enc_left_ticks * 2.0 * M_PI / TICKS_PER_REV;
    joint_state_msg.position.data[1] = (double)enc_right_ticks * 2.0 * M_PI / TICKS_PER_REV;
    
    // Velocity (could calculate from position change, set to 0 for now)
    joint_state_msg.velocity.data[0] = 0.0;
    joint_state_msg.velocity.data[1] = 0.0;
    
    // Timestamp
    joint_state_msg.header.stamp.sec = millis() / 1000;
    joint_state_msg.header.stamp.nanosec = (millis() % 1000) * 1000000;
    
    rcl_publish(&joint_state_pub, &joint_state_msg, NULL);
    last_encoder_pub_ms = millis();
  }

  // Safety: stop if cmd stream goes stale
  if (millis() - last_cmd_ms > CMD_TIMEOUT_MS) {
    stop_motors();
    v_left_ms = v_right_ms = 0.0f;
  }

  // Keep Wi-Fi alive
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.disconnect(true);
    WiFi.begin(WIFI_SSID, WIFI_PSK);
  }

  delay(2);
}
