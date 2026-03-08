#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>
#include <math.h>

// =======================
// ======  PINES  ========
// =======================
#define LED_PIN  23
#define PWM_PIN  22
#define IN1_PIN  19
#define IN2_PIN  18
#define ENC_A    25
#define ENC_B    26

// =======================
// ======  PWM SETUP =====
// =======================
#define PWM_FRQ   980
#define PWM_RES   8
#define PWM_CHNL  0

// =======================
// ====== MOTOR ==========
// =======================
#define PPR        44.5f   // pulsos por revolución del motor
#define GEAR_RATIO 45.0f   // reducción
#define TS         0.01f    // mismo periodo que el MPC

// =======================
// ====== ENCODER ========
// =======================
volatile long encoder_count = 0;
volatile float target_duty = 0.0f;

void IRAM_ATTR encoder_isr() {
  encoder_count++;
}

// =======================
// ===== MICRO-ROS =======
// =======================
rcl_node_t node;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;

rcl_subscription_t subscriber;
rcl_publisher_t publisher;
rcl_timer_t timer;

std_msgs__msg__Float32 msg_sub;
std_msgs__msg__Float32 msg_pub;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

// =======================
// ====== CALLBACK =======
// =======================
void subscription_callback(const void * msgin)
{
  const std_msgs__msg__Float32 * msg =
    (const std_msgs__msg__Float32 *)msgin;

  // El MPC manda voltaje en [-6, 6]V → convertir a duty [-1, 1]
  target_duty = msg->data;
  
  

}

// =======================
// ===== TIMER PUB =======
// =======================
void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer == NULL) return;

  // Dirección
  if (target_duty < 0.0f) {
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
    target_duty *= -1.0;
  } else {
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, HIGH);
  }

  uint32_t pwm_value = (uint32_t)((pow(2, PWM_RES) - 1) * target_duty);
  ledcWrite(PWM_CHNL, pwm_value);

  // Calcular velocidad del eje de salida
  static long prev_count = 0;
  long current_count = encoder_count;
  long delta = current_count - prev_count;
  prev_count = current_count;

  float speed_rps_motor  = (float)delta / (PPR * TS);
  float speed_rps_output = speed_rps_motor / GEAR_RATIO;
  float speed_raw        = speed_rps_output * 2.0f * M_PI;

  // Filtro promedio móvil
  #define FILTER_SIZE 5
  static float speed_buffer[FILTER_SIZE] = {0};
  static int filter_idx = 0;

  speed_buffer[filter_idx] = speed_raw;
  filter_idx = (filter_idx + 1) % FILTER_SIZE;

  float speed_filtered = 0.0f;
  for (int i = 0; i < FILTER_SIZE; i++) {
    speed_filtered += speed_buffer[i];
  }
  speed_filtered /= FILTER_SIZE;

  msg_pub.data = speed_filtered;
  RCSOFTCHECK(rcl_publish(&publisher, &msg_pub, NULL));
}

// =======================
// ========= SETUP =======
// =======================
void setup() {
  set_microros_transports();

  pinMode(LED_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);
  pinMode(ENC_A, INPUT);
  pinMode(ENC_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENC_A), encoder_isr, RISING);

  ledcSetup(PWM_CHNL, PWM_FRQ, PWM_RES);
  ledcAttachPin(PWM_PIN, PWM_CHNL);
  ledcWrite(PWM_CHNL, 0);

  /*digitalWrite(IN1_PIN, HIGH);
  digitalWrite(IN2_PIN, LOW);
  ledcWrite(PWM_CHNL, 255);*/

  delay(2000);

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "motor_driver_node", "", &support));


 //  Suscriptor: recibe voltaje del MPC en V
  RCCHECK(rclc_subscription_init_default(
    &subscriber, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/motor_duty"));

  // Publicador: manda velocidad en rad/s al MPC
  RCCHECK(rclc_publisher_init_default(
    &publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/motor_speed"));

  // Timer a 10ms, igual que el MPC
  RCCHECK(rclc_timer_init_default(
    &timer, &support,
    RCL_MS_TO_NS(10),
    timer_callback));

  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(
    &executor, &subscriber, &msg_sub,
    &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
}

// =======================
// ========= LOOP ========
// =======================
void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
}
