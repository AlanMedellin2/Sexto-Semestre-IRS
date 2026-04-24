// Include Libraries to be used
#include <micro_ros_arduino.h>          //micro-ros-arduino library
#include <rcl/rcl.h>                    //Core ROS 2 Client Library (RCL) for node management.
#include <rcl/error_handling.h>         //Error handling utilities for Micro-ROS.
#include <rclc/rclc.h>                  //Micro-ROS Client library for embedded devices.
#include <rclc/executor.h>              //Micro-ROS Executor to manage callbacks
#include <std_msgs/msg/float32.h>       //Predefined ROS 2 message type
#include <rmw_microros/rmw_microros.h>  //ROS Middleware for Micro-ROS, provides functions for interfacing Micro-ROS with DDS.
#include <stdio.h>                      //Standard I/O library for debugging.
#include <WiFi.h>
#include <math.h>
#include <geometry_msgs/msg/twist.h>

//Pines de encoder derecho
const int PIN_rightA = 13;
const int PIN_rightB = 41;

//Pines de encoder izquierdo
const int PIN_leftA = 34;
const int PIN_leftB = 44;

// motor izquierdo
const int PWM_left = 25;
const int DIR_left1 = 26; //checar
const int DIR_left2 = 27;

//motor derecho
const int PWM_right = 14;
const int DIR_right1 = 12; //checar
const int DIR_right2 = 19;

const int PWM_max = 255;
const float K_PWM = 100.0;   // ganancia de prueba, converte referencia de rueda a pwm
const float Wheel_axes = 0.18; // distancia entre ejes

//pulsos por revolución
const float PPR = 540; //Por definir

//contador de pulsos
volatile int32_t counter_left_encoder = 0;
volatile int32_t counter_right_encoder = 0;

//contadores anteriores, saber cuantos pulsos nuevos aparecieron antes 
int32_t prev_left= 0;
int32_t prev_right= 0;

float cmd_linear = 0.0;
float cmd_angular = 0.0;

void isr_right() {
  counter_right_encoder++;
}

void isr_left() {
  counter_left_encoder++;
}

//Declare nodes to be used
rcl_node_t node;  //Represents a ROS 2 Node running on the microcontroller.

//Instantiate executor and its support classes
rclc_executor_t executor;   //Manages task execution (timers, callbacks, etc.).
rclc_support_t support;     //Data structure that holds the execution context of Micro-ROS, including its communication state, memory management, and initialization data.
rcl_allocator_t allocator;  //Manages memory allocation.

//Declare Publishers to be used
rcl_publisher_t publisher;  //Declares a ROS 2 publisher for sending messages.
rcl_subscription_t subscriber;

//Declare timers to be used
rcl_timer_t timer;  //Creates a timer to execute functions at intervals.

//Declare Messages to be used
std_msgs__msg__Float32 msg;  //Defines a message of type int32.
geometry_msgs__msg__Twist cmd_msg; //mnsaje que recibe el subscriber


float dt = 0.05; //cada 50 ms se lee los datos del enconder

//Define Macros to be used
//Executes fn and returns false if it fails.
#define RCCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { return false; } \
  }

// Executes fn, but ignores failures.
#define RCSOFTCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) {} \
  }

#define EXECUTE_EVERY_N_MS(MS, X) \
  do { \
    static int64_t init = -1; \
    if (init == -1) { \
      init = uxr_millis(); \
    } \
    if (uxr_millis() - init > (MS)) { \
      X; \
      init = uxr_millis(); \
    } \
  } while (0)

  //Defines State Machine States
  enum states {
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
  } state;

//Define callbacks
void timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
float change_pulseL = 0;
float change_pulseR = 0;
float delta_VL = 0;
float delta_VR = 0;
float vel_left = 0;
float vel_right = 0;
//conteo actual del encoder
int32_t actual_left = counter_left_encoder;
int32_t actual_right = counter_right_encoder;

  //cambios de pulsos
  change_pulseL = actual_left - prev_left;
  change_pulseR = actual_right - prev_right;

  //cambio a vueltas de intervalo
  delta_VL = change_pulseL / PPR;
  delta_VR = change_pulseR / PPR;

  //vueltas sobre segundo, velocidad
  vel_left = delta_VL /dt;
  vel_right = delta_VR / dt;

  msg.data = vel_left;
  RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));

  prev_left = actual_left;
  prev_right = actual_right;

}

//función para mover un motor
void set_motor(int pwm_pin, int dir1, int dir2, float ref)
{
  int pwm_value = (int)(K_PWM * fabs(ref));

  if (pwm_value > PWM_max) {
    pwm_value = PWM_max;
  }

  if (ref > 0.0) {
    digitalWrite(dir1, HIGH);
    digitalWrite(dir2, LOW);
    analogWrite(pwm_pin, pwm_value);
  }
  else if (ref < 0.0) {
    digitalWrite(dir1, LOW);
    digitalWrite(dir2, HIGH);
    analogWrite(pwm_pin, pwm_value);
  }
  else {
    digitalWrite(dir1, LOW);
    digitalWrite(dir2, LOW);
    analogWrite(pwm_pin, 0);
  }
}

void update_motors_from_cmd() // Convierte cmd_vel a ruedas
{
  float ref_left  = cmd_linear - (cmd_angular * Wheel_axes / 2.0);
  float ref_right = cmd_linear + (cmd_angular * Wheel_axes / 2.0);

  set_motor(PWM_left, DIR_left1, DIR_left2, ref_left);
  set_motor(PWM_right, DIR_right1, DIR_right2, ref_right);
}
void cmd_vel_callback(const void * msgin) //void* --> no sabes que tipo de mensaje llega 
{
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin; //declaras que el mensaje que llego es un twist

  cmd_linear = msg->linear.x;
  cmd_angular = msg->angular.z;
  update_motors_from_cmd();
}

bool create_entities() {
  //Initializes memory allocation for Micro-ROS operations.
  allocator = rcl_get_default_allocator();

  //Creates a ROS 2 support structure to manage the execution context.
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_pub_node", "", &support));

  rmw_qos_profile_t qos = rmw_qos_profile_sensor_data;

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/sensor_  set_microros_wifi_transports(ssid, password, agent_ip, agent_port);  //Wifi agent
signal"));

      // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), //recibe velocidad
    "/cmd_vel"));

    // create timer,
  const unsigned int timer_timeout = 50;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create zero initialised executor (no configured) to avoid memory problems
  executor = rclc_executor_get_zero_initialized_executor();
  // Initializes the Micro-ROS Executor, which manages tasks and callbacks.
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  // Register timer with executor
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &cmd_msg, &cmd_vel_callback, ON_NEW_DATA));
  return true;
}

void destroy_entities() {
  rmw_context_t* rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&publisher, &node);
  rcl_subscription_fini(&subscriber, &node);
  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

//Setup
void setup() {

  pinMode(PIN_rightA, INPUT);
  pinMode(PIN_rightB, INPUT);
  pinMode(PIN_leftA, INPUT);
  pinMode(PIN_leftB, INPUT);

  pinMode(PWM_left, OUTPUT);
  pinMode(DIR_left1, OUTPUT);
  pinMode(DIR_left2, OUTPUT);

  pinMode(PWM_right, OUTPUT);
  pinMode(DIR_right1, OUTPUT);
  pinMode(DIR_right2, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(PIN_rightA), isr_right, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_leftA), isr_left, RISING);

  //Initial State
  state = WAITING_AGENT;

  // Initialise message
  msg.data = 0;
}

void loop() {
  switch (state) {

    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);  //ejecuta cada 500 ms
      break;

    case AGENT_AVAILABLE:
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;  //si encontró agente crea las entidades
      if (state == WAITING_AGENT) {
        destroy_entities();  //destriye entidades si no encuentra agente
      };
      break;

    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED) {  //si lo encontró checa lo demás, hace speen
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
      }
      break;

    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;

    default:
      break;
  }
}
