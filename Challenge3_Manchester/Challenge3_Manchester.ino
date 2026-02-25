// Include Libraries to be used
#include <micro_ros_arduino.h>    //micro-ros-arduino library
#include <rcl/rcl.h>              //Core ROS 2 Client Library (RCL) for node management.
#include <rcl/error_handling.h>   //Error handling utilities for Micro-ROS.
#include <rclc/rclc.h>            //Micro-ROS Client library for embedded devices.
#include <rclc/executor.h>        //Micro-ROS Executor to manage callbacks
#include <std_msgs/msg/int32.h>   //Predefined ROS 2 message type
#include <rmw_microros/rmw_microros.h>
#include <stdio.h>                //Standard I/O library for debugging.

rcl_node_t node; //Declarar node /motor 

//Instantiate executor and its support classes
rclc_executor_t executor;   //Manages task execution (timers, callbacks, etc.), dice quien va primero y ordena
rclc_support_t support;     //Handles initialization & communication setup.
rcl_allocator_t allocator;  //Manages memory allocation.

rcl_publisher_t publisher; //Crear publisher
rcl_timer_t timer; //crea timer

std_msgs__msg__Int32 msg; //define mensajes

//Posibles fallos

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}} //errores críticos, entra a error_loop()

#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}} //errores menores

//  Executes a given statement (X) periodically every MS milliseconds
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \    //si es la primera ves que lo llaman (-1) inicia el conteo, es estática para que siempre pueda cecar
  if (init == -1) { init = uxr_millis();} \ //actualiza el tiempo si cumple 
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \ //ejecuta la acción X dependiendo de la condición, sino vuelve a checar
} while (0)\

//#define LED_PIN 22  Por definir, pin donde mostrará el error si es que hay

/*void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN)); //Por si el agent no responde, parpadeda rápido
    delay(100);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    msg.data++;
  }
}*/

//Defines State Machine States
enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

bool create_entities()
{
   //Initializes memory allocation for Micro-ROS operations.
  allocator = rcl_get_default_allocator();

   //Creates a ROS 2 support structure to manage the execution context.
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_pub_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "/cmd_pwm"));

    // create timer,
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create zero initialised executor (no configured) to avoid memory problems
  executor = rclc_executor_get_zero_initialized_executor();
  // Initializes the Micro-ROS Executor, which manages tasks and callbacks.
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  // Register timer with executor
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  return true;
}

void destroy_entities()
{
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&publisher, &node);
  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void setup() {
  set_microros_transports(); // Le da un canal por donde se pueda comunicar micro-ROS

  allocator = rcl_get_default_allocator(); //define como se asigna la memoriapara crear entidades *checar
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator)); //*checar
  RCCHECK(rclc_node_init_default(&node, "/motor", "", &support)); //nombre del nodo y lo inicializas con support

  // crear publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,                         
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),  //tipo de mensaje
    "micro_ros_counter"));

  // crear timer,
  const unsigned int timer_timeout = 1000; //cada segundo evía mensaje
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,                             //Todo usa el support
    RCL_MS_TO_NS(timer_timeout), 
    timer_callback));                     //lo que voy a hacer cuando se termine el timer

  // Initializes the Micro-ROS Executor, which manages tasks and callbacks.
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator)); //decirle cuantos timers, hamlers tenemos pq hace separación de memoria
  // Register timer with executor
  RCCHECK(rclc_executor_add_timer(&executor, &timer)); //registramos timer en ejector para checar que se vaya cumpliedo

  // Initialise message
  msg.data = 0;
}

void loop() {
  // put your main code here, to run repeatedly:

}
