#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np

class OpenLoopCtrl(Node):
    def __init__(self):
        super().__init__('open_loop_ctrl')
        
        self.wait_for_ros_time()
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        #Constantes 
        self.VEL_LIN_MIN = 0.1549
        self.VEL_LIN_MAX = 0.83
        self.VEL_ANG_MIN = 0.82 
        self.VEL_ANG_MAX = 8.25

        #Datos figura
        self.side_length = 1.0  
        self.side_num = 5

        #Velocidades (la funcion clamp es para que no se salga de los limites)
        self.linear_speed = self.clamp(0.3, self.VEL_LIN_MIN, self.VEL_LIN_MAX)
        self.angular_speed = self.clamp(1.0, self.VEL_ANG_MIN, self.VEL_ANG_MAX)

        # Tiempos de rampa 
        self.ramp_time_linear = 1.0  
        self.ramp_time_angular = 0.8 

        #Formula para calcular los angulos externos de las figuras y convertidos a radianes
        self.angle = 360/self.side_num
        self.angle_rad = np.deg2rad(self.angle)
        #self.angle = ((self.side_num*2)*180)/self.side_num Esta fromula es paar los angulos internos pero no se ocupa xd

        # Cálculo de tiempos de ejecución 
        self.straight_time = self.side_length / self.linear_speed
        self.turn_time = self.angle_rad / self.angular_speed
        
        # Máquina de estados
        self.state = 'STRAIGHT'
        self.side_count = 0
        self.state_start_time = self.get_clock().now()

        self.timer_period = 0.05
        self.timer = self.create_timer(self.timer_period, self.control_loop)

        self.get_logger().info('Controlador Puzzlebot Iniciado')

    def wait_for_ros_time(self):
        self.get_logger().info('Esperando tiempo de ROS...')
        while rclpy.ok():
            now = self.get_clock().now()
            if now.nanoseconds > 0:
                break
            rclpy.spin_once(self, timeout_sec=0.1)
        
    def clamp(self, value, min_val, max_val):
        return max(min_val, min(value, max_val))

    def ramp_speed(self, target_speed, min_speed, max_speed, elapsed_time, ramp_time):

        target_speed = self.clamp(target_speed, min_speed, max_speed)

        if elapsed_time >= ramp_time:
            return target_speed

        # Interpolación lineal
        current_vel = min_speed + (target_speed - min_speed) * (elapsed_time / ramp_time)

        # Clamp final por seguridad
        return self.clamp(current_vel, min_speed, max_speed)

    def control_loop(self):
        now = self.get_clock().now()
        elapsed = (now - self.state_start_time).nanoseconds * 1e-9
        cmd = Twist()

        if self.state == 'STRAIGHT':
            # Aplicamos rampa lineal
            cmd.linear.x = self.ramp_speed(
                self.linear_speed,
                self.VEL_LIN_MIN,
                self.VEL_LIN_MAX,
                elapsed,
                self.ramp_time_linear
            )
            cmd.angular.z = 0.0

            if elapsed >= self.straight_time:
                self.state = 'TURN'
                self.state_start_time = now
                self.get_logger().info(f'Lado {self.side_count + 1} completado. Girando...')

        elif self.state == 'TURN':
            cmd.linear.x = 0.0
            # Aplicamos rampa angular
            cmd.angular.z = self.ramp_speed(
                self.angular_speed,
                self.VEL_ANG_MIN,
                self.VEL_ANG_MAX,
                elapsed,
                self.ramp_time_angular
            )

            if elapsed >= self.turn_time:
                self.side_count += 1
                # Si ya completó todos los lados
                if self.side_count >= self.side_num:
                    self.state = 'STOP'
                    self.state_start_time = now
                    self.get_logger().info('Figura completada. Deteniéndose...')
                
                # Si aún faltan lados
                else:
                    self.state = 'STRAIGHT'
                    self.state_start_time = now
                    self.get_logger().info(f'Lado {self.side_count + 1} de {self.side_num}')

        elif self.state == 'STOP':
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd)
            self.get_logger().info('Figura finalizada con éxito.')
            self.timer.cancel()
            return

        # Publicar el comando
        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = OpenLoopCtrl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
