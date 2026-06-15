#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time

class RotateRobotPI(Node):
    def __init__(self):
        super().__init__('rotate_pi_node')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry, '/encoder_odometry', self.odom_callback, 10)
        
        # Parámetros de consigna
        self.goal_angle = 1.5708  # 90 grados
        self.target_yaw = 0.0
        self.initial_yaw = None
        
        # --- PARÁMETROS DEL CONTROL PI ---
        self.kp = 1.1               # Ganancia Proporcional (reacción inmediata)
        self.ki = 0.05              # Ganancia Integral (vence la fricción acumulada)
        self.integral_error = 0.0   # Acumulador del error
        self.max_integral = 0.5     # Anti-windup (limita la acumulación para evitar oscilaciones)
        
        self.min_vel = 0.22         # Velocidad mínima de seguridad
        self.max_vel = 0.8          # Velocidad máxima
        self.tolerance = 0.01       # Precisión (aprox 0.5 grados)

    def get_yaw_from_quaternion(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def odom_callback(self, msg):
        orientation = msg.pose.pose.orientation
        current_yaw = self.get_yaw_from_quaternion(orientation)
        
        if self.initial_yaw is None:
            self.initial_yaw = current_yaw
            self.target_yaw = self.initial_yaw + self.goal_angle
            self.get_logger().info(f'Giro PI iniciado. Objetivo: {self.target_yaw:.2f} rad')

        # 1. Calcular Error
        error = self.target_yaw - current_yaw
        # Normalizar error entre -PI y PI
        error = (error + math.pi) % (2 * math.pi) - math.pi

        twist = Twist()

        if abs(error) > self.tolerance:
            # 2. Término Proporcional
            p_term = self.kp * error
            
            # 3. Término Integral (solo si no hemos llegado)
            self.integral_error += error
            # Anti-windup: Evita que la integral crezca infinito si el robot está bloqueado físicamente
            self.integral_error = max(min(self.integral_error, self.max_integral), -self.max_integral)
            i_term = self.ki * self.integral_error
            
            # 4. Suma de Control
            v_angular = p_term + i_term
            
            # Aplicar límites y dirección
            sign = 1.0 if v_angular > 0 else -1.0
            abs_v = abs(v_angular)
            
            if abs_v < self.min_vel:
                twist.angular.z = sign * self.min_vel
            elif abs_v > self.max_vel:
                twist.angular.z = sign * self.max_vel
            else:
                twist.angular.z = v_angular
        else:
            # Frenado final
            twist.angular.z = 0.0
            self.publisher.publish(twist)
            self.get_logger().info('¡Giro PI completado!')
            time.sleep(0.5)
            self.destroy_node()
            rclpy.shutdown()
            exit()

        self.publisher.publish(twist)

def main():
    rclpy.init()
    node = RotateRobotPI()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass

if __name__ == '__main__':
    main()
