#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class RotateRobot(Node):
    def __init__(self):
        super().__init__('rotate_precise_node')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry, '/encoder_odometry', self.odom_callback, 10)
        
        # Configuración del giro (360 grados en radianes)
        self.target_rotation = 2 * math.pi  
        self.total_rotated = 0.0
        
        self.last_yaw = None
        self.kp = 0.8  # Ganancia proporcional
        self.min_speed = 0.05  # Velocidad mínima para romper la inercia
        self.max_speed = 0.5   # Velocidad máxima permitida

    def get_yaw_from_quaternion(self, q):
        # Convierte cuaternión de odometría a ángulo Euler (Yaw)
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def odom_callback(self, msg):
        orientation = msg.pose.pose.orientation
        current_yaw = self.get_yaw_from_quaternion(orientation)
        
        # Inicialización en la primera lectura
        if self.last_yaw is None:
            self.last_yaw = current_yaw
            self.get_logger().info('Iniciando giro de 360 grados...')
            return

        # Calcular la diferencia de ángulo desde el último callback
        delta_yaw = current_yaw - self.last_yaw
        
        # Corregir la discontinuidad de atan2 (cuando pasa de PI a -PI o viceversa)
        if delta_yaw > math.pi:
            delta_yaw -= 2 * math.pi
        elif delta_yaw < -math.pi:
            delta_yaw += 2 * math.pi

        # Acumular el giro total (usamos valor absoluto por si gira al revés)
        self.total_rotated += abs(delta_yaw)
        self.last_yaw = current_yaw

        # Calcular el error restante para llegar a los 360°
        error = self.target_rotation - self.total_rotated

        twist = Twist()

        # Tolerancia de error (aprox. 0.5 grados)
        if error > 0.01:
            # Control Proporcional
            speed = self.kp * error
            
            # Acotar velocidad entre el máximo y el mínimo útil
            twist.angular.z = max(self.min_speed, min(speed, self.max_speed))
        else:
            # Detener el robot por completo
            twist.angular.z = 0.0
            self.publisher.publish(twist)
            self.get_logger().info(f'¡Giro de 360° completado con éxito! Rotado: {math.degrees(self.total_rotated):.2f}°')
            
            # Destruir nodo limpiamente
            self.destroy_node()
            rclpy.shutdown()
            exit()

        self.publisher.publish(twist)

def main():
    rclpy.init()
    node = RotateRobot()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass  # Maneja la salida limpia del exit()

if __name__ == '__main__':
    main()
