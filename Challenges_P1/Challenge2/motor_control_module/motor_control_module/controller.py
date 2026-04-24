#!/usr/bin/env python 

import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32 
from rcl_interfaces.msg import SetParametersResult
from custom_interfaces.msg import Init       

class ControleNode(Node):
    def __init__(self):
        super().__init__('control_node')

        self.declare_parameter('kp', 1.0)
        self.kp = self.get_parameter('kp').value

        self.declare_parameter('ki', 1.0)
        self.ki = self.get_parameter('ki').value

        self.declare_parameter('kd', 1.0)
        self.kd = self.get_parameter('kd').value

        self.dt = 0.02
        self.setp = 0.0
        self.y = 0.0
        self.anterior_error = 0.0
        self.sumatoria_error = 0.0
        self.active = False

        # Límites de saturación
        self.u_max = 10.0
        self.windup_limit = 50.0

        self.publisher = self.create_publisher(Float32, 'motor_input_u', 10)

        self.setp_sub = self.create_subscription(Float32, 'set_point', self.setp_callback, 10)
        self.y_sub = self.create_subscription(Float32, 'motor_speed_y', self.y_callback, 10)
        self.init_subscriber = self.create_subscription(Init, 'init_system', self.init_callback, 10)

        self.timer = self.create_timer(self.dt, self.timer_cb)

        self.add_on_set_parameters_callback(self.parameters_callback)

        self.get_logger().info("Control Node Started")

    def init_callback(self, msg):
        if msg.info.data == 'resume':
            self.active = True
        else:
            self.active = False
            # Resetear integrador al desactivar
            self.sumatoria_error = 0.0
            self.anterior_error = 0.0

    def timer_cb(self):
        if not self.active:
            return

        error = self.setp - self.y

        # Integración correcta con dt
        self.sumatoria_error += error * self.dt
        
        self.sumatoria_error = np.clip(self.sumatoria_error, -self.windup_limit, self.windup_limit)

        p_term = self.kp * error
        i_term = self.ki * self.sumatoria_error
        d_term = self.kd * (error - self.anterior_error) / self.dt

        u = p_term + i_term + d_term

        # Saturar salida
        u = np.clip(u, -self.u_max, self.u_max)

        self.anterior_error = error

        msg = Float32()
        msg.data = float(u)
        self.publisher.publish(msg)

    def setp_callback(self, msg):
        self.setp = msg.data

    def y_callback(self, msg):
        self.y = msg.data

    def parameters_callback(self, params):
        for param in params:
            if param.name == "kp":
                if param.value < 0.0:
                    self.get_logger().warn("Valor kp incorrecto.")
                    return SetParametersResult(successful=False, reason="kp no puede ser negativo")
                self.kp = param.value
                self.get_logger().info(f"kp ahora es {self.kp}")

            if param.name == "ki":
                if param.value < 0.0:
                    self.get_logger().warn("Valor ki incorrecto.")
                    return SetParametersResult(successful=False, reason="ki no puede ser negativo")
                self.ki = param.value
                self.sumatoria_error = 0.0
                self.get_logger().info(f"ki ahora es {self.ki}")

            if param.name == "kd":
                if param.value < 0.0:
                    self.get_logger().warn("Valor kd incorrecto.")
                    return SetParametersResult(successful=False, reason="kd no puede ser negativo")
                self.kd = param.value
                self.get_logger().info(f"kd ahora es {self.kd}")

        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    controle_node = ControleNode()

    try:
        rclpy.spin(controle_node)
    except KeyboardInterrupt:
        pass
    finally:
        controle_node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
