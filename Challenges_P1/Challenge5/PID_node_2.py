#!/usr/bin/env python 
import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32 
from rcl_interfaces.msg import SetParametersResult      

class ControleNode(Node):
    def __init__(self):
        super().__init__('control_node')
        self.declare_parameter('kp', 0.4)
        self.kp = self.get_parameter('kp').value
        self.declare_parameter('ki', 1.4)
        self.ki = self.get_parameter('ki').value
        self.declare_parameter('kd', 0.0)
        self.kd = self.get_parameter('kd').value
        self.dt = 0.01
        self.y = 0.0
        self.setp = 0.0          
        self.anterior_error = 0.0
        self.sumatoria_error = 0.0

        self.u_max = 5.4       
        self.windup_limit = 5.4

        self.publisher = self.create_publisher(Float32, '/motor_duty', 10)

        self.y_sub = self.create_subscription(
            Float32, '/motor_speed', self.y_callback, 10
        )


        self.setp_sub = self.create_subscription(
            Float32, 'set_point', self.setp_callback, 10
        )

        self.timer = self.create_timer(self.dt, self.timer_cb)
        self.add_on_set_parameters_callback(self.parameters_callback)
        self.get_logger().info("Control Node Started - Modo Hardware Real")

    # ✅ NUEVO callback
    def setp_callback(self, msg):
        self.setp = msg.data

    def apply_deadzone(self, duty, deadzone=0.3):
        if abs(duty) < 0.01:
            return 0.0
        elif duty > 0:
            return 0.3 + (duty * (0.9 - 0.3) / 0.9)
        else:
            return -0.3 + (duty * (0.9 - 0.3) / 0.9)
        
    def timer_cb(self):
        error = self.setp - self.y
        self.sumatoria_error += error * self.dt
        self.sumatoria_error = np.clip(self.sumatoria_error, -self.windup_limit, self.windup_limit)

        p_term = self.kp * error
        i_term = self.ki * self.sumatoria_error
        d_term = self.kd * (error - self.anterior_error) / self.dt

        u = p_term + i_term + d_term
        u = np.clip(u, -self.u_max, self.u_max)
        self.anterior_error = error

        duty = u / 6.0
        duty = float(np.clip(duty, -0.9, 0.9))
        duty = self.apply_deadzone(duty)

        msg = Float32()
        msg.data = float(duty)
        self.publisher.publish(msg)

        self.get_logger().debug(f"SP: {self.setp:.2f} | Y: {self.y:.2f} | U: {u:.2f}")

    def y_callback(self, msg):
        self.y = msg.data

    def parameters_callback(self, params):
        for param in params:
            if param.name == "kp":
                if param.value < 0.0:
                    return SetParametersResult(successful=False, reason="kp no puede ser negativo")
                self.kp = param.value
            if param.name == "ki":
                if param.value < 0.0:
                    return SetParametersResult(successful=False, reason="ki no puede ser negativo")
                self.ki = param.value
                self.sumatoria_error = 0.0
            if param.name == "kd":
                if param.value < 0.0:
                    return SetParametersResult(successful=False, reason="kd no puede ser negativo")
                self.kd = param.value
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
