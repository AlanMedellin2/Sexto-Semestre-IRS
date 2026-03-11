#!/usr/bin/env python   
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32
from rcl_interfaces.msg import SetParametersResult

class SetPointPublisher(Node):
    def __init__(self):
        super().__init__('set_point_node')

        self.declare_parameter('signal_type', 'sine')
        self.signal_type = self.get_parameter('signal_type').get_parameter_value().string_value

        self.amplitude = 2.0
        self.omega = 1.0
        self.timer_period = 0.2

        valid_types = ['sine', 'square', 'triangle']
        if self.signal_type not in valid_types:
            self.signal_type = 'sine'

        self.signal_publisher = self.create_publisher(Float32, 'set_point', 10)
        self.timer = self.create_timer(self.timer_period, self.timer_cb)

        self.signal_msg = Float32()
        self.start_time = self.get_clock().now()

        self.add_on_set_parameters_callback(self.parameters_callback)
        self.get_logger().info(f"SetPoint Node Started | Signal type: '{self.signal_type}'")

    def parameters_callback(self, params):
        valid_types = ['sine', 'square', 'triangle']
        for param in params:
            if param.name == 'signal_type':
                if param.value not in valid_types:
                    self.get_logger().warn(f"Tipo de señal inválido: '{param.value}'")
                    return SetParametersResult(
                        successful=False,
                        reason=f"signal_type debe ser uno de: {valid_types}"
                    )
                self.signal_type = param.value
                self.get_logger().info(f"Signal type cambiado a: '{self.signal_type}'")

        return SetParametersResult(successful=True)

    def generate_signal(self, t):
        phase = self.omega * t

        if self.signal_type == 'sine':
            return self.amplitude * np.sin(phase)

        elif self.signal_type == 'square':
            return self.amplitude * np.sign(np.sin(phase))

        elif self.signal_type == 'triangle':
            period = 2 * np.pi / self.omega
            t_mod = t % period
            if t_mod < period / 2:
                return self.amplitude * (4 * t_mod / period - 1)
            else:
                return self.amplitude * (3 - 4 * t_mod / period)

    def timer_cb(self):
        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        self.signal_msg.data = self.generate_signal(elapsed_time)
        self.signal_publisher.publish(self.signal_msg)

def main(args=None):
    rclpy.init(args=args)
    set_point = SetPointPublisher()

    try:
        rclpy.spin(set_point)
    except KeyboardInterrupt:
        pass
    finally:
        set_point.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
