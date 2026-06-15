#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist


class LineFollowerPID(Node):

    def __init__(self):
        super().__init__('line_follower_pid')

        self.subscription = self.create_subscription(
            Int32,
            '/line_error',
            self.error_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # PID
        self.kp = 0.1
        self.ki = 0.0
        self.kd = 0.1

        self.error = 0.0
        self.prev_error = 0.0
        self.integral = 0.0
        self.dt = 0.05

        # Robot
        self.base_speed = 0.10
        self.deadband = 30
        self.max_linear = 0.10
        self.max_angular = 0.5

        # Rampas
        self.current_linear = 0.0
        self.current_angular = 0.0
        self.linear_accel = 0.02
        self.angular_accel = 0.10

        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info("Line follower PID iniciado")

    def error_callback(self, msg):
        self.error = float(msg.data)

    def saturate(self, value, limit):
        return max(-limit, min(limit, value))

    def ramp(self, target, current, step):
        if target > current:
            current = min(current + step, target)
        elif target < current:
            current = max(current - step, target)
        return current

    def control_loop(self):

        proportional = self.error
        self.integral += self.error * self.dt
        derivative = (self.error - self.prev_error) / self.dt

        angular_pid = (
            self.kp * proportional +
            self.ki * self.integral +
            self.kd * derivative
        )
        self.prev_error = self.error

        angular_pid = self.saturate(angular_pid, self.max_angular)

        if abs(self.error) < self.deadband:
            target_linear = self.base_speed
            target_angular = 0.0
        else:
            target_linear = 0.0 if abs(self.error) > 150 else self.base_speed
            target_angular = -angular_pid

        self.current_linear = self.ramp(
            target_linear, self.current_linear, self.linear_accel
        )
        self.current_angular = self.ramp(
            target_angular, self.current_angular, self.angular_accel
        )

        self.current_linear = self.saturate(self.current_linear, self.max_linear)
        self.current_angular = self.saturate(self.current_angular, self.max_angular)

        cmd = Twist()
        cmd.linear.x = self.current_linear
        cmd.angular.z = self.current_angular
        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f'Error: {self.error:.1f} | '
            f'Lin: {self.current_linear:.2f} | '
            f'Ang: {self.current_angular:.2f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerPID()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
