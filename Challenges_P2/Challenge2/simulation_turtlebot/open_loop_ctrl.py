import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np



class OpenLoopCtrl(Node):
    def __init__(self):
        super().__init__('open_loop_ctrl')
        
        self.wait_for_ros_time()
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)    #pub

        # Variables cuadrado
        self.side_length = 2.0  # metros
        self.linear_speed = 0.2   # m/s
        self.angular_speed = 0.5     # rad/s

        # Tiempos de rampa
        self.ramp_time_linear = 0.8   # s
        self.ramp_time_angular = 0.6 

        # t
        self.straight_time = self.side_length / self.linear_speed
        self.turn_time = (np.pi / 2.0) / self.angular_speed
        
        
        # STATE Machine
        self.state = 'STRAIGHT'
        self.side_count = 0
        self.state_start_time = self.get_clock().now()

        self.timer_period = 0.05
        self.timer = self.create_timer(self.timer_period, self.control_loop)

        self.get_logger().info('Controller Square init')
        self.get_logger().info(f'Straight t: {self.straight_time:.2f} s')
        self.get_logger().info(f'Turn t: {self.turn_time:.2f} s')

    def wait_for_ros_time(self):
        self.get_logger().info('Waiting ROS t...')
        while rclpy.ok():
            now = self.get_clock().now()
            if now.nanoseconds > 0:
                break
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info('ROS t ACTIVE.')



    def ramp_speed(self, target_speed, elapsed_time, ramp_time):
        if ramp_time <= 0.0:
            return target_speed

        if elapsed_time >= ramp_time:
            return target_speed

        return target_speed * (elapsed_time / ramp_time)




    def control_loop(self):
        now = self.get_clock().now()
        elapsed = (now - self.state_start_time).nanoseconds * 1e-9

        cmd = Twist()

        if self.state == 'STRAIGHT':
            cmd.linear.x = self.ramp_speed(
                self.linear_speed,
                elapsed,
                self.ramp_time_linear
            )
            cmd.angular.z = 0.0


            if elapsed >= self.straight_time:
                self.state = 'TURN'
                self.state_start_time = now
                self.get_logger().info(f'Finished side {self.side_count + 1}, starting turn.')

        elif self.state == 'TURN':
            cmd.linear.x = 0.0
            cmd.angular.z = self.ramp_speed(
                self.angular_speed,
                elapsed,
                self.ramp_time_angular
            )

            if elapsed >= self.turn_time:
                self.side_count += 1
                self.state_start_time = now

                if self.side_count >= 4:
                    self.state = 'STOP'
                    self.get_logger().info('Square READY.')
                else:
                    self.state = 'STRAIGHT'
                    self.get_logger().info(f'Turn completed. Side {self.side_count + 1}.')


        elif self.state == 'STOP':
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd)
            self.timer.cancel()
            return

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
