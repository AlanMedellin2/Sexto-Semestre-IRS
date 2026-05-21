#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry as OdomMsg
from sensor_msgs.msg import Imu
import math
from collections import deque


class Odometry_IMU(Node):
    def __init__(self):
        super().__init__("node_imu_odometry")

        self.last_time = None
        
        self.theta = 0.0

        self.w = 0.0

        self.w_window = deque(maxlen=5)

        self.bias = 0.007 #calculado manualmente en bias_imu.py

        self.sub_imu = self.create_subscription(Imu, "/r1/imu", self.imu_callback, 10)

        self.odom_pub = self.create_publisher(OdomMsg, "/imu_odometry", 10)


    def wrap_to_pi(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi


    def imu_callback(self, msg):

        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self.last_time is None:
            self.last_time = current_time
            return
        
        dt = current_time - self.last_time
        self.last_time = current_time

        self.w = msg.angular_velocity.z - self.bias

        self.w_window.append(self.w)
        w_filtered = sum(self.w_window) / len(self.w_window)

        #self.get_logger().info(f"vel:{self.w:.3f}")

        if abs(w_filtered) < 0.03: w_filtered = 0.0

        #Integrar velocidad angular
        self.theta = self.wrap_to_pi(self.theta + w_filtered * dt)

        # publicar odometría
        odom = OdomMsg()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = 0.0
        odom.pose.pose.position.y = 0.0
        odom.pose.pose.position.z = 0.0

        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)

        self.odom_pub.publish(odom)

        self.get_logger().info(f"theta:{math.degrees(self.theta):.1f}°")


def main(args=None):
    #Inicializar comunicaciones ROS RDS
    rclpy.init(args=args)
    my_node = Odometry_IMU()

    try:
        rclpy.spin(my_node)
    except KeyboardInterrupt:
        pass
    finally:
        my_node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    main()
