#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry as OdomMsg
from sensor_msgs.msg import Imu
import math


class Odometry_IMU(Node):
    def __init__(self):
        super().__init__("node_imu_odometry")
        
        self.sum = 0.0

        self.bias = 0.0

        self.contador = 0

        self.sub_imu = self.create_subscription(Imu, "/r1/imu", self.imu_callback, 10)


    def timer_callback(self):

        #Integrar velocidad angular
        self.theta = self.wrap_to_pi(self.theta + self.w * self.period)

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

        #Con esta linea podemos ver la odometría en terminal
        self.get_logger().info(f"theta:{math.degrees(self.theta):.1f}°")


    def imu_callback(self, msg):
        self.sum += msg.angular_velocity.z
        self.contador += 1

        if(self.contador == 499):
            self.bias = self.sum / 500
            self.get_logger().info(f"bias:{self.bias:.3f}")




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
