#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry as OdomMsg
import math
from rclpy.qos import ReliabilityPolicy, QoSProfile

class Odometry(Node):
    def __init__(self):
        super().__init__("node_odometry")
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.wr = 0.0 #última velocidad D
        self.wl = 0.0 #última velocidad recivida Iz
        #velocidad angular --> velocidad tangencial 
        self._r = 0.05 #radio de rueda (revisar, estos datos los saque de la presentación de Manchester)
        #diferencia de velocidades --> giro del robot
        self._l = 0.18 #distancia entre ejes (revisar)

        self.period = 0.05

        self.sub_r = self.create_subscription(
            Float32, "/VelocityEncR", self.right_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        self.sub_l = self.create_subscription(
            Float32, "/VelocityEncL", self.left_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )

        self.odom_pub = self.create_publisher(OdomMsg, "/encoder_odometry", 10)
        self.timer_=self.create_timer(self.period, self.timer_callback)

    def wrap_to_pi(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi


    def timer_callback(self):
        #velocidad tangencial
        v_r = self._r * self.wr
        v_l = self._r * self.wl

        #velocidad lineal
        V = (v_r + v_l)/2.0

        #velocidad angular
        w = (v_r - v_l)/self._l

        #modelo cinemático
        self.x = self.x + V * math.cos(self.theta)* self.period
        self.y = self.y + V * math.sin(self.theta)* self.period
        self.theta = self.wrap_to_pi(self.theta + w * self.period)

        # publicar odometría
        odom = OdomMsg()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)

        odom.twist.twist.linear.x = V
        odom.twist.twist.angular.z = w
        self.odom_pub.publish(odom)

        #Con esta linea podemos ver la odometría en terminal
        self.get_logger().info(f"x:{self.x:.2f} y:{self.y:.2f} theta:{math.degrees(self.theta):.1f}°")


    def right_callback(self,msg):
        self.wr = msg.data
    
    def left_callback(self,msg):
        self.wl = msg.data


def main(args=None):
    #Inicializar comunicaciones ROS RDS
    rclpy.init(args=args)
    my_node = Odometry()

    try:
        rclpy.spin(my_node)
    except KeyboardInterrupt:
        pass
    finally:
        my_node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    main()
