#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import   Twist


class moveTurtleBot(Node):
    def __init__(self):
        super().__init__("track_point")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.timer = self.create_timer(0.1,self.publish_hello_world)
        self.i=0

    def publish_hello_world(self):

        msg= Twist()
        msg.linear.x = 0.5
        msg.angular.z = 0.0

        self.pub.publish(msg)
        self.i += 1

def main(args=None):
    rclpy.init()
    my_pub = moveTurtleBot()
    print("Publisher Node Running")

    try:
        rclpy.spin(my_pub)
    except KeyboardInterrupt:
        my_pub.destroy_node()


if __name__=='__main__':
    main()
