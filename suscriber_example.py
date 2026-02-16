#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class HellowSuscriber(Node):
    def __init__(self):
        super().__init__("hello_world_sub_node")
        self.sub = self.create_subscription(String, "hello_world", self.suscriber_callback ,10)

    def suscriber_callback(self,msg):
        print("Recieved: " + msg.data)

def main(args=None):
    #Inicializar comunicaciones ROS RDS
    rclpy.init()
    my_sub = HellowSuscriber()
    print("Waiting for data")

    try:
        rclpy.spin(my_sub)
    except KeyboardInterrupt:
        my_sub.destroy_node()


if __name__=='__main__':
    main()
