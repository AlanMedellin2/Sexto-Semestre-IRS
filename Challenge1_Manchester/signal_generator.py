#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import math

class SenoPublisher(Node):

    def __init__(self):
        super().__init__("seno_data_pub_node")

        self.pub = self.create_publisher(Float32, "seno_data", 10)
        self.timer = self.create_timer(0.01, self.publish_seno_data)
        self.tiempo = 0.0   
        self.aumento = 0.05 
        self.get_logger().info("Seno continuo corriendo ")

    def publish_seno_data(self):
        msg = Float32()
        # Seno continuo
        msg.data = math.sin(self.tiempo)
        self.pub.publish(msg)
        # Incrementar tiempo
        self.tiempo += self.aumento

def main(args=None):
    rclpy.init()
    node = SenoPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

