# Imports
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32
import time as t

#Class Definition
class PWMPublisher(Node):
    def __init__(self):
        super().__init__('pwm_node')


        ## Publisher
        self.publisher = self.create_publisher(Float32, '/cmd_pwm', 10)



        self.get_logger().info("PWMPoint Node Started \U0001F680")


    def publish_pwm(self):
        msg = Float32()
        msg.data = 0.0
        self.publisher.publish(msg)
        t.sleep(1)
        msg.data = 1.0
        self.publisher.publish(msg)
        t.sleep(3)
        msg.data = 0.0
        self.publisher.publish(msg)
        t.sleep(2)
        msg.data = -1.0
        self.publisher.publish(msg)
        t.sleep(3)
        msg.data = 0.0
        self.publisher.publish(msg)
        t.sleep(1)


   


#Main
def main(args=None):
    rclpy.init(args=args)

    pwm_node = PWMPublisher()
    pwm_node.publish_pwm()
    pwm_node.destroy_node()
    rclpy.try_shutdown()

#Execute Node
if __name__ == '__main__':
    main()
