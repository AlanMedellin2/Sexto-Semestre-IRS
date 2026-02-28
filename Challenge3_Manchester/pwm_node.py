# Imports
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32

#Class Definition
class PWMPublisher(Node):
    def __init__(self):
        super().__init__('pwm_node')



        self.get_logger().info("SetPoint Node Started \U0001F680")

   


#Main
def main(args=None):
    rclpy.init(args=args)

    pwm_node = PWMPublisher()

    try:
        rclpy.spin(pwm_node)
    except KeyboardInterrupt:
        pass
    finally:
        pwm_node.destroy_node()
        rclpy.try_shutdown()

#Execute Node
if __name__ == '__main__':
    main()
