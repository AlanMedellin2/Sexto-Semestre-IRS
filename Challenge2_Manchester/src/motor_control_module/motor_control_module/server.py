#!/usr/bin/env python 

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import Float32
from custom_interfaces.srv import Initiate
from custom_interfaces.msg import Init


class RobotService(Node):
    def __init__(self):
        super().__init__('initaite_motor_service') #Nombre del nodo

        #Publisher
        self.service = self.create_service(Initiate, '/init', self.init_callback)
        self.init_publisher = self.create_publisher(Init, '/init_system',10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.msg = Init()

    def init_callback (self, request, response):
        response.success = True
        my_request = request.command.data
        if(my_request == 'resume'):
            self.msg.info.data = 'resume'
            
        elif(my_request == 'stop'):
            self.msg.info.data = 'stop'
            
        else:
            self.get_logger().info("Incorrect command")
            

        return response
    
    def timer_callback(self):
        self.init_publisher.publish(self.msg)
        



def main (args=None):
    rclpy.init(args=args)
    robot_service_node = RobotService()
    rclpy.spin(robot_service_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
