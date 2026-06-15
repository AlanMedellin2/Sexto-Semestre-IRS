import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class Grabber(Node):
    def __init__(self):
        super().__init__('grabber')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, '/camera/image', self.cb, 1)
    def cb(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg)
        cv2.imwrite('/home/ubuntu/frame.png', img)
        print('Guardado!')
        rclpy.shutdown()

rclpy.init()
rclpy.spin(Grabber())
