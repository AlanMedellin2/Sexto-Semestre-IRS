#!/usr/bin/env python3
import cv2 as cv
import rclpy

from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class CameraRawPublisher(Node):

    def __init__(self):
        super().__init__('camera_raw_publisher')

        # Único publisher requerido
        self.raw_pub = self.create_publisher(Image, '/camera/raw', 10)

        self.bridge = CvBridge()

        # Configuración de la cámara
        self.cap = cv.VideoCapture('/dev/video0', cv.CAP_V4L2)
        
        self.cap.set(cv.CAP_PROP_FRAME_WIDTH,  320)
        self.cap.set(cv.CAP_PROP_FRAME_HEIGHT, 240)
        self.cap.set(cv.CAP_PROP_FPS,          30)
        self.cap.set(cv.CAP_PROP_BUFFERSIZE,   1)

        if not self.cap.isOpened():
            self.get_logger().error("Cannot open camera /dev/video0")
            exit()

        # Timer para capturar y publicar a ~30 FPS
        self.timer = self.create_timer(0.033, self.process_frame)
        self.get_logger().info("Nodo de cámara (/camera/raw) iniciado")

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning("Cannot receive frame")
            return

        # Publicar directamente la resolución cruda de la cámara
        self.raw_pub.publish(self.bridge.cv2_to_imgmsg(frame, encoding='bgr8'))

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraRawPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
