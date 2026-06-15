#!/usr/bin/env python3

import rclpy
import cv2
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class SnapshotDebug(Node):
    def __init__(self):
        super().__init__('snapshot_debug')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image,
            '/camera/image',
            self.cb,
            10
        )

    def cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imwrite('/home/ubuntu/line_debug.jpg', frame)
        self.get_logger().info('Snapshot guardado en /home/ubuntu/line_debug.jpg')


def main():
    rclpy.init()
    node = SnapshotDebug()
    rclpy.spin_once(node, timeout_sec=2.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
