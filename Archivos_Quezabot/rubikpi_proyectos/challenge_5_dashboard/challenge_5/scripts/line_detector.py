#!/usr/bin/env python3
from collections import deque
import cv2 as cv
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class LineDetector(Node):

    def __init__(self):
        super().__init__('line_detector')

        self.publisher_ = self.create_publisher(Int32, '/line_error', 10)
        self.img_pub = self.create_publisher(Image, '/camera/image', 10)
        self.bridge = CvBridge()

        self.cap = cv.VideoCapture('/dev/video0', cv.CAP_V4L2)
        self.Area_min = 200
        self.Area_max = 50000

        if not self.cap.isOpened():
            self.get_logger().error("Cannot open camera")
            exit()

        self.H_history = deque(maxlen=10)

        self.timer = self.create_timer(0.03, self.process_frame)

    def process_frame(self):
        ret, originalFr = self.cap.read()
        if not ret:
            self.get_logger().warning("Cannot receive frame")
            return

        h, w = originalFr.shape[:2]
        roi = originalFr[int(h*0.6):h, :]
        roi_h, roi_w = roi.shape[:2]


        ############################################################
        # HSV

        H_roi = roi[int(roi_h*0.60):roi_h, int(roi_w*0.30):int(roi_w*0.70)]
        hsv_roi = cv.cvtColor(H_roi, cv.COLOR_BGR2HSV)    
        H_channel = hsv_roi[:, :, 2] #Toma solo H
        H_mean = np.mean(H_channel)
        self.H_history.append(H_mean)
        H_mean_smooth = np.mean(self.H_history)
        print(H_mean_smooth)

   
        ############################################################


        gris_image = cv.cvtColor(roi, cv.COLOR_BGR2GRAY)
        blurred = cv.GaussianBlur(gris_image, (5, 5), 0)

        if H_mean_smooth < 110.00:
            cutting = 100
        elif H_mean_smooth >= 110.00 and H_mean_smooth < 135.0:
            cutting = 140
        elif H_mean_smooth >= 135.00 and H_mean_smooth < 150.0:
            cutting = 140
        elif H_mean_smooth >= 150.00 and H_mean_smooth < 155.0:
            cutting = 115
        elif H_mean_smooth >= 155.00 and H_mean_smooth < 165.0:
            cutting = 120
        elif H_mean_smooth >= 165.00 and H_mean_smooth < 170.0:
            cutting = 125
        elif H_mean_smooth >= 170.00 and H_mean_smooth < 190.0:
            cutting = 130
        elif H_mean_smooth >= 190.00 and H_mean_smooth < 200.0:
            cutting = 140
        elif H_mean_smooth >= 200.00:
            cutting = 160

        _, binary = cv.threshold(blurred, cutting, 255, cv.THRESH_BINARY_INV) #Estaba en 80

        top_width = int(roi_w * 0.6)
        top_y = int(roi_h * 0.3)
        trapezoid = np.array([[
            ((roi_w - top_width) // 2, top_y),
            ((roi_w + top_width) // 2, top_y),
            (roi_w, roi_h),
            (0, roi_h)
        ]], dtype=np.int32)

        mask = np.zeros((roi_h, roi_w), dtype=np.uint8)
        cv.fillPoly(mask, trapezoid, 255)
        binary_masked = cv.bitwise_and(binary, binary, mask=mask)

        kernel = np.ones((3, 3), np.uint8)
        morph = cv.erode(binary_masked, kernel, iterations=1)
        morph = cv.dilate(morph, kernel, iterations=1)

        num_labels, labels, stats, centroids = cv.connectedComponentsWithStats(
            morph, connectivity=8
        )

        ref_x = roi_w // 2
        candidatos = []
        for i in range(1, num_labels):
            x, y, bw, bh, area = stats[i]
            cx, cy = centroids[i]
            if self.Area_min <= area <= self.Area_max:
                candidatos.append((cx, cy))

        if len(candidatos) > 0:
            cx, cy = min(candidatos, key=lambda p: abs(p[0] - ref_x))
            error_x = int(cx - ref_x)
            msg = Int32()
            msg.data = error_x
            self.publisher_.publish(msg)
            self.get_logger().info(f'Error publicado: {error_x}')

        self.img_pub.publish(self.bridge.cv2_to_imgmsg(morph, encoding='mono8'))


def main(args=None):
    rclpy.init(args=args)
    node = LineDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
