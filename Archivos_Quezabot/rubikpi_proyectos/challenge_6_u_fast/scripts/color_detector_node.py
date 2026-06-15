#!/home/ubuntu/ros2_ws/src/challenge_6_u_fast/yolo_ros_env/bin/python

import cv2
import rclpy
import numpy as np

from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO


class ColorDetectorNode(Node):

    def __init__(self):
        super().__init__('color_detector_node')

        self.bridge = CvBridge()

        self.color_pub = self.create_publisher(Float32, '/color', 10)

        self.sub = self.create_subscription(
            Image,
            '/camera/raw',
            self.image_callback,
            10
        )

        self.model = YOLO("/home/ubuntu/ros2_ws/src/challenge_6_u_fast/models/best.pt")

        # 0.0 = nada
        # 1.0 = amarillo
        # 2.0 = verde
        # 3.0 = rojo
        self.color_values = {
            "Amarillo": 1.0,
            "Verde": 2.0,
            "Rojo": 3.0
        }

        self.color_ranges = {
            "Amarillo": [
                (np.array([20, 90, 100]), np.array([35, 255, 255]))
            ],
            "Verde": [
                (np.array([45, 100, 90]), np.array([85, 255, 255]))
            ],
            "Rojo": [
                (np.array([0, 110, 90]), np.array([10, 255, 255])),
                (np.array([165, 110, 90]), np.array([180, 255, 255]))
            ]
        }

        self.kernel = np.ones((5, 5), np.uint8)

        # Dentro del bbox el área será menor, por eso no puede ser tan alto
        self.min_area = 80
        self.min_area_ratio = 0.01

        # Redundancia temporal de color
        self.last_candidate = 0.0
        self.same_count = 0
        self.required_count = 3

        self.frame_count = 0

        self.get_logger().info("Color detector YOLO iniciado: color SOLO dentro de bbox traffic_light")

    def detectar_color_roi(self, roi):

        if roi.size == 0:
            return 0.0, "Nada", 0.0, 0.0

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        roi_area = roi.shape[0] * roi.shape[1]

        best_color = "Nada"
        best_value = 0.0
        best_area = 0.0

        for color_name, ranges in self.color_ranges.items():

            mask_total = np.zeros(hsv.shape[:2], dtype=np.uint8)

            for lower, upper in ranges:
                mask = cv2.inRange(hsv, lower, upper)
                mask_total = cv2.bitwise_or(mask_total, mask)

            mask_total = cv2.morphologyEx(mask_total, cv2.MORPH_OPEN, self.kernel)
            mask_total = cv2.morphologyEx(mask_total, cv2.MORPH_CLOSE, self.kernel)

            contours, _ = cv2.findContours(
                mask_total,
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE
            )

            color_area = 0.0

            for cnt in contours:
                area = cv2.contourArea(cnt)

                if area >= self.min_area:
                    color_area += area

            if color_area > best_area:
                best_area = color_area
                best_color = color_name
                best_value = self.color_values[color_name]

        ratio = best_area / roi_area if roi_area > 0 else 0.0

        if best_area < self.min_area or ratio < self.min_area_ratio:
            return 0.0, "Nada", best_area, ratio

        return best_value, best_color, best_area, ratio

    def image_callback(self, msg):

        self.frame_count += 1

        # Para no saturar Rubik
        if self.frame_count % 6 != 0:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        results = self.model(
            frame,
            imgsz=416,
            conf=0.55,
            verbose=False
        )[0]

        best_bbox = None
        best_conf = 0.0
        best_area = 0.0

        for box in results.boxes:
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])
            name = self.model.names[cls_id]

            if name != "traffic_light":
                continue

            if conf < 0.70:
                continue

            x1, y1, x2, y2 = map(int, box.xyxy[0])
            area = (x2 - x1) * (y2 - y1)

            if area < 1500:
                continue

            if conf > best_conf:
                best_conf = conf
                best_bbox = (x1, y1, x2, y2)
                best_area = area

        if best_bbox is None:
            candidate = 0.0
            best_color = "Nada"
            color_area = 0.0
            ratio = 0.0

        else:
            x1, y1, x2, y2 = best_bbox
            roi = frame[y1:y2, x1:x2]
            candidate, best_color, color_area, ratio = self.detectar_color_roi(roi)

        # Redundancia temporal: necesita ver el mismo color 3 veces
        if candidate == self.last_candidate:
            self.same_count += 1
        else:
            self.same_count = 1
            self.last_candidate = candidate

        if self.same_count >= self.required_count:
            detected_value = candidate
        else:
            detected_value = 0.0

        msg_color = Float32()
        msg_color.data = float(detected_value)
        self.color_pub.publish(msg_color)

        self.get_logger().info(
            f"traffic_light_conf={best_conf:.2f} | "
            f"bbox_area={best_area:.0f} | "
            f"color={best_color} | "
            f"color_area={color_area:.0f} | "
            f"ratio={ratio:.3f} | "
            f"count={self.same_count}/{self.required_count} | "
            f"publicado={detected_value}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = ColorDetectorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
