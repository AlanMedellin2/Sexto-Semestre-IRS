#!/usr/bin/env python3

import cv2
import rclpy
import numpy as np

from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


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

        # 0.0 = nada
        # 1.0 = amarillo
        # 2.0 = verde
        # 3.0 = rojo
        self.color_values = {
            "Amarillo": 1.0,
            "Verde": 2.0,
            "Rojo": 3.0
        }

        # Rangos HSV más estrictos para evitar falsos verdes
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

        # Evita detectar ruido pequeño como bandera
        self.min_area = 1500

        # También exige que el color ocupe cierto porcentaje del ROI
        self.min_area_ratio = 0.015

        # Filtro temporal: necesita ver el mismo color varias veces
        self.last_candidate = 0.0
        self.same_count = 0
        self.required_count = 5

        self.get_logger().info("Color detector iniciado en /camera/raw")

    def image_callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        h, w = frame.shape[:2]

        # ROI superior-central para evitar piso, línea negra y reflejos bajos
        roi = frame[0:int(h * 0.60), int(w * 0.20):int(w * 0.80)]

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

        # Validación fuerte para evitar colores falsos
        if best_area < self.min_area or (best_area / roi_area) < self.min_area_ratio:
            candidate = 0.0
            best_color = "Nada"
        else:
            candidate = best_value

        # Estabilidad temporal
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
            f"Color candidato: {best_color} | "
            f"area: {best_area:.0f} | "
            f"ratio: {(best_area / roi_area):.4f} | "
            f"publicado: {detected_value}"
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
