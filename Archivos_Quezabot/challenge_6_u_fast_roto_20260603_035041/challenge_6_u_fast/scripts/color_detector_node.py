#!/usr/bin/env python3

import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import numpy as np


class ColorDetectorNode(Node):
    def __init__(self):
        super().__init__('color_detector_node')

        self.Ts = 0.1
        self.color_pub = self.create_publisher(Float32, '/color', 10)
        self.timer = self.create_timer(self.Ts, self.main_loop)

        self.get_logger().info("Iniciando deteccion de semaforo...")

        # Cambia el numero si tu camara esta en /dev/video1 o /dev/video2
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

        if not self.cap.isOpened():
            self.get_logger().error("No se pudo abrir la camara")
            return

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        # 0.0 = nada
        # 1.0 = amarillo
        # 2.0 = verde
        # 3.0 = rojo

        self.color_ranges = {
            "Amarillo": [
                (np.array([20, 100, 120]), np.array([35, 255, 255]))
            ],
            "Verde": [
                (np.array([40, 70, 70]), np.array([90, 255, 255]))
            ],
            "Rojo": [
                (np.array([0, 120, 80]), np.array([10, 255, 255])),
                (np.array([160, 120, 80]), np.array([180, 255, 255]))
            ]
        }

        self.draw_colors = {
            "Amarillo": (0, 255, 255),
            "Verde": (0, 255, 0),
            "Rojo": (0, 0, 255)
        }

        self.color_values = {
            "Amarillo": 1.0,
            "Verde": 2.0,
            "Rojo": 3.0
        }

        self.min_area = 800
        self.kernel = np.ones((5, 5), np.uint8)

        # Para que no cambie por ruido de un frame
        self.last_detected = 0.0
        self.same_count = 0
        self.required_count = 2

        self.show_window = False

    def main_loop(self):
        if not self.cap.isOpened():
            msg = Float32()
            msg.data = 0.0
            self.color_pub.publish(msg)
            return

        ret, frame = self.cap.read()

        if not ret:
            self.get_logger().warn("No se pudo leer frame")
            msg = Float32()
            msg.data = 0.0
            self.color_pub.publish(msg)
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        frame_h, frame_w = frame.shape[:2]

        best_color = "Nada"
        best_value = 0.0
        best_area = 0.0
        best_box = None

        for color_name, ranges in self.color_ranges.items():
            mask = np.zeros((frame_h, frame_w), dtype=np.uint8)

            for lower, upper in ranges:
                partial_mask = cv2.inRange(hsv, lower, upper)
                mask = cv2.bitwise_or(mask, partial_mask)

            # Limpiar ruido
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)

            contours, _ = cv2.findContours(
                mask,
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE
            )

            for cnt in contours:
                area = cv2.contourArea(cnt)

                if area < self.min_area:
                    continue

                perimeter = cv2.arcLength(cnt, True)
                if perimeter == 0:
                    continue

                circularity = 4 * np.pi * area / (perimeter * perimeter)

                x, y, w, h = cv2.boundingRect(cnt)

                if h == 0:
                    continue

                aspect_ratio = float(w) / float(h)

                # Filtro para evitar detectar manchas raras
                if 0.45 < circularity < 1.3 and 0.55 < aspect_ratio < 1.6:
                    if area > best_area:
                        best_area = area
                        best_color = color_name
                        best_value = self.color_values[color_name]
                        best_box = (x, y, w, h)

        # Filtro temporal simple para que no salte por ruido
        if best_value == self.last_detected:
            self.same_count += 1
        else:
            self.same_count = 0
            self.last_detected = best_value

        if self.same_count >= self.required_count:
            detected_value = best_value
        else:
            detected_value = 0.0

        msg = Float32()
        msg.data = float(detected_value)
        self.color_pub.publish(msg)

        if best_box is not None:
            x, y, w, h = best_box
            cv2.rectangle(frame, (x, y), (x + w, y + h), self.draw_colors[best_color], 2)
            cv2.putText(
                frame,
                f"{best_color} area:{int(best_area)}",
                (x, y - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                self.draw_colors[best_color],
                2
            )

        cv2.putText(
            frame,
            f"Publicado /color: {detected_value}",
            (20, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2
        )

        if self.show_window:
            cv2.imshow("Deteccion de semaforo", frame)
            key = cv2.waitKey(1)

            if key == 27:
                rclpy.shutdown()

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


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
