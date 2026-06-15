#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import String

from cv_bridge import CvBridge

import cv2
from ultralytics import YOLO


class YoloDoubleDetector(Node):

    def __init__(self):
        super().__init__('yolo_double_detector')

        self.bridge = CvBridge()

        # =========================
        # MODELOS
        # =========================

        self.modelo_senales = YOLO(
            '/home/ubuntu/ros2_ws/src/challenge_6_u_fast/models/best.pt'
        )

        self.modelo_obstaculos = YOLO(
            '/home/ubuntu/ros2_ws/src/challenge_6_u_fast/models/yolo11n.pt'
        )

        # =========================
        # SUSCRIPCION CAMARA
        # =========================

        self.subscription = self.create_subscription(
            Image,
            '/video_source/raw',
            self.image_callback,
            10
        )

        # =========================
        # PUBLICADORES
        # =========================

        self.pub_priority = self.create_publisher(
            String,
            '/yolo/priority',
            10
        )

        # =========================
        # MEMORIA ANTIFLICKER
        # =========================

        self.ultima_prioridad = "none"
        self.frames_memoria = 8
        self.contador_memoria = 0

        self.get_logger().info("YOLO DOUBLE DETECTOR INICIADO")

    def image_callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        frame_out = frame.copy()

        # =========================
        # YOLO SEÑALES
        # =========================

        resultados_senales = self.modelo_senales(
            frame,
            imgsz=416,
            conf=0.45,
            verbose=False
        )[0]

        # =========================
        # YOLO OBSTACULOS
        # =========================

        resultados_obstaculos = self.modelo_obstaculos(
            frame,
            imgsz=320,
            conf=0.45,
            verbose=False
        )[0]

        mejor_score = 0
        mejor_texto = "none"

        # =========================
        # SEÑALES
        # =========================

        for box in resultados_senales.boxes:

            cls_id = int(box.cls[0])
            conf = float(box.conf[0])

            nombre = self.modelo_senales.names[cls_id]

            x1, y1, x2, y2 = map(int, box.xyxy[0])

            area = (x2 - x1) * (y2 - y1)

            prioridad = 10000

            score = area * conf * prioridad

            if score > mejor_score:
                mejor_score = score
                mejor_texto = f"senal:{nombre}"

            cv2.rectangle(
                frame_out,
                (x1, y1),
                (x2, y2),
                (255, 0, 0),
                2
            )

            cv2.putText(
                frame_out,
                f"{nombre} {conf:.2f}",
                (x1, y1 - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 0, 0),
                2
            )

        # =========================
        # OBSTACULOS
        # =========================

        clases_obstaculos = {
            "person": 1,
            "car": 1,
            "truck": 1,
            "bus": 1,
            "bicycle": 1,
            "motorcycle": 1
        }

        for box in resultados_obstaculos.boxes:

            cls_id = int(box.cls[0])

            nombre = self.modelo_obstaculos.names[cls_id]

            if nombre not in clases_obstaculos:
                continue

            conf = float(box.conf[0])

            x1, y1, x2, y2 = map(int, box.xyxy[0])

            area = (x2 - x1) * (y2 - y1)

            prioridad = clases_obstaculos[nombre]

            score = area * conf * prioridad

            if score > mejor_score:
                mejor_score = score
                mejor_texto = f"obstaculo:{nombre}"

            cv2.rectangle(
                frame_out,
                (x1, y1),
                (x2, y2),
                (0, 0, 255),
                2
            )

            cv2.putText(
                frame_out,
                f"{nombre} {conf:.2f}",
                (x1, y1 - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2
            )

        # =========================
        # MEMORIA ANTIFLICKER
        # =========================

        if mejor_texto != "none":
            self.ultima_prioridad = mejor_texto
            self.contador_memoria = self.frames_memoria

        elif self.contador_memoria > 0:
            mejor_texto = self.ultima_prioridad
            self.contador_memoria -= 1

        # =========================
        # PUBLICAR
        # =========================

        msg_out = String()
        msg_out.data = mejor_texto

        self.pub_priority.publish(msg_out)

        # =========================
        # TEXTO PRIORIDAD
        # =========================

        cv2.putText(
            frame_out,
            f"PRIORIDAD: {mejor_texto}",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 255, 255),
            3
        )

        cv2.imshow("YOLO DOBLE", frame_out)

        cv2.waitKey(1)


def main(args=None):

    rclpy.init(args=args)

    node = YoloDoubleDetector()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
