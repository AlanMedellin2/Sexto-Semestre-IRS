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

        # MODELOS
        # Nuevo modelo de señales con turn_around
        self.modelo_senales = YOLO(
            '/home/ubuntu/models/best_turnaround.pt'
        )

        # Modelo de obstáculos se queda igual
        self.modelo_obstaculos = YOLO(
            '/home/ubuntu/ros2_ws/src/challenge_6_u_fast/models/yolo11n.pt'
        )

        self.subscription = self.create_subscription(
            Image,
            '/video_source/raw',
            self.image_callback,
            10
        )

        self.pub_priority = self.create_publisher(
            String,
            '/yolo/priority',
            10
        )

        self.ultima_prioridad = "none"
        self.frames_memoria = 8
        self.contador_memoria = 0

        self.thresholds_senales = {
            "stop": 0.45,
            "straight": 0.35,
            "turn_right": 0.35,
            "turn_left": 0.35,
            "speed_limit_30": 0.45,
            "traffic_light": 0.50,
            "turn_around": 0.88
        }

        self.prioridad_senales = {
            "stop": 10,
            "turn_left": 9,
            "turn_right": 9,
            "straight": 8,
            "speed_limit_30": 7,
            "traffic_light": 6,
            "turn_around": 4
        }

        self.clases_obstaculos = {
            "person": 5,
            "car": 5,
            "truck": 5,
            "bus": 5,
            "bicycle": 5,
            "motorcycle": 5
        }

        self.get_logger().info("YOLO DOUBLE DETECTOR INICIADO CON MODELO NUEVO")

    def image_callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        frame_out = frame.copy()

        resultados_senales = self.modelo_senales(
            frame,
            imgsz=192,
            conf=0.30,
            verbose=False
        )[0]

        resultados_obstaculos = self.modelo_obstaculos(
            frame,
            imgsz=160,
            conf=0.45,
            verbose=False
        )[0]

        mejor_score = 0
        mejor_texto = "none"

        detecciones_senales = []

        for box in resultados_senales.boxes:

            cls_id = int(box.cls[0])
            conf = float(box.conf[0])
            nombre = self.modelo_senales.names[cls_id]

            min_conf = self.thresholds_senales.get(nombre, 0.50)

            if conf < min_conf:
                continue

            detecciones_senales.append((nombre, conf, box))

        # Si hay flechas normales, no dejamos que turn_around gane
        hay_direccion = any(
            nombre in ["straight", "turn_left", "turn_right"]
            for nombre, conf, box in detecciones_senales
        )

        if hay_direccion:
            detecciones_senales = [
                det for det in detecciones_senales
                if det[0] != "turn_around"
            ]

        for nombre, conf, box in detecciones_senales:

            x1, y1, x2, y2 = map(int, box.xyxy[0])
            area = (x2 - x1) * (y2 - y1)

            prioridad = self.prioridad_senales.get(nombre, 1)
            score = area * conf * prioridad

            if score > mejor_score:
                mejor_score = score
                mejor_texto = f"senal:{nombre}"

            cv2.rectangle(frame_out, (x1, y1), (x2, y2), (255, 0, 0), 2)

            cv2.putText(
                frame_out,
                f"{nombre} {conf:.2f}",
                (x1, max(20, y1 - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 0, 0),
                2
            )

        for box in resultados_obstaculos.boxes:

            cls_id = int(box.cls[0])
            nombre = self.modelo_obstaculos.names[cls_id]

            if nombre not in self.clases_obstaculos:
                continue

            conf = float(box.conf[0])

            x1, y1, x2, y2 = map(int, box.xyxy[0])
            area = (x2 - x1) * (y2 - y1)

            prioridad = self.clases_obstaculos[nombre]
            score = area * conf * prioridad

            if score > mejor_score:
                mejor_score = score
                mejor_texto = f"obstaculo:{nombre}"

            cv2.rectangle(frame_out, (x1, y1), (x2, y2), (0, 0, 255), 2)

            cv2.putText(
                frame_out,
                f"{nombre} {conf:.2f}",
                (x1, max(20, y1 - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 0, 255),
                2
            )

        if mejor_texto != "none":
            self.ultima_prioridad = mejor_texto
            self.contador_memoria = self.frames_memoria

        elif self.contador_memoria > 0:
            mejor_texto = self.ultima_prioridad
            self.contador_memoria -= 1

        msg_out = String()
        msg_out.data = mejor_texto
        self.pub_priority.publish(msg_out)

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
