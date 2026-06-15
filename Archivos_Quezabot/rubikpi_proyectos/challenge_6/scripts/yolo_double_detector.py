#!/usr/bin/env python3

import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import cv2
from ultralytics import YOLO


def validar_flecha_por_zonas(frame, box):
    x1, y1, x2, y2 = map(int, box.xyxy[0])

    roi = frame[y1:y2, x1:x2]

    if roi.size == 0:
        return None

    roi = cv2.resize(roi, (120, 120))

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    # Detectar blanco de la flecha
    lower_white = np.array([0, 0, 110])
    upper_white = np.array([180, 90, 255])
    mask = cv2.inRange(hsv, lower_white, upper_white)

    # Limpieza para quitar ruido
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    h, w = mask.shape

    # Dividir bbox en zonas
    left = np.sum(mask[:, :w // 3])
    center = np.sum(mask[:, w // 3: 2 * w // 3])
    right = np.sum(mask[:, 2 * w // 3:])

    top = np.sum(mask[:h // 3, :])
    middle = np.sum(mask[h // 3: 2 * h // 3, :])
    bottom = np.sum(mask[2 * h // 3:, :])

    total = left + center + right

    if total < 1000:
        return None

    left_ratio = left / total
    center_ratio = center / total
    right_ratio = right / total

    top_ratio = top / total
    middle_ratio = middle / total
    bottom_ratio = bottom / total

    # Si la flecha está distribuida en casi toda la bbox, puede ser vuelta
    zonas_activas = sum([
        left_ratio > 0.20,
        center_ratio > 0.20,
        right_ratio > 0.20,
        top_ratio > 0.20,
        middle_ratio > 0.20,
        bottom_ratio > 0.20
    ])

    if zonas_activas >= 5:
        return "turn_around"

    # Si el blanco está cargado a la derecha
    if right_ratio > 0.40 and right_ratio > left_ratio * 1.20:
        return "turn_right"

    # Si el blanco está cargado a la izquierda
    if left_ratio > 0.40 and left_ratio > right_ratio * 1.20:
        return "turn_left"

    # Si está más arriba y centrado
    if top_ratio > 0.32 and center_ratio > 0.24:
        return "straight"

    return None


class YoloSignalDetector(Node):

    def __init__(self):
        super().__init__('yolo_signal_detector')

        print("Cargando modelo de señales...")
        self.modelo_senales = YOLO('/home/ubuntu/models/best_turnaround.pt')

        self.pub_priority = self.create_publisher(String, '/yolo/priority', 10)

        print("Abriendo cámara...")
        self.cap = cv2.VideoCapture(0)

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 160)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 120)
        self.cap.set(cv2.CAP_PROP_FPS, 5)

        if not self.cap.isOpened():
            self.get_logger().error("No se pudo abrir la cámara")
            return

        self.thresholds_senales = {
            "stop": 0.45,
            "straight": 0.20,
            "turn_right": 0.20,
            "turn_left": 0.20,
            "speed_limit_30": 0.45,
            "traffic_light": 0.50,
            "turn_around": 0.85
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

        self.ultima_prioridad = "none"
        self.frames_memoria = 4
        self.contador_memoria = 0

        self.timer = self.create_timer(1.0, self.main_loop)

        self.get_logger().info("YOLO señales + zonas iniciado SIN ventana")

    def main_loop(self):

        ret, frame = self.cap.read()

        if not ret:
            self.get_logger().warn("No se pudo leer frame")
            return

        mejor_score = 0
        mejor_texto = "none"

        resultados_senales = self.modelo_senales(
            frame,
            imgsz=128,
            conf=0.20,
            verbose=False
        )[0]

        detecciones_senales = []

        for box in resultados_senales.boxes:
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])
            nombre_yolo = self.modelo_senales.names[cls_id]

            nombre_final = nombre_yolo

            if nombre_yolo in ["turn_left", "turn_right", "straight", "turn_around"]:
                validacion = validar_flecha_por_zonas(frame, box)

                if validacion is not None:
                    nombre_final = validacion

            if conf < self.thresholds_senales.get(nombre_final, 0.50):
                continue

            detecciones_senales.append((nombre_final, conf, box, nombre_yolo))

        # Si hay flecha normal, bloquea turn_around para que no gane tan fácil
        hay_direccion = any(
            nombre in ["straight", "turn_left", "turn_right"]
            for nombre, conf, box, nombre_yolo in detecciones_senales
        )

        if hay_direccion:
            detecciones_senales = [
                det for det in detecciones_senales
                if det[0] != "turn_around"
            ]

        for nombre, conf, box, nombre_yolo in detecciones_senales:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            area = (x2 - x1) * (y2 - y1)

            score = area * conf * self.prioridad_senales.get(nombre, 1)

            if score > mejor_score:
                mejor_score = score
                mejor_texto = f"senal:{nombre}"

        if mejor_texto != "none":
            self.ultima_prioridad = mejor_texto
            self.contador_memoria = self.frames_memoria

        elif self.contador_memoria > 0:
            mejor_texto = self.ultima_prioridad
            self.contador_memoria -= 1

        msg = String()
        msg.data = mejor_texto
        self.pub_priority.publish(msg)

        print(f"PRIORIDAD: {mejor_texto}")


def main(args=None):
    rclpy.init(args=args)

    node = YoloSignalDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    if hasattr(node, "cap"):
        node.cap.release()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
