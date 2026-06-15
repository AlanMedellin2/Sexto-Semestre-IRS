#!/usr/bin/env python3

import time
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from flask import Flask, Response
from ultralytics import YOLO
import threading


app = Flask(__name__)
output_frame = None
lock = threading.Lock()


class YoloStreamDetector(Node):

    def __init__(self):
        super().__init__('yolo_stream_detector')

        self.modelo = YOLO('/home/ubuntu/models/best_turnaround.pt')
        self.pub_priority = self.create_publisher(String, '/yolo/priority', 10)

        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        self.cap.set(cv2.CAP_PROP_FPS, 3)

        self.thresholds = {
            "stop": 0.45,
            "straight": 0.20,
            "turn_right": 0.20,
            "turn_left": 0.20,
            "speed_limit_30": 0.45,
            "traffic_light": 0.50,
            "turn_around": 0.85,
        }

        self.prioridad = {
            "stop": 10,
            "turn_left": 9,
            "turn_right": 9,
            "straight": 8,
            "speed_limit_30": 7,
            "traffic_light": 6,
            "turn_around": 4,
        }

        self.timer = self.create_timer(0.6, self.main_loop)
        self.get_logger().info("YOLO stream detector iniciado")

    def main_loop(self):
        global output_frame

        ok, frame = self.cap.read()
        if not ok:
            return

        frame_out = frame.copy()
        mejor_score = 0
        mejor_texto = "none"

        results = self.modelo(frame, imgsz=160, conf=0.20, verbose=False)[0]

        detecciones = []

        for box in results.boxes:
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])
            nombre = self.modelo.names[cls_id]

            if conf < self.thresholds.get(nombre, 0.50):
                continue

            detecciones.append((nombre, conf, box))

        hay_direccion = any(
            nombre in ["straight", "turn_left", "turn_right"]
            for nombre, conf, box in detecciones
        )

        if hay_direccion:
            detecciones = [
                det for det in detecciones
                if det[0] != "turn_around"
            ]

        for nombre, conf, box in detecciones:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            area = (x2 - x1) * (y2 - y1)
            score = area * conf * self.prioridad.get(nombre, 1)

            if score > mejor_score:
                mejor_score = score
                mejor_texto = f"senal:{nombre}"

            cv2.rectangle(frame_out, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(
                frame_out,
                f"{nombre} {conf:.2f}",
                (x1, max(15, y1 - 5)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (0, 255, 0),
                1,
            )

        msg = String()
        msg.data = mejor_texto
        self.pub_priority.publish(msg)

        cv2.putText(
            frame_out,
            f"PRIORIDAD: {mejor_texto}",
            (5, 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (0, 255, 255),
            1,
        )

        with lock:
            output_frame = frame_out.copy()

        print(f"PRIORIDAD: {mejor_texto}")


def generate():
    global output_frame

    while True:
        with lock:
            if output_frame is None:
                continue

            ok, encoded = cv2.imencode(
                ".jpg",
                output_frame,
                [cv2.IMWRITE_JPEG_QUALITY, 45]
            )

            if not ok:
                continue

            frame = encoded.tobytes()

        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
        )

        time.sleep(0.15)


@app.route("/")
def index():
    return """
    <html>
    <body>
    <h2>YOLO Stream Detector</h2>
    <img src="/video_feed" width="640">
    </body>
    </html>
    """


@app.route("/video_feed")
def video_feed():
    return Response(
        generate(),
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )


def ros_thread():
    rclpy.init()
    node = YoloStreamDetector()
    rclpy.spin(node)

    node.cap.release()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    thread = threading.Thread(target=ros_thread)
    thread.daemon = True
    thread.start()

    app.run(host="0.0.0.0", port=8080, threaded=True)
