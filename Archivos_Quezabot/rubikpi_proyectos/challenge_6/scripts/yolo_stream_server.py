#!/usr/bin/env python3

from flask import Flask, Response
from ultralytics import YOLO
import cv2
import numpy as np
import time

app = Flask(__name__)

# =========================
# MODELOS
# =========================

model_signs = YOLO("/home/ubuntu/ros2_ws/src/challenge_6/models/best.pt")
model_obs = YOLO("/home/ubuntu/ros2_ws/src/challenge_6/models/yolo11n.pt")

# =========================
# CALIBRACION
# =========================

data = np.load("/home/ubuntu/calibracion_v4l2/calibracion_v4l2.npz")
camera_matrix = data["mtx"]
dist_coeffs = data["dist"]

# =========================
# PRIORIDADES
# =========================

prioridad_senal = {
    "stop": 300,
    "turn_left": 280,
    "turn_right": 280,
    "straight": 260,
    "speed_limit_30": 240,
    "traffic_light": 180
}

prioridad_obstaculo = {
    "car": 30,
    "bicycle": 25,
    "motorcycle": 25,
    "bus": 20,
    "truck": 20
}

# =========================
# CAMARA
# =========================

cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FPS, 10)

frame_count = 0
ultimo_frame_out = None


# =========================
# FUNCION FLECHAS POR BORDES
# =========================

def clasificar_flecha_por_bordes(frame, x1, y1, x2, y2):
    roi = frame[y1:y2, x1:x2]

    if roi.size == 0:
        return None

    gris = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gris, (5, 5), 0)
    edges = cv2.Canny(blur, 60, 150)

    h, w = edges.shape

    zona_izq = edges[:, :w // 3]
    zona_der = edges[:, 2 * w // 3:]
    zona_arriba = edges[:h // 3, :]
    zona_abajo = edges[2 * h // 3:, :]

    score_izq = cv2.countNonZero(zona_izq)
    score_der = cv2.countNonZero(zona_der)
    score_arriba = cv2.countNonZero(zona_arriba)
    score_abajo = cv2.countNonZero(zona_abajo)

    margen = 80

    if score_der > score_izq + margen and score_der > score_arriba:
        return "turn_right"

    elif score_izq > score_der + margen and score_izq > score_arriba:
        return "turn_left"

    elif score_arriba > score_abajo:
        return "straight"

    return None


# =========================
# GENERADOR DE FRAMES
# =========================

def generar_frames():
    global frame_count, ultimo_frame_out

    while True:
        ret, frame = cap.read()

        if not ret:
            print("No se pudo leer camara")
            time.sleep(0.1)
            continue

        frame_count += 1

        # Corregir distorsion
        frame = cv2.undistort(frame, camera_matrix, dist_coeffs)

        # Para no saturar, YOLO corre cada 5 frames
        if frame_count % 5 != 0 and ultimo_frame_out is not None:
            frame_out = ultimo_frame_out.copy()

        else:
            frame_out = frame.copy()

            mejor_score = 0
            mejor_texto = "none"

            # =========================
            # YOLO SEÑALES
            # =========================

            res_signs = model_signs(
                frame,
                imgsz=256,
                conf=0.70,
                verbose=False
            )[0]

            for box in res_signs.boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                name = model_signs.names[cls_id]

                x1, y1, x2, y2 = map(int, box.xyxy[0])
                area = (x2 - x1) * (y2 - y1)

                if name == "traffic_light" and conf < 0.75:
                    continue

                if name == "traffic_light" and area < 1200:
                    continue

                if name in ["turn_left", "turn_right", "straight"]:
                    nuevo = clasificar_flecha_por_bordes(
                        frame,
                        x1,
                        y1,
                        x2,
                        y2
                    )

                    if nuevo is not None:
                        name = nuevo

                score = prioridad_senal.get(name, 100) * area * conf

                if score > mejor_score:
                    mejor_score = score
                    mejor_texto = f"SENAL:{name}"

                cv2.rectangle(
                    frame_out,
                    (x1, y1),
                    (x2, y2),
                    (255, 0, 0),
                    2
                )

                cv2.putText(
                    frame_out,
                    f"{name} {conf:.2f}",
                    (x1, max(20, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.45,
                    (255, 0, 0),
                    2
                )

            # =========================
            # YOLO OBSTACULOS
            # =========================

            res_obs = model_obs(
                frame,
                imgsz=256,
                conf=0.45,
                verbose=False
            )[0]

            for box in res_obs.boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                name = model_obs.names[cls_id]

                if name not in prioridad_obstaculo:
                    continue

                x1, y1, x2, y2 = map(int, box.xyxy[0])
                area = (x2 - x1) * (y2 - y1)

                score = prioridad_obstaculo[name] * area * conf

                if score > mejor_score:
                    mejor_score = score
                    mejor_texto = f"OBST:{name}"

                cv2.rectangle(
                    frame_out,
                    (x1, y1),
                    (x2, y2),
                    (0, 255, 255),
                    2
                )

                cv2.putText(
                    frame_out,
                    f"obs:{name} {conf:.2f}",
                    (x1, max(20, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.45,
                    (0, 255, 255),
                    2
                )

            cv2.putText(
                frame_out,
                f"PRIORIDAD: {mejor_texto}",
                (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 0, 255),
                2
            )

            cv2.putText(
                frame_out,
                f"frame:{frame_count}",
                (10, 50),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 0, 255),
                2
            )

            ultimo_frame_out = frame_out.copy()

        ok, buffer = cv2.imencode(
            ".jpg",
            frame_out,
            [int(cv2.IMWRITE_JPEG_QUALITY), 35]
        )

        if not ok:
            continue

        frame_bytes = buffer.tobytes()

        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n" +
            frame_bytes +
            b"\r\n"
        )

        time.sleep(0.03)


# =========================
# RUTAS FLASK
# =========================

@app.route("/")
def index():
    return """
    <html>
        <head>
            <title>YOLO Rubik Stream</title>
        </head>
        <body style="background-color:#111;color:white;text-align:center;font-family:Arial;">
            <h2>YOLO doble en RubikPi</h2>
            <p>Señales + semáforos + obstáculos</p>
            <img src="/video_feed" width="640">
        </body>
    </html>
    """


@app.route("/video_feed")
def video_feed():
    return Response(
        generar_frames(),
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )


# =========================
# MAIN
# =========================

if __name__ == "__main__":
    print("Servidor YOLO iniciado")
    print("Abrir en laptop:")
    print("http://10.48.190.132:5000")
    print("o")
    print("http://192.168.0.115:5000")

    app.run(
        host="0.0.0.0",
        port=5000,
        threaded=True
    )
