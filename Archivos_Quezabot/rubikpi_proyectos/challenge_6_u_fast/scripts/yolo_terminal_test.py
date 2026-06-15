#!/usr/bin/env python3

from ultralytics import YOLO
import cv2
import time
import numpy as np

# =========================
# MODELOS
# =========================

model_signs = YOLO("/home/ubuntu/ros2_ws/src/challenge_6_u_fast/models/best.pt")
model_obs = YOLO("/home/ubuntu/ros2_ws/src/challenge_6_u_fast/models/yolo11n.pt")

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
# FUNCION OPENCV
# =========================

def clasificar_flecha_por_bordes(frame, x1, y1, x2, y2):

    roi = frame[y1:y2, x1:x2]

    if roi.size == 0:
        return None

    gris = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

    blur = cv2.GaussianBlur(gris, (5, 5), 0)

    edges = cv2.Canny(blur, 60, 150)

    h, w = edges.shape

    zona_izq = edges[:, :w//3]
    zona_der = edges[:, 2*w//3:]
    zona_arriba = edges[:h//3, :]
    zona_abajo = edges[2*h//3:, :]

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
# CAMARA
# =========================

cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FPS, 10)

frame_count = 0

ultimo_print = time.time()

print("YOLO terminal iniciado")

try:

    while True:

        ret, frame = cap.read()

        if not ret:
            print("No se pudo leer cámara")
            time.sleep(0.5)
            continue

        # =========================
        # UNDISTORT
        # =========================

        frame = cv2.undistort(
            frame,
            camera_matrix,
            dist_coeffs
        )

        frame_count += 1

        if frame_count % 5 != 0:
            continue

        detecciones = []

        mejor_score = 0
        mejor_texto = "none"

        # =========================
        # SEÑALES
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

            x1, y1, x2, y2 = map(
                int,
                box.xyxy[0]
            )

            area = (x2 - x1) * (y2 - y1)

            if name == "traffic_light" and conf < 0.75:
                continue

            if name == "traffic_light" and area < 1200:
                continue

            # =========================
            # OPENCV FLECHAS
            # =========================

            if name in [
                "turn_left",
                "turn_right",
                "straight"
            ]:

                nuevo = clasificar_flecha_por_bordes(
                    frame,
                    x1,
                    y1,
                    x2,
                    y2
                )

                if nuevo is not None:
                    name = nuevo

            score = prioridad_senal.get(
                name,
                100
            ) * area * conf

            detecciones.append(
                f"SENAL:{name} conf={conf:.2f} score={int(score)}"
            )

            if score > mejor_score:
                mejor_score = score
                mejor_texto = f"SENAL:{name}"

        # =========================
        # OBSTACULOS
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

            x1, y1, x2, y2 = map(
                int,
                box.xyxy[0]
            )

            area = (x2 - x1) * (y2 - y1)

            score = prioridad_obstaculo[name] * area * conf

            detecciones.append(
                f"OBST:{name} conf={conf:.2f} score={int(score)}"
            )

            if score > mejor_score:
                mejor_score = score
                mejor_texto = f"OBST:{name}"

        if time.time() - ultimo_print > 0.5:

            print("\n====================")
            print(f"Frame: {frame_count}")
            print(f"PRIORIDAD: {mejor_texto}")
            print(f"SCORE: {int(mejor_score)}")

            if len(detecciones) == 0:
                print("Sin detecciones")

            else:
                for det in detecciones:
                    print(det)

            ultimo_print = time.time()

except KeyboardInterrupt:

    print("\nSaliendo...")

cap.release()
