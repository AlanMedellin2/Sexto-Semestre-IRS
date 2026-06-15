#!/usr/bin/env python3

from flask import Flask, Response
from ultralytics import YOLO
import cv2
import time

app = Flask(__name__)

# =========================
# MODELOS
# =========================

model_signs = YOLO("/home/ubuntu/ros2_ws/src/challenge_6_u_fast/models/best.pt")
model_obs = YOLO("/home/ubuntu/ros2_ws/src/challenge_6_u_fast/models/yolo11n.pt")

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
# DOUBLE CHECK GIROS
# =========================

def double_check_giro(frame, x1, y1, x2, y2):

    roi = frame[y1:y2, x1:x2]

    if roi.size == 0:
        return None, {}

    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

    blur = cv2.GaussianBlur(gray, (5, 5), 0)

    edges = cv2.Canny(blur, 80, 180)

    h, w = edges.shape

    if h < 20 or w < 20:
        return None, {}

    left_zone = edges[:, :w // 3]
    center_zone = edges[:, w // 3:2 * w // 3]
    right_zone = edges[:, 2 * w // 3:]

    left_score = cv2.countNonZero(left_zone)
    center_score = cv2.countNonZero(center_zone)
    right_score = cv2.countNonZero(right_zone)

    total = left_score + center_score + right_score

    scores = {
        "left": left_score,
        "center": center_score,
        "right": right_score,
        "total": total
    }

    if total < 120:
        return None, scores

    margen_absoluto = 120
    margen_ratio = 1.45

    if right_score > left_score + margen_absoluto and right_score > center_score and right_score > left_score * margen_ratio:
        return "turn_right", scores

    if left_score > right_score + margen_absoluto and left_score > center_score and left_score > right_score * margen_ratio:
        return "turn_left", scores

    return None, scores


# =========================
# DETECCION SEÑALES
# =========================

def procesar_senales(frame, offset_x=0, zona="full"):

    resultados = []

    res = model_signs(
        frame,
        imgsz=320,
        conf=0.55,
        verbose=False
    )[0]

    for box in res.boxes:

        cls_id = int(box.cls[0])

        conf = float(box.conf[0])

        name_yolo = model_signs.names[cls_id]

        name_final = name_yolo

        x1, y1, x2, y2 = map(int, box.xyxy[0])

        x1_global = x1 + offset_x
        x2_global = x2 + offset_x

        area = (x2 - x1) * (y2 - y1)

        if conf < 0.55:
            continue

        if name_yolo == "traffic_light" and conf < 0.70:
            continue

        if name_yolo == "traffic_light" and area < 1500:
            continue

        if area < 250:
            continue

        edge_info = ""

        if name_yolo in ["turn_left", "turn_right"]:

            edge_result, scores = double_check_giro(
                frame,
                x1,
                y1,
                x2,
                y2
            )

            if edge_result is not None:
                name_final = edge_result

                edge_info = (
                    f"{edge_result} "
                    f"L={scores.get('left', 0)} "
                    f"R={scores.get('right', 0)}"
                )

        score = prioridad_senal.get(name_final, 100) * area * conf

        resultados.append(
            (
                name_yolo,
                name_final,
                conf,
                area,
                score,
                edge_info,
                zona,
                x1_global,
                y1,
                x2_global,
                y2
            )
        )

    return resultados


# =========================
# CAMARA
# =========================

cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FPS, 10)

frame_count = 0
ultimo_frame = None

# =========================
# STREAM
# =========================

def generar_frames():

    global frame_count
    global ultimo_frame

    while True:

        ret, frame = cap.read()

        if not ret:
            continue

        frame_count += 1

        frame_out = frame.copy()

        h, w, _ = frame.shape

        mejor_score = 0
        mejor_texto = "none"

        # =========================
        # SEÑALES
        # =========================

        detecciones_signs = []

        detecciones_full = procesar_senales(
            frame,
            offset_x=0,
            zona="full"
        )

        detecciones_signs.extend(detecciones_full)

        if len(detecciones_full) == 0:

            left_crop = frame[:, :w // 2]
            right_crop = frame[:, w // 2:]

            detecciones_left = procesar_senales(
                left_crop,
                offset_x=0,
                zona="left"
            )

            detecciones_right = procesar_senales(
                right_crop,
                offset_x=w // 2,
                zona="right"
            )

            detecciones_signs.extend(detecciones_left)
            detecciones_signs.extend(detecciones_right)

        # =========================
        # DIBUJAR SEÑALES
        # =========================

        for (
            name_yolo,
            name_final,
            conf,
            area,
            score,
            edge_info,
            zona,
            x1,
            y1,
            x2,
            y2
        ) in detecciones_signs:

            if score > mejor_score:
                mejor_score = score
                mejor_texto = f"SENAL:{name_final}"

            cv2.rectangle(
                frame_out,
                (x1, y1),
                (x2, y2),
                (255, 0, 0),
                2
            )

            texto = (
                f"{name_final} "
                f"{conf:.2f} "
                f"{zona}"
            )

            cv2.putText(
                frame_out,
                texto,
                (x1, max(20, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 0, 0),
                2
            )

        # =========================
        # OBSTACULOS
        # =========================

        if frame_count % 5 == 0:

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

                if area < 300:
                    continue

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
                    f"OBS:{name} {conf:.2f}",
                    (x1, max(20, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.45,
                    (0, 255, 255),
                    2
                )

        # =========================
        # TEXTO GENERAL
        # =========================

        cv2.putText(
            frame_out,
            f"PRIORIDAD: {mejor_texto}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 0, 255),
            2
        )

        cv2.putText(
            frame_out,
            f"FRAME: {frame_count}",
            (10, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 0, 255),
            2
        )

        ultimo_frame = frame_out.copy()

        _, buffer = cv2.imencode(
            '.jpg',
            frame_out,
            [int(cv2.IMWRITE_JPEG_QUALITY), 60]
        )

        frame_bytes = buffer.tobytes()

        yield (
            b'--frame\r\n'
            b'Content-Type: image/jpeg\r\n\r\n' +
            frame_bytes +
            b'\r\n'
        )

        time.sleep(0.03)


# =========================
# FLASK
# =========================

@app.route('/')
def index():

    return """
    <html>
    <head>
        <title>YOLO Rubik Stream</title>
    </head>

    <body style="background:#111;color:white;text-align:center;font-family:Arial;">

        <h2>YOLO RubikPi</h2>

        <p>Señales + obstáculos + double check</p>

        <img src="/video_feed" width="900">

    </body>
    </html>
    """


@app.route('/video_feed')
def video_feed():

    return Response(
        generar_frames(),
        mimetype='multipart/x-mixed-replace; boundary=frame'
    )


# =========================
# MAIN
# =========================

if __name__ == "__main__":

    print("\n========================")
    print("STREAM INICIADO")
    print("========================")
    print("Abrir en laptop:")
    print("http://10.48.190.132:5000")
    print("========================\n")

    app.run(
        host='0.0.0.0',
        port=5000,
        threaded=True
    )
