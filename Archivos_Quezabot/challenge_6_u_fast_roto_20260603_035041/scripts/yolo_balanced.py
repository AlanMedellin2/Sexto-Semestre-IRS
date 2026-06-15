#!/usr/bin/env python3

from ultralytics import YOLO
import cv2
import time

model_signs = YOLO("/home/ubuntu/ros2_ws/src/challenge_6_u_fast/models/best.pt")
model_obs = YOLO("/home/ubuntu/ros2_ws/src/challenge_6_u_fast/models/yolo11n.pt")

prioridad_senal = {
    "stop": 300,
    "turn_left": 280,
    "turn_right": 280,
    "straight": 260,
    "speed_limit_30": 240,
    "traffic_light": 180
}

# Obstáculos activados, pero con baja prioridad
prioridad_obstaculo = {
    "car": 30,
    "bicycle": 25,
    "motorcycle": 25,
    "bus": 20,
    "truck": 20
}


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


def procesar_senales_en_frame(frame, offset_x=0, etiqueta_zona="full"):
    resultados = []

    res = model_signs(
        frame,
        imgsz=416,
        conf=0.55,
        verbose=False
    )[0]

    for box in res.boxes:
        cls_id = int(box.cls[0])
        conf = float(box.conf[0])
        name_yolo = model_signs.names[cls_id]
        name_final = name_yolo

        x1, y1, x2, y2 = map(int, box.xyxy[0])

        # Convertir coordenadas si viene de crop lateral
        x1_global = x1 + offset_x
        x2_global = x2 + offset_x
        y1_global = y1
        y2_global = y2

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

        # Double check SOLO para giros
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
                    f" | edge_check={edge_result}"
                    f" L={scores.get('left', 0)}"
                    f" C={scores.get('center', 0)}"
                    f" R={scores.get('right', 0)}"
                    f" T={scores.get('total', 0)}"
                )
            else:
                name_final = name_yolo
                edge_info = (
                    f" | edge_check=keep_yolo"
                    f" L={scores.get('left', 0)}"
                    f" C={scores.get('center', 0)}"
                    f" R={scores.get('right', 0)}"
                    f" T={scores.get('total', 0)}"
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
                etiqueta_zona,
                x1_global,
                y1_global,
                x2_global,
                y2_global
            )
        )

    return resultados


cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FPS, 10)

frame_count = 0
ultimo_print = time.time()

ultima_senal = []
ultimo_obstaculo = []

print("YOLO balanced iniciado. CTRL+C para salir.")
print("Señales: full frame + fallback lateral")
print("Obstáculos: car, bicycle, motorcycle, bus, truck")
print("Double check SOLO para turn_left y turn_right")
print("straight se queda con YOLO")

try:
    while True:
        ret, frame = cap.read()

        if not ret:
            print("No lee camara")
            time.sleep(0.5)
            continue

        frame_count += 1
        detecciones = []
        mejor_score = 0
        mejor_texto = "none"

        h, w, _ = frame.shape

        # =========================
        # SEÑALES
        # =========================

        if frame_count % 6 == 0:
            ultima_senal = []

            # 1. Primero frame completo
            detecciones_full = procesar_senales_en_frame(
                frame,
                offset_x=0,
                etiqueta_zona="full"
            )

            ultima_senal.extend(detecciones_full)

            # 2. Si no encontró señales, busca en laterales
            if len(detecciones_full) == 0:
                left_crop = frame[:, :w // 2]
                right_crop = frame[:, w // 2:]

                detecciones_left = procesar_senales_en_frame(
                    left_crop,
                    offset_x=0,
                    etiqueta_zona="left_crop"
                )

                detecciones_right = procesar_senales_en_frame(
                    right_crop,
                    offset_x=w // 2,
                    etiqueta_zona="right_crop"
                )

                ultima_senal.extend(detecciones_left)
                ultima_senal.extend(detecciones_right)

        # =========================
        # OBSTÁCULOS
        # =========================

        if frame_count % 18 == 0:
            res_obs = model_obs(
                frame,
                imgsz=256,
                conf=0.45,
                verbose=False
            )[0]

            ultimo_obstaculo = []

            for box in res_obs.boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                name = model_obs.names[cls_id]

                if name not in prioridad_obstaculo:
                    continue

                x1, y1, x2, y2 = map(int, box.xyxy[0])
                area = (x2 - x1) * (y2 - y1)

                if area < 300:
                    continue

                score = prioridad_obstaculo[name] * area * conf

                ultimo_obstaculo.append(
                    (name, conf, area, score)
                )

        # =========================
        # DECISIÓN FINAL
        # =========================

        for name_yolo, name_final, conf, area, score, edge_info, zona, x1, y1, x2, y2 in ultima_senal:
            detecciones.append(
                f"SENAL:{name_final} "
                f"(YOLO:{name_yolo}) "
                f"zona={zona} "
                f"conf={conf:.2f} "
                f"area={area} "
                f"score={int(score)}"
                f"{edge_info}"
            )

            if score > mejor_score:
                mejor_score = score
                mejor_texto = f"SENAL:{name_final}"

        for name, conf, area, score in ultimo_obstaculo:
            detecciones.append(
                f"OBST:{name} conf={conf:.2f} area={area} score={int(score)}"
            )

            if score > mejor_score:
                mejor_score = score
                mejor_texto = f"OBST:{name}"

        # =========================
        # PRINT
        # =========================

        if time.time() - ultimo_print > 0.5:
            print("\n====================")
            print(f"Frame: {frame_count}")
            print(f"PRIORIDAD: {mejor_texto}")
            print(f"SCORE: {int(mejor_score)}")

            if detecciones:
                for d in detecciones:
                    print(d)
            else:
                print("Sin detecciones")

            ultimo_print = time.time()

except KeyboardInterrupt:
    print("\nSaliendo...")

cap.release()
