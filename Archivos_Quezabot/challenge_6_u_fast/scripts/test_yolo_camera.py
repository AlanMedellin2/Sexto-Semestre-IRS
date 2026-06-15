#!/usr/bin/env python3

from ultralytics import YOLO
import cv2
import time

model_signs = YOLO("/home/ubuntu/ros2_ws/src/challenge_6_u_fast/models/best.pt")
model_obs = YOLO("/home/ubuntu/ros2_ws/src/challenge_6_u_fast/models/yolo11n.pt")

cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
cap.set(cv2.CAP_PROP_FPS, 10)

prioridad_senal = {
    "traffic_light": 300,
    "stop": 260,
    "turn_left": 220,
    "turn_right": 220,
    "straight": 200,
    "speed_limit_30": 180
}

prioridad_obstaculo = {
    "car": 30,
    "bicycle": 25,
    "motorcycle": 25,
    "bus": 20,
    "truck": 20
}

frame_count = 0
start = time.time()

while True:
    ret, frame = cap.read()

    if not ret:
        print("No se pudo leer la camara")
        break

    frame_count += 1

    if frame_count % 3 != 0:
        continue

    frame_out = frame.copy()
    mejor_score = 0
    mejor_texto = "none"

    res_signs = model_signs(frame, imgsz=320, conf=0.55, verbose=False)[0]

    for box in res_signs.boxes:
        cls_id = int(box.cls[0])
        conf = float(box.conf[0])
        name = model_signs.names[cls_id]
        x1, y1, x2, y2 = map(int, box.xyxy[0])

        area = (x2 - x1) * (y2 - y1)
        score = prioridad_senal.get(name, 100) * area * conf

        if score > mejor_score:
            mejor_score = score
            mejor_texto = f"senal:{name}"

        cv2.rectangle(frame_out, (x1, y1), (x2, y2), (255, 0, 0), 2)
        cv2.putText(frame_out, f"{name} {conf:.2f}", (x1, max(20, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 0, 0), 2)

    res_obs = model_obs(frame, imgsz=320, conf=0.45, verbose=False)[0]

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
            mejor_texto = f"obstaculo:{name}"

        cv2.rectangle(frame_out, (x1, y1), (x2, y2), (0, 255, 255), 2)
        cv2.putText(frame_out, f"obs:{name} {conf:.2f}", (x1, max(20, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 2)

    cv2.putText(frame_out, f"PRIORIDAD: {mejor_texto}", (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)

    if frame_count % 30 == 0:
        path = f"/home/ubuntu/yolo_test_{frame_count}.jpg"
        cv2.imwrite(path, frame_out)
        print(f"{mejor_texto} | guardado {path}")

    if time.time() - start > 30:
        break

cap.release()
print("Prueba terminada")
