#!/usr/bin/env python3

from ultralytics import YOLO
import cv2
import time

model = YOLO("/home/ubuntu/ros2_ws/src/challenge_6/models/best.pt")

cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
cap.set(cv2.CAP_PROP_FPS, 10)

frame_count = 0
ultimo_print = time.time()

print("YOLO minimo iniciado. CTRL+C para salir.")

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("No lee camara")
            time.sleep(0.5)
            continue

        frame_count += 1

        if frame_count % 8 != 0:
            continue

        results = model(frame, imgsz=320, conf=0.35, verbose=False)[0]

        detecciones = []

        for box in results.boxes:
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])
            name = model.names[cls_id]
            detecciones.append(f"{name} {conf:.2f}")

        if time.time() - ultimo_print > 0.5:
            print("\nFrame:", frame_count)

            if detecciones:
                for d in detecciones:
                    print("Detectado:", d)
            else:
                print("Sin detecciones")

            ultimo_print = time.time()

except KeyboardInterrupt:
    print("\nSaliendo...")

cap.release()
