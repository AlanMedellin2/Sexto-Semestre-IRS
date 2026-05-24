from ultralytics import YOLO
import cv2

model = YOLO("runs/detect/train/weights/best.pt")
cap = cv2.VideoCapture(2)

if not cap.isOpened():
    print("No se pudo abrir la cámara")
    exit()

# Umbral de confianza: súbelo si sigue detectando cosas falsas
CONF_THRESHOLD = 0.48

# Filtro opcional por tamaño de caja respecto al frame
# Evita que detecte objetos absurdamente grandes como toda la cara/pared
MAX_AREA_RATIO = 0.35   # 35% del frame
MIN_AREA_RATIO = 0.001  # evita ruido muy pequeño

print("Presiona ESC para salir")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    h, w, _ = frame.shape
    frame_area = w * h

    results = model(frame, imgsz=416, conf=0.05, verbose=False)
    annotated = frame.copy()

    valid_detections = 0

    if len(results) > 0 and results[0].boxes is not None:
        boxes = results[0].boxes

        for box in boxes:
            conf = float(box.conf[0])
            cls = int(box.cls[0])
            label = results[0].names[cls]

            x1, y1, x2, y2 = map(int, box.xyxy[0])
            box_area = max(0, x2 - x1) * max(0, y2 - y1)
            area_ratio = box_area / frame_area if frame_area > 0 else 0

            # Solo aceptar detecciones suficientemente confiables
            # y con tamaño razonable
            if conf < CONF_THRESHOLD:
                continue

            if area_ratio > MAX_AREA_RATIO or area_ratio < MIN_AREA_RATIO:
                continue

            valid_detections += 1

            cv2.rectangle(annotated, (x1, y1), (x2, y2), (255, 0, 0), 2)
            cv2.putText(
                annotated,
                f"{label} {conf:.2f}",
                (x1, max(30, y1 - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                (255, 0, 0),
                2
            )

    # Si no hubo detecciones válidas, no dibuja ninguna caja
    if valid_detections == 0:
        cv2.putText(
            annotated,
            "sin senal",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 0, 255),
            2
        )

    cv2.imshow("Deteccion filtrada", annotated)

    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
