from ultralytics import YOLO
import cv2
import numpy as np

# =========================
# CONFIG
# =========================
MODEL_PATH = "runs/detect/train-2/weights/best.pt"
CAMERA_INDEX = 2
YOLO_CONF = 0.45
YOLO_IMGSZ = 416

# Filtros de contornos
MIN_CONTOUR_AREA = 1500
MAX_CONTOUR_AREA = 120000
MIN_ASPECT_RATIO = 0.6
MAX_ASPECT_RATIO = 1.4

# Margen extra alrededor del recorte
PADDING = 10

# =========================
# MODELO
# =========================
model = YOLO(MODEL_PATH)

cap = cv2.VideoCapture(CAMERA_INDEX)
if not cap.isOpened():
    print("No se pudo abrir la cámara")
    exit()

print("Presiona ESC para salir")

while True:
    ret, frame = cap.read()
    if not ret:
        print("No se pudo leer el frame")
        break

    output = frame.copy()
    h, w = frame.shape[:2]

    # =========================
    # 1) BGR -> HSV
    # =========================
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # =========================
    # 2) MASCARAS DE COLOR
    # =========================

    # Azul
    lower_blue = np.array([90, 80, 60])
    upper_blue = np.array([135, 255, 255])
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

    # Rojo (dos rangos en HSV)
    lower_red1 = np.array([0, 80, 60])
    upper_red1 = np.array([10, 255, 255])

    lower_red2 = np.array([170, 80, 60])
    upper_red2 = np.array([180, 255, 255])

    mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask_red = cv2.bitwise_or(mask_red1, mask_red2)

    # Máscara combinada
    mask = cv2.bitwise_or(mask_blue, mask_red)

    # =========================
    # 3) LIMPIEZA MORFOLÓGICA
    # =========================
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.dilate(mask, kernel, iterations=1)

    # =========================
    # 4) CONTORNOS CANDIDATOS
    # =========================
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    found_valid_detection = False

    for cnt in contours:
        area = cv2.contourArea(cnt)

        if area < MIN_CONTOUR_AREA or area > MAX_CONTOUR_AREA:
            continue

        x, y, bw, bh = cv2.boundingRect(cnt)

        if bh == 0:
            continue

        aspect_ratio = bw / bh
        if aspect_ratio < MIN_ASPECT_RATIO or aspect_ratio > MAX_ASPECT_RATIO:
            continue

        # Agregar padding
        x1 = max(0, x - PADDING)
        y1 = max(0, y - PADDING)
        x2 = min(w, x + bw + PADDING)
        y2 = min(h, y + bh + PADDING)

        roi = frame[y1:y2, x1:x2]
        if roi.size == 0:
            continue

        # =========================
        # 5) YOLO SOLO EN ROI
        # =========================
        results = model(roi, imgsz=YOLO_IMGSZ, conf=YOLO_CONF, verbose=False)

        if results[0].boxes is None or len(results[0].boxes) == 0:
            continue

        # Tomar solo la detección más confiable dentro del ROI
        best_box = None
        best_conf = -1.0
        best_label = None

        for box in results[0].boxes:
            conf = float(box.conf[0])
            cls = int(box.cls[0])
            label = results[0].names[cls]

            if conf > best_conf:
                best_conf = conf
                best_box = box
                best_label = label

        if best_box is None:
            continue

        # Coordenadas de la caja dentro del ROI
        bx1, by1, bx2, by2 = map(int, best_box.xyxy[0])

        # Pasarlas a coordenadas globales del frame
        gx1 = x1 + bx1
        gy1 = y1 + by1
        gx2 = x1 + bx2
        gy2 = y1 + by2

        # Dibujar SOLO detecciones confirmadas por YOLO
        cv2.rectangle(output, (gx1, gy1), (gx2, gy2), (255, 0, 0), 2)
        cv2.putText(
            output,
            f"{best_label} {best_conf:.2f}",
            (gx1, max(30, gy1 - 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 0, 0),
            2
        )

        found_valid_detection = True

    if not found_valid_detection:
        cv2.putText(
            output,
            "sin senal",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 0, 255),
            2
        )

    # Ventanas
    cv2.imshow("Deteccion con mascara + YOLO", output)
    cv2.imshow("Mascara", mask)

    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
