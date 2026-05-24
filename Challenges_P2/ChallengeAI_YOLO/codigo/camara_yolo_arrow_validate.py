from ultralytics import YOLO
import cv2
import numpy as np
from collections import deque, Counter


MODEL_PATH = "runs/detect/train-2/weights/best.pt"
CAMERA_INDEX = 2

YOLO_CONF = 0.45
IMG_SIZE = 416

# filtros geométricos generales
MIN_BOX_AREA_RATIO = 0.002
MAX_BOX_AREA_RATIO = 0.25
MIN_BOX_ASPECT = 0.6
MAX_BOX_ASPECT = 1.4

history = deque(maxlen=5)
MIN_FRAMES_FOR_CONFIRM = 2



# MODELO
# =========================
model = YOLO(MODEL_PATH)




def classify_arrow_direction(roi_bgr):
    # Intenta clasificar la flecha dentro de una señal azul:
    # devuelve 'turn_left', 'turn_right', 'straight' o None

    if roi_bgr is None or roi_bgr.size == 0:
        return None

    h, w = roi_bgr.shape[:2]
    if h < 20 or w < 20:
        return None

    hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)


    # Blanco de la flecha
    lower_white = np.array([0, 0, 150])
    upper_white = np.array([180, 90, 255])
    white_mask = cv2.inRange(hsv, lower_white, upper_white)



    # limpieza
    kernel = np.ones((3, 3), np.uint8)

    white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, kernel)
    white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(white_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if not contours:
        return None

    # contorno blanco más grande
    cnt = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(cnt)
    if area < 0.01 * (h * w):
        return None


    # bounding contorno blanco
    x, y, bw, bh = cv2.boundingRect(cnt)
    if bw == 0 or bh == 0:
        return None

    aspect = bw / bh



    # Centroide 
    M = cv2.moments(cnt)
    if M["m00"] == 0:
        return None

    cx = M["m10"] / M["m00"]
    cy = M["m01"] / M["m00"]


    # Extremos del contorno
    pts = cnt.reshape(-1, 2)
    leftmost = pts[np.argmin(pts[:, 0])]
    rightmost = pts[np.argmax(pts[:, 0])]

    topmost = pts[np.argmin(pts[:, 1])]
    bottommost = pts[np.argmax(pts[:, 1])]


    # Distancias desde centroide a extremos
    dist_left = np.linalg.norm(leftmost - np.array([cx, cy]))
    dist_right = np.linalg.norm(rightmost - np.array([cx, cy]))
    dist_top = np.linalg.norm(topmost - np.array([cx, cy]))
    dist_bottom = np.linalg.norm(bottommost - np.array([cx, cy]))

    # Reglas OU YEA:
    # - izq/der: extremo horizontal domina
    # - straight: extremo superior domina y forma más vertical
    horizontal_dom = max(dist_left, dist_right)
    vertical_dom = max(dist_top, dist_bottom)

    # caso recto
    if dist_top > dist_left * 1.10 and dist_top > dist_right * 1.10 and aspect < 1.15:
        return "straight"

    # caso derecha
    if dist_right > dist_left * 1.10:
        return "turn_right"

    # caso izquierda
    if dist_left > dist_right * 1.10:
        return "turn_left"

    # fallback por posición relativa del centroide y extremos
    if rightmost[0] - cx > cx - leftmost[0]:
        return "turn_right"
    elif cx - leftmost[0] > rightmost[0] - cx:
        return "turn_left"

    return None


def validate_detection(label, roi_bgr):
    # valida por color/estructura.
    # para flechas, además valida la orientación interna.

    hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)

    # azul
    lower_blue = np.array([90, 80, 60])
    upper_blue = np.array([135, 255, 255])
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)


    # rojo
    lower_red1 = np.array([0, 80, 60])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 80, 60])
    upper_red2 = np.array([180, 255, 255])
    mask_red = cv2.bitwise_or(
        cv2.inRange(hsv, lower_red1, upper_red1),
        cv2.inRange(hsv, lower_red2, upper_red2)
    )

    # blanco
    lower_white = np.array([0, 0, 150])
    upper_white = np.array([180, 90, 255])
    mask_white = cv2.inRange(hsv, lower_white, upper_white)

    total = roi_bgr.shape[0] * roi_bgr.shape[1]
    if total == 0:
        return False

    blue_ratio = np.count_nonzero(mask_blue) / total
    red_ratio = np.count_nonzero(mask_red) / total
    white_ratio = np.count_nonzero(mask_white) / total

    # Reglas por clase
    if label in ["turn_left", "turn_right", "straight"]:
        # Debe verse clearly azul y blanco
        if blue_ratio < 0.10 or white_ratio < 0.08:
            return False

        arrow_label = classify_arrow_direction(roi_bgr)
        if arrow_label is None:
            return False

        return arrow_label == label
    

    elif label == "stop":
        # rojo dominante
        if red_ratio < 0.08:
            return False
        return True

    elif label == "speed_limit_30":
        # rojo + blanco
        if red_ratio < 0.03 or white_ratio < 0.12:
            return False
        return True

    return False


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
    frame_area = h * w

    results = model(frame, imgsz=IMG_SIZE, conf=0.05, verbose=False)

    current_labels = []
    detections_to_draw = []

    if results[0].boxes is not None and len(results[0].boxes) > 0:
        for box in results[0].boxes:
            conf = float(box.conf[0])
            cls = int(box.cls[0])
            label = results[0].names[cls]


            if conf < YOLO_CONF:
                continue

            x1, y1, x2, y2 = map(int, box.xyxy[0])

            box_w = max(1, x2 - x1)
            box_h = max(1, y2 - y1)
            area_ratio = (box_w * box_h) / frame_area
            aspect_ratio = box_w / box_h if box_h > 0 else 0

            if area_ratio < MIN_BOX_AREA_RATIO or area_ratio > MAX_BOX_AREA_RATIO:
                continue

            if aspect_ratio < MIN_BOX_ASPECT or aspect_ratio > MAX_BOX_ASPECT:
                continue

            roi = frame[max(0, y1):min(h, y2), max(0, x1):min(w, x2)]
            if roi.size == 0:
                continue

            if not validate_detection(label, roi):
                continue

            current_labels.append(label)
            detections_to_draw.append((label, conf, x1, y1, x2, y2))


    history.append(current_labels)
    flat = [lab for labs in history for lab in labs]
    counts = Counter(flat)


    drawn = False
    for label, conf, x1, y1, x2, y2 in detections_to_draw:
        if counts[label] >= MIN_FRAMES_FOR_CONFIRM:

            drawn = True
            cv2.rectangle(output, (x1, y1), (x2, y2), (255, 0, 0), 2)
            cv2.putText(
                output,
                f"{label} {conf:.2f}",
                (x1, max(30, y1 - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (255, 0, 0),
                2
            )

    if not drawn:
        cv2.putText(
            output,
            "sin senal",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 0, 255),
            2
        )

    cv2.imshow("YOLO + validacion de flecha", output)

    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
