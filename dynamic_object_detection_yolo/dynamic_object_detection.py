import cv2
import numpy as np
import torch
from ultralytics import YOLO

# Utilizo mi GPU
device = 'cuda' if torch.cuda.is_available() else 'cpu'
print(f"--- Trabajando con: {device} ---")

# Modelo de segmentación pre entrenado
model = YOLO('yolov8n-seg.pt').to(device)

# Dataset COCO: Persona, bici, auto, moto, bus, camión
DYNAMIC_CLASSES = [0, 1, 2, 3, 5, 7]

cap = cv2.VideoCapture('people_moving.mp4')
#cap = cv2.VideoCapture('6387-191695740.mp4') # Otro video
#cap = cv2.VideoCapture(1) # Cámara conectada a la compu

# Configurar resolución para cámara conectada
#cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
#cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

while cap.isOpened():
    success, frame = cap.read()
    if not success:
        break

    # Redimensionar 
    frame = cv2.resize(frame, (640, 480))

    # Inferencia en GPU
    results = model.track(frame, persist=True, device=0, conf=0.4, verbose=False)
    
    slam_mask = np.zeros(frame.shape[:2], dtype=np.uint8)

    if results[0].masks is not None:
        masks = results[0].masks.xy
        clss = results[0].boxes.cls.cpu().int().tolist()

        for mask_poly, cls in zip(masks, clss):
            if cls in DYNAMIC_CLASSES:
                # Rellenar la silueta en la máscara con blanco
                points = np.int32([mask_poly])
                cv2.fillPoly(slam_mask, points, 255)

                # ORBE
                M = cv2.moments(points)
                if M['m00'] != 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    cv2.circle(frame, (cx, cy), 12, (0, 255, 255), 2) 
                    cv2.circle(frame, (cx, cy), 6, (255, 255, 255), -1) 

    # dilatación
    kernel = np.ones((10, 10), np.uint8)
    slam_mask = cv2.dilate(slam_mask, kernel, iterations=1)

    # Invertimos máscara y aplicamos AND
    inv_mask = cv2.bitwise_not(slam_mask)
    frame_for_slam = cv2.bitwise_and(frame, frame, mask=inv_mask)

    cv2.imshow("1. Visualizacion (Orbes)", frame)
    cv2.imshow("2. Input para ORB-SLAM3", frame_for_slam)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
