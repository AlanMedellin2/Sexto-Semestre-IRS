from ultralytics import YOLO
import cv2

# Modelo nuevo
model = YOLO("runs/detect/train-2/weights/best.pt")

cap = cv2.VideoCapture(2)

if not cap.isOpened():
    print("No se pudo abrir la cámara")
    exit()

CONF_THRESHOLD = 0.60  # prueba 0.7 si detecta cosas raras
IMG_SIZE = 416

print("Presiona ESC para salir")

while True:
    ret, frame = cap.read()
    if not ret:
        print("No se pudo leer el frame")
        break

    results = model(frame, imgsz=IMG_SIZE, conf=CONF_THRESHOLD, verbose=False)

    # Dibuja cajas y etiquetas
    annotated = results[0].plot()

    # Si no detecta nada, muestra texto
    if results[0].boxes is None or len(results[0].boxes) == 0:
        cv2.putText(
            annotated,
            "sin senal",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 0, 255),
            2
        )

    cv2.imshow("Deteccion de senales", annotated)

    if cv2.waitKey(1) & 0xFF == 27:  # ESC
        break

cap.release()
cv2.destroyAllWindows()
