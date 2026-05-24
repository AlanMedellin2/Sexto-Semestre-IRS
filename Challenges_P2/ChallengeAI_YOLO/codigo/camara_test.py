from ultralytics import YOLO
import cv2

# Cargar modelo entrenado
model = YOLO("runs/classify/train/weights/best.pt")

# Abrir cámara
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("No se pudo abrir la cámara")
    exit()

print("Presiona ESC para salir")

while True:
    ret, frame = cap.read()
    if not ret:
        print("No se pudo leer el frame")
        break

    # Inferencia
    results = model(frame, imgsz=64, verbose=False)

    r = results[0]
    top1 = r.probs.top1
    conf = float(r.probs.top1conf)
    label = r.names[top1]

    texto = f"{label} {conf:.2f}"

    # Mostrar predicción sobre la imagen
    cv2.putText(
        frame,
        texto,
        (20, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        1,
        (0, 255, 0),
        2
    )

    cv2.imshow("Clasificacion de senales", frame)

    if cv2.waitKey(1) & 0xFF == 27:  # ESC
        break

cap.release()
cv2.destroyAllWindows()
