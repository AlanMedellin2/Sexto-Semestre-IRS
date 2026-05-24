from ultralytics import YOLO
import cv2

model = YOLO("runs/classify/train/weights/best.pt")
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("No se pudo abrir la cámara")
    exit()

UNKNOWN_THRESHOLD = 0.85  # prueba también 0.90 si aún se equivoca mucho

print("Presiona ESC para salir")

while True:
    ret, frame = cap.read()
    if not ret:
        print("No se pudo leer el frame")
        break

    results = model(frame, imgsz=64, verbose=False)
    r = results[0]

    top1 = r.probs.top1
    conf = float(r.probs.top1conf)
    pred_label = r.names[top1]

    if conf < UNKNOWN_THRESHOLD:
        label = "desconocido"
    else:
        label = pred_label

    texto = f"{label} {conf:.2f}"

    cv2.putText(
        frame,
        texto,
        (20, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        1,
        (0, 255, 0),
        2
    )

    cv2.imshow("Clasificacion con desconocido", frame)

    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
