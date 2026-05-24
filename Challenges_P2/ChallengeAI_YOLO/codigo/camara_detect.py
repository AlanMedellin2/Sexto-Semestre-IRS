from ultralytics import YOLO
import cv2

model = YOLO("runs/detect/train/weights/best.pt")
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    results = model(frame, imgsz=416, conf=0.5, verbose=False)
    annotated = results[0].plot()

    cv2.imshow("Deteccion", annotated)

    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
