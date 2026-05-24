from ultralytics import YOLO
import cv2
import time

modelo_senales = YOLO("runs/detect/train-4/weights/best.pt")
modelo_obstaculos = YOLO("yolo11n.pt")

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

clases_obstaculos = list(prioridad_obstaculo.keys())

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
cap.set(cv2.CAP_PROP_FPS, 10)

cv2.namedWindow("deteccion_prioridad", cv2.WINDOW_NORMAL)

frame_count = 0
ultimo_objetivo = None
ultimo_frame_out = None
score_suavizado = 0

alpha = 0.10  # más bajo = menos parpadeo en score

while True:
    ret, frame = cap.read()

    if not ret:
        print("No se pudo leer la camara")
        break

    frame_count += 1

    # En frames saltados, muestra la última imagen con detecciones
    if frame_count % 3 != 0:
        if ultimo_frame_out is not None:
            cv2.imshow("deteccion_prioridad", ultimo_frame_out)
        else:
            cv2.imshow("deteccion_prioridad", frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

        continue

    frame_out = frame.copy()
    mejor_objetivo = None
    mejor_score = 0
    detecciones = []

    # =========================
    # YOLO SEÑALES
    # =========================

    resultados_senales = modelo_senales(
        frame,
        imgsz=320,
        conf=0.60,   # subido para evitar falsas señales
        verbose=False
    )[0]

    for box in resultados_senales.boxes:
        cls_id = int(box.cls[0])
        conf = float(box.conf[0])
        nombre = modelo_senales.names[cls_id]

        x1, y1, x2, y2 = map(int, box.xyxy[0])
        area = (x2 - x1) * (y2 - y1)
        prioridad = prioridad_senal.get(nombre, 40)
        score = prioridad * area * conf

        detecciones.append(("senal", nombre, conf, x1, y1, x2, y2, score))

        if score > mejor_score:
            mejor_score = score
            mejor_objetivo = ("senal", nombre, conf, x1, y1, x2, y2, score)

    # =========================
    # YOLO OBSTACULOS
    # =========================

    resultados_obstaculos = modelo_obstaculos(
        frame,
        imgsz=320,
        conf=0.45,
        verbose=False
    )[0]

    for box in resultados_obstaculos.boxes:
        cls_id = int(box.cls[0])
        conf = float(box.conf[0])
        nombre = modelo_obstaculos.names[cls_id]

        if nombre not in clases_obstaculos:
            continue

        x1, y1, x2, y2 = map(int, box.xyxy[0])
        area = (x2 - x1) * (y2 - y1)
        prioridad = prioridad_obstaculo.get(nombre, 20)
        score = prioridad * area * conf

        detecciones.append(("obstaculo", nombre, conf, x1, y1, x2, y2, score))

        if score > mejor_score:
            mejor_score = score
            mejor_objetivo = ("obstaculo", nombre, conf, x1, y1, x2, y2, score)

    # =========================
    # SUAVIZADO DEL SCORE
    # =========================

    if mejor_objetivo is not None:
        tipo, nombre, conf, x1, y1, x2, y2, score = mejor_objetivo

        if ultimo_objetivo == (tipo, nombre):
            score_suavizado = alpha * score + (1 - alpha) * score_suavizado
        else:
            score_suavizado = score
            ultimo_objetivo = (tipo, nombre)

        mejor_objetivo = (tipo, nombre, conf, x1, y1, x2, y2, score_suavizado)
    else:
        ultimo_objetivo = None
        score_suavizado = 0

    # =========================
    # DIBUJAR DETECCIONES
    # =========================

    for tipo, nombre, conf, x1, y1, x2, y2, score in detecciones:
        if tipo == "senal":
            color = (255, 0, 0)
            texto = f"senal: {nombre} {conf:.2f}"
        else:
            color = (0, 255, 255)
            texto = f"obs: {nombre} {conf:.2f}"

        cv2.rectangle(frame_out, (x1, y1), (x2, y2), color, 2)

        cv2.putText(
            frame_out,
            texto,
            (x1, max(20, y1 - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            color,
            2
        )

    # =========================
    # OBJETIVO PRIORITARIO
    # =========================

    if mejor_objetivo is not None:
        tipo, nombre, conf, x1, y1, x2, y2, score = mejor_objetivo
        score_mostrar = round(score / 1000) * 1000

        cv2.rectangle(frame_out, (x1, y1), (x2, y2), (0, 0, 255), 3)

        cv2.putText(
            frame_out,
            f"PRIORIDAD: {tipo} {nombre}",
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 0, 255),
            2
        )

        cv2.putText(
            frame_out,
            f"score: {int(score_mostrar)}",
            (10, 50),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 0, 255),
            2
        )

        if frame_count % 30 == 0:
            print(f"Prioridad actual: {tipo} - {nombre} | score={int(score_mostrar)}")
    else:
        cv2.putText(
            frame_out,
            "Sin deteccion prioritaria",
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 0, 255),
            2
        )

    ultimo_frame_out = frame_out.copy()
    cv2.imshow("deteccion_prioridad", frame_out)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

    time.sleep(0.03)

cap.release()
cv2.destroyAllWindows()
