#!/usr/bin/env python3
"""
yolo_double_detector_fixed.py
==============================
Versión optimizada para Rubik Pi 3.

CAMBIOS vs el original:
  1. NO abre la cámara — se suscribe a /camera/raw (ya la abre line_detector.py)
     → elimina conflicto de /dev/video0 y libera CPU
  2. Publica /yolo/command  (era /yolo/priority — no lo leía nadie)
  3. Publica /sign_area     (Float32) para el sistema difuso
  4. USE_OBSTACLES=False por defecto — el modelo de obstáculos es el más pesado
     cámbialo a True solo si la Rubik no se pasma
  5. Procesa 1 de cada N frames (SKIP_FRAMES=3) para no saturar

Todo lo demás idéntico: Hough+zonas, antiflicker, prioridades, thresholds.
"""

import os
import cv2
import numpy as np
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32
from ultralytics import YOLO


# ── Configuración rápida ──────────────────────────────────────────────────────
USE_OBSTACLES = False       # True solo si la Rubik aguanta dos modelos
SKIP_FRAMES   = 3           # procesa 1 de cada 3 frames del topic
SIGN_IMGSZ    = 192         # baja a 160 si sigue lento
OBS_IMGSZ     = 128

SIGN_MODEL_CANDIDATES = [
    "/home/ubuntu/models/best_turnaround.pt",
    "/home/ubuntu/ros2_ws/src/challenge_6/models/best_turnaround.pt",
    "/home/ubuntu/ros2_ws/src/challenge_6/models/best.pt",
]
OBSTACLE_MODEL_CANDIDATES = [
    "/home/ubuntu/ros2_ws/src/challenge_6/models/yolo11n.pt",
    "/home/ubuntu/yolo11n.pt",
]

CLASES_FLECHAS = ["straight", "turn_right", "turn_left", "turn_around"]


def first_existing(paths):
    for p in paths:
        if os.path.exists(p):
            return p
    return paths[0]


# ── Validación Hough + zonas: IDÉNTICA al original ────────────────────────────

def extraer_mascara_blanca(roi):
    if roi.size == 0:
        return None
    roi = cv2.resize(roi, (120, 120))
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array([0, 0, 105]), np.array([180, 105, 255]))
    k = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
    return mask


def validar_flecha_hough_zonas(frame, box):
    x1, y1, x2, y2 = map(int, box.xyxy[0])
    hf, wf = frame.shape[:2]
    px = int((x2-x1)*0.08); py = int((y2-y1)*0.08)
    x1 = max(0,x1-px); y1 = max(0,y1-py)
    x2 = min(wf,x2+px); y2 = min(hf,y2+py)

    roi  = frame[y1:y2, x1:x2]
    mask = extraer_mascara_blanca(roi)
    if mask is None:
        return None, 0.0

    h, w  = mask.shape
    total = float(np.sum(mask)/255.0)
    if total < 35:
        return None, 0.0

    L = float(np.sum(mask[:, :w//3])/255.0)            / total
    C = float(np.sum(mask[:, w//3:2*w//3])/255.0)      / total
    R = float(np.sum(mask[:, 2*w//3:])/255.0)           / total
    T = float(np.sum(mask[:h//3,:])/255.0)              / total
    M = float(np.sum(mask[h//3:2*h//3,:])/255.0)        / total
    B = float(np.sum(mask[2*h//3:,:])/255.0)            / total

    edges = cv2.Canny(mask, 50, 150)
    lines = cv2.HoughLinesP(edges, 1, np.pi/180,
                            threshold=14, minLineLength=10, maxLineGap=8)
    H = V = D = n = 0
    if lines is not None:
        for l in lines:
            xa,ya,xb,yb = l[0]
            dx=xb-xa; dy=yb-ya
            if (dx*dx+dy*dy)**0.5 < 8: continue
            n += 1
            a = np.degrees(np.arctan2(dy,dx))
            if a<-90: a+=180
            elif a>90: a-=180
            if   abs(a)<25: H+=1
            elif abs(a)>65: V+=1
            else:            D+=1

    zones = sum([L>0.20, C>0.20, R>0.20, T>0.20, M>0.20, B>0.20])

    sr = max(0.0, R-L)*2.0 + (0.35 if R>0.38 else 0)
    sl = max(0.0, L-R)*2.0 + (0.35 if L>0.38 else 0)
    ss = (0.65 if T>0.30 and C>0.22 else 0) + (0.20 if V>=1 else 0) \
       + (0.15 if C>max(L,R) else 0)
    sa = (0.75 if zones>=5 else 0) + (0.25 if D>=2 else 0) \
       + (0.20 if n>=5 and H<=2 else 0)

    scores = {"turn_right":sr,"turn_left":sl,"straight":ss,"turn_around":sa}
    best   = max(scores, key=scores.get)
    bscore = scores[best]

    return (best, bscore) if bscore >= 0.35 else (None, bscore)


# ── Nodo ──────────────────────────────────────────────────────────────────────

class YoloDoubleDetector(Node):

    def __init__(self):
        super().__init__('yolo_double_detector')

        sign_path = first_existing(SIGN_MODEL_CANDIDATES)
        self.get_logger().info(f"Modelo señales: {sign_path}")
        self.model_signs = YOLO(sign_path)

        self.model_obs = None
        if USE_OBSTACLES:
            obs_path = first_existing(OBSTACLE_MODEL_CANDIDATES)
            self.get_logger().info(f"Modelo obstáculos: {obs_path}")
            self.model_obs = YOLO(obs_path)

        self.bridge = CvBridge()

        # ── Publishers ────────────────────────────────────────────────────
        self.pub_cmd  = self.create_publisher(String,  '/yolo/command', 10)
        self.pub_area = self.create_publisher(Float32, '/sign_area',    10)

        # ── Subscriber a /camera/raw (ya lo publica line_detector.py) ─────
        self.create_subscription(Image, '/camera/raw', self._cb_image, 10)

        self.thresholds = {
            "stop":0.45,"straight":0.18,"turn_right":0.18,
            "turn_left":0.18,"speed_limit_30":0.45,
            "traffic_light":0.50,"turn_around":0.75,
        }
        self.prioridad = {
            "stop":12,"turn_left":10,"turn_right":10,
            "straight":9,"speed_limit_30":7,
            "traffic_light":6,"turn_around":4,
        }
        self.obs_clases = {
            "person":8,"car":8,"truck":8,
            "bus":8,"bicycle":8,"motorcycle":8,
        }

        self.ultima     = "none"
        self.mem_count  = 0
        self.mem_frames = 3
        self.frame_n    = 0
        self.obs_mem    = []

        self.get_logger().info(
            f"YoloDoubleDetector listo — suscrito a /camera/raw "
            f"(skip={SKIP_FRAMES}, obstáculos={'ON' if USE_OBSTACLES else 'OFF'})")

    def _cb_image(self, msg: Image):
        self.frame_n += 1
        if self.frame_n % SKIP_FRAMES != 0:
            return

        frame      = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w       = frame.shape[:2]
        frame_area = float(h * w)

        mejor_score = 0.0
        mejor_texto = "none"
        mejor_area  = 0.0

        # ── Señales ───────────────────────────────────────────────────────
        res = self.model_signs(frame, imgsz=SIGN_IMGSZ, conf=0.18, verbose=False)[0]

        detecciones = []
        for box in res.boxes:
            cls_id      = int(box.cls[0])
            conf        = float(box.conf[0])
            nombre_yolo = self.model_signs.names[cls_id]
            nombre_final = nombre_yolo
            val_score    = 0.0

            if nombre_yolo in CLASES_FLECHAS:
                validacion, val_score = validar_flecha_hough_zonas(frame, box)
                if validacion is not None:
                    nombre_final = validacion

            if conf < self.thresholds.get(nombre_final, 0.50):
                continue

            detecciones.append((nombre_final, conf, box))

        # Filtro turn_around si hay dirección clara
        hay_dir = any(n in ("straight","turn_left","turn_right")
                      for n,_,_ in detecciones)
        if hay_dir:
            detecciones = [d for d in detecciones if d[0] != "turn_around"]

        for nombre, conf, box in detecciones:
            x1,y1,x2,y2 = map(int, box.xyxy[0])
            area  = float((x2-x1)*(y2-y1))
            score = area * conf * self.prioridad.get(nombre, 1)
            if score > mejor_score:
                mejor_score = score
                mejor_texto = f"senal:{nombre}"
                mejor_area  = area

        # ── Obstáculos (opcional) ─────────────────────────────────────────
        if USE_OBSTACLES and self.frame_n % (SKIP_FRAMES * 4) == 0:
            res_obs = self.model_obs(
                frame, imgsz=OBS_IMGSZ, conf=0.45, verbose=False)[0]
            self.obs_mem = []
            for box in res_obs.boxes:
                cls_id = int(box.cls[0])
                conf   = float(box.conf[0])
                nombre = self.model_obs.names[cls_id]
                if nombre not in self.obs_clases: continue
                x1,y1,x2,y2 = map(int, box.xyxy[0])
                area = float((x2-x1)*(y2-y1))
                if area < 300: continue
                self.obs_mem.append((nombre, conf, area))

        for nombre, conf, area in self.obs_mem:
            score = area * conf * self.obs_clases[nombre]
            if score > mejor_score:
                mejor_score = score
                mejor_texto = f"obstaculo:{nombre}"
                mejor_area  = area

        # ── Antiflicker ───────────────────────────────────────────────────
        if mejor_texto != "none":
            self.ultima    = mejor_texto
            self.mem_count = self.mem_frames
        elif self.mem_count > 0:
            mejor_texto = self.ultima
            self.mem_count -= 1
            mejor_area  = 0.0   # área de memoria no es confiable

        # ── Publicar ──────────────────────────────────────────────────────
        cmd_msg      = String()
        cmd_msg.data = mejor_texto
        self.pub_cmd.publish(cmd_msg)

        area_norm     = mejor_area / frame_area if frame_area > 0 else 0.0
        area_msg      = Float32()
        area_msg.data = float(area_norm)
        self.pub_area.publish(area_msg)

        self.get_logger().info(
            f"cmd={mejor_texto}  area={area_norm:.4f}",
            throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = YoloDoubleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
