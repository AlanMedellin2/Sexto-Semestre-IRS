#!/home/ubuntu/ros2_ws/src/challenge_6/yolo_ros_env/bin/python

import time
import cv2
import numpy as np
import rclpy

from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32
from cv_bridge import CvBridge
from ultralytics import YOLO

class YoloDecisionNode(Node):

    def __init__(self):
        super().__init__("yolo_decision_node")

        self.bridge = CvBridge()
        self.model_signs = YOLO("/home/ubuntu/ros2_ws/src/challenge_6/models/best_v3.pt")

        self.sub         = self.create_subscription(Image, "/camera/raw", self.image_callback, 10)
        self.command_pub = self.create_publisher(String,  "/yolo/command", 10)
        self.color_pub   = self.create_publisher(Float32, "/color",        10)
        self.debug_pub   = self.create_publisher(Image,   "/yolo/debug",   10)
        self.area_pub    = self.create_publisher(Float32, "/yolo/sign_area", 10)

        self.prioridad_senal = {
            "stop": 300, "turn_left": 280, "turn_right": 280,
            "straight": 260, "roadwork_ahead": 240, "give_way": 220,
            "traffic_light": 180, "turn_around": 150
        }

        self.frame_count      = 0
        self.frame_color      = 0   # procesa color cada 2 frames
        self.last_published   = "none"
        self.published_at     = 0.0
        self.publish_cooldown = 6.0

        self.color_ranges = {
            "yellow": [
                (np.array([15, 60,  60]),  np.array([40, 255, 255])),
                (np.array([18, 40,  80]),  np.array([38, 255, 255])),
                (np.array([20, 100, 100]), np.array([35, 255, 255])),
                (np.array([10, 50,  50]),  np.array([45, 255, 255]))
            ],
            "green": [
                (np.array([35, 60,  60]),  np.array([90, 255, 255])),
                (np.array([40, 40,  40]),  np.array([85, 255, 255])),
                (np.array([36, 80,  50]),  np.array([88, 255, 255])),
                (np.array([50, 30,  30]),  np.array([80, 255, 255])),
                (np.array([38, 50,  80]),  np.array([82, 220, 255]))
            ],
            "red": [
                (np.array([0,   80, 60]),  np.array([12, 255, 255])),
                (np.array([160, 80, 60]),  np.array([180,255, 255])),
                (np.array([0,   60, 60]),  np.array([8,  255, 255])),
                (np.array([170, 50, 50]),  np.array([180,255, 255])),
                (np.array([0,   50, 80]),  np.array([10, 255, 255]))
            ]
        }

        self.color_values = {"yellow": 1.0, "green": 2.0, "red": 3.0}
        self.kernel = np.ones((5, 5), np.uint8)

        # Último color conocido del semáforo (persiste entre frames)
        self.last_color_value = 0.0
        self.last_color_name  = "none"

        import os
        template_dir = "/home/ubuntu/ros2_ws/src/challenge_6/templates"
        self.templates = {}
        for name in ["turn_left", "turn_right", "straight"]:
            path = os.path.join(template_dir, f"{name}.png")
            if os.path.exists(path):
                img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
                self.templates[name] = cv2.resize(img, (64, 64))
                self.get_logger().info(f"Template cargado: {name}")

        self.get_logger().info("YOLO decision node iniciado")

    def template_check(self, frame, x1, y1, x2, y2):
        if not self.templates:
            return None
        roi = frame[y1:y2, x1:x2]
        if roi.size == 0 or roi.shape[0] < 20 or roi.shape[1] < 20:
            return None
        gray_r = cv2.resize(cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY), (64, 64))
        best_score, best_name = -1, None
        for name, tmpl in self.templates.items():
            score = float(cv2.matchTemplate(gray_r, tmpl, cv2.TM_CCOEFF_NORMED)[0][0])
            if score > best_score:
                best_score, best_name = score, name
        return best_name if best_score > 0.45 else None

    def double_check_giro(self, frame, x1, y1, x2, y2):
        roi = frame[y1:y2, x1:x2]
        if roi.size == 0: return None
        edges = cv2.Canny(cv2.GaussianBlur(
            cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY), (5,5), 0), 80, 180)
        h, w = edges.shape
        if h < 20 or w < 20: return None
        l = cv2.countNonZero(edges[:, :w//3])
        c = cv2.countNonZero(edges[:, w//3:2*w//3])
        r = cv2.countNonZero(edges[:, 2*w//3:])
        if l+c+r < 120: return None
        if r > l+120 and r > c and r > l*1.45: return "turn_right"
        if l > r+120 and l > c and l > r*1.45: return "turn_left"
        return None

    def detectar_color(self, frame, x1, y1, x2, y2):
        roi = frame[y1:y2, x1:x2]
        if roi.size == 0: return 0.0, "none"
        hsv      = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        roi_area = roi.shape[0] * roi.shape[1]
        best_color, best_value, best_area = "none", 0.0, 0.0
        for color_name, ranges in self.color_ranges.items():
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for lo, hi in ranges:
                mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lo, hi))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  self.kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)
            area = sum(cv2.contourArea(c) for c in
                       cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_SIMPLE)[0]
                       if cv2.contourArea(c) >= 20)
            if area > best_area:
                best_area, best_color, best_value = area, color_name, self.color_values[color_name]
        if best_area < 20 or best_area / roi_area < 0.003:
            return 0.0, "none"
        return best_value, best_color

    def procesar_senales(self, frame):
        resultados = []
        # Conf baja para no perder el semáforo
        res = self.model_signs(frame, imgsz=416, conf=0.35, verbose=False)[0]
        for box in res.boxes:
            cls_id    = int(box.cls[0])
            conf      = float(box.conf[0])
            name_yolo = self.model_signs.names[cls_id]
            name_final = name_yolo
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            area = (x2-x1)*(y2-y1)

            # Filtros por tipo
            if name_yolo == "traffic_light":
                if conf < 0.30 or area < 150: continue
            else:
                if conf < 0.45 or area < 150: continue

            if name_yolo in ["turn_left", "turn_right", "straight"]:
                tmpl = self.template_check(frame, x1, y1, x2, y2)
                if tmpl:
                    name_final = tmpl
                else:
                    edge = self.double_check_giro(frame, x1, y1, x2, y2)
                    if edge: name_final = edge

            score = self.prioridad_senal.get(name_final, 100) * area * conf
            resultados.append((name_yolo, name_final, conf, area, score,
                                x1, y1, x2, y2))
        return resultados

    def publicar_comando(self, candidate):
        now = time.time()
        msg = String()
        if (candidate == "none" or candidate == "traffic_light" or
                (candidate == self.last_published and
                 now - self.published_at < self.publish_cooldown)):
            msg.data = "none"
            self.command_pub.publish(msg)
            return "none"
        self.last_published = candidate
        self.published_at   = now
        msg.data = candidate
        self.command_pub.publish(msg)
        self.get_logger().info(f"COMANDO: {candidate}")
        return candidate

    def image_callback(self, msg):
        self.frame_count += 1

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # ---- DETECCIÓN DE COLOR — cada 2 frames (rápido) ----
        self.frame_color += 1
        if self.frame_color % 6 == 0:
            # Busca solo traffic_light con conf muy baja
            res_tl = self.model_signs(frame, imgsz=320, conf=0.25, verbose=False)[0]
            for box in res_tl.boxes:
                name = self.model_signs.names[int(box.cls[0])]
                if name == "traffic_light":
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    area = (x2-x1)*(y2-y1)
                    if area >= 100:
                        cv, cn = self.detectar_color(frame, x1, y1, x2, y2)
                        if cn != "none":
                            self.last_color_value = cv
                            self.last_color_name  = cn
                        break

            color_msg = Float32()
            color_msg.data = self.last_color_value
            self.color_pub.publish(color_msg)

        # ---- DETECCIÓN DE SEÑALES — cada 6 frames ----
        if self.frame_count % 6 != 0:
            return

        detecciones = self.procesar_senales(frame)

        mejor_score, mejor_command, mejor_info = 0, "none", "none"
        traffic_light_bbox = None
        mejor_area = 0.0

        for (ny, nf, cf, ar, sc, x1, y1, x2, y2) in detecciones:
            if ny == "traffic_light":
                traffic_light_bbox = (x1, y1, x2, y2)
            if nf != "traffic_light" and sc > mejor_score:
                mejor_score   = sc
                mejor_command = nf
                mejor_info    = f"{nf} conf={cf:.2f} area={ar}"
                mejor_area    = float(ar)

        # Publica área de la mejor señal (para difuso)
        area_msg = Float32()
        area_msg.data = mejor_area
        self.area_pub.publish(area_msg)

        stable_command = self.publicar_comando(mejor_command)

        self.get_logger().info(
            f"cand={mejor_command} CMD={stable_command} "
            f"color={self.last_color_name}:{self.last_color_value} | {mejor_info}"
        )

        # Debug visual
        debug_frame = frame.copy()
        for (ny, nf, cf, ar, sc, x1, y1, x2, y2) in detecciones:
            col = (0, 165, 255) if ny == "traffic_light" else (0, 255, 0)
            cv2.rectangle(debug_frame, (x1, y1), (x2, y2), col, 2)
            cv2.putText(debug_frame, f"{nf} {cf:.2f}",
                        (x1, max(y1-5, 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1)
        if traffic_light_bbox:
            x1, y1, x2, y2 = traffic_light_bbox
            cv2.putText(debug_frame,
                        f"COLOR:{self.last_color_name}({self.last_color_value})",
                        (x1, y2+15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 1)
        cv2.putText(debug_frame, f"CMD:{stable_command}", (5, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        small = cv2.resize(debug_frame, (160, 120))
        self.debug_pub.publish(
            self.bridge.cv2_to_imgmsg(small, encoding="bgr8"))


def main(args=None):
    rclpy.init(args=args)
    node = YoloDecisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
