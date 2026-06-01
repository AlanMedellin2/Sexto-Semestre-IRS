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
        self.model_obs = YOLO("/home/ubuntu/ros2_ws/src/challenge_6/models/yolo11n.pt")

        self.sub = self.create_subscription(
            Image,
            "/camera/raw",
            self.image_callback,
            10
        )

        self.command_pub = self.create_publisher(String, "/yolo/command", 10)
        self.color_pub   = self.create_publisher(Float32, "/color", 10)
        self.debug_pub   = self.create_publisher(Image, "/yolo/debug", 10)

        self.prioridad_senal = {
            "stop": 300,
            "turn_left": 280,
            "turn_right": 280,
            "straight": 260,
            "roadwork_ahead": 240,
            "give_way": 220,
            "traffic_light": 180,
            "turn_around": 150
        }

        self.prioridad_obstaculo = {
            "car": 30,
            "bicycle": 25,
            "motorcycle": 25,
            "bus": 20,
            "truck": 20
        }

        # Redundancia: misma señal 3 veces antes de publicar instrucción
        self.last_candidate = "none"
        self.same_count = 0
        self.required_count = 3
        self.last_confirmed = "none"
        self.pending_command = None
        self.pending_at      = None

        self.frame_count = 0

        self.color_ranges = {
            "yellow": [
                (np.array([15, 60, 60]),  np.array([40, 255, 255])),  # amarillo amplio
                (np.array([18, 40, 80]),  np.array([38, 255, 255]))   # amarillo tenue
            ],
            "green": [
                (np.array([35, 60, 60]),  np.array([90, 255, 255])),  # verde amplio
                (np.array([40, 40, 40]),  np.array([85, 255, 255]))   # verde tenue
            ],
            "red": [
                (np.array([0,  80, 60]),  np.array([12, 255, 255])),  # rojo bajo
                (np.array([160, 80, 60]), np.array([180, 255, 255])), # rojo alto
                (np.array([0,  60, 60]),  np.array([8,  255, 255]))   # rojo muy bajo
            ]
        }

        self.color_values = {
            "yellow": 1.0,
            "green": 2.0,
            "red": 3.0
        }

        self.kernel = np.ones((5, 5), np.uint8)

        self.get_logger().info("YOLO decision node iniciado")

    def double_check_giro(self, frame, x1, y1, x2, y2):
        roi = frame[y1:y2, x1:x2]

        if roi.size == 0:
            return None

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, 80, 180)

        h, w = edges.shape

        if h < 20 or w < 20:
            return None

        left_zone = edges[:, :w // 3]
        center_zone = edges[:, w // 3:2 * w // 3]
        right_zone = edges[:, 2 * w // 3:]

        left_score = cv2.countNonZero(left_zone)
        center_score = cv2.countNonZero(center_zone)
        right_score = cv2.countNonZero(right_zone)

        total = left_score + center_score + right_score

        if total < 120:
            return None

        margen_absoluto = 120
        margen_ratio = 1.45

        if (
            right_score > left_score + margen_absoluto
            and right_score > center_score
            and right_score > left_score * margen_ratio
        ):
            return "turn_right"

        if (
            left_score > right_score + margen_absoluto
            and left_score > center_score
            and left_score > right_score * margen_ratio
        ):
            return "turn_left"

        return None

    def detectar_color_en_traffic_light(self, frame, x1, y1, x2, y2):
        roi = frame[y1:y2, x1:x2]

        if roi.size == 0:
            return 0.0, "none"

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        roi_area = roi.shape[0] * roi.shape[1]

        best_color = "none"
        best_value = 0.0
        best_area = 0.0

        for color_name, ranges in self.color_ranges.items():
            mask_total = np.zeros(hsv.shape[:2], dtype=np.uint8)

            for lower, upper in ranges:
                mask = cv2.inRange(hsv, lower, upper)
                mask_total = cv2.bitwise_or(mask_total, mask)

            mask_total = cv2.morphologyEx(mask_total, cv2.MORPH_OPEN, self.kernel)
            mask_total = cv2.morphologyEx(mask_total, cv2.MORPH_CLOSE, self.kernel)

            contours, _ = cv2.findContours(
                mask_total,
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE
            )

            color_area = 0.0

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area >= 30:
                    color_area += area

            if color_area > best_area:
                best_area = color_area
                best_color = color_name
                best_value = self.color_values[color_name]

        ratio = best_area / roi_area if roi_area > 0 else 0.0

        if best_area < 30 or ratio < 0.005:
            return 0.0, "none"

        return best_value, best_color

    def procesar_senales(self, frame, offset_x=0, zona="full"):
        resultados = []

        res = self.model_signs(
            frame,
            imgsz=416,
            conf=0.55,
            verbose=False
        )[0]

        for box in res.boxes:
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])
            name_yolo = self.model_signs.names[cls_id]
            name_final = name_yolo

            x1, y1, x2, y2 = map(int, box.xyxy[0])

            x1_global = x1 + offset_x
            x2_global = x2 + offset_x
            y1_global = y1
            y2_global = y2

            area = (x2 - x1) * (y2 - y1)

            if conf < 0.45:
                continue

            if name_yolo == "traffic_light" and conf < 0.45:
                continue

            if name_yolo == "traffic_light" and area < 400:
                continue

            if area < 150:
                continue

            if name_yolo in ["turn_left", "turn_right"]:
                edge_result = self.double_check_giro(frame, x1, y1, x2, y2)

                if edge_result is not None:
                    name_final = edge_result

            score = self.prioridad_senal.get(name_final, 100) * area * conf

            resultados.append(
                (
                    name_yolo,
                    name_final,
                    conf,
                    area,
                    score,
                    zona,
                    x1_global,
                    y1_global,
                    x2_global,
                    y2_global
                )
            )

        return resultados

    def publicar_comando_estable(self, candidate):
        now = time.time()

        if candidate == self.last_candidate:
            self.same_count += 1
        else:
            self.last_candidate = candidate
            self.same_count = 1
            self.pending_command = None
            self.pending_at      = None

        DELAY_30CM = 3.0

        if self.same_count >= self.required_count:
            if not hasattr(self, 'pending_command') or self.pending_command != candidate:
                self.pending_command = candidate
                self.pending_at      = now
                self.get_logger().info(
                    f"Senal '{candidate}' confirmada — ejecutando en {DELAY_30CM}s (30cm)")

        command = "none"
        if (hasattr(self, 'pending_command')
                and self.pending_command is not None
                and self.pending_at is not None
                and now - self.pending_at >= DELAY_30CM):
            command = self.pending_command
            self.pending_command = None
            self.pending_at      = None
            self.same_count      = 0
            self.last_candidate  = "none"

        msg = String()
        msg.data = command
        self.command_pub.publish(msg)

        return command

    def image_callback(self, msg):
        self.frame_count += 1

        # Para no saturar Rubik
        if self.frame_count % 3 != 0:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        h, w, _ = frame.shape

        # Señales y semáforo SOLO en la mitad derecha del frame
        right_half = frame[:, w // 2:]

        detecciones_signs = self.procesar_senales(
            right_half,
            offset_x=w // 2,
            zona="right"
        )

        mejor_score = 0
        mejor_command = "none"
        mejor_info = "none"

        traffic_light_bbox = None

        for (
            name_yolo,
            name_final,
            conf,
            area,
            score,
            zona,
            x1,
            y1,
            x2,
            y2
        ) in detecciones_signs:

            if name_final == "traffic_light":
                traffic_light_bbox = (x1, y1, x2, y2)

            if score > mejor_score:
                mejor_score = score
                mejor_command = name_final
                mejor_info = (
                    f"SENAL:{name_final} YOLO:{name_yolo} "
                    f"conf={conf:.2f} area={area} zona={zona}"
                )

        # Obstáculos con baja prioridad
        if self.frame_count % 18 == 0:
            res_obs = self.model_obs(
                frame,
                imgsz=256,
                conf=0.45,
                verbose=False
            )[0]

            for box in res_obs.boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                name = self.model_obs.names[cls_id]

                if name not in self.prioridad_obstaculo:
                    continue

                x1, y1, x2, y2 = map(int, box.xyxy[0])
                area = (x2 - x1) * (y2 - y1)

                if area < 300:
                    continue

                score = self.prioridad_obstaculo[name] * area * conf

                if score > mejor_score:
                    mejor_score = score
                    mejor_command = f"obstacle_{name}"
                    mejor_info = f"OBST:{name} conf={conf:.2f} area={area}"

        # Color SOLO dentro del bbox traffic_light
        color_value = 0.0
        color_name = "none"

        if traffic_light_bbox is not None:
            x1, y1, x2, y2 = traffic_light_bbox
            color_value, color_name = self.detectar_color_en_traffic_light(
                frame,
                x1,
                y1,
                x2,
                y2
            )

        color_msg = Float32()
        color_msg.data = float(color_value)
        self.color_pub.publish(color_msg)

        stable_command = self.publicar_comando_estable(mejor_command)

        self.get_logger().info(
            f"cand={mejor_command} stable={stable_command} "
            f"count={self.same_count}/{self.required_count} "
            f"color={color_name}:{color_value} | {mejor_info}"
        )

        # Debug visual con bounding boxes
        debug_frame = frame.copy()
        for (ny, nf, cf, ar, sc, zo, x1, y1, x2, y2) in detecciones_signs:
            color_box = (0, 255, 0) if nf != "traffic_light" else (0, 165, 255)
            cv2.rectangle(debug_frame, (x1, y1), (x2, y2), color_box, 2)
            label = f"{nf} {cf:.2f}"
            cv2.putText(debug_frame, label, (x1, max(y1-5, 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, color_box, 1)
        # Semáforo color
        if traffic_light_bbox is not None:
            x1, y1, x2, y2 = traffic_light_bbox
            cv2.putText(debug_frame, f"COLOR:{color_name}({color_value})",
                        (x1, y2+15), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255,255,0), 1)
        # Comando actual
        cv2.putText(debug_frame, f"CMD:{stable_command}", (5, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)
        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug_frame, encoding="bgr8"))


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
