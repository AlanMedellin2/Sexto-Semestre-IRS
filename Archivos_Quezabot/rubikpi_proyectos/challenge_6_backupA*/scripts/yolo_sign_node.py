#!/home/ubuntu/ros2_ws/src/challenge_6/yolo_ros_env/bin/python

import cv2
import rclpy

from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO


class YoloSignNode(Node):

    def __init__(self):
        super().__init__("yolo_sign_node")

        self.bridge = CvBridge()

        self.model = YOLO("/home/ubuntu/ros2_ws/src/challenge_6/models/best.pt")

        self.sub = self.create_subscription(
            Image,
            "/camera/raw",
            self.image_callback,
            10
        )

        self.command_pub = self.create_publisher(String, "/yolo/command", 10)

        self.prioridad_senal = {
            "stop": 300,
            "turn_left": 280,
            "turn_right": 280,
            "straight": 260,
            "speed_limit_30": 240
        }

        self.last_candidate = "none"
        self.same_count = 0
        self.required_count = 3

        self.frame_count = 0

        self.get_logger().info("YOLO sign node iniciado: redundancia 3 detecciones")

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

    def procesar_frame(self, frame, offset_x=0):
        detecciones = []

        res = self.model(
            frame,
            imgsz=416,
            conf=0.55,
            verbose=False
        )[0]

        for box in res.boxes:
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])
            name_yolo = self.model.names[cls_id]
            name_final = name_yolo

            if name_yolo == "traffic_light":
                continue

            if name_yolo not in self.prioridad_senal:
                continue

            x1, y1, x2, y2 = map(int, box.xyxy[0])
            area = (x2 - x1) * (y2 - y1)

            if conf < 0.55:
                continue

            if area < 250:
                continue

            if name_yolo in ["turn_left", "turn_right"]:
                edge_result = self.double_check_giro(frame, x1, y1, x2, y2)

                if edge_result is not None:
                    name_final = edge_result

            score = self.prioridad_senal.get(name_final, 100) * area * conf

            detecciones.append((name_final, name_yolo, conf, area, score))

        return detecciones

    def publicar_estable(self, candidate):
        if candidate == self.last_candidate:
            self.same_count += 1
        else:
            self.last_candidate = candidate
            self.same_count = 1

        msg = String()

        if candidate != "none" and self.same_count >= self.required_count:
            msg.data = candidate
        else:
            msg.data = "none"

        self.command_pub.publish(msg)
        return msg.data

    def image_callback(self, msg):
        self.frame_count += 1

        if self.frame_count % 6 != 0:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        h, w, _ = frame.shape

        detecciones = self.procesar_frame(frame)

        # Fallback lateral
        if len(detecciones) == 0:
            left_crop = frame[:, :w // 2]
            right_crop = frame[:, w // 2:]

            detecciones.extend(self.procesar_frame(left_crop, offset_x=0))
            detecciones.extend(self.procesar_frame(right_crop, offset_x=w // 2))

        mejor_score = 0
        mejor = "none"
        info = "none"

        for name_final, name_yolo, conf, area, score in detecciones:
            if score > mejor_score:
                mejor_score = score
                mejor = name_final
                info = f"{name_final} YOLO:{name_yolo} conf={conf:.2f} area={area}"

        publicado = self.publicar_estable(mejor)

        self.get_logger().info(
            f"cand={mejor} publicado={publicado} "
            f"count={self.same_count}/{self.required_count} | {info}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = YoloSignNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
