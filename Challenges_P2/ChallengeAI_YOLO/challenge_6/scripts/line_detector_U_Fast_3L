#!/usr/bin/env python3
from collections import deque
import cv2 as cv
import numpy as np
import rclpy

from rclpy.node import Node
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data


class LineDetector(Node):

    def __init__(self):
        super().__init__('line_detector')

        self.publisher_ = self.create_publisher(Int32, '/line_error', 10)
        self.img_pub    = self.create_publisher(Image, '/image_raw', 10)
        self.line_ufast_h = self.create_publisher(Image, '/camera/line_ufast_h', 10)
        self.raw_pub    = self.create_publisher(Image, '/camera/raw',   10)

        self.bridge = CvBridge()

        """
        self.cap = cv.VideoCapture('/dev/video0', cv.CAP_V4L2)

        # Resolución baja para streaming fluido
        self.cap.set(cv.CAP_PROP_FRAME_WIDTH,  320)
        self.cap.set(cv.CAP_PROP_FRAME_HEIGHT, 240)
        self.cap.set(cv.CAP_PROP_FPS, 30)

        if not self.cap.isOpened():
            self.get_logger().error("Cannot open camera /dev/video2")
            exit()
        """

        self.H_history    = deque(maxlen=10)
        self.error_history = deque(maxlen=5)
        self.last_error   = 0
        
        #---Párametros para detección por filas---
        self.num_row_anchors = 9
        self.band_height = 8

        self.min_candidate_width = 3
        self.max_candidate_width = 70
        self.min_points_for_lane = 3

        self.last_lane_points = []
        self.target_lane = "center"
        self.debug_windows = True

    #   self.timer = self.create_timer(0.033, self.process_frame)
        self.image_sub = self.create_subscription(
        Image,
        '/camera/raw',
        self.image_callback,
        qos_profile_sensor_data
        )

        self.get_logger().info("LineDetector Hough 320x240 iniciado")

    # ------------------------------------------------------------------
    def get_threshold(self, H):
        if   H < 110: return 100
        elif H < 135: return 140
        elif H < 150: return 140
        elif H < 155: return 115
        elif H < 165: return 120
        elif H < 170: return 125
        elif H < 190: return 130
        elif H < 200: return 140
        else:         return 160

    # ------------------------------------------------------------------
    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Error convirtiendo imagen: {e}")
            return

        self.process_frame(frame)
    #-------------------------------------------------------------------
    def process_frame(self, frame):
         
        h, w = frame.shape[:2]

        # ---- ROI recortando arriba y abajo ------------------------
        # y1 más pequeño = el ROI empieza más arriba
        # y2 menor que h = recortas la parte más baja de la imagen

        y1 = int(h * 0.20)   # empieza al 35% de la altura
        y2 = int(h * 0.80)   # termina al 90%, recorta el 10% inferior

        # Para ver todo el ancho:
        x1 = 0
        x2 = w

        roi = frame[y1:y2, x1:x2]

        roi_h, roi_w = roi.shape[:2]
        debug = roi.copy()

        # ---- Brillo adaptativo ------------------------------------
        H_roi = roi[int(roi_h * 0.5):, int(roi_w * 0.3):int(roi_w * 0.7)]
        hsv   = cv.cvtColor(H_roi, cv.COLOR_BGR2HSV)
        V_mean = np.mean(hsv[:, :, 2])
        self.H_history.append(V_mean)
        V_smooth = np.mean(self.H_history)
        cutting  = 105

        # ---- Binarización -----------------------------------------
        gray    = cv.cvtColor(roi, cv.COLOR_BGR2GRAY)
        blurred = cv.GaussianBlur(gray, (5, 5), 0)
        _, binary = cv.threshold(blurred, cutting, 255, cv.THRESH_BINARY_INV)

        # ---- Trapecio alargado centrado ---------------------------
        top_w = int(roi_w * 0.90) #ancho
        bottom_w = int(roi_w * 0.90) #forma del trapecio
        top_y = int(roi_h * 0.40) #altura
        bottom_y = roi_h

        trap = np.array([[
            (int(roi_w * 0.10), int(roi_h * 0.35)),
            (int(roi_w * 0.80), int(roi_h * 0.35)),#punto 2
            (int(roi_w * 0.98), int(roi_h * 0.60)), #punto 3
            (int(roi_w * 0.99), int(roi_h * 0.95)),
            (int(roi_w * 0.01), int(roi_h * 0.95)),
            (int(roi_w * 0.01), int(roi_h * 0.50)) #punto 6
        ]], dtype=np.int32)

        mask = np.zeros((roi_h, roi_w), dtype=np.uint8)
        cv.fillPoly(mask, trap, 255)
        masked = cv.bitwise_and(binary, binary, mask=mask)

        # ---- Morfología -------------------------------------------
# ---- Morfología -------------------------------------------
        k_open = np.ones((3, 3), np.uint8)
        morph = cv.morphologyEx(masked, cv.MORPH_OPEN, k_open, iterations=1)

        k_close = np.ones((5, 3), np.uint8)
        morph = cv.morphologyEx(morph, cv.MORPH_CLOSE, k_close, iterations=1)

        # ---- Hough ------------------------------------------------
        #"""""
        edges = cv.Canny(morph, 50, 150)
        lines = cv.HoughLinesP(edges, 1, np.pi/180,
                               threshold=125,
                               minLineLength=1,
                               maxLineGap=25)
        
        ref_x   = roi_w // 2
        error_x = self.last_error
        

        cv.polylines(debug, trap, True, (255, 0, 255), 1)
        cv.line(debug, (ref_x, 0), (ref_x, roi_h), (255, 0, 0), 1)

        if lines is not None:
            valid = []
            for l in lines:
                x1, y1, x2, y2 = l[0]
                angle = abs(np.degrees(np.arctan2(y2-y1, x2-x1)))
                if 20 < angle < 160:
                    valid.append((x1, y1, x2, y2))

            if valid:
                total_len  = 0.0
                wx, wy     = 0.0, 0.0
                for x1, y1, x2, y2 in valid:
                    mx  = (x1+x2)/2.0
                    my  = (y1+y2)/2.0
                    seg = np.hypot(x2-x1, y2-y1)
                    wx += mx*seg;  wy += my*seg
                    total_len += seg
                    cv.line(debug, (x1,y1), (x2,y2), (0,255,0), 1)

                cx = wx / total_len
                cy = wy / total_len

                error_x = int(cx - ref_x)
                self.error_history.append(error_x)
                error_x = int(np.mean(self.error_history))
                self.last_error = error_x

                cv.circle(debug, (int(cx), int(cy)), 5, (0,255,255), -1)
                cv.line(debug, (ref_x, roi_h), (int(cx), int(cy)), (0,255,255), 1)
        else:
            cv.putText(debug, "Sin linea", (5, roi_h-5),
                       cv.FONT_HERSHEY_SIMPLEX, 0.4, (0,0,255), 1)
                    
        #-----------Row anchors-------------------------------------------------------------

                #----------- Row anchors para detectar 3 líneas -----------------------------
        #""""" 
        
        # Bordes solo para debug
        edges = cv.Canny(morph, 50, 150)

        ref_x = roi_w // 2
        error_x = self.last_error
         

        # Debug visual base
        cv.polylines(debug, trap, True, (255, 0, 255), 1)
        cv.line(debug, (ref_x, 0), (ref_x, roi_h), (255, 0, 0), 1)

        row_anchors = np.linspace(
            int(roi_h * 0.35),
            int(roi_h * 0.95),
            self.num_row_anchors
        ).astype(int)

        # Aquí guardamos todos los candidatos encontrados por fila
        three_candidates = []

        for y in row_anchors:
            band_h = self.band_height

            y1 = max(0, int(y - band_h // 2))
            y2 = min(roi_h, int(y + band_h // 2))

            band = morph[y1:y2, :]

            # Dibujar row anchor en debug
            cv.line(debug, (0, int(y)), (roi_w, int(y)), (80, 80, 80), 1)

            # Sumamos columnas activas dentro de la banda
            col_sum = np.sum(band > 0, axis=0)

            # Más estricto que col_sum > 1 para reducir ruido
            min_pixels_in_col = int(self.band_height * 0.50)
            active_cols = np.where(col_sum >= min_pixels_in_col)[0]

            if len(active_cols) == 0:
                continue

            # Separar grupos de columnas continuas
            splits = np.where(np.diff(active_cols) > 6)[0] + 1
            groups = np.split(active_cols, splits)

            candidates = []

            for g in groups:
                if len(g) < self.min_candidate_width:
                    continue

                x_min = int(g[0])
                x_max = int(g[-1])
                width = x_max - x_min

                if width > self.max_candidate_width:
                    continue

                cx_candidate = int(np.mean(g))
                candidates.append(cx_candidate)

            if not candidates:
                continue

            # Guardamos TODOS los candidatos, no solo uno
            three_candidates.append((int(y), candidates))

            # Dibujamos todos los candidatos detectados
            for cx_candidate in candidates:
                cv.circle(debug, (cx_candidate, int(y)), 3, (0, 255, 255), -1)

        # ---- Separar candidatos en izquierda, centro y derecha ----

        self.left_line = []
        self.center_line = []
        self.right_line = []

        for y, candidates in three_candidates:
            for cx in candidates:
                if cx < roi_w * 0.33:
                    self.left_line.append((cx, y))
                    cv.circle(debug, (cx, y), 4, (255, 0, 0), -1)

                elif cx < roi_w * 0.66:
                    self.center_line.append((cx, y))
                    cv.circle(debug, (cx, y), 4, (0, 255, 255), -1)

                else:
                    self.right_line.append((cx, y))
                    cv.circle(debug, (cx, y), 4, (0, 0, 255), -1)

        # ---- Elegir qué línea seguir -------------------------------

        if self.target_lane == "left":
            target_points = self.left_line

        elif self.target_lane == "right":
            target_points = self.right_line

        else:
            target_points = self.center_line

        # ---- Calcular error usando la línea objetivo ---------------

        if len(target_points) >= self.min_points_for_lane:
            pts = np.array(target_points)

            # Damos más peso a las filas inferiores porque están más cerca del robot
            weights = pts[:, 1] / max(1, roi_h)

            cx = int(np.average(pts[:, 0], weights=weights))
            cy = int(np.average(pts[:, 1], weights=weights))

            error_x = int(cx - ref_x)

            self.error_history.append(error_x)
            error_x = int(np.mean(self.error_history))
            self.last_error = error_x
            self.last_lane_points = target_points

            cv.circle(debug, (cx, cy), 7, (0, 255, 255), -1)
            cv.line(debug, (ref_x, roi_h), (cx, cy), (0, 255, 255), 1)

        else:
            error_x = self.last_error
            cv.putText(debug, f"Sin puntos en {self.target_lane}", (5, roi_h - 5),
                       cv.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)

        # ---- Publicar error ---------------------------------------
        msg = Int32()
        msg.data = error_x
        self.publisher_.publish(msg)

        # ---- HUD compacto -----------------------------------------
        cv.putText(debug, f"E:{error_x}", (3, 15),
                   cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

        cv.putText(debug, f"V:{V_smooth:.0f} C:{cutting}", (3, 30),
                   cv.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1)

        cv.putText(debug, f"L:{len(self.left_line)} C:{len(self.center_line)} R:{len(self.right_line)}", 
           (3, 45),
           cv.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1)

        # ---- Debug local con OpenCV -------------------------------
        if self.debug_windows:
            cv.imshow('1_frame_original', frame)
            cv.imshow('2_roi', roi)
            #cv.imshow('3_binary', binary)
            #cv.imshow('4_masked', masked)
            cv.imshow('5_morph', morph)
            cv.imshow('6_edges', edges)
            cv.imshow('7_debug_final', debug)

            key = cv.waitKey(1) & 0xFF
            if key == ord('q'):
                self.get_logger().info("Cerrando por tecla q")
                if rclpy.ok():
                    rclpy.shutdown()

        self.get_logger().info(
            f'Error: {error_x} | Brillo: {V_smooth:.1f} | Cutting: {cutting} | '
            f'L:{len(self.left_line)} C:{len(self.center_line)} R:{len(self.right_line)} | '
            f'Objetivo:{self.target_lane}'
        )

        self.img_pub.publish(self.bridge.cv2_to_imgmsg(debug, encoding='bgr8'))
        self.line_ufast_h.publish(self.bridge.cv2_to_imgmsg(debug, encoding='bgr8'))

    def destroy_node(self):
        cv.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LineDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
