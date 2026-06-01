#!/usr/bin/env python3
from collections import deque
import cv2 as cv
import numpy as np
import rclpy

from rclpy.node import Node
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class LineDetector(Node):

    def __init__(self):
        super().__init__('line_detector')

        self.publisher_ = self.create_publisher(Int32, '/line_error', 10)
        self.img_pub    = self.create_publisher(Image, '/camera/image', 10)
        self.raw_pub    = self.create_publisher(Image, '/camera/raw',   10)

        self.bridge = CvBridge()

        self.cap = cv.VideoCapture('/dev/video0', cv.CAP_V4L2)

        # Resolución baja para streaming fluido
        self.cap.set(cv.CAP_PROP_FRAME_WIDTH,  320)
        self.cap.set(cv.CAP_PROP_FRAME_HEIGHT, 240)
        self.cap.set(cv.CAP_PROP_FPS, 30)

        if not self.cap.isOpened():
            self.get_logger().error("Cannot open camera /dev/video2")
            exit()

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
        self.debug_windows = True

        self.timer = self.create_timer(0.033, self.process_frame)
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
    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning("Cannot receive frame")
            return

        h, w = frame.shape[:2]   # 240 x 320

        # Publica frame crudo para YOLO (tamaño completo)
        self.raw_pub.publish(self.bridge.cv2_to_imgmsg(frame, encoding='bgr8'))

        # ---- ROI: 65% inferior ------------------------------------
        roi = frame[int(h * 0.40):h, :]
        roi_h, roi_w = roi.shape[:2]
        debug = roi.copy()

        # ---- Brillo adaptativo ------------------------------------
        H_roi = roi[int(roi_h * 0.5):, int(roi_w * 0.3):int(roi_w * 0.7)]
        hsv   = cv.cvtColor(H_roi, cv.COLOR_BGR2HSV)
        V_mean = np.mean(hsv[:, :, 2])
        self.H_history.append(V_mean)
        V_smooth = np.mean(self.H_history)
        cutting  = self.get_threshold(V_smooth)

        # ---- Binarización -----------------------------------------
        gray    = cv.cvtColor(roi, cv.COLOR_BGR2GRAY)
        blurred = cv.GaussianBlur(gray, (5, 5), 0)
        _, binary = cv.threshold(blurred, cutting, 255, cv.THRESH_BINARY_INV)

        # ---- Trapecio alargado centrado ---------------------------
        top_w = int(roi_w * 0.20) #------------------------- PROBAR CON UN NÜMERO MÄS GRANDE (ej. 45)
        top_y = int(roi_h * 0.05)
        trap  = np.array([[
            ((roi_w - top_w) // 2, top_y),
            ((roi_w + top_w) // 2, top_y),
            (roi_w, roi_h),
            (0,     roi_h)
        ]], dtype=np.int32)

        mask = np.zeros((roi_h, roi_w), dtype=np.uint8)
        cv.fillPoly(mask, trap, 255)
        masked = cv.bitwise_and(binary, binary, mask=mask)

        # ---- Morfología -------------------------------------------
        k     = np.ones((3, 3), np.uint8)
        morph = cv.erode(masked, k, iterations=1)
        morph = cv.dilate(morph, k, iterations=2)

        # ---- Hough ------------------------------------------------
        """
        edges = cv.Canny(morph, 50, 150)
        lines = cv.HoughLinesP(edges, 1, np.pi/180,
                               threshold=25,
                               minLineLength=20,
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
        """               
        #-----------Row anchors-------------------------------------------------------------
        #----------- Row anchors estilo U-Fast clásico -----------------------------

        ref_x = roi_w // 2
        error_x = self.last_error
        points = []

        # Debug visual base
        cv.polylines(debug, trap, True, (255, 0, 255), 1)
        cv.line(debug, (ref_x, 0), (ref_x, roi_h), (255, 0, 0), 1)

        # Si quieres ver bordes, los calculamos solo para debug.
        # Ya no se usan para Hough.
        edges = cv.Canny(morph, 50, 150)

        # Filas adaptadas al tamaño de TU ROI.
        # No copiamos CULane/TuSimple porque esos anchors son para 288x800.
        row_anchors = np.linspace(
            int(roi_h * 0.15),
            roi_h - 5,
            self.num_row_anchors
        ).astype(int)

        # Usamos el error anterior como referencia para evitar saltos bruscos
        search_ref = ref_x + self.last_error

        for y in row_anchors:
            band_h = self.band_height

            y1 = max(0, int(y - band_h // 2))
            y2 = min(roi_h, int(y + band_h // 2))

            band = morph[y1:y2, :]

            # Dibujar row anchor en debug
            cv.line(debug, (0, int(y)), (roi_w, int(y)), (80, 80, 80), 1)

            # Sumamos columnas activas dentro de la banda
            col_sum = np.sum(band > 0, axis=0)
            active_cols = np.where(col_sum > 1)[0]

            if len(active_cols) == 0:
                continue

            # Separamos grupos de columnas continuas
            splits = np.where(np.diff(active_cols) > 6)[0] + 1
            groups = np.split(active_cols, splits)

            candidates = []

            for g in groups:
                if len(g) < self.min_candidate_width:
                    continue

                x_min = int(g[0])
                x_max = int(g[-1])
                width = x_max - x_min

                # Evita manchas grandes como sombras o bloques de ruido
                if width > self.max_candidate_width:
                    continue

                cx_candidate = int(np.mean(g))
                candidates.append(cx_candidate)

            if not candidates:
                continue

            # Por ahora seguimos el candidato más cercano a la posición esperada
            cx = min(candidates, key=lambda x: abs(x - search_ref))

            points.append((cx, int(y)))
            cv.circle(debug, (cx, int(y)), 3, (0, 255, 255), -1)

        # ---- Calcular error usando los puntos encontrados ----------
        if len(points) >= self.min_points_for_lane:
            pts = np.array(points)

            # Damos más peso a las filas inferiores porque están más cerca del robot
            weights = pts[:, 1] / max(1, roi_h)

            cx = int(np.average(pts[:, 0], weights=weights))
            cy = int(np.average(pts[:, 1], weights=weights))

            error_x = int(cx - ref_x)

            self.error_history.append(error_x)
            error_x = int(np.mean(self.error_history))
            self.last_error = error_x
            self.last_lane_points = points

            cv.circle(debug, (cx, cy), 6, (0, 255, 255), -1)
            cv.line(debug, (ref_x, roi_h), (cx, cy), (0, 255, 255), 1)

        else:
            error_x = self.last_error
            cv.putText(debug, "Sin suficientes puntos", (5, roi_h - 5),
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

        cv.putText(debug, f"P:{len(points)}", (3, 45),
                   cv.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1)

        # ---- Debug local con OpenCV -------------------------------
        if self.debug_windows:
            cv.imshow('1_frame_original', frame)
            cv.imshow('2_roi', roi)
            cv.imshow('3_binary', binary)
            cv.imshow('4_masked', masked)
            cv.imshow('5_morph', morph)
            cv.imshow('6_edges', edges)
            cv.imshow('7_debug_final', debug)

            key = cv.waitKey(1) & 0xFF
            if key == ord('q'):
                self.get_logger().info("Cerrando por tecla q")
                rclpy.shutdown()

        self.get_logger().info(
            f'Error: {error_x} | Brillo: {V_smooth:.1f} | Cutting: {cutting} | Puntos: {len(points)}'
        )

        self.img_pub.publish(self.bridge.cv2_to_imgmsg(debug, encoding='bgr8'))

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
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
