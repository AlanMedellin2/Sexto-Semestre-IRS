import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import numpy as np

class ColorDetectorNode(Node):
    def __init__(self):
        super().__init__('color_detector_node')
        # Timer a 10Hz para mayor fluidez
        self.Ts = 0.1 
        self.color_pub = self.create_publisher(Float32, '/color', 10)
        self.timer = self.create_timer(self.Ts, self.main_loop)

        self.get_logger().info("Iniciando detección de semáforo...")

        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        # RANGOS HSV (Ajustados para luces brillantes)
        self.color_ranges = {
            "Amarillo": [(np.array([20, 100, 150]), np.array([35, 255, 255]))],
            "Verde": [(np.array([40, 70, 70]), np.array([90, 255, 255]))],
            "Rojo": [
                (np.array([0, 150, 100]), np.array([10, 255, 255])),
                (np.array([160, 150, 100]), np.array([180, 255, 255]))
            ]
        }

        self.draw_colors = {
            "Amarillo": (0, 255, 255),
            "Verde": (0, 255, 0),
            "Rojo": (0, 0, 255)
        }

    def main_loop(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        frame_h, frame_w = frame.shape[:2]
        total_objects = 0
        detected_color_val = 0.0 # Por defecto nada

        for color_name, ranges in self.color_ranges.items():
            mask = np.zeros((frame_h, frame_w), dtype=np.uint8)
            for lower, upper in ranges:
                partial_mask = cv2.inRange(hsv, lower, upper)
                mask = cv2.bitwise_or(mask, partial_mask)

            # Limpieza morfológica
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < 800: # Ignorar ruidos pequeños
                    continue

                # --- FILTRO 1: CIRCULARIDAD ---
                # Un círculo perfecto tiene circularidad = 1
                perimeter = cv2.arcLength(cnt, True)
                if perimeter == 0: continue
                circularity = 4 * np.pi * (area / (perimeter * perimeter))

                # --- FILTRO 2: RELACIÓN DE ASPECTO ---
                x, y, w, h = cv2.boundingRect(cnt)
                aspect_ratio = float(w)/h

                # Un semáforo circular debería tener circularidad > 0.7 
                # y un aspect ratio cercano a 1 (cuadrado/círculo)
                if 0.6 < circularity < 1.2 and 0.7 < aspect_ratio < 1.3:
                    total_objects += 1
                    
                    # Dibujar detección
                    cv2.rectangle(frame, (x, y), (x + w, y + h), self.draw_colors[color_name], 2)
                    cv2.putText(frame, f"Semaforo: {color_name}", (x, y - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.draw_colors[color_name], 2)

                    # Asignar valor para publicar
                    if color_name == 'Amarillo': detected_color_val = 1.0
                    elif color_name == 'Verde': detected_color_val = 2.0
                    elif color_name == 'Rojo': detected_color_val = 3.0

        # Publicar el resultado (solo el último detectado o 0 si ninguno)
        msg = Float32()
        msg.data = detected_color_val
        self.color_pub.publish(msg)

        cv2.imshow("Deteccion de Semaforo", frame)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = ColorDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
