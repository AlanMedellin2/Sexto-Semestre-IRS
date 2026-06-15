#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class ColorCmdVel(Node):
    def __init__(self):
        super().__init__('color_cmd_vel')

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.cap = cv2.VideoCapture(2)

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        self.cap.set(cv2.CAP_PROP_FPS, 10)

        self.timer = self.create_timer(0.1, self.loop)

        self.get_logger().info("Nodo color_cmd_vel iniciado")

    def send_cmd(self, linear_x, angular_z=0.0):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.pub.publish(msg)

    def loop(self):
        ret, frame = self.cap.read()

        if not ret:
            self.get_logger().warn("No se pudo leer cámara")
            self.send_cmd(0.0, 0.0)
            return

        frame = cv2.resize(frame, (320, 240))
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)



        # Rangos HSV

        # Verde
        lower_green = np.array([35, 80, 60])
        upper_green = np.array([85, 255, 255])

        # Amarillo
        lower_yellow = np.array([20, 80, 80])
        upper_yellow = np.array([35, 255, 255])

        # Rojo
        lower_red1 = np.array([0, 80, 60])
        upper_red1 = np.array([10, 255, 255])

        lower_red2 = np.array([170, 80, 60])
        upper_red2 = np.array([180, 255, 255])

        # MASK
        mask_green = cv2.inRange(hsv, lower_green, upper_green)
        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
        mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask_red = mask_red1 + mask_red2

        # Limpieza de ruido
        kernel = np.ones((5, 5), np.uint8)

        mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel)
        mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_CLOSE, kernel)

        mask_yellow = cv2.morphologyEx(mask_yellow, cv2.MORPH_OPEN, kernel)
        mask_yellow = cv2.morphologyEx(mask_yellow, cv2.MORPH_CLOSE, kernel)

        mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)
        mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, kernel)

        #Áreas 
        area_green = cv2.countNonZero(mask_green)
        area_yellow = cv2.countNonZero(mask_yellow)
        area_red = cv2.countNonZero(mask_red)

        min_area = 800

        linear = 0.0
        angular = 0.0
        selected_mask = None
        detected_color = "NONE"




        # Decisión por color 
        if area_red > min_area:
            detected_color = "ROJO - STOP"
            linear = 0.00
            angular = 0.00
            selected_mask = mask_red

        elif area_yellow > min_area:
            detected_color = "AMARILLO - LENTO"
            linear = 0.10
            selected_mask = mask_yellow

        elif area_green > min_area:
            detected_color = "VERDE - RAPIDO"
            linear = 0.50
            selected_mask = mask_green

        else:
            detected_color = "SIN COLOR - STOP"
            self.send_cmd(0.0, 0.0)

            cv2.putText(frame, detected_color, (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

            cv2.imshow("Camera - Color Detection", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.send_cmd(0.0, 0.0)
                rclpy.shutdown()

            return

        # Dirección según posición del color
        moments = cv2.moments(selected_mask)

        cx = None
        error = 0

        if moments["m00"] > 0:
            cx = int(moments["m10"] / moments["m00"])
            cy = int(moments["m01"] / moments["m00"])

            width = frame.shape[1]
            center_x = width // 2

            error = cx - center_x

            # Control proporcional para girar
            k_ang = 0.002
            angular = -k_ang * error

            # Limitar giro
            max_angular = 0.6
            angular = max(min(angular, max_angular), -max_angular)

            # Dibujar centro objeto
            cv2.circle(frame, (cx, cy), 8, (255, 0, 0), -1)

            # Dibujar línea del centro de imagen
            cv2.line(frame, (center_x, 0), (center_x, frame.shape[0]),
                     (255, 255, 255), 2)





        #  Enviar comando
        self.send_cmd(linear, angular)

        cv2.putText(frame, detected_color, (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        cv2.putText(frame, f"linear: {linear:.2f}", (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        cv2.putText(frame, f"angular: {angular:.2f}", (10, 75),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        cv2.putText(frame, f"error: {error}", (10, 100),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        cv2.imshow("Camera - Color Detection", frame)
        cv2.imshow("Selected Mask", selected_mask)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.send_cmd(0.0, 0.0)
            rclpy.shutdown()

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ColorCmdVel()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.send_cmd(0.0, 0.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
