#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Bool

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber_node')
        
        self.subscription = self.create_subscription(
            Image,
            '/camera/raw',
            self.image_callback,
            10 
        )
        self.subscription  
        self.inter_pub  = self.create_publisher(Bool,  '/intersection_line',  10)
        self.debug_pub  = self.create_publisher(Image, '/intersection_debug', 10)
        
        self.bridge = CvBridge()
        self.get_logger().info('Nodo Suscriptor de Imagen Iniciado.')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            h, w = cv_image.shape[:2] 
            roi = cv_image[int(h*0.6):h, :]
            roi_h, roi_w = roi.shape[:2]
            gris_image = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gris_image, (5, 5), 0)
            _, binary = cv2.threshold(blurred, 80, 255, cv2.THRESH_BINARY_INV)

            kernel = np.ones((3,3), np.uint8)
            morph = cv2.erode(binary, kernel, iterations=1)
            morph = cv2.dilate(morph, kernel, iterations=1)

            num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(morph, connectivity=8)
            zona_central_min = int(roi_h * 0.40)
            zona_central_max = int(roi_h * 0.60)
            contador_candidatos = 0

            debug_bgr = cv2.cvtColor(morph, cv2.COLOR_GRAY2BGR)
            cv2.line(debug_bgr, (0, zona_central_min), (roi_w, zona_central_min), (0, 255, 0), 1)
            cv2.line(debug_bgr, (0, zona_central_max), (roi_w, zona_central_max), (0, 255, 0), 1)

            for i in range(1, num_labels):
                x, y, bw, bh, area = stats[i]
                cx, cy = centroids[i]
                if area >= 100:
                    if zona_central_min <= cy <= zona_central_max:
                        contador_candidatos += 1
                        cv2.circle(debug_bgr, (int(cx), int(cy)), 5, (0, 0, 255), -1)
                    else:
                        cv2.circle(debug_bgr, (int(cx), int(cy)), 3, (255, 0, 0), -1)

            msg_interseccion = Bool()
            if contador_candidatos >= 6:
                print("Interseccion")
                msg_interseccion.data = True
                self.get_logger().info("¡Intersección Detectada! Enviando True...")
                cv2.putText(debug_bgr, "INTER!", (5, 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            else:
                msg_interseccion.data = False

            cv2.putText(debug_bgr, f"cands:{contador_candidatos}", (5, roi_h - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

            self.inter_pub.publish(msg_interseccion)
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug_bgr, encoding='bgr8'))

            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.get_logger().info('Cerrando ventana por petición del usuario.')
                
        except CvBridgeError as e:
            self.get_logger().error(f'Error al convertir la imagen: {e}')

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    try:
        rclpy.spin(image_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        image_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
