#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

class ErrorCalculus(Node):
    def __init__(self):
        super().__init__("error_node")
        
        self.pub_ED= self.create_publisher(
            Float32, "/r1/error_distance", 10
        )

        self.pub_Etheta = self.create_publisher(
            Float32, "/r1/error_theta", 10
        )

        self.pub_estado= self.create_publisher(
            Float32, "/r1/estado", 10
        )

        self.sub_groundtruth = self.create_subscription(
            PoseStamped, "/r1/ground_truth", self.ground_truth_callback, 10
        )

        self.sub_groundtruth = self.create_subscription(
            PoseStamped, "goals", self.goal_callback, 10
        )


        # --- BUFFER Y ESTADO ---
        self.estado = 1.0
        self.buffer_metas = []  # Aquí guardaremos todas las metas que lleguen
        self.umbral_llegada = 0.1  # 10 cm para considerar que llegó a la meta
        self.current_pos = None

    def goal_callback(self, msg):
        self.buffer_metas.append(msg.pose)
        self.get_logger().info(f"Meta recibida y guardada. Buffer: {len(self.buffer_metas)} metas.")
   
    def ground_truth_callback(self, msg):
        self.current_pos = msg.pose
        
        # Si no hay metas en el buffer, no calculamos nada
        if not self.buffer_metas:
            return
            
        # Tomamos siempre la primera meta (la más vieja en el buffer)
        meta_objetivo = self.buffer_metas[0]
        
        # Calculamos errores (usando la lógica anterior)
        x_r, y_r = -self.current_pos.position.x, -self.current_pos.position.y
        x_g, y_g = meta_objetivo.position.x, meta_objetivo.position.y

        dist_error = math.sqrt((x_g - x_r)**2 + (y_g - y_r)**2)
        
        # --- LÓGICA DE ACTUALIZACIÓN DEL BUFFER ---
        if dist_error < self.umbral_llegada:
            self.get_logger().info("¡Meta alcanzada! Pasando a la siguiente en el buffer.")
            self.buffer_metas.pop(0) # Eliminamos la meta actual para pasar a la siguiente
            if not self.buffer_metas:
                self.get_logger().info("Se han completado todas las metas del buffer.")
                self.estado = 0.0
                self.estado_final = Float32()
                self.estado_final.data = self.estado
                self.pub_estado.publish(self.estado_final)

                return
            self.estado = 1.0
            # Actualizamos meta_objetivo a la nueva primera meta
            meta_objetivo = self.buffer_metas[0]
            # Recalculamos distancia para la nueva meta
            dist_error = math.sqrt((meta_objetivo.position.x - x_r)**2 + (meta_objetivo.position.y - y_r)**2)

        # Cálculo de Error de Ángulo
        curr_yaw = self.euler_from_quaternion(self.current_pos.orientation)
        desired_yaw = math.atan2(y_g - y_r, x_g - x_r)
        angle_error = math.atan2(math.sin(desired_yaw - curr_yaw), math.cos(desired_yaw - curr_yaw))

        # Publicar
        self.publish_metrics(dist_error, angle_error)

    def publish_metrics(self, d, t):
        msg_d = Float32()
        msg_d.data = float(d)
        self.pub_ED.publish(msg_d)

        msg_t = Float32()
        msg_t.data = float(t)
        self.pub_Etheta.publish(msg_t)

        self.estado_final = Float32()
        self.estado_final.data = self.estado
        self.pub_estado.publish(self.estado_final)

    def euler_from_quaternion(self, q):
        t3 = +2.0 * (q.w * q.z + q.x * q.y)
        t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(t3, t4)


def main(args=None):
    #Inicializar comunicaciones ROS RDS
    rclpy.init(args=args)
    my_sub = ErrorCalculus()

    try:
        rclpy.spin(my_sub)
    except KeyboardInterrupt:
        pass
    finally:
        my_sub.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    main()
