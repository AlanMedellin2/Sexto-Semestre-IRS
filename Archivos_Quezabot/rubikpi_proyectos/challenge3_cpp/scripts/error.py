#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

UMBRAL_LLEGADA = 0.13

class ErrorCalculus(Node):
    def __init__(self):
        super().__init__("error_node")

        self.pub_ED = self.create_publisher(Float32, "/error_distance", 10)
        self.pub_Etheta = self.create_publisher(Float32, "/error_theta", 10)
        self.pub_estado = self.create_publisher(Float32, "/estado", 10)

        self.sub_groundtruth = self.create_subscription(
            Odometry, "/ekf_odom", self.ground_truth_callback, 10
        )
        self.sub_goals = self.create_subscription(
            PoseStamped, "a_star_goals", self.goal_callback, 10
        )

        self.estado = 1.0
        self.buffer_metas = []
        self.current_pos = None

    def goal_callback(self, msg):
        self.buffer_metas.append(msg.pose)
        self.estado = 1.0
        self.get_logger().info(
            f"Meta recibida: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}. "
            f"Buffer: {len(self.buffer_metas)} metas."
        )

    def ground_truth_callback(self, msg):
        self.current_pos = msg.pose.pose  # Odometry tiene pose.pose

        if not self.buffer_metas:
            return

        meta_objetivo = self.buffer_metas[0]

        x_r = self.current_pos.position.x
        y_r = self.current_pos.position.y

        x_g = meta_objetivo.position.x
        y_g = meta_objetivo.position.y

        dist_error = math.sqrt((x_g - x_r)**2 + (y_g - y_r)**2)

        if dist_error < UMBRAL_LLEGADA:
            self.get_logger().info(
                f"Meta ({x_g:.2f}, {y_g:.2f}) alcanzada. Pasando a la siguiente."
            )
            self.buffer_metas.pop(0)

            if not self.buffer_metas:
                self.get_logger().info("Todas las metas completadas.")
                self.estado = 0.0
                self._publicar_estado()
                return

            # Señal de handshake para node_extra
            self.estado = 0.0
            self._publicar_estado()
            self.estado = 1.0

            # Actualizar meta
            meta_objetivo = self.buffer_metas[0]
            x_g = meta_objetivo.position.x
            y_g = meta_objetivo.position.y
            dist_error = math.sqrt((x_g - x_r)**2 + (y_g - y_r)**2)

        curr_yaw = self.euler_from_quaternion(self.current_pos.orientation)
        desired_yaw = math.atan2(y_g - y_r, x_g - x_r)
        angle_error = math.atan2(
            math.sin(desired_yaw - curr_yaw),
            math.cos(desired_yaw - curr_yaw)
        )

        self.publish_metrics(dist_error, angle_error)

    def publish_metrics(self, d, t):
        msg_d = Float32()
        msg_d.data = float(d)
        self.pub_ED.publish(msg_d)

        msg_t = Float32()
        msg_t.data = float(t)
        self.pub_Etheta.publish(msg_t)

        self._publicar_estado()

    def _publicar_estado(self):
        msg = Float32()
        msg.data = float(self.estado)
        self.pub_estado.publish(msg)

    def euler_from_quaternion(self, q):
        t3 = +2.0 * (q.w * q.z + q.x * q.y)
        t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(t3, t4)


def main(args=None):
    rclpy.init(args=args)
    my_sub = ErrorCalculus()
    try:
        rclpy.spin(my_sub)
    except KeyboardInterrupt:
        pass
    finally:
        my_sub.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
