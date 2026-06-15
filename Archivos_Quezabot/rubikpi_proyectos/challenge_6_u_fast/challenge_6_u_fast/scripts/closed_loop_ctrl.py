#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist


class ControlPColor(Node):
    def __init__(self):
        super().__init__("control_color_node")

        # ── Suscripciones de control ──────────────────────────────────────
        self.sub_ED = self.create_subscription(
            Float32, "/error_distance", self.distance_callback, 10
        )
        self.sub_Etheta = self.create_subscription(
            Float32, "/error_theta", self.angle_callback, 10
        )
        self.sub_estado = self.create_subscription(
            Float32, "/estado", self.estado_callback, 10
        )

        # ── Suscripción semáforo ──────────────────────────────────────────
        # Escucha strings: "red" | "yellow" | "green" | "" (sin detección)
        self.sub_color = self.create_subscription(
            Float32, "/color", self.color_callback, 10
        )

        # ── Publisher ─────────────────────────────────────────────────────
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # ── Estado interno ────────────────────────────────────────────────
        self.error_d     = 0.0
        self.error_theta = 0.0
        self.estado      = 0.0   # 0 = inactivo, 1 = activo (viene del nodo de goal)

        # Estado semáforo — arranca detenido hasta recibir verde
        # Cambia a "GO" si quieres que empiece moviéndose sin semáforo
        self.estado_semaforo = "GO"

        # ── Ganancias ─────────────────────────────────────────────────────
        self.kd      = 1.5
        self.k_theta = 3.0

        # ── Límites ───────────────────────────────────────────────────────
        self.max_V       = 0.1
        self.max_W       = 0.5
        self.factor_lento = 0.3   # fracción de velocidad en amarillo

        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info("control_color_node iniciado — esperando semáforo...")

    # ── Callbacks ─────────────────────────────────────────────────────────

    def distance_callback(self, msg):
        self.error_d = msg.data

    def angle_callback(self, msg):
        self.error_theta = msg.data

    def estado_callback(self, msg):
        self.estado = msg.data

    def color_callback(self, msg):

        val = msg.data
        if val == 3.0:
            self.estado_semaforo = "STOP"
        elif val == 2.0:
            self.estado_semaforo = "GO"
        elif val == 1.0:
            if self.estado_semaforo != "STOP":
                self.estado_semaforo = "SLOW"

        # val == 0.0 → sin detección → mantiene último estado

    # ── Utilidades ────────────────────────────────────────────────────────

    def saturate(self, value, limit):
        return max(-limit, min(limit, value))

    # ── Loop principal ────────────────────────────────────────────────────

    def timer_callback(self):
        # Control proporcional base
        v = self.kd      * self.error_d
        w = self.k_theta * self.error_theta

        # Si el error angular es grande, girar en sitio primero
        if abs(self.error_theta) > math.radians(30):
            v = 0.0

        # Nodo de goal dice "no moverse aún"
        if self.estado == 0.0:
            v = 0.0
            w = 0.0

        # ── Aplicar estado del semáforo ───────────────────────────────────
        if self.estado_semaforo == "STOP":
            v = 0.0
            w = 0.0

        elif self.estado_semaforo == "SLOW":
            v *= self.factor_lento
            w *= self.factor_lento

        # elif "GO": velocidades sin modificar

        # Saturar y publicar
        cmd = Twist()
        cmd.linear.x  = self.saturate(v, self.max_V)
        cmd.angular.z = self.saturate(w, self.max_W)
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ControlPColor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

