#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PointStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math
import numpy as np


# ─────────────────────────────
# Parámetros ajustados para TurtleBot sim
# ─────────────────────────────
ANGLE_TOL   = 0.017
MAX_ANG_VEL = 0.05
DT          = 0.1   

# Ganancias LQR (modelo 2° orden: error, omega)
K_LQR = [0.4472, 1.0461]


def wrapToPi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi

def quaternion_to_yaw(z, w):
    return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)


class TurnLQRSecondOrder(Node):

    def __init__(self):
        super().__init__("turn_lqr_second_order")

        self.sub_goal = self.create_subscription(
            PointStamped, "/clicked_point", self.cb_goal, 10)

        
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_callback,
            10
        )

        """self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )"""

        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)

        self.timer = self.create_timer(DT, self.control_loop)

        #PARA LOS ANGULOS
        self.omega = 0.0
        self.theta = 0.0

        #POSICIONES
        self.goal_x = None
        self.goal_y = None
        self.pose_x = 0.0
        self.pose_y = 0.0

        self.kp_atrac = 2.0

        self.omega_cmd = 0.0

        self.FR_x = 0
        self.FR_y = 0

        self.get_logger().info("LQR 2° orden estable listo.")

    # ─────────────────────────────

    def cb_goal(self, msg: PointStamped):
        self.goal_x = msg.point.x
        self.goal_y = msg.point.y
        self.omega_cmd = 0.0  # reset
        self.get_logger().info(
            f"Nuevo objetivo: ({self.goal_x:.2f}, {self.goal_y:.2f})"
        )

    def amcl_callback(self, msg):
        # Guardamos la posición en la memoria
        self.pose_x = msg.pose.pose.position.x
        self.pose_y = msg.pose.pose.position.y
        
        # Extraemos el cuaternión y lo convertimos a radianes
        rot_z = msg.pose.pose.orientation.z
        rot_w = msg.pose.pose.orientation.w
        self.theta = quaternion_to_yaw(rot_z, rot_w)

    # ─────────────────────────────

    def control_loop(self):

         # --- 1. NO HACE NADA SI NO HA LLEGADO UN PUNTO ---
        if self.goal_x is None:
            return
        
        # --- 2. CALUCLAR DISTANCIA Y EVALUAR ---
        dist = math.hypot(self.goal_x-self.pose_x,self.goal_y-self.pose_y)
        if dist<0.5:
            self.get_logger().info("¡Meta alcanzada!")
            cmd = Twist()
            self.pub_cmd.publish(cmd)
            self.goal_x = None 
            self.goal_y = None
            cmd.angular.z = 0.0
            cmd.linear.x = 0.0
            return
            
        # --- 3. CAMPOS POTENCIALES (FUERZA ATRACTIVA) ---
        FA_x = self.kp_atrac*(self.goal_x-self.pose_x)
        FA_y = self.kp_atrac*(self.goal_y-self.pose_y)


        # --- 4. FUERZA TOTAL ---
        Ftot_x = FA_x + self.FR_x
        Ftot_y = FA_y + self.FR_y

        dx = self.goal_x - self.pose_x
        dy = self.goal_y - self.pose_y

        angle_to_goal = math.atan2(dy, dx)

        self.get_logger().info(f"theta: {self.theta:.3f}")

        error = wrapToPi(self.theta - angle_to_goal)

         # --- 5. LQR (Control de Dirección) ---
        x_state = np.array([error, self.omega])
        u = -np.dot(K_LQR, x_state)

        # Actualización de Euler
        self.omega = self.omega + (u * DT)
        self.omega = np.clip(self.omega, -2.8, 2.8)

        # --- 6. MODULACIÓN DE VELOCIDAD LINEAL ---
        v = MAX_ANG_VEL * max(0.0, math.cos(error))

        # --- PUBLICAR COMANDOS ---
        cmd = Twist()
        cmd.angular.z = float(self.omega)
        
        alineado = abs(error) < ANGLE_TOL
        # Condición de parada
        self.get_logger().info(f"error: {error:.4f}")

        if alineado:
            self.get_logger().info("Alineado correctamente.")
            cmd.angular.z = 0.0
            cmd.linear.x = float(v)
        
        else:
            cmd.angular.z = float(self.omega)
            cmd.linear.x = 0.0

        self.pub_cmd.publish(cmd)

        


def main(args=None):
    rclpy.init(args=args)
    node = TurnLQRSecondOrder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
