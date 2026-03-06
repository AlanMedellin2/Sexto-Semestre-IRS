#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PointStamped
from sensor_msgs.msg import LaserScan
import math
import numpy as np
import signal

ANGLE_TOL   = 0.1   
MAX_LIN_VEL = 0.5   # m/s razonable para TurtleBot3
DT          = 0.1

K_LQR = [0.4472, 1.0461]

def wrapToPi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi

def quaternion_to_yaw(z, w):
    return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)


class TurnLQRSecondOrder(Node):

    def __init__(self):
        super().__init__("turn_lqr_second_order")

        #suscripsiones
        self.sub_goal = self.create_subscription(PointStamped, "/clicked_point", self.cb_goal, 10)
        self.amcl_sub = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        #Publishers
        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.timer = self.create_timer(DT, self.control_loop)

        self.omega = 0.0
        self.theta = 0.0
        self.goal_x = None
        self.goal_y = None
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.kp_atrac = 1.0

        self.FR_x = 0.0
        self.FR_y = 0.0
        self.FR_x_local = 0.0
        self.FR_y_local = 0.0
        self.front_distance = 3.5

        self.get_logger().info("LQR 2° orden estable listo.")


    def stop_turtle(self):
        stop_msg = Twist()
        self.pub_cmd.publish(stop_msg)

    def cb_goal(self, msg: PointStamped):
        self.goal_x = msg.point.x
        self.goal_y = msg.point.y
        self.omega = 0.0  # reset omega al recibir nuevo objetivo
        self.get_logger().info(f"Nuevo objetivo: ({self.goal_x:.2f}, {self.goal_y:.2f})")

    def amcl_callback(self, msg):
        self.pose_x = msg.pose.pose.position.x
        self.pose_y = msg.pose.pose.position.y
        rot_z = msg.pose.pose.orientation.z
        rot_w = msg.pose.pose.orientation.w
        self.theta = quaternion_to_yaw(rot_z, rot_w)

    def scan_callback(self, msg):
        d_max = 0.4
        k_rep = 1.2
        dir_x_sum = 0.0
        dir_y_sum = 0.0
        d_min = float('inf')

        for i, dist in enumerate(msg.ranges):
            if dist < 0.1 or math.isinf(dist) or math.isnan(dist):
                continue
            if dist < d_max:
                if dist < d_min:
                    d_min = dist
                angle_local = msg.angle_min + i * msg.angle_increment   
                dir_x_sum -= math.cos(angle_local)
                dir_y_sum -= math.sin(angle_local)

        if d_min < d_max:
            norm = math.hypot(dir_x_sum, dir_y_sum)
            if norm > 0:
                dir_x_norm = dir_x_sum / norm
                dir_y_norm = dir_y_sum / norm
                magnitude = k_rep * (1.0/d_min - 1.0/d_max) / (d_min**2)
                magnitude = min(magnitude, 3.0)
                self.FR_x_local = dir_x_norm * magnitude
                self.FR_y_local = dir_y_norm * magnitude
        else:
            self.FR_x_local = 0.0
            self.FR_y_local = 0.0

        self.FR_x = self.FR_x_local * math.cos(self.theta) - self.FR_y_local * math.sin(self.theta)
        self.FR_y = self.FR_x_local * math.sin(self.theta) + self.FR_y_local * math.cos(self.theta)

        front_index = int((0.0 - msg.angle_min) / msg.angle_increment)
        dist_frente = msg.ranges[front_index]
        if math.isinf(dist_frente) or math.isnan(dist_frente) or dist_frente < 0.05:
            self.front_distance = 0.0
        else:
            self.front_distance = dist_frente

    def control_loop(self):

         # --- 1. NO HACE NADA SI NO HA LLEGADO UN PUNTO ---
        if self.goal_x is None:
            return

        # --- 2. CALUCLAR DISTANCIA Y EVALUAR ---
        dist = math.hypot(self.goal_x - self.pose_x, self.goal_y - self.pose_y)
        if dist < 0.5:
            self.get_logger().info("¡Meta alcanzada!")
            self.pub_cmd.publish(Twist())
            self.goal_x = None
            self.goal_y = None
            self.omega = 0.0
            return

        # --- 3. CAMPOS POTENCIALES (FUERZA ATRACTIVA) ---
        dx = self.goal_x - self.pose_x
        dy = self.goal_y - self.pose_y

        FA_x = self.kp_atrac * (dx)
        FA_y = self.kp_atrac * (dy)

         # --- 4. FUERZA TOTAL ---
        atenuacion = min(1.0, dist)
        Ftot_x = FA_x + (self.FR_x * atenuacion)
        Ftot_y = FA_y + (self.FR_y * atenuacion)

        angle_to_goal = math.atan2(Ftot_y, Ftot_x)
        #angle_to_goal = math.atan2(dy, dx)

        self.get_logger().info(f"theta: {self.theta:.3f}")

        error = wrapToPi(self.theta - angle_to_goal)

        # --- 5. LQR (Control de Dirección) ---
        x_state = np.array([error, self.omega])
        u = -np.dot(K_LQR, x_state)

        # Actualización de Euler
        self.omega = self.omega + (u * DT)
        self.omega = np.clip(self.omega, -2.8, 2.8)

        v = MAX_LIN_VEL * max(0.0, math.cos(error))

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

        self.get_logger().info(f"goal_angle:{angle_to_goal:.2f} theta:{self.theta:.2f} error:{error:.2f}")
        self.pub_cmd.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = TurnLQRSecondOrder()
    
    def signal_handler(sig,frame):
        node.stop_turtle()
        node.destroy_node()
        rclpy.shutdown()

    signal.signal(signal.SIGINT,signal_handler)

    rclpy.spin(node)


if __name__ == "__main__":
    main()
