#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import   Twist,PointStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math
import numpy as np


class moveTurtleBot(Node):
    def __init__(self):
        super().__init__("track_point")

        #Creacion de Publisher y Suscribers
        self.sus = self.create_subscription(PointStamped,"/clicked_point",self.sus_callback,10) #Punto a donde vamos
        self.sus_odo = self.create_subscription(Odometry,"/odom",self.sus_odo_callback,10) #Posicion Actual del robot
        #self.sus_scan = self.create_subscription(LaserScan,"/scan",self.sus__scan_callback,10) #Obstaculos

        self.pub = self.create_publisher(Twist, "/cmd_vel", 10) #Publicar para mover el turtle bot
        self.timer = self.create_timer(0.01,self.publish_position_to_point)
        self.i=0

        #Valor kp para el control proporcional
        self.kp_att = 0.2
        self.kp_ang =2

        #Definiciones iniciales
        self.pos_x_actual = 0.0
        self.pos_y_actual = 0.0
        self.pos_x_to_go = None
        self.pos_y_to_go = None
        self.yaw = 0.0


    def sus_odo_callback(self,msg):

        #Nos dicen en que parte del mapa esta el robot
        self.pos_x_actual = msg.pose.pose.position.x 
        self.pos_y_actual = msg.pose.pose.position.y

        self.pos_robot = np.array([self.pos_x_actual,self.pos_y_actual])

        #Se calcula su orientacion a partir de quaterniones
        self.q_x = msg.pose.pose.orientation.x
        self.q_y = msg.pose.pose.orientation.y
        self.q_z = msg.pose.pose.orientation.z
        self.q_w = msg.pose.pose.orientation.w

        self.yaw = math.atan2(2*(self.q_w*self.q_z+self.q_x*self.q_y),
                              1-2*(self.q_y*self.q_y+self.q_z*self.q_z)
                              )

    def sus_callback(self,msg):

        #Guardamos las coordenadas del punto a donde queremos ir
        self.pos_x_to_go= msg.point.x
        self.pos_y_to_go = msg.point.y

        self.pos_to_go = np.array([self.pos_x_to_go,self.pos_y_to_go])

    def publish_position_to_point(self):

        if self.pos_x_to_go is None: #Si no se ha recibido un punto a donde ir, no hace nada
            return
        
        msg= Twist()

        #-----FUERZA DE ATRACCION-----#

        #Direccion
        self.ref_angulo = math.atan2(self.pos_y_to_go-self.pos_y_actual,
                                     self.pos_x_to_go-self.pos_x_actual) 
        
        #Error de direccion en radianes
        self.error = self.ref_angulo-self.yaw
        self.error_rad = math.atan2(math.sin(self.error),math.cos(self.error))

        #Fuerza de atraccion
        diff = self.pos_to_go-self.pos_robot
        self.fuerzaAtt = self.kp_att*(diff)

        #Distancia
        self.dist = np.sqrt(diff[0]**2+diff[1]**2)
        
        
        # Si ya llegaste, detente
        if self.dist < 0.1:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.pos_x_to_go = None
            self.pos_y_to_go = None
        else:
            msg.angular.z = self.kp_ang * self.error_rad
            msg.linear.x  = self.kp_att * self.dist

        self.pub.publish(msg)

        self.i += 1

def main(args=None):
    rclpy.init()
    my_pub = moveTurtleBot()
    print("Publisher Node Running")
    

    try:
        rclpy.spin(my_pub)
    except KeyboardInterrupt:
        my_pub.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    main()
