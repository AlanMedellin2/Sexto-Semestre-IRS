#!/usr/bin/env python3
import rclpy #nos deja usar las funciones de Ros
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class PointGoals(Node):
    def __init__(self):
        super().__init__("node_goals") #definir una vez

        self.pub_ = self.create_publisher(PoseStamped, "goals", 10) #va a publicar Pose, nombre tópico, num cola

        self.period_ = 1
        self.meta_actual = 0 #Punto 0 de 4

        self.metas = [
            (0.5, 0.0),             #arrancamos de el vértice inferior izquierdo, no inicia desde (0,0) porque suponemos que se encuentra ahí
            (0.5, 0.5),
            (0.0, 0.5),
            (0.0, 0.0),
        ]
       
        self.get_logger().info("Publishing at %d s" %self.period_) #Se ve en la terminal

        self.timer_=self.create_timer(self.period_, self.timerCallback) #nuevo timer que ejecuta una función específica repetidamente con su periodo
    
    def timerCallback(self): #todo lo que se repite
        #contruir mensaje de meta actual

        if self.meta_actual >= len(self.metas): #len por la lista
            self.get_logger().info("Todas las metas publicadas")
            self.timer_.cancel() #Si es que queremos que termine (revisar)
            return

        msg = PoseStamped()
        x_meta, y_meta = self.metas[self.meta_actual] #llenamos el mensaje

        msg.pose.position.x = x_meta
        msg.pose.position.y = y_meta
        msg.pose.position.z = 0.0

        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0

        #sin esto perdemos contexto del mensaje
        msg.header.stamp = self.get_clock().now().to_msg() #marca de tiempo
        msg.header.frame_id = "map" #marco de referencia

        self.pub_.publish(msg)
        self.meta_actual += 1

        
def main():
    rclpy.init() #inicializar interfaz
    point_goals = PointGoals()
    rclpy.spin(point_goals) #para que siempre envíe 
    point_goals.destroy_node() #se asegura de destruir con ctrl + C
    rclpy.shutdown()

if __name__ == '__main__':
    main()
