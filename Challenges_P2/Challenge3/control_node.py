#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

class ControlP(Node):
    def __init__(self):
        super().__init__("control_node")
        
        self.sub_ED = self.create_subscription(
            Float32, "/error_distance", self.distance_callback, 10
        )
        self.sub_Etheta = self.create_subscription(
            Float32, "/error_theta", self.angle_callback, 10
        )

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        #Errores
        self.error_d = 0.0
        self.error_theta = 0.0

        #ganancias
        self.kd = 0.0   #que tan fuerte responde el robot a la distancia
        self.k_theta = 0.0 #que tanto va a girar

        #saturaciones
        self.max_V = 0.0
        self.max_W = 0.0

        self.periodo = 0.05

        self.timer = self.create_timer(self.periodo, self.timer_callback)
    
    def distance_callback(self, msg):
        self.error_d = msg.data

    def angle_callback(self, msg):
        self.error_theta = msg.data

    def saturate(self, value, limit):
        if value > limit:
            return limit
        elif value < -limit:
            return -limit
        return value
    
    def timer_callback(self):
        v = self.kd * self.error_d
        w = self.k_theta * self.error_theta

        #saturación
        v = self.saturate(v, self.max_V)
        w = self.saturate(w, self.max_W)

        #publicar cmd_vel
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)



def main(args=None):
    #Inicializar comunicaciones ROS RDS
    rclpy.init(args=args)
    my_sub = ControlP()

    try:
        rclpy.spin(my_sub)
    except KeyboardInterrupt:
        pass
    finally:
        my_sub.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    main()
