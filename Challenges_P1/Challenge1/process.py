#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import math

class Procesar(Node):
    def __init__(self):
        super().__init__("process") #Nombre del Nodo

        #Creacion suscribers
        self.sub_time = self.create_subscription(Float32,"time",self.time_callback,10)
        self.sub_signal = self.create_subscription(Float32, "signal", self.signal_callback ,10)

        #Creacion publisher

        self.rate= 0.1 #10 Hz, se pude cambiar

        self.pub_signal = self.create_publisher(Float32, "proc_signal",10)
        self.timer = self.create_timer(self.rate, self.publish_proc_data) 

        self.signal=0.0
        self.time=0.0

        self.phaseShift = 0.5 #Radianes
        self.procSignal = 0

    def time_callback(self,msg):
        self.time=msg.data

    def signal_callback(self,msg):
        self.signal=msg.data

    def publish_proc_data(self):

        #Amplitud a la mitad y siempre postiva Rango [0,1] sin phase shift
        #self.procSignal = 0.5*self.signal+0.5

        #Procesamiento con phase shift recreando con el valor de tiempo
        self.procSignal = 0.5*math.sin(self.time+self.phaseShift)+0.5

        msg_out=Float32()
        msg_out.data = self.procSignal

        self.pub_signal.publish(msg_out)


def main(args=None):
    rclpy.init()
    my_sub = Procesar()
    print("Waiting for data")

    try:
        rclpy.spin(my_sub)
    except KeyboardInterrupt:
        my_sub.destroy_node()


if __name__=='__main__':
    main()
