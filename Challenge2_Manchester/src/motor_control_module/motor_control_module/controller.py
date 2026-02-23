import rclpy                #para programar en python
from rclpy.node import Node #clases nodo
from std_msgs.msg import Float32           

class ControleNode(Node):               #definimos la clase
   def __init__(self):
    super().__init__('control_node')         #'nombre de nodo'

    self.declare_parameter('kp', 1) #parámetro de ganancia
    self.kp = self.get_parameter('kp'). value

    self.setp = 0
    self.y = 0 #Salida y del sistema
    self.publisher = self.create_publisher(Float32, 'motor_input_u', 10) # (dato tipo string, 'nombre de tópico', 10 datos se almacenan)
    timer_period = 0.5

    self.setp_sub = self.create_subscription(Float32, 'set_point', self.setp_callback, 10)
    self.y_sub = self.create_subscription(Float32, 'motor_speed_y', self.y_callback, 10)                                       

    self.timer = self.create_timer(timer_period, self.timer_cb)     #cada 0.5 segundos se llama al callback
    self.i= 0

   def timer_cb(self):
    msg = Float32()
    error = self.setp - self.y #fórmula oara error
    u = self.kp * error  # u = error, para que pueda ir cambiando respecto a kp
    msg.data = u

    self.publisher.publish(msg) #Se manda error

    self.i+= 1 #va a aumentar por cada callback

   def setp_callback(self, msg):
    self.setp=msg.data

   def y_callback(self, msg):
    self.y = msg.data


def main(args=None):
    rclpy.init(args=args)                   #se inicializa el nodo

    controle_node = ControleNode() #inicializamos la clase y generamos objeto

    try:
        rclpy.spin(controle_node)       #donde ros llama a la clase, poner a correr el nodo
    except KeyboardInterrupt:               # con ctrl C, detenemos el nodo(shutdown)
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        controle_node.destroy_node()


if __name__ == '__main__':
    main()
