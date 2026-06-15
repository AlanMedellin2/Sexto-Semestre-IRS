#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import smbus2
import time

# Direcciones I2C del MPU6050
MPU6050_ADDR = 0x68
REG_PWR_MGMT_1 = 0x6B
REG_ACCEL_XOUT_H = 0x3B
REG_GYRO_XOUT_H = 0x43

class Mpu6050CustomNode(Node):
    def __init__(self):
        super().__init__('mpu6050_custom_node')
        
        # 1. Crear el Publisher de ROS 2 (Tópico: /imu/data_raw)
        self.publisher_ = self.create_publisher(Imu, 'imu/data_raw', 10)
        
        # 2. Inicializar el bus I2C (Bus 1, que es el de los pines de tu Rubik Pi)
        self.bus = smbus2.SMBus(1)
        
        # Despertar al MPU6050 (escribir 0 en el registro de energía)
        self.bus.write_byte_data(MPU6050_ADDR, REG_PWR_MGMT_1, 0)
        self.get_logger().info("¡MPU6050 Inicializado y Despierto!")

        # 3. Crear un Timer para leer datos cada 0.05 segundos (20 Hz)
        self.timer = self.create_timer(0.05, self.timer_callback)

    def read_raw_data(self, addr):
        # El sensor devuelve dos bytes por cada eje (High y Low)
        high = self.bus.read_byte_data(MPU6050_ADDR, addr)
        low = self.bus.read_byte_data(MPU6050_ADDR, addr + 1)
        
        # Combinar los dos bytes en un valor de 16 bits
        value = (high << 8) | low
        
        # Convertir a valor con signo (el MPU6050 usa complemento a dos)
        if value > 32768:
            value = value - 65536
        return value

    def timer_callback(self):
        # --- LECTURA DE HARDWARE DESDE CERO ---
        # Leer valores crudos (raw) de la memoria
        raw_ax = self.read_raw_data(REG_ACCEL_XOUT_H)
        raw_ay = self.read_raw_data(REG_ACCEL_XOUT_H + 2)
        raw_az = self.read_raw_data(REG_ACCEL_XOUT_H + 4)
        
        raw_gx = self.read_raw_data(REG_GYRO_XOUT_H)
        raw_gy = self.read_raw_data(REG_GYRO_XOUT_H + 2)
        raw_gz = self.read_raw_data(REG_GYRO_XOUT_H + 4)

        # --- CONVERSIÓN A UNIDADES FÍSICAS REALES ---
        # Por defecto, la escala del acelerómetro es +/- 2g. Factor de conversión: 16384 LSB/g.
        # En ROS 2, la aceleración debe estar en m/s² (1g = 9.80665 m/s²).
        ax = (raw_ax / 16384.0) * 9.80665
        ay = (raw_ay / 16384.0) * 9.80665
        az = (raw_az / 16384.0) * 9.80665

        # Por defecto, la escala del giróscopo es +/- 250 °/s. Factor: 131.0 LSB/(°/s).
        # En ROS 2, la velocidad angular debe estar en Radianes por segundo (Rad/s).
        # Conversión: grados a radianes = grados * (pi / 180)
        import math
        gx = (raw_gx / 131.0) * (math.pi / 180.0)
        gy = (raw_gy / 131.0) * (math.pi / 180.0)
        gz = (raw_gz / 131.0) * (math.pi / 180.0)

        # --- CONSTRUCCIÓN DEL MENSAJE DE ROS 2 ---
        msg = Imu()
        
        # Llenar la cabecera (Header)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'
        
        # Asignar las velocidades angulares (Giróscopo)
        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz
        
        # Asignar las aceleraciones lineales (Acelerómetro)
        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az

        # La orientación (cuaternión) se deja en 0 ya que el MPU6050 crudo no calcula orientación básica
        msg.orientation.w = 1.0 

        # Publicar el mensaje en el tópico
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Mpu6050CustomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
