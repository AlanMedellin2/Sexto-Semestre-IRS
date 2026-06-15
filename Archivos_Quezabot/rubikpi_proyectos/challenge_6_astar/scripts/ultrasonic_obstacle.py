#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Twist
import time

try:
    import gpiod
    GPIOD_AVAILABLE = True
except ImportError:
    GPIOD_AVAILABLE = False

TRIG_LINE = 8    # GPIO4 = Pin físico 7
ECHO_LINE = 24   # GPIO14 = Pin físico 8
GPIO_CHIP  = "/dev/gpiochip4"

STOP_DISTANCE_CM = 10.0


class UltrasonicObstacle(Node):

    def __init__(self):
        super().__init__('ultrasonic_obstacle')

        self.obstacle_pub = self.create_publisher(Bool,    '/obstacle_detected_real',   10)
        self.dist_pub     = self.create_publisher(Float32, '/ultrasonic_distance',  10)
        self.cmd_pub      = self.create_publisher(Twist,   '/cmd_vel',              10)

        self.trig = None
        self.echo = None

        if GPIOD_AVAILABLE:
            try:
                self.trig = gpiod.request_lines(
                    GPIO_CHIP,
                    consumer="ultrasonic_trig",
                    config={TRIG_LINE: gpiod.LineSettings(direction=gpiod.line.Direction.OUTPUT, output_value=gpiod.line.Value.INACTIVE)}
                )
                self.echo = gpiod.request_lines(
                    GPIO_CHIP,
                    consumer="ultrasonic_echo",
                    config={ECHO_LINE: gpiod.LineSettings(direction=gpiod.line.Direction.INPUT)}
                )
                self.get_logger().info(f"gpiod v2 listo — TRIG=GPIO{TRIG_LINE} ECHO=GPIO{ECHO_LINE}")
            except Exception as e:
                self.get_logger().error(f"Error GPIO: {e}")
                self.trig = None
                self.echo = None
        else:
            self.get_logger().warning("gpiod no disponible")

        self.timer = self.create_timer(0.1, self.medir)
        self.get_logger().info("Ultrasonic obstacle node iniciado")

    def medir(self):
        if self.trig is None or self.echo is None:
            return

        try:
            self.trig.set_value(TRIG_LINE, gpiod.line.Value.ACTIVE)
            time.sleep(0.00001)
            self.trig.set_value(TRIG_LINE, gpiod.line.Value.INACTIVE)

            t_start = time.time()
            while self.echo.get_value(ECHO_LINE) == gpiod.line.Value.INACTIVE:
                if time.time() - t_start > 0.04:
                    return
            pulse_start = time.time()

            while self.echo.get_value(ECHO_LINE) == gpiod.line.Value.ACTIVE:
                if time.time() - pulse_start > 0.04:
                    return
            pulse_end = time.time()

            distance = ((pulse_end - pulse_start) * 34300) / 2.0

            dist_msg = Float32()
            dist_msg.data = float(distance)
            self.dist_pub.publish(dist_msg)

            detected = distance < STOP_DISTANCE_CM
            obs_msg = Bool()
            obs_msg.data = detected
            self.obstacle_pub.publish(obs_msg)

            if detected:
                self.cmd_pub.publish(Twist())
                self.get_logger().info(f"OBSTÁCULO a {distance:.1f} cm — STOP")
            else:
                self.get_logger().info(f"Distancia: {distance:.1f} cm")

        except Exception as e:
            self.get_logger().error(f"Error sensor: {e}")

    def destroy_node(self):
        if self.trig: self.trig.release()
        if self.echo: self.echo.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicObstacle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
