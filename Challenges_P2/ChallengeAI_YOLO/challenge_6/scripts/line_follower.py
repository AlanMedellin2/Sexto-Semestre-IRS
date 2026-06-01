#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32, String
from geometry_msgs.msg import Twist


class LineFollowerPID(Node):

    def __init__(self):
        super().__init__('line_follower_pid')

        self.subscription = self.create_subscription(
            Int32,
            '/line_error',
            self.error_callback,
            10
        )

        self.color_subscription = self.create_subscription(
            Float32,
            '/color',
            self.color_callback,
            10
        )

        self.yolo_subscription = self.create_subscription(
            String,
            '/yolo/command',
            self.yolo_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # PID
        self.kp = 0.1
        self.ki = 0.0
        self.kd = 0.1

        self.error = 0.0
        self.prev_error = 0.0
        self.integral = 0.0
        self.dt = 0.05

        # Robot
        self.base_speed = 0.10
        self.deadband = 30
        self.max_linear = 0.10
        self.max_angular = 0.5

        # Rampas
        self.current_linear = 0.0
        self.current_angular = 0.0
        self.linear_accel = 0.02
        self.angular_accel = 0.10

        # Semáforo:
        # 0.0 = nada
        # 1.0 = amarillo
        # 2.0 = verde
        # 3.0 = rojo
        #
        # Inicia detenido hasta ver verde
        self.last_valid_color = 3.0
        self.slow_factor = 0.35

        # YOLO
        self.yolo_command = "none"
        self.action_until = None
        self.action_command = "none"
        self.cooldown_until = None

        # Giro en 2 fases:
        # Fase 1: avanza para pasar la intersección (TURN_ADVANCE_TIME s)
        # Fase 2: gira en su lugar hasta completar ~90°  (TURN_ROTATE_TIME s)
        self.TURN_ADVANCE_TIME = 0.3   # segundos avanzando antes de girar
        self.TURN_ROTATE_TIME  = 2.8   # segundos girando (ajusta en pista)
        self.TURN_LINEAR       = 0.08  # velocidad avanzando en fase 1
        self.TURN_ANGULAR      = 0.75  # velocidad angular girando en fase 2
        self.TURNAROUND_TIME   = 4.5   # segundos para vuelta en U (~180°)

        self.turn_phase     = "none"   # "advance" | "rotate" | "none"
        self.turn_direction = 0        # +1 izquierda, -1 derecha
        self.turn_phase_end = None

        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info("Line follower iniciado: esperando bandera VERDE + YOLO")

    def now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def error_callback(self, msg):
        self.error = float(msg.data)

    def color_callback(self, msg):
        color = float(msg.data)

        # Si no detecta nada, mantiene la última instrucción válida
        if color != 0.0:
            self.last_valid_color = color

            if color == 1.0:
                self.get_logger().info("Bandera AMARILLA detectada: lento")
            elif color == 2.0:
                self.get_logger().info("Bandera VERDE detectada: avanzar")
            elif color == 3.0:
                self.get_logger().info("Bandera ROJA detectada: stop")

    def yolo_callback(self, msg):
        cmd = msg.data

        if cmd == "none":
            return

        now = self.now_sec()

        if self.cooldown_until is not None and now < self.cooldown_until:
            return

        self.yolo_command = cmd

        if cmd == "turn_right":
            self.turn_direction = -1
            self.turn_phase = "advance"
            self.turn_phase_end = now + self.TURN_ADVANCE_TIME
            total = self.TURN_ADVANCE_TIME + self.TURN_ROTATE_TIME
            self.cooldown_until = now + total + 3.0
            self.get_logger().info("YOLO confirmado: TURN RIGHT → fase advance")

        elif cmd == "turn_left":
            self.turn_direction = +1
            self.turn_phase = "advance"
            self.turn_phase_end = now + self.TURN_ADVANCE_TIME
            total = self.TURN_ADVANCE_TIME + self.TURN_ROTATE_TIME
            self.cooldown_until = now + total + 3.0
            self.get_logger().info("YOLO confirmado: TURN LEFT → fase advance")

        elif cmd == "stop":
            self.action_command = "stop"
            self.action_until = now + 1.5
            self.cooldown_until = now + 2.5
            self.get_logger().info("YOLO confirmado: STOP")

        elif cmd == "roadwork_ahead":
            self.action_command = "slow"
            self.action_until = now + 3.0
            self.cooldown_until = now + 3.5
            self.get_logger().info("YOLO confirmado: ROADWORK AHEAD - lento")

        elif cmd == "give_way":
            self.action_command = "slow"
            self.action_until = now + 2.0
            self.cooldown_until = now + 3.0
            self.get_logger().info("YOLO confirmado: GIVE WAY - lento")

        elif cmd == "turn_around":
            self.turn_direction = +1
            self.turn_phase = "advance"
            self.turn_phase_end = now + self.TURN_ADVANCE_TIME
            self.action_command = "turn_around"
            total = self.TURN_ADVANCE_TIME + self.TURNAROUND_TIME
            self.cooldown_until = now + total + 3.0
            self.get_logger().info("YOLO confirmado: TURN AROUND → fase advance")

        elif cmd == "straight":
            # No cambia el control, solo deja que siga la línea.
            self.get_logger().info("YOLO confirmado: STRAIGHT, sigue línea")

    def saturate(self, value, limit):
        return max(-limit, min(limit, value))

    def ramp(self, target, current, step):
        if target > current:
            current = min(current + step, target)
        elif target < current:
            current = max(current - step, target)
        return current

    def control_loop(self):

        proportional = self.error
        self.integral += self.error * self.dt
        derivative = (self.error - self.prev_error) / self.dt

        angular_pid = (
            self.kp * proportional +
            self.ki * self.integral +
            self.kd * derivative
        )

        self.prev_error = self.error
        angular_pid = self.saturate(angular_pid, self.max_angular)

        if abs(self.error) < self.deadband:
            target_linear = self.base_speed
            target_angular = 0.0
        else:
            target_linear = 0.0 if abs(self.error) > 150 else self.base_speed
            target_angular = -angular_pid

        self.current_linear = self.ramp(
            target_linear,
            self.current_linear,
            self.linear_accel
        )

        self.current_angular = self.ramp(
            target_angular,
            self.current_angular,
            self.angular_accel
        )

        self.current_linear = self.saturate(self.current_linear, self.max_linear)
        self.current_angular = self.saturate(self.current_angular, self.max_angular)

        # ==========================
        # Lógica de color ORIGINAL
        # ==========================
        if self.last_valid_color == 3.0:
            self.current_linear = 0.0
            self.current_angular = 0.0
            estado_color = "ROJO/INICIO - STOP"

        elif self.last_valid_color == 1.0:
            self.current_linear *= self.slow_factor
            self.current_angular *= self.slow_factor
            estado_color = "AMARILLO - LENTO"

        elif self.last_valid_color == 2.0:
            estado_color = "VERDE - SIGUE"

        else:
            self.current_linear = 0.0
            self.current_angular = 0.0
            estado_color = "SIN ESTADO - STOP"

        # ==========================
        # Acciones YOLO confirmadas
        # ==========================
        now = self.now_sec()

        if self.action_until is not None and now < self.action_until:

            if self.action_command in ("turn_right", "turn_left"):
                # Fase 1: avanzar para cruzar la intersección
                if self.turn_phase == "advance":
                    self.current_linear  = self.TURN_LINEAR
                    self.current_angular = 0.0
                    estado_color = f"YOLO {self.action_command.upper()} fase ADVANCE"
                    if now >= self.turn_phase_end:
                        self.turn_phase = "rotate"
                        self.turn_phase_end = now + self.TURN_ROTATE_TIME
                        self.get_logger().info(f"{self.action_command.upper()} → fase ROTATE")
                # Fase 2: girar limpio
                elif self.turn_phase == "rotate":
                    self.current_linear  = 0.0
                    self.current_angular = self.TURN_ANGULAR * self.turn_direction
                    estado_color = f"YOLO {self.action_command.upper()} fase ROTATE"
                    if now >= self.turn_phase_end:
                        self.turn_phase = "none"
                        self.action_command = "none"
                        self.action_until = None
                        self.get_logger().info(f"{self.action_command.upper()} completado → PID")

            elif self.action_command == "stop":
                self.current_linear = 0.0
                self.current_angular = 0.0
                estado_color = "YOLO STOP"

            elif self.action_command == "slow":
                self.current_linear *= 0.5
                self.current_angular *= 0.5
                estado_color = "YOLO SPEED LIMIT - LENTO"

            elif self.action_command == "turn_around":
                if self.turn_phase == "advance":
                    self.current_linear  = self.TURN_LINEAR
                    self.current_angular = 0.0
                    estado_color = "YOLO TURN AROUND fase ADVANCE"
                    if now >= self.turn_phase_end:
                        self.turn_phase = "rotate"
                        self.turn_phase_end = now + self.TURNAROUND_TIME
                        self.get_logger().info("TURN AROUND → fase ROTATE")
                elif self.turn_phase == "rotate":
                    self.current_linear  = 0.0
                    self.current_angular = self.TURN_ANGULAR * self.turn_direction
                    estado_color = "YOLO TURN AROUND fase ROTATE"
                    if now >= self.turn_phase_end:
                        self.turn_phase = "none"
                        self.action_command = "none"
                        self.action_until = None
                        self.get_logger().info("TURN AROUND completado → PID")

        elif self.action_until is not None and now >= self.action_until:
            self.action_until = None
            self.action_command = "none"

        cmd = Twist()
        cmd.linear.x = self.current_linear
        cmd.angular.z = self.current_angular
        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f'Error: {self.error:.1f} | '
            f'Color/Estado: {estado_color} | '
            f'YOLO: {self.yolo_command} | '
            f'Lin: {self.current_linear:.2f} | '
            f'Ang: {self.current_angular:.2f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerPID()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        stop_cmd = Twist()
        node.cmd_pub.publish(stop_cmd)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
