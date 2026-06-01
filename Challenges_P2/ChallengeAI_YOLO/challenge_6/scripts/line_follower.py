#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32, String
from geometry_msgs.msg import Twist


class LineFollowerPID(Node):

    def __init__(self):
        super().__init__('line_follower_pid')

        self.subscription = self.create_subscription(Int32, '/line_error', self.error_callback, 10)
        self.color_subscription = self.create_subscription(Float32, '/color', self.color_callback, 10)
        self.yolo_subscription = self.create_subscription(String, '/yolo/command', self.yolo_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.kp = 0.1
        self.ki = 0.0
        self.kd = 0.1
        self.error = 0.0
        self.prev_error = 0.0
        self.integral = 0.0
        self.dt = 0.05

        self.base_speed   = 0.10
        self.deadband     = 30
        self.max_linear   = 0.10
        self.max_angular  = 0.5
        self.linear_accel = 0.02
        self.angular_accel = 0.10

        self.current_linear  = 0.0
        self.current_angular = 0.0

        self.last_valid_color = 3.0
        self.slow_factor = 0.35

        self.action_command = "none"
        self.action_until   = None
        self.cooldown_until = None

        self.TURN_ADVANCE_TIME = 0.3
        self.TURN_ROTATE_TIME  = 3.2
        self.TURN_LINEAR       = 0.08
        self.TURN_ANGULAR      = 0.90
        self.TURNAROUND_TIME   = 5.5

        self.turn_phase     = "none"
        self.turn_direction = 0
        self.turn_phase_end = None

        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info("Line follower iniciado")

    def now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def error_callback(self, msg):
        self.error = float(msg.data)

    def color_callback(self, msg):
        color = float(msg.data)
        if color != 0.0:
            self.last_valid_color = color
            if color == 1.0:
                self.get_logger().info("AMARILLO: lento")
            elif color == 2.0:
                self.get_logger().info("VERDE: avanzar")
            elif color == 3.0:
                self.get_logger().info("ROJO: stop")

    def yolo_callback(self, msg):
        cmd = msg.data
        if cmd == "none":
            return

        now = self.now_sec()

        if self.cooldown_until is not None and now < self.cooldown_until:
            return

        if cmd == "turn_right":
            self.turn_direction  = -1
            self.turn_phase      = "advance"
            self.turn_phase_end  = now + self.TURN_ADVANCE_TIME
            total = self.TURN_ADVANCE_TIME + self.TURN_ROTATE_TIME
            self.action_command  = "turn_right"
            self.action_until    = now + total
            self.cooldown_until  = now + total + 3.0
            self.get_logger().info("YOLO: TURN RIGHT")

        elif cmd == "turn_left":
            self.turn_direction  = +1
            self.turn_phase      = "advance"
            self.turn_phase_end  = now + self.TURN_ADVANCE_TIME
            total = self.TURN_ADVANCE_TIME + self.TURN_ROTATE_TIME
            self.action_command  = "turn_left"
            self.action_until    = now + total
            self.cooldown_until  = now + total + 3.0
            self.get_logger().info("YOLO: TURN LEFT")

        elif cmd == "stop":
            self.action_command  = "stop"
            self.action_until    = now + 999.0
            self.cooldown_until  = now + 5.0
            self.get_logger().info("YOLO: STOP — esperando verde")

        elif cmd == "roadwork_ahead":
            self.action_command  = "slow"
            self.action_until    = now + 4.0
            self.cooldown_until  = now + 5.0
            self.get_logger().info("YOLO: ROADWORK — lento")

        elif cmd == "give_way":
            self.action_command  = "give_way"
            self.action_until    = now + 3.0
            self.cooldown_until  = now + 4.0
            self.get_logger().info("YOLO: GIVE WAY — cediendo paso")

        elif cmd == "turn_around":
            self.turn_direction  = +1
            self.turn_phase      = "advance"
            self.turn_phase_end  = now + self.TURN_ADVANCE_TIME
            total = self.TURN_ADVANCE_TIME + self.TURNAROUND_TIME
            self.action_command  = "turn_around"
            self.action_until    = now + total
            self.cooldown_until  = now + total + 3.0
            self.get_logger().info("YOLO: TURN AROUND")

        elif cmd == "straight":
            self.get_logger().info("YOLO: STRAIGHT")

    def saturate(self, value, limit):
        return max(-limit, min(limit, value))

    def ramp(self, target, current, step):
        if target > current:
            return min(current + step, target)
        elif target < current:
            return max(current - step, target)
        return current

    def control_loop(self):
        proportional = self.error
        self.integral += self.error * self.dt
        derivative = (self.error - self.prev_error) / self.dt
        angular_pid = self.kp * proportional + self.ki * self.integral + self.kd * derivative
        self.prev_error = self.error
        angular_pid = self.saturate(angular_pid, self.max_angular)

        if abs(self.error) < self.deadband:
            target_linear  = self.base_speed
            target_angular = 0.0
        else:
            target_linear  = 0.0 if abs(self.error) > 150 else self.base_speed
            target_angular = -angular_pid

        self.current_linear  = self.ramp(target_linear,  self.current_linear,  self.linear_accel)
        self.current_angular = self.ramp(target_angular, self.current_angular, self.angular_accel)
        self.current_linear  = self.saturate(self.current_linear,  self.max_linear)
        self.current_angular = self.saturate(self.current_angular, self.max_angular)

        estado = "VERDE - SIGUE"

        if self.last_valid_color == 3.0:
            self.current_linear  = 0.0
            self.current_angular = 0.0
            estado = "ROJO - STOP"

        elif self.last_valid_color == 1.0:
            self.current_linear  *= self.slow_factor
            self.current_angular *= self.slow_factor
            estado = "AMARILLO - LENTO"

        now = self.now_sec()

        if self.action_until is not None and now < self.action_until:

            if self.action_command in ("turn_right", "turn_left"):
                if self.turn_phase == "advance":
                    self.current_linear  = self.TURN_LINEAR
                    self.current_angular = 0.0
                    estado = f"{self.action_command.upper()} ADVANCE"
                    if now >= self.turn_phase_end:
                        self.turn_phase     = "rotate"
                        self.turn_phase_end = now + self.TURN_ROTATE_TIME
                        self.get_logger().info(f"{self.action_command.upper()} → ROTATE")
                elif self.turn_phase == "rotate":
                    self.current_linear  = 0.0
                    self.current_angular = self.TURN_ANGULAR * self.turn_direction
                    estado = f"{self.action_command.upper()} ROTATE"
                    if now >= self.turn_phase_end:
                        self.turn_phase     = "none"
                        self.action_command = "none"
                        self.action_until   = None
                        self.get_logger().info("GIRO completado → PID")

            elif self.action_command == "stop":
                self.current_linear  = 0.0
                self.current_angular = 0.0
                estado = "YOLO STOP"
                if self.last_valid_color == 2.0:
                    self.action_command = "none"
                    self.action_until   = None
                    self.get_logger().info("STOP liberado por VERDE")

            elif self.action_command == "slow":
                self.current_linear  *= 0.5
                self.current_angular *= 0.5
                estado = "YOLO SLOW"

            elif self.action_command == "give_way":
                self.current_linear  *= 0.3
                self.current_angular *= 0.5
                estado = "YOLO GIVE WAY"

            elif self.action_command == "turn_around":
                if self.turn_phase == "advance":
                    self.current_linear  = self.TURN_LINEAR
                    self.current_angular = 0.0
                    estado = "TURN AROUND ADVANCE"
                    if now >= self.turn_phase_end:
                        self.turn_phase     = "rotate"
                        self.turn_phase_end = now + self.TURNAROUND_TIME
                        self.get_logger().info("TURN AROUND → ROTATE")
                elif self.turn_phase == "rotate":
                    self.current_linear  = 0.0
                    self.current_angular = self.TURN_ANGULAR * self.turn_direction
                    estado = "TURN AROUND ROTATE"
                    if now >= self.turn_phase_end:
                        self.turn_phase     = "none"
                        self.action_command = "none"
                        self.action_until   = None
                        self.get_logger().info("TURN AROUND completado → PID")

        elif self.action_until is not None and now >= self.action_until:
            self.action_until   = None
            self.action_command = "none"

        cmd = Twist()
        cmd.linear.x  = self.current_linear
        cmd.angular.z = self.current_angular
        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f'Error:{self.error:.0f} | {estado} | '
            f'YOLO:{self.action_command} | '
            f'Lin:{self.current_linear:.2f} Ang:{self.current_angular:.2f}'
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
