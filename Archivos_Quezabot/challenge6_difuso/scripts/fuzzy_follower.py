#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32, String, Bool
from geometry_msgs.msg import Twist
import numpy as np


# ================================================================
# FUNCIONES DE MEMBRESÍA
# ================================================================

def trimf(x, a, b, c):
    if x <= a or x >= c: return 0.0
    if x <= b: return (x-a)/(b-a) if b!=a else 1.0
    return (c-x)/(c-b) if c!=b else 1.0

def trapmf(x, a, b, c, d):
    if x <= a or x >= d: return 0.0
    if x <= b: return (x-a)/(b-a) if b!=a else 1.0
    if x <= c: return 1.0
    return (d-x)/(d-c) if d!=c else 1.0


# ================================================================
# SISTEMA DIFUSO 1: SEGUIMIENTO DE LÍNEA
# Error [-320,320] → angular_z [-0.5, 0.5]
# ================================================================

def mu_eNL(e): return trapmf(e,-320,-320,-100,-30)
def mu_eNM(e): return trimf(e, -100, -50,  -5)
def mu_eZ(e):  return trimf(e,  -25,   0,  25)
def mu_ePM(e): return trimf(e,    5,  50, 100)
def mu_ePL(e): return trapmf(e,  30, 100, 320, 320)

OUT_U = np.linspace(-0.5, 0.5, 200)
def mu_NL(u): return trapmf(u,-0.5,-0.5,-0.30,-0.12)
def mu_NM(u): return trimf(u, -0.28,-0.18,-0.04)
def mu_Z(u):  return trimf(u,  -0.08, 0.0,  0.08)
def mu_PM(u): return trimf(u,   0.04, 0.18, 0.28)
def mu_PL(u): return trapmf(u,  0.12, 0.30, 0.5, 0.5)

OUT_SETS = [mu_NL, mu_NM, mu_Z, mu_PM, mu_PL]

def defuzz(activations):
    agg = np.zeros(len(OUT_U))
    for mu_f, alpha in activations:
        if alpha > 0:
            agg = np.maximum(agg, alpha * np.array([mu_f(u) for u in OUT_U]))
    d = np.sum(agg)
    return float(np.dot(OUT_U, agg)/d) if d > 1e-6 else 0.0

def fuzzy_line(error):
    eNL=mu_eNL(error); eNM=mu_eNM(error); eZ=mu_eZ(error)
    ePM=mu_ePM(error); ePL=mu_ePL(error)
    rules = [
        (mu_NL, ePL),
        (mu_NM, ePM),
        (mu_Z,  eZ),
        (mu_PM, eNM),
        (mu_PL, eNL),
    ]
    return defuzz(rules)


# ================================================================
# SISTEMA DIFUSO 2: VELOCIDAD SEGÚN ÁREA DE SEÑAL Y ERROR DE LÍNEA
# Entradas: area_norm [0,1], error_norm [0,1]
# Salida:   speed_factor [0.3, 1.0]
# ================================================================

def mu_area_LEJOS(a):  return trapmf(a, 0.0, 0.0, 0.10, 0.25)
def mu_area_MEDIA(a):  return trimf(a,  0.15, 0.35, 0.60)
def mu_area_CERCA(a):  return trapmf(a, 0.45, 0.70, 1.0, 1.0)

def mu_err_BAJO(e):   return trapmf(e, 0.0, 0.0, 0.15, 0.30)
def mu_err_MEDIO(e):  return trimf(e,  0.20, 0.40, 0.65)
def mu_err_ALTO(e):   return trapmf(e, 0.55, 0.75, 1.0, 1.0)

SPD_U = np.linspace(0.3, 1.0, 100)
def mu_spd_MUY_LENTO(s): return trapmf(s, 0.3, 0.3, 0.40, 0.52)
def mu_spd_LENTO(s):     return trimf(s,  0.42, 0.58, 0.72)
def mu_spd_NORMAL(s):    return trapmf(s, 0.65, 0.80, 1.0, 1.0)

def defuzz_speed(activations):
    agg = np.zeros(len(SPD_U))
    for mu_f, alpha in activations:
        if alpha > 0:
            agg = np.maximum(agg, alpha * np.array([mu_f(s) for s in SPD_U]))
    d = np.sum(agg)
    return float(np.dot(SPD_U, agg)/d) if d > 1e-6 else 0.85

def fuzzy_speed(area_norm, error_norm):
    """
    Reglas:
    1. area=LEJOS                     → speed=NORMAL
    2. area=MEDIA, error=BAJO         → speed=NORMAL
    3. area=MEDIA, error=MEDIO        → speed=LENTO
    4. area=MEDIA, error=ALTO         → speed=MUY_LENTO
    5. area=CERCA, error=BAJO         → speed=LENTO
    6. area=CERCA, error=MEDIO/ALTO   → speed=MUY_LENTO
    """
    aL = mu_area_LEJOS(area_norm)
    aM = mu_area_MEDIA(area_norm)
    aC = mu_area_CERCA(area_norm)
    eB = mu_err_BAJO(error_norm)
    eM = mu_err_MEDIO(error_norm)
    eA = mu_err_ALTO(error_norm)

    rules = [
        (mu_spd_NORMAL,    aL),                        # R1
        (mu_spd_NORMAL,    min(aM, eB)),               # R2
        (mu_spd_LENTO,     min(aM, eM)),               # R3
        (mu_spd_MUY_LENTO, min(aM, eA)),               # R4
        (mu_spd_LENTO,     min(aC, eB)),               # R5
        (mu_spd_MUY_LENTO, min(aC, max(eM, eA))),      # R6
    ]
    return defuzz_speed(rules)


# ================================================================
# NODO ROS 2
# ================================================================

class FuzzyFollower(Node):

    def __init__(self):
        super().__init__('fuzzy_controller')

        self.sub_error  = self.create_subscription(Int32,   '/line_error',       self.error_cb,   10)
        self.sub_color  = self.create_subscription(Float32, '/color',             self.color_cb,   10)
        self.sub_yolo   = self.create_subscription(String,  '/yolo/command',      self.yolo_cb,    10)
        self.sub_finish = self.create_subscription(Bool,    '/finish_line',       self.finish_cb,  10)
        self.sub_inter  = self.create_subscription(Bool,    '/intersection_line', self.inter_cb,   10)
        self.sub_area   = self.create_subscription(Float32, '/yolo/sign_area',    self.area_cb,    10)
        self.sub_obs    = self.create_subscription(Bool,    '/obstacle_detected', self.obs_cb,     10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.dt = 0.05

        self.error            = 0.0
        self.last_valid_color = 3.0
        self.finished         = False
        self.obstacle         = False
        self.sign_area_raw    = 0.0
        self.give_way_active  = False

        self.action_command  = "none"
        self.action_phase    = "none"
        self.phase_end       = None
        self.cooldown_until  = None
        self.turn_direction  = 0

        self.PAUSE_TIME            = 1.0
        self.ADVANCE_TIME_VERDE    = 5.0
        self.ADVANCE_TIME_AMARILLO = 1.5
        self.ROTATE_TIME           = 3.0
        self.TURN_LINEAR           = 0.08
        self.TURN_ANGULAR          = 0.45
        self.INSTANT_WAIT          = 2.0

        self.base_speed    = 0.09
        self.max_linear    = 0.09
        self.max_angular   = 0.50
        self.slow_factor   = 0.55
        self.deadband      = 25

        self.current_linear  = 0.0
        self.current_angular = 0.0
        self.linear_accel    = 0.02
        self.angular_accel   = 0.12

        self.AREA_NORM_MAX = 30000.0

        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info("Fuzzy follower iniciado")

    def now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def error_cb(self, msg):
        self.error = float(msg.data)

    def color_cb(self, msg):
        c = float(msg.data)
        if c != 0.0:
            self.last_valid_color = c

    def finish_cb(self, msg):
        if msg.data and not self.finished:
            self.finished = True
            self.get_logger().info("META — detenido")

    def obs_cb(self, msg):
        self.obstacle = bool(msg.data)
        if self.obstacle:
            self.get_logger().info("OBSTÁCULO detectado — pausado")

    def inter_cb(self, msg):
        if not msg.data:
            return

        # Si give_way activo, intersección lo cancela
        if self.give_way_active:
            self.give_way_active = False
            self.action_command  = "none"
            self.action_phase    = "none"
            self.get_logger().info("GIVE WAY cancelado por intersección → FUZZY")
            return

        if self.action_command in ("turn_right", "turn_left", "straight") and self.action_phase == "none":
            advance = self.ADVANCE_TIME_AMARILLO if self.last_valid_color == 1.0 else self.ADVANCE_TIME_VERDE
            self.action_phase = "advance"
            self.phase_end    = self.now_sec() + advance
            self.get_logger().info(f"INTERSECCIÓN — {self.action_command.upper()} → ADVANCE {advance}s")

    def area_cb(self, msg):
        self.sign_area_raw = float(msg.data)

    def yolo_cb(self, msg):
        cmd = msg.data
        if cmd == "none":
            return
        now = self.now_sec()
        if self.cooldown_until is not None and now < self.cooldown_until:
            return

        if cmd == "turn_right":
            self.action_command = "turn_right"
            self.turn_direction = -1
            self.action_phase   = "none"
            self.cooldown_until = now + self.ADVANCE_TIME_VERDE + self.PAUSE_TIME + self.ROTATE_TIME + 5.0
            self.get_logger().info("YOLO: TURN RIGHT")

        elif cmd == "turn_left":
            self.action_command = "turn_left"
            self.turn_direction = +1
            self.action_phase   = "none"
            self.cooldown_until = now + self.ADVANCE_TIME_VERDE + self.PAUSE_TIME + self.ROTATE_TIME + 5.0
            self.get_logger().info("YOLO: TURN LEFT")

        elif cmd == "stop":
            self.action_command = "stop"
            self.action_phase   = "active"
            self.phase_end      = now + 3.0
            self.cooldown_until = now + 5.0
            self.get_logger().info("YOLO: STOP 3s")

        elif cmd == "roadwork_ahead":
            self.action_command = "slow"
            self.action_phase   = "active"
            self.phase_end      = now + 8.0
            self.cooldown_until = now + 10.0
            self.get_logger().info("YOLO: ROADWORK — lento 8s")

        elif cmd == "give_way":
            self.action_command  = "give_way"
            self.action_phase    = "wait"
            self.phase_end       = now + self.INSTANT_WAIT
            self.give_way_active = True
            self.cooldown_until  = now + self.INSTANT_WAIT + 30.0
            self.get_logger().info("YOLO: GIVE WAY — lento hasta intersección")

        elif cmd == "straight":
            self.action_command = "straight"
            self.action_phase   = "none"
            self.cooldown_until = now + self.ADVANCE_TIME_VERDE + 3.0
            self.get_logger().info("YOLO: STRAIGHT")

    def saturate(self, v, lim):
        return max(-lim, min(lim, v))

    def ramp(self, target, current, step):
        if target > current: return min(current+step, target)
        if target < current: return max(current-step, target)
        return current

    def control_loop(self):
        now = self.now_sec()

        if self.finished:
            self.cmd_pub.publish(Twist())
            return

        if self.obstacle:
            self.cmd_pub.publish(Twist())
            self.get_logger().info("OBSTÁCULO — detenido")
            return

        if self.last_valid_color == 3.0:
            self.cmd_pub.publish(Twist())
            self.get_logger().info("ROJO-STOP")
            return

        # ── GIVE WAY ACTIVE: prioridad sobre semáforo ──
        if self.action_command == "give_way" and self.action_phase == "active":
            if self.last_valid_color == 3.0:
                self.cmd_pub.publish(Twist())
                return
            angular_fuzzy = fuzzy_line(self.error)
            cmd = Twist()
            cmd.linear.x  = self.base_speed * 0.45
            cmd.angular.z = self.saturate(angular_fuzzy * 0.5, self.max_angular)
            self.cmd_pub.publish(cmd)
            self.get_logger().info(f"GIVE_WAY-SLOW|Lin:{cmd.linear.x:.2f}|Ang:{cmd.angular.z:.2f}")
            return

        # ── CONTROL DIFUSO DE VELOCIDAD POR SEÑAL ──
        area_norm  = min(1.0, self.sign_area_raw / self.AREA_NORM_MAX)
        error_norm = min(1.0, abs(self.error) / 320.0)
        speed_factor = fuzzy_speed(area_norm, error_norm) if self.action_command != "none" else 1.0

        # ── CONTROL DIFUSO DE LÍNEA ──
        angular_fuzzy = fuzzy_line(self.error)

        if abs(self.error) < self.deadband:
            target_linear  = self.base_speed * speed_factor
            target_angular = 0.0
        else:
            target_linear  = self.base_speed * speed_factor
            target_angular = self.saturate(angular_fuzzy, self.max_angular)

        # Semáforo amarillo
        if self.last_valid_color == 1.0:
            target_linear  *= self.slow_factor
            target_angular *= self.slow_factor
            estado = "AMARILLO-LENTO"
        else:
            estado = "FUZZY"

        self.current_linear  = self.ramp(target_linear,  self.current_linear,  self.linear_accel)
        self.current_angular = self.ramp(target_angular, self.current_angular, self.angular_accel)

        # ── ACCIONES YOLO ──
        if self.action_command != "none" and self.action_phase != "none":

            if self.action_command == "stop":
                self.current_linear  = 0.0
                self.current_angular = 0.0
                estado = "YOLO-STOP"
                if now >= self.phase_end:
                    self.action_command = "none"
                    self.action_phase   = "none"
                    self.get_logger().info("STOP completado → FUZZY")

            elif self.action_command in ("turn_right", "turn_left"):
                if self.action_phase == "advance":
                    self.current_linear  = self.TURN_LINEAR
                    self.current_angular = 0.0
                    estado = f"{self.action_command.upper()}-ADVANCE"
                    if now >= self.phase_end:
                        self.action_phase = "pause"
                        self.phase_end    = now + self.PAUSE_TIME

                elif self.action_phase == "pause":
                    self.current_linear  = 0.0
                    self.current_angular = 0.0
                    estado = f"{self.action_command.upper()}-PAUSE"
                    if now >= self.phase_end:
                        self.action_phase = "rotate"
                        self.phase_end    = now + self.ROTATE_TIME

                elif self.action_phase == "rotate":
                    self.current_linear  = 0.0
                    self.current_angular = self.TURN_ANGULAR * self.turn_direction
                    estado = f"{self.action_command.upper()}-ROTATE"
                    if now >= self.phase_end:
                        self.action_command = "none"
                        self.action_phase   = "none"
                        self.get_logger().info("GIRO completado → FUZZY")

            elif self.action_command == "straight":
                if self.action_phase == "advance":
                    self.current_linear  = self.TURN_LINEAR
                    self.current_angular = 0.0
                    estado = "STRAIGHT-ADVANCE"
                    if now >= self.phase_end:
                        self.action_command = "none"
                        self.action_phase   = "none"

            elif self.action_command == "slow":
                if self.action_phase == "active":
                    self.current_linear  *= 0.65
                    self.current_angular *= 0.5
                    estado = "ROADWORK-SLOW"
                    if now >= self.phase_end:
                        self.action_command = "none"
                        self.action_phase   = "none"

            elif self.action_command == "give_way":
                if self.action_phase == "wait":
                    estado = "GIVE_WAY-WAIT"
                    if now >= self.phase_end:
                        self.action_phase = "active"
                        self.phase_end    = None
                        self.get_logger().info("GIVE WAY → ACTIVE lento hasta intersección")

        cmd = Twist()
        cmd.linear.x  = self.saturate(self.current_linear,  self.max_linear)
        cmd.angular.z = self.saturate(self.current_angular, self.max_angular)
        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f"E:{self.error:.0f}|{estado}|"
            f"area:{area_norm:.2f} spd_f:{speed_factor:.2f}|"
            f"fuz:{angular_fuzzy:.3f}|"
            f"Lin:{cmd.linear.x:.2f}|Ang:{cmd.angular.z:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = FuzzyFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
