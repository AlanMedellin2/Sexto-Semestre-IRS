#!/usr/bin/env python3
"""
fuzzy_follower.py
=================
Reemplaza a line_follower.py en challenge_6_u_fast.

Topics (IDÉNTICOS a los que ya usa tu paquete):
  SUB  /line_error    (std_msgs/Int32)   — del line_detector.py original
  SUB  /yolo/command  (std_msgs/String)  — del yolo_decision_node.py original
  SUB  /color         (std_msgs/Float32) — semáforo del yolo_decision_node.py
  PUB  /cmd_vel       (geometry_msgs/Twist)

Cambios respecto al PID original:
  - El control angular ya NO es un PID → es un sistema difuso
  - La velocidad lineal también es difusa (error grande = va más lento)
  - Las señales YOLO con área grande desaceleran progresivamente
    ANTES de ejecutar la acción (stop, giro)
  - La lógica de semáforo (color) y cooldown YOLO se conserva intacta
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32, String
from geometry_msgs.msg import Twist


# ════════════════════════════════════════════════════════════════════
#  FUNCIONES DE MEMBRESÍA  (no dependen de scikit-fuzzy)
# ════════════════════════════════════════════════════════════════════

def trimf(x, a, b, c):
    if x <= a or x >= c:
        return 0.0
    if x <= b:
        return (x - a) / (b - a + 1e-9)
    return (c - x) / (c - b + 1e-9)


def trapmf(x, a, b, c, d):
    if x <= a or x >= d:
        return 0.0
    if b <= x <= c:
        return 1.0
    if x < b:
        return (x - a) / (b - a + 1e-9)
    return (d - x) / (d - c + 1e-9)


def defuzz(fired):
    """Centroide. fired = lista de (grado, valor_crisp)."""
    num = sum(w * v for w, v in fired)
    den = sum(w       for w, _ in fired)
    return num / den if den > 1e-9 else 0.0


# ════════════════════════════════════════════════════════════════════
#  UNIVERSO 1: error lateral  [-320, 320] píxeles  (Int32)
# ════════════════════════════════════════════════════════════════════

def error_mem(e):
    return {
        'NB': trapmf(e, -320, -320, -150,  -60),
        'NM': trimf (e, -150,  -70,  -10),
        'NS': trimf (e,  -70,  -25,   -5),
        'Z':  trimf (e,  -30,    0,   30),
        'PS': trimf (e,    5,   25,   70),
        'PM': trimf (e,   10,   70,  150),
        'PB': trapmf(e,   60,  150,  320,  320),
    }

# Tabla: error_set → (linear_crisp, angular_crisp)
# linear  [0.0 – 0.10 m/s]  (respeta tu max_linear original)
# angular [−0.5 – 0.5 rad/s] (respeta tu max_angular original)
LINE_RULES = {
    'NB': (0.03,  0.50),
    'NM': (0.06,  0.30),
    'NS': (0.09,  0.15),
    'Z':  (0.10,  0.00),
    'PS': (0.09, -0.15),
    'PM': (0.06, -0.30),
    'PB': (0.03, -0.50),
}


def fuzzy_line(error):
    mem = error_mem(error)
    lf, af = [], []
    for s, (lv, av) in LINE_RULES.items():
        w = mem[s]
        if w > 0:
            lf.append((w, lv))
            af.append((w, av))
    return defuzz(lf), defuzz(af)


# ════════════════════════════════════════════════════════════════════
#  UNIVERSO 2: sign_area  [0.0 – 1.0]
#  → factor de velocidad lineal
# ════════════════════════════════════════════════════════════════════

def area_mem(a):
    return {
        'far':    trapmf(a, 0.00, 0.00, 0.01, 0.04),
        'medium': trimf (a, 0.02, 0.05, 0.10),
        'close':  trapmf(a, 0.07, 0.12, 1.00, 1.00),
    }

# factor 1.0 = sin cambio, 0.0 = parar
SIGN_SPEED = {
    ('stop',         'far'):    1.00,
    ('stop',         'medium'): 0.45,
    ('stop',         'close'):  0.00,
    ('speed_limit_30','far'):   0.80,
    ('speed_limit_30','medium'):0.55,
    ('speed_limit_30','close'): 0.45,
    ('turn_right',   'far'):    1.00,
    ('turn_right',   'medium'): 0.70,
    ('turn_right',   'close'):  0.40,
    ('turn_left',    'far'):    1.00,
    ('turn_left',    'medium'): 0.70,
    ('turn_left',    'close'):  0.40,
}


def fuzzy_sign_factor(sign, area):
    am = area_mem(area)
    fired = []
    for (s, aset), factor in SIGN_SPEED.items():
        if s == sign:
            w = am.get(aset, 0.0)
            if w > 0:
                fired.append((w, factor))
    return defuzz(fired) if fired else 1.0


# ════════════════════════════════════════════════════════════════════
#  NODO
# ════════════════════════════════════════════════════════════════════

class FuzzyFollower(Node):

    def __init__(self):
        super().__init__('line_follower_pid')   # mismo nombre que el original

        # ── Subscriptores (mismos topics que el original) ─────────────────
        self.create_subscription(Int32,  '/line_error',   self._cb_error, 10)
        self.create_subscription(Float32,'/color',        self._cb_color, 10)
        self.create_subscription(String, '/yolo/command', self._cb_yolo,  10)

        # ── Publisher (igual que el original) ─────────────────────────────
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ── Estado ────────────────────────────────────────────────────────
        self.error          = 0.0
        self.last_color     = 3.0    # arranca en rojo (detenido), igual que el original
        self.slow_factor    = 0.35   # igual que el original para amarillo

        self.yolo_command   = 'none'
        self.sign_area      = 0.0    # área del bbox de la señal activa
        self.action_command = 'none'
        self.action_until   = None
        self.cooldown_until = None

        # Rampas (igual que el original)
        self.cur_lin = 0.0
        self.cur_ang = 0.0
        self.lin_acc = 0.02
        self.ang_acc = 0.10

        self.dt = 0.05
        self.create_timer(self.dt, self.control_loop)

        self.get_logger().info('FuzzyFollower iniciado — esperando VERDE')

    # ── Callbacks ────────────────────────────────────────────────────

    def _cb_error(self, msg):
        self.error = float(msg.data)

    def _cb_color(self, msg):
        c = float(msg.data)
        if c != 0.0:
            self.last_color = c

    def _cb_yolo(self, msg):
        cmd = msg.data
        if cmd == 'none':
            return

        now = self._now()
        if self.cooldown_until and now < self.cooldown_until:
            return

        self.yolo_command = cmd

        # Acciones temporizadas (igual que el original)
        if cmd == 'turn_right':
            self.action_command = 'turn_right'
            self.action_until   = now + 1.0
            self.cooldown_until = now + 2.5

        elif cmd == 'turn_left':
            self.action_command = 'turn_left'
            self.action_until   = now + 1.0
            self.cooldown_until = now + 2.5

        elif cmd == 'stop':
            self.action_command = 'stop'
            self.action_until   = now + 1.5
            self.cooldown_until = now + 2.5

        elif cmd == 'speed_limit_30':
            self.action_command = 'slow'
            self.action_until   = now + 3.0
            self.cooldown_until = now + 3.5

        self.get_logger().info(f'YOLO: {cmd}')

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    # ── Loop principal ────────────────────────────────────────────────

    def control_loop(self):
        # 1. Sistema difuso de línea
        v_line, w_line = fuzzy_line(self.error)

        # 2. Factor difuso por señal y distancia estimada
        #    (usa el comando YOLO activo como señal)
        active_sign = self.action_command if self.action_command != 'none' \
                      else self.yolo_command
        factor = fuzzy_sign_factor(active_sign, self.sign_area)

        target_lin = v_line * factor
        target_ang = w_line

        # 3. Rampas (igual que el original)
        self.cur_lin = self._ramp(target_lin, self.cur_lin, self.lin_acc)
        self.cur_ang = self._ramp(target_ang, self.cur_ang, self.ang_acc)

        # 4. Lógica de semáforo (idéntica al original)
        estado = 'DIFUSO'
        if self.last_color == 3.0:
            self.cur_lin = 0.0
            self.cur_ang = 0.0
            estado = 'ROJO-STOP'
        elif self.last_color == 1.0:
            self.cur_lin *= self.slow_factor
            self.cur_ang *= self.slow_factor
            estado = 'AMARILLO-LENTO'

        # 5. Acciones YOLO temporizadas (idénticas al original)
        now = self._now()
        if self.action_until and now < self.action_until:
            if self.action_command == 'turn_right':
                self.cur_lin = 0.04
                self.cur_ang = -0.35
                estado = 'YOLO-TURN_RIGHT'
            elif self.action_command == 'turn_left':
                self.cur_lin = 0.04
                self.cur_ang = 0.35
                estado = 'YOLO-TURN_LEFT'
            elif self.action_command == 'stop':
                self.cur_lin = 0.0
                self.cur_ang = 0.0
                estado = 'YOLO-STOP'
            elif self.action_command == 'slow':
                self.cur_lin *= 0.5
                self.cur_ang *= 0.5
                estado = 'YOLO-SLOW'
        elif self.action_until and now >= self.action_until:
            self.action_until   = None
            self.action_command = 'none'

        # 6. Publicar
        cmd = Twist()
        cmd.linear.x  = float(max(0.0, min(self.cur_lin, 0.10)))
        cmd.angular.z = float(max(-0.5, min(self.cur_ang, 0.5)))
        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f'err={self.error:.0f} | {estado} | '
            f'factor={factor:.2f} | '
            f'v={cmd.linear.x:.3f} w={cmd.angular.z:.3f}'
        )

    def _ramp(self, target, current, step):
        if target > current:
            return min(current + step, target)
        if target < current:
            return max(current - step, target)
        return current


def main(args=None):
    rclpy.init(args=args)
    node = FuzzyFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop = Twist()
        node.cmd_pub.publish(stop)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
