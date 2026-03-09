import roboticstoolbox as rtb
import numpy as np
from spatialmath import SE3
from math import pi

# ================================
# 1. PARÁMETROS DH (en metros)
# ================================
d1 = 0.089
a2 = -0.425
a3 = -0.392
d4 = 0.109
d5 = 0.094
d6 = 0.082

# ================================
# 2. CREACIÓN DEL ROBOT
# ================================
robot = rtb.DHRobot([

    rtb.RevoluteDH(d=d1, a=0,     alpha=pi/2),
    rtb.RevoluteDH(d=0,  a=a2,    alpha=0),
    rtb.RevoluteDH(d=0,  a=a3,    alpha=0),
    rtb.RevoluteDH(d=d4, a=0,     alpha=pi/2),
    rtb.RevoluteDH(d=d5, a=0,     alpha=-pi/2),
    rtb.RevoluteDH(d=d6, a=0,     alpha=0)

], name="Robot_6DOF")

print("=== TABLA DH DEL ROBOT ===")
print(robot)

# ==================================
# 3. CONFIGURACIÓN 1
# Todas las articulaciones en 0°
# ==================================
q1 = [0,0,0,0,0,0]

T1 = robot.fkine(q1)

print("\nConfiguración 1 (todo en 0°)")
print(T1)

robot.plot(q1, block=False)

# ==================================
# 4. CONFIGURACIÓN 2
# 45° y -45° alternados
# ==================================
q2 = [
    pi/4,
    -pi/4,
    pi/4,
    -pi/4,
    pi/4,
    -pi/4
]

T2 = robot.fkine(q2)

print("\nConfiguración 2 (±45°)")
print(T2)

robot.plot(q2, block=False)

# ==================================
# 5. CONFIGURACIÓN 3
# Configuración libre
# ==================================
q3 = [
    pi/6,
    -pi/3,
    pi/4,
    pi/6,
    -pi/4,
    pi/3
]

T3 = robot.fkine(q3)

print("\nConfiguración 3 (libre)")
print(T3)

robot.plot(q3, block=True)

sol = robot.ikine_LM(T3)

print("¿Solución encontrada?: ", sol.success)

q_inv = sol.q

print("Ángulos encontrados (rad):")
print(np.round(q_inv,4))

print("Ángulos encontrados (deg):")
print(np.round(np.rad2deg(q_inv),2))
