#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32

# ─── CONFIGURACIÓN ────────────────────────────────────────────────────────────
CELL_SIZE = 0.40        # metros por celda — ajusta aquí cuando quieras cambiar escala

# El A* usa (row, col). La conversión al mundo es:
#   x_mundo = (col - START_COL) * CELL_SIZE
#   y_mundo = (row - START_ROW) * CELL_SIZE   (positivo hacia abajo en el grid)
# Ajusta el signo de y_mundo si tu robot tiene el eje Y invertido.
# ──────────────────────────────────────────────────────────────────────────────

'''
00 01 02 03 04 05 06
10 XX XX 13 XX XX XX
20 XX XX 23 XX XX XX
30 31 32 33 XX XX XX
'''

grid = [
    [0, 0, 0, 0, 0, 0, 0],
    [0, 1, 1, 0, 1, 1, 1],
    [0, 1, 1, 0, 1, 1, 1],
    [0, 0, 0, 0, 1, 1, 1]
]


class AStarCell:
    def __init__(self, x, y, g=0, h=0, parent=None):
        self.x = x
        self.y = y
        self.g = g
        self.h = h
        self.f = g + h
        self.parent = parent


dx = [-1, 1, 0, 0]
dy = [0, 0, -1, 1]

def mostrar_mapa(grid):
    print("\nCoordenadas disponibles")

    for row in range(len(grid)):
        for col in range(len(grid[0])):

            if grid[row][col] == 0:
                print(f"{row}{col}", end="  ")
            else:
                print("XX", end="  ")

        print()

def heuristica(x1, y1, x2, y2):
    return abs(x1 - x2) + abs(y1 - y2)


def is_valid(x, y, grid):
    return (
        0 <= x < len(grid)
        and 0 <= y < len(grid[0])
        and grid[x][y] == 0
    )


def get_lowest_f(open_list):
    best = open_list[0]
    for cell in open_list:
        if cell.f < best.f:
            best = cell
    return best


def a_estrella(start, goal, grid):
    open_list = []
    closed_list = []

    start_node = AStarCell(start[0], start[1])
    start_node.h = heuristica(start[0], start[1], goal[0], goal[1])
    start_node.f = start_node.g + start_node.h
    open_list.append(start_node)

    while open_list:
        actual = get_lowest_f(open_list)

        if (actual.x, actual.y) == goal:
            return actual

        open_list.remove(actual)
        closed_list.append(actual)

        for i in range(4):
            nx = actual.x + dx[i]
            ny = actual.y + dy[i]

            if not is_valid(nx, ny, grid):
                continue
            if any(n.x == nx and n.y == ny for n in closed_list):
                continue

            new_g = actual.g + 1
            existente = None
            for n in open_list:
                if n.x == nx and n.y == ny:
                    existente = n
                    break

            if existente:
                if new_g < existente.g:
                    existente.g = new_g
                    existente.f = existente.g + existente.h
                    existente.parent = actual
            else:
                vecino = AStarCell(nx, ny, new_g)
                vecino.h = heuristica(nx, ny, goal[0], goal[1])
                vecino.f = vecino.g + vecino.h
                vecino.parent = actual
                open_list.append(vecino)

    return None


def reconstruir_path(node):
    path = []
    while node:
        path.append((node.x, node.y))
        node = node.parent
    return path[::-1]


def path_to_world_coords(path, start_row, start_col, cell_size):
    """
    Convierte el camino de índices de celda a coordenadas del mundo en metros,
    relativas a la posición inicial del robot (que la odometría ve como 0,0).

    (row, col) → x = (col - start_col) * cell_size
                  y = (row - start_row) * cell_size
    """
    coords = []
    for (row, col) in path[1:]:   # saltamos el nodo inicial (el robot ya está ahí)
        y = (col - start_col) * cell_size
        x = (row - start_row) * cell_size
        coords.append((x, y))
    return coords


class PointGoals(Node):

    def __init__(self):
        super().__init__("a_star_goals_publisher")

        self.pub_ = self.create_publisher(PoseStamped, "a_star_goals", 10)

        # Suscripción al estado: solo publicamos la siguiente meta
        # cuando error.py confirma que el robot llegó a la anterior.
        self.sub_estado = self.create_subscription(
            Float32, "/estado", self.estado_callback, 10
        )

        mostrar_mapa(grid)

        self.declare_parameter("start_row", 0)
        self.declare_parameter("start_col", 0)

        self.declare_parameter("goal_row", 3)
        self.declare_parameter("goal_col", 3)

        start_row = self.get_parameter("start_row").value
        start_col = self.get_parameter("start_col").value

        goal_row = self.get_parameter("goal_row").value
        goal_col = self.get_parameter("goal_col").value

        self.counter_ = 0
        self.listo_para_siguiente = False   # handshake con error.py

        start = (start_row, start_col)
        goal = (goal_row, goal_col)     

        final = a_estrella(start, goal, grid)

        if final is None:
            self.get_logger().error("No se encontró camino con A*")
            self.metas = []
        else:
            path = reconstruir_path(final)
            self.get_logger().info(f"Camino A* (celdas): {path}")
            self.get_logger().info(f"Costo total: {final.g} celdas")

            self.metas = path_to_world_coords(
            path,
            start_row,
            start_col,
            CELL_SIZE
            )
            self.get_logger().info(f"Metas en metros: {self.metas}")

        # Publicamos la primera meta al arrancar sin esperar el estado
        self.timer_ = self.create_timer(0.5, self.timerCallback)

    def estado_callback(self, msg):
        # error.py publica estado=0.0 cuando el robot llegó a una meta
        if msg.data == 0.0 and not self.listo_para_siguiente:
            self.listo_para_siguiente = True

    def timerCallback(self):
        if not self.metas:
            return

        if self.counter_ >= len(self.metas):
            self.get_logger().info("Todas las metas completadas.")
            self.timer_.cancel()
            return

        # Primera meta: publicar de inmediato al arrancar
        # Metas siguientes: esperar confirmación de llegada
        if self.counter_ > 0 and not self.listo_para_siguiente:
            return

        self.listo_para_siguiente = False

        x_meta, y_meta = self.metas[self.counter_]

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"

        msg.pose.position.x = float(x_meta)
        msg.pose.position.y = float(y_meta)
        msg.pose.position.z = 0.0

        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0

        self.pub_.publish(msg)
        self.get_logger().info(
            f"Meta {self.counter_ + 1}/{len(self.metas)} publicada: "
            f"x={x_meta:.2f}, y={y_meta:.2f}"
        )
        self.counter_ += 1


def main():
    rclpy.init()
    point_goals = PointGoals()
    rclpy.spin(point_goals)
    point_goals.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
