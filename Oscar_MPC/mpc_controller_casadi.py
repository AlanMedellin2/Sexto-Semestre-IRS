#!/usr/bin/env python3
"""
Simple Path-Following MPC (CasADi + IPOPT) for a unicycle in ODOM frame.


ROS:
- Sub: /drawn_plan (nav_msgs/Path)  [odom]
- Sub: /odom       (nav_msgs/Odometry)
- Pub: /cmd_vel    (geometry_msgs/Twist)

Behavior:
- Uses a “carrot” point: advance along the path by a fixed distance ahead of the closest path point.
- MPC tracks a constant reference pose (carrot + heading-to-carrot) across the whole horizon (super stable demo).
- Logs  include sanity checks of u* ranges.
"""

import math
import numpy as np
import casadi as ca

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist


# ----------------- small utils -----------------

def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_pi(a):
    return math.atan2(math.sin(a), math.cos(a))


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


# ----------------- node -----------------

class SimplePathMPCCasadi(Node):
    def __init__(self):
        super().__init__("simple_path_mpc_casadi")

        # ---------- params ----------
        self.declare_parameter("dt", 0.1)
        self.declare_parameter("N", 10)

        self.declare_parameter("v_min", 0.0)
        self.declare_parameter("v_max", 0.32)
        self.declare_parameter("w_min", -2.0)
        self.declare_parameter("w_max",  2.0)

        self.declare_parameter("carrot_dist", 0.70)   # meters ahead along path
        self.declare_parameter("advance_eps", 0.20)   # meters: when close to base index, allow advance
        self.declare_parameter("log_every", 10)
        self.declare_parameter("ipopt_verbose", False)

        self.dt = float(self.get_parameter("dt").value)
        self.N = int(self.get_parameter("N").value)

        self.v_min = float(self.get_parameter("v_min").value)
        self.v_max = float(self.get_parameter("v_max").value)
        self.w_min = float(self.get_parameter("w_min").value)
        self.w_max = float(self.get_parameter("w_max").value)

        self.carrot_dist = float(self.get_parameter("carrot_dist").value)
        self.advance_eps = float(self.get_parameter("advance_eps").value)
        self.log_every = int(self.get_parameter("log_every").value)
        self.ipopt_verbose = bool(self.get_parameter("ipopt_verbose").value)

        # ---------- weights (simple, stable demo) ----------
        # state = [x,y,th], control=[v,w]
        self.Q = np.diag([8.0, 8.0, 2.0])
        self.R = np.diag([0.6, 0.25])
        self.Rd = np.diag([0.8, 0.35])

        # ---------- ROS I/O ----------
        self.sub_path = self.create_subscription(Path, "/drawn_plan", self.path_cb, 10)
        self.sub_odom = self.create_subscription(Odometry, "/odom", self.odom_cb, 20)
        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)

        # ---------- state ----------
        self.have_pose = False
        self.have_path = False

        self.rx = 0.0
        self.ry = 0.0
        self.rth = 0.0

        self.path_xy = []
        self.base_idx = 0  # progress along path (monotonic-ish)

        self.u_prev = np.array([0.0, 0.0], dtype=float)

        self.tick_count = 0
        self.last_status = ""

        # ---------- solver ----------
        self.solver, self.struct = self.build_solver()

        self.timer = self.create_timer(self.dt, self.tick)

        self.get_logger().info(
            "SimplePathMPC ready.\n"
            f"- dt={self.dt} N={self.N}\n"
            f"- v[{self.v_min:+.2f},{self.v_max:+.2f}] w[{self.w_min:+.2f},{self.w_max:+.2f}]\n"
            f"- carrot_dist={self.carrot_dist:.2f}\n"
            f"- ipopt_verbose={self.ipopt_verbose} log_every={self.log_every}\n"
        )

    # ---------- callbacks ----------

    def path_cb(self, msg: Path):
        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.path_xy = pts
        self.have_path = (len(pts) >= 2)
        # reset progress if a new path comes in (simple rule)
        self.base_idx = 0
        self.get_logger().warn(f"PATH: n={len(pts)} frame='{msg.header.frame_id}' base_idx reset")

    def odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.rx = float(p.x)
        self.ry = float(p.y)
        self.rth = yaw_from_quat(q)
        self.have_pose = True

    # ---------- cmd publish ----------

    def publish_cmd(self, v, w):
        # Creamos el objeto Twist (que ya importaste correctamente)
        m = Twist()
        
        # Twist NO tiene header, así que eliminamos las líneas de stamp y frame_id
        # Los valores de velocidad se asignan directamente a .linear y .angular
        m.linear.x = float(v)
        m.angular.z = float(w)
        
        # Publicamos
        self.pub_cmd.publish(m)

    # ---------- path helpers ----------

    def closest_index(self, start_idx=0):
        pts = self.path_xy
        if not pts:
            return 0, float("inf")
        start_idx = int(clamp(start_idx, 0, len(pts) - 1))

        best_i = start_idx
        best_d2 = float("inf")

        # local search forward only (fast + monotonic)
        # scan a window ahead; if you want more robust, increase window
        window = 40
        end = min(len(pts), start_idx + window)
        for i in range(start_idx, end):
            dx = pts[i][0] - self.rx
            dy = pts[i][1] - self.ry
            d2 = dx * dx + dy * dy
            if d2 < best_d2:
                best_d2 = d2
                best_i = i
        return best_i, math.sqrt(best_d2)

    def carrot_from(self, idx0):
        """
        Walk forward along the polyline until we've accumulated carrot_dist.
        Returns (carrot_x, carrot_y, carrot_idx)
        """
        pts = self.path_xy
        n = len(pts)
        if n < 2:
            return None

        idx = int(clamp(idx0, 0, n - 1))
        cx, cy = pts[idx]

        remain = self.carrot_dist
        j = idx
        while remain > 1e-9 and j < n - 1:
            x0, y0 = pts[j]
            x1, y1 = pts[j + 1]
            seg = math.hypot(x1 - x0, y1 - y0)
            if seg < 1e-9:
                j += 1
                continue
            if seg >= remain:
                a = remain / seg
                cx = x0 + a * (x1 - x0)
                cy = y0 + a * (y1 - y0)
                remain = 0.0
                return (cx, cy, j)
            else:
                remain -= seg
                j += 1

        # clamp at end
        cx, cy = pts[-1]
        return (cx, cy, n - 1)

    # ---------- MPC ----------

    def build_solver(self):
        nx, nu, N = 3, 2, self.N   # 3 variables (x,y,th), 2 controls (v,w), horizon N
        dt = self.dt

        X = ca.SX.sym("X", nx, N + 1) #So X is a symbolic matrix:  # each column k is [x; y; th] at step k

        ## STATE MATRIX X
        ##
        ## X ∈ ℝ^{3 × (N+1)}
        ##
        ##          k=0      k=1      k=2      ...      k=N
        ##      ----------------------------------------------
        ## x  |     x₀       x₁       x₂       ...       x_N
        ## y  |     y₀       y₁       y₂       ...       y_N
        ## θ  |     θ₀       θ₁       θ₂       ...       θ_N
        ##
        ## Each column:
        ##
        ## X[:,k] = [x_k, y_k, θ_k]ᵀ
        ##
        ## Interpretation:
        ##
        ## X₀ = current robot state
        ## X₁ = predicted state after dt
        ## X₂ = predicted state after 2dt
        ## ...
        ## X_N = predicted state at horizon end
        U = ca.SX.sym("U", nu, N)  # each column k is [v; w]# control matrix U
        # Parameter Vector P=[x0​,y0​,θ0​,x∗,y∗,θ∗,vprev​,wprev​]
        # P = x0(3) + ref_pose(3) + u_prev(2)
        P = ca.SX.sym("P", 3 + 3 + 2)
        x0 = P[0:3]
        ref_pose = P[3:6]          # constant reference for the entire horizon
        u_prev = P[6:8]
        Q = ca.DM(self.Q)           #LQR weights as CasADi DM (dense matrix)|   
        R = ca.DM(self.R)           # control effort weights as CasADi DM (dense matrix)
        Rd = ca.DM(self.Rd)         # control change rate weights as CasADi DM (dense matrix)
        def f(x, u):
            th = x[2]
            v = u[0]
            w = u[1]
            return ca.vertcat(
                x[0] + v * ca.cos(th) * dt,
                x[1] + v * ca.sin(th) * dt,
                x[2] + w * dt
            )
        ### CRystall ball the future Euler integration of the unicycle dynamics, given current state x and control u, to predict the next state after dt.
        ### This function f encapsulates the discrete-time dynamics of the unicycle model
        obj = 0
        g = []                      #constr list for the optimization problem, will include initial condition and dynamics constraints
        g.append(X[:, 0] - x0)      # initial condition constraint: the first state in the trajectory must match the current state of the robot (x0)
        for k in range(N):
            e = X[:, k] - ref_pose
            obj += ca.mtimes([e.T, Q, e]) + ca.mtimes([U[:, k].T, R, U[:, k]])

            du = (U[:, k] - u_prev) if k == 0 else (U[:, k] - U[:, k - 1])
            obj += ca.mtimes([du.T, Rd, du])

            g.append(X[:, k + 1] - f(X[:, k], U[:, k]))

        g = ca.vertcat(*g)
        
        U_pack = ca.vertcat(*[U[:, k] for k in range(N)])  # (2N x 1)
        OPT = ca.vertcat(ca.reshape(X, -1, 1), U_pack)
        
        ##NON LINEAR programming porblem#######################3
        nlp = {"x": OPT, "f": obj, "g": g, "p": P}

        opts = {
            "ipopt.print_level": 5 if self.ipopt_verbose else 0,
            "print_time": 1 if self.ipopt_verbose else 0,
            "ipopt.sb": "no" if self.ipopt_verbose else "yes",
            "ipopt.max_iter": 60,
            "ipopt.tol": 1e-3,
            "ipopt.acceptable_tol": 1e-2,
        }

        solver = ca.nlpsol("solver", "ipopt", nlp, opts)

        # bounds
        lbg = np.zeros(g.size1())
        ubg = np.zeros(g.size1())

        nX = nx * (N + 1)
        nU = 2 * N
        nOPT = nX + nU

        lbx = -1e20 * np.ones(nOPT)
        ubx =  1e20 * np.ones(nOPT)

        u_start = nX
        for k in range(N):
            lbx[u_start + 2 * k + 0] = self.v_min
            ubx[u_start + 2 * k + 0] = self.v_max
            lbx[u_start + 2 * k + 1] = self.w_min
            ubx[u_start + 2 * k + 1] = self.w_max

        return solver, {"nx": nx, "N": N, "lbx": lbx, "ubx": ubx, "lbg": lbg, "ubg": ubg}

    def tick(self):
        self.tick_count += 1

        status = f"pose={'OK' if self.have_pose else 'NO'} path={'OK' if self.have_path else 'NO'} n={len(self.path_xy)} base={self.base_idx}"
        if status != self.last_status:
            self.get_logger().warn(f"STATUS: {status}")
            self.last_status = status

        if not self.have_pose or not self.have_path:
            self.publish_cmd(0.0, 0.0)
            return

        # 1) update closest index (forward-only)
        s_idx, d_path = self.closest_index(start_idx=self.base_idx)
        # (prevents jumping backward)
        if s_idx > self.base_idx and d_path < (self.carrot_dist + self.advance_eps):
            self.base_idx = s_idx

        # 2) compute carrot ahead of s_idx
        carrot = self.carrot_from(s_idx)
        if carrot is None:
            self.publish_cmd(0.0, 0.0)
            return

        cx, cy, carrot_seg_idx = carrot
        dx = cx - self.rx
        dy = cy - self.ry
        d_carrot = math.hypot(dx, dy)
        bearing = math.atan2(dy, dx)

        # desired heading points toward carrot
        th_ref = bearing
        e_th = wrap_pi(th_ref - self.rth)

        # reference pose (constant over horizon)
        ref_pose = np.array([cx, cy, th_ref], dtype=float)

        # 3) solve MPC
        x0 = np.array([self.rx, self.ry, self.rth], dtype=float)
        p = np.concatenate([x0, ref_pose, self.u_prev])

        nx, N = self.struct["nx"], self.struct["N"]

        # warm start for X
        X_guess = np.zeros((nx, N + 1), dtype=float)
        X_guess[:, 0] = x0
        for k in range(N):
            X_guess[:, k + 1] = ref_pose

        u_init = np.zeros(2 * N, dtype=float)
        for k in range(N):
            u_init[2 * k + 0] = float(self.u_prev[0])
            u_init[2 * k + 1] = float(self.u_prev[1])
        x_init = np.concatenate([X_guess.reshape(-1), u_init])

        t0 = self.get_clock().now().nanoseconds
        try:
            sol = self.solver(
                x0=x_init,
                p=p,
                lbx=self.struct["lbx"],
                ubx=self.struct["ubx"],
                lbg=self.struct["lbg"],
                ubg=self.struct["ubg"],
            )
        except Exception as e:
            self.get_logger().error(f"SOLVER EXCEPTION: {e}")
            self.publish_cmd(0.0, 0.0)
            return
        
        
        t1 = self.get_clock().now().nanoseconds
        solve_ms = (t1 - t0) / 1e6

        stats = self.solver.stats()
        ok = bool(stats.get("success", False))
        rs = stats.get("return_status", "UNKNOWN")
        if not ok:
            self.get_logger().error(f"IPOPT FAIL: {rs} solve={solve_ms:.1f}ms")
            self.publish_cmd(0.0, 0.0)
            return

        z = np.array(sol["x"]).flatten()
        nX = nx * (N + 1)

        # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
        # CRITICAL FIX: unpack interleaved controls
        u_flat = z[nX:]  # [v0,w0,v1,w1,...]
        v = float(u_flat[0])
        w = float(u_flat[1])
        # <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

        # clip as  (should be unnecessary once packing is correct)
        v_cmd = v #clamp(v, self.v_min, self.v_max)
        w_cmd = w #clamp(w, self.w_min, self.w_max)


        self.u_prev[:] = [v_cmd, w_cmd]
        self.publish_cmd(v_cmd, w_cmd)

        # ---- clean logs (not spam) ----
        if self.tick_count % self.log_every == 0:
            # u* preview
            K = min(5, N)
            pairs = []
            for k in range(K):
                pairs.append((float(u_flat[2*k + 0]), float(u_flat[2*k + 1])))

            min_v = float(np.min(u_flat[0::2])) if u_flat.size else 0.0
            max_v = float(np.max(u_flat[0::2])) if u_flat.size else 0.0
            min_w = float(np.min(u_flat[1::2])) if u_flat.size else 0.0
            max_w = float(np.max(u_flat[1::2])) if u_flat.size else 0.0
            # sign sanity: if e_th is negative, we EXPECT w to be negative (usually)
            sign_mismatch = (abs(e_th) > 0.35) and (e_th * w_cmd < 0.0)  # opposite sign than expected
            sm = "  !!!SIGN_MISMATCH!!!" if sign_mismatch else ""

            u_star_txt = " ".join([f"({vk:+.2f},{wk:+.2f})" for (vk, wk) in pairs])

            self.get_logger().info(
                f"tick={self.tick_count} ok={rs} solve={solve_ms:.1f}ms | "
                f"pose=({self.rx:.2f},{self.ry:.2f},th={self.rth:.2f}) | "
                f"s={s_idx}/{max(0,len(self.path_xy)-1)} base={self.base_idx} d_path={d_path:.2f} | "
                f"carrot=({cx:.2f},{cy:.2f}) d_carrot={d_carrot:.2f} | "
                f"bearing={bearing:+.2f} e_th={e_th:+.2f} | "
                f"cmd(v={v_cmd:.3f},w={w_cmd:.3f}) | "
                f"u*[0:{K-1}]={u_star_txt} | "
                f"u_range v[{min_v:+.2f},{max_v:+.2f}] w[{min_w:+.2f},{max_w:+.2f}]"
                f"{sm}"
            )


def main():
    rclpy.init()
    node = SimplePathMPCCasadi()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
