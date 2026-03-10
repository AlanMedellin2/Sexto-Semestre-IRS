#!/usr/bin/env python3
"""
Path-horizon MPC (CasADi + IPOPT) for unicycle in ODOM frame.

- Subscribes: /drawn_plan (nav_msgs/Path)  [odom]
- Subscribes: /odom (nav_msgs/Odometry)
- Publishes : /cmd_vel (geometry_msgs/TwistStamped)

Reference over the horizon:
  ref[:,k] = [x_ref_k, y_ref_k, th_ref_k]

Main fixes:
- Explicit interleaved control packing: [v0,w0,v1,w1,...]
- Explicit warm-start packing for X and U
- Explicit control unpacking from flat solver output
- Horizon starts a little ahead of the closest point
"""

import math
import numpy as np
import casadi as ca

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32



def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_to_pi(a):
    return math.atan2(math.sin(a), math.cos(a))


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class PathMPC(Node):
    def __init__(self):
        super().__init__("path_mpc_casadi")

        # ---------- Params ----------
        self.declare_parameter("dt", 0.1) #Tiempo de Muestreo
        self.declare_parameter("N", 10) #Horizonte

        #Limites fiscos del robot
        self.declare_parameter("v_min", -0.20)
        self.declare_parameter("v_max", 0.32)
        self.declare_parameter("w_min", -2.0)
        self.declare_parameter("w_max", 2.0)

        self.declare_parameter("ipopt_verbose", False)
        self.declare_parameter("log_every", 10)

        self.declare_parameter("stride", 1)
        self.declare_parameter("min_ref_step", 0.02)

        self.declare_parameter("stop_if_far", 5.0)
        self.declare_parameter("slowdown_dist", 0.30)

        # El horizonte no comienza exactamente en el punto más cercano de la trayectoria
        #Empieza unos puntos delantes, lo que evita oscilaciones y hace el seguimiento mas suave
        self.declare_parameter("lookahead_pts", 4)

        self.dt = float(self.get_parameter("dt").value)
        self.N = int(self.get_parameter("N").value)

        self.v_min = float(self.get_parameter("v_min").value)
        self.v_max = float(self.get_parameter("v_max").value)
        self.w_min = float(self.get_parameter("w_min").value)
        self.w_max = float(self.get_parameter("w_max").value)

        self.ipopt_verbose = bool(self.get_parameter("ipopt_verbose").value)
        self.log_every = int(self.get_parameter("log_every").value)

        self.stride = int(self.get_parameter("stride").value)
        self.min_ref_step = float(self.get_parameter("min_ref_step").value)

        self.stop_if_far = float(self.get_parameter("stop_if_far").value)
        self.slowdown_dist = float(self.get_parameter("slowdown_dist").value)
        self.lookahead_pts = int(self.get_parameter("lookahead_pts").value)

        # ---------- Weights ----------
        # TUNNING ZONE!!
        ##
        self.Q = np.diag([10.0, 10.0, 4.0])
        self.R = np.diag([0.3, 0.2])
        #self.Rd = np.diag([0.20, 0.02])
        self.Rd = np.diag([1, 1])   # A derivative term for LQR ( only use if smoothness is an issue)
    
        # ---------- ROS ----------
        self.sub_path = self.create_subscription(Path, "/drawn_plan", self.path_cb, 10)
        self.sub_odom = self.create_subscription(Odometry, "/odom", self.odom_cb, 20)
        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pub_error = self.create_publisher(Float32, "/error", 10)
        self.pub_cte = self.create_publisher(Float32, "/cross_track_error", 10)

        # ---------- State ----------
        self.have_pose = False
        self.have_path = False

        self.rx = 0.0
        self.ry = 0.0
        self.rth = 0.0

        self.path_xy = []
        self.u_prev = np.zeros(2, dtype=float)

        self.tick_count = 0
        self.last_status = ""
        self.last_cmd = (0.0, 0.0)

        self.solver, self.struct = self.build_solver()

        self.timer = self.create_timer(self.dt, self.tick)

        self.get_logger().info(
            "Path MPC ready.\n"
            f"- dt={self.dt} N={self.N} stride={self.stride} lookahead_pts={self.lookahead_pts}\n"
            f"- bounds: v[{self.v_min:+.2f},{self.v_max:+.2f}] w[{self.w_min:+.2f},{self.w_max:+.2f}]\n"
            f"- IPOPT verbose={self.ipopt_verbose}, log_every={self.log_every}\n"
            "Waiting for /odom and /drawn_plan..."
        )

    # ---------- Callbacks ----------
    def path_cb(self, msg: Path):
        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.path_xy = pts
        self.have_path = (len(pts) >= 2)
        self.get_logger().warn(f"PATH: n={len(pts)} frame='{msg.header.frame_id}'")

    def odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.rx = float(p.x)
        self.ry = float(p.y)
        self.rth = yaw_from_quat(q)
        self.have_pose = True

    # ---------- Command publish ----------
    # ---------- Command publish ----------
    def publish_cmd(self, v, w, reason=""):
        m = Twist()
        # Twist no tiene header, así que quitamos m.header.stamp y m.header.frame_id
        # También quitamos el prefijo .twist para ir directo a .linear y .angular
        m.linear.x = float(v)
        m.angular.z = float(w)
        self.pub_cmd.publish(m)

        # Mantenemos tu lógica de logs para que veas qué está pasando
        if abs(v - self.last_cmd[0]) > 1e-3 or abs(w - self.last_cmd[1]) > 1e-3:
            if reason:
                self.get_logger().info(f"CMD {reason}: v={v:.3f} w={w:.3f}")
            else:
                self.get_logger().info(f"CMD: v={v:.3f} w={w:.3f}")
        self.last_cmd = (v, w)

    # ---------- Packing helpers ----------
    # Dont think of this in terms of code, think of this in terms of the actual memory layout that CasADi expects for the optimization variables.
    # In other words shaping matricies the right way 
    
    def pack_X_colmajor(self, X):
        """
        Pack X[:,0], X[:,1], ..., X[:,N] explicitly:
        [x0,y0,th0,x1,y1,th1,...]
        """
        nx, ncols = X.shape
        out = np.zeros(nx * ncols, dtype=float)
        t = 0
        for k in range(ncols):
            for i in range(nx):
                out[t] = float(X[i, k])
                t += 1
        return out

    def pack_U_interleaved(self, U):
        """
        Pack U[:,0], U[:,1], ..., U[:,N-1] explicitly:
        [v0,w0,v1,w1,...]
        """
        nu, ncols = U.shape
        out = np.zeros(nu * ncols, dtype=float)
        t = 0
        for k in range(ncols):
            for i in range(nu):
                out[t] = float(U[i, k])
                t += 1
        return out

    # ---------- CasADi / IPOPT ----------
    def build_solver(self):
        nx, nu, N = 3, 2, self.N
        dt = self.dt

        #Variables simbolicas
        X = ca.SX.sym("X", nx, N + 1)
        U = ca.SX.sym("U", nu, N)

        # Parameters: x0(3) + ref(3*N) + u_prev(2)
        P = ca.SX.sym("P", 3 + 3 * N + 2)
        x0 = P[0:3]
        ref = ca.reshape(P[3:3 + 3 * N], 3, N)
        u_prev = P[3 + 3 * N: 3 + 3 * N + 2]

        Q = ca.DM(self.Q)
        R = ca.DM(self.R)
        Rd = ca.DM(self.Rd)

        def f(x, u):
            th = x[2]
            v = u[0]
            w = u[1]
            return ca.vertcat(                                      #trycylce dynamics
                x[0] + v * ca.cos(th) * dt,                         #diff drive sometime?
                x[1] + v * ca.sin(th) * dt,                         #non linear :O    
                x[2] + w * dt
            )

        obj = 0
        g = []
        g.append(X[:, 0] - x0)
        



        for k in range(N):
            e = X[:, k] - ref[:, k]
            obj += ca.mtimes([e.T, Q, e]) + ca.mtimes([U[:, k].T, R, U[:, k]])
            du = (U[:, k] - u_prev) if k == 0 else (U[:, k] - U[:, k - 1])
            # Optional derivative term for smoothness ( only use if smoothness is an issue, otherwise it can cause sluggishness)
            obj += ca.mtimes([du.T, Rd, du])#If not smooth enough, implement a kind of derivative R ( from LQR)
            
            g.append(X[:, k + 1] - f(X[:, k], U[:, k]))
            ### TODO   add a" forbiudden safety zone"

        g = ca.vertcat(*g)

        # EXPLICIT control packing to avoid reshape ambiguity
        U_pack = ca.vertcat(*[U[:, k] for k in range(N)])   # [v0,w0,v1,w1,...]
        OPT = ca.vertcat(ca.reshape(X, -1, 1), U_pack)

        nlp = {"x": OPT, "f": obj, "g": g, "p": P}              # this is the optimization problem definition that CasADi will use to generate the solver. It includes the optimization variables (x), the objective function (f), the constraints (g), and the parameters (p).

        opts = {
            "ipopt.print_level": 5 if self.ipopt_verbose else 0,
            "print_time": 1 if self.ipopt_verbose else 0,
            "ipopt.sb": "no" if self.ipopt_verbose else "yes",
            "ipopt.max_iter": 60,
            "ipopt.tol": 1e-3,
            "ipopt.acceptable_tol": 1e-2,
        }

        solver = ca.nlpsol("solver", "ipopt", nlp, opts)

        lbg = np.zeros(g.size1())
        ubg = np.zeros(g.size1())

        nX = nx * (N + 1)
        nU = nu * N
        nOPT = nX + nU

        lbx = -1e20 * np.ones(nOPT)
        ubx = 1e20 * np.ones(nOPT)

        u_start = nX
        for k in range(N):
            lbx[u_start + 2 * k + 0] = self.v_min
            ubx[u_start + 2 * k + 0] = self.v_max
            lbx[u_start + 2 * k + 1] = self.w_min
            ubx[u_start + 2 * k + 1] = self.w_max

        return solver, {
            "nx": nx,
            "nu": nu,
            "N": N,
            "lbx": lbx,
            "ubx": ubx,
            "lbg": lbg,
            "ubg": ubg,
        }

    # ---------- Reference horizon ----------
    def closest_path_index(self):
        pts = self.path_xy
        dx = np.array([px - self.rx for (px, py) in pts], dtype=float)
        dy = np.array([py - self.ry for (px, py) in pts], dtype=float)
        d2 = dx * dx + dy * dy
        i0 = int(np.argmin(d2))
        d0 = float(math.sqrt(d2[i0]))
        return i0, d0

    def build_ref_horizon(self):
        pts = self.path_xy
        if len(pts) < 2:
            return None

        i_closest, d_path = self.closest_path_index()

        if d_path > self.stop_if_far:
            return None

        # START AHEAD OF THE RAW CLOSEST POINT
        i0 = min(i_closest + max(0, self.lookahead_pts), len(pts) - 1)

        stride = max(1, self.stride)

        ref_xy = []
        idx = i0
        for _ in range(self.N):
            ref_xy.append(pts[idx])
            idx = min(idx + stride, len(pts) - 1)

        ref = np.zeros((self.N, 3), dtype=float)
        bad_steps = 0

        for k in range(self.N):
            xk, yk = ref_xy[k]

            if k < self.N - 1:
                xk2, yk2 = ref_xy[k + 1]
            else:
                xk2, yk2 = ref_xy[k]

            if math.hypot(xk2 - xk, yk2 - yk) < self.min_ref_step:
                # At the very end / repeated points, keep heading sensible
                if k > 0:
                    thk = ref[k - 1, 2]
                else:
                    thk = self.rth
                bad_steps += 1
            else:
                thk = math.atan2(yk2 - yk, xk2 - xk)

            ref[k, :] = [xk, yk, thk]

        if bad_steps > int(0.7 * self.N):
            return None

        d_ref0 = float(math.hypot(ref[0, 0] - self.rx, ref[0, 1] - self.ry))

        end_x, end_y = pts[-1]
        d_to_end = float(math.hypot(end_x - self.rx, end_y - self.ry))

        info = {
            "i_closest": i_closest,
            "i0": i0,
            "d_path": d_path,
            "d_ref0": d_ref0,
            "d_to_end": d_to_end,
            "stride": stride,
            "ref0": ref[0, :].copy(),
            "refN": ref[-1, :].copy(),
        }
        return ref, info

    # ---------- Main loop ----------
    def tick(self):
        self.tick_count += 1

        status = f"pose={'OK' if self.have_pose else 'NO'} path={'OK' if self.have_path else 'NO'} n={len(self.path_xy)}"
        if status != self.last_status:
            self.get_logger().warn(f"STATUS: {status}")
            self.last_status = status

        if not self.have_pose or not self.have_path:
            self.publish_cmd(0.0, 0.0, reason="HOLD(waiting)")
            return

        out = self.build_ref_horizon()

        if out is None:
            self.publish_cmd(0.0, 0.0, reason="HOLD(bad_ref)")
            if self.tick_count % self.log_every == 0:
                i0, d_path = self.closest_path_index() if self.have_path else (-1, float("nan"))
                self.get_logger().warn(f"tick={self.tick_count} HOLD(bad_ref) d_path={d_path:.2f} i_closest={i0}")
            return

        ref, info = out

        # Cross track error firmado
        i        = info["i_closest"]
        px,  py  = self.path_xy[i]
        i2       = min(i + 1, len(self.path_xy) - 1)
        px2, py2 = self.path_xy[i2]

        tx   = px2 - px          # vector tangente del path
        ty   = py2 - py
        rx_  = self.rx - px      # vector robot respecto al punto mas cercano
        ry_  = self.ry - py

        norm = math.hypot(tx, ty)
        cte  = (tx * ry_ - ty * rx_) / norm if norm > 1e-9 else 0.0

        if info["d_to_end"] < self.slowdown_dist:  # 0.30 m por defecto
            a = clamp(info["d_to_end"] / max(self.slowdown_dist, 1e-6), 0.0, 1.0)
            v *= a

        x0 = np.array([self.rx, self.ry, self.rth], dtype=float)
        p = np.concatenate([x0, ref.reshape(-1), self.u_prev])

        nx, nu, N = self.struct["nx"], self.struct["nu"], self.struct["N"]

        # Warm start packed EXPLICITLY to match CasADi layout
        X_guess = np.zeros((nx, N + 1), dtype=float)
        X_guess[:, 0] = x0
        for k in range(N):
            X_guess[:, k + 1] = ref[k, :]

        U_guess = np.tile(self.u_prev.reshape(2, 1), (1, N))

        x_init = np.concatenate([
            self.pack_X_colmajor(X_guess),
            self.pack_U_interleaved(U_guess)
        ])

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
            self.publish_cmd(0.0, 0.0, reason="HOLD(exception)")
            return

        t1 = self.get_clock().now().nanoseconds
        solve_ms = (t1 - t0) / 1e6

        stats = self.solver.stats()
        ok = bool(stats.get("success", False))
        rs = stats.get("return_status", "UNKNOWN")

        if not ok:
            self.get_logger().error(f"IPOPT FAIL: {rs} solve={solve_ms:.1f}ms")
            self.publish_cmd(0.0, 0.0, reason="HOLD(ipopt_fail)")
            return

        z = np.array(sol["x"]).flatten()
        nX = nx * (N + 1)

        # EXPLICIT flat interleaved control extraction
        u_flat = z[nX:]   # [v0,w0,v1,w1,...]
        v = float(u_flat[0])
        w = float(u_flat[1])

        # Gentle slowdown near end-of-path
        if info["d_to_end"] < self.slowdown_dist:
            a = clamp(info["d_to_end"] / max(self.slowdown_dist, 1e-6), 0.0, 1.0)
            v *= a

        # Safety clip, though with correct packing it should already respect bounds
        #v = clamp(v, self.v_min, self.v_max)
        #w = clamp(w, self.w_min, self.w_max)

        self.u_prev[:] = [v, w]                #Twist to publish

        self.publish_cmd(v, w, reason="MPC")    #publiish

        # --- Publicar error ---

        cte_msg = Float32()
        cte_msg.data = cte
        self.pub_cte.publish(cte_msg)

        
        bearing = math.atan2(info["ref0"][1] - self.ry, info["ref0"][0] - self.rx)
        e_th = wrap_to_pi(bearing - self.rth)

        error_msg = Float32()
        error_msg.data = e_th
        self.pub_error.publish(error_msg)

        if self.tick_count % self.log_every == 0:    # SPAM CONTROL

            show_k = min(5, N)
            pairs = []
            for k in range(show_k):
                pairs.append((float(u_flat[2 * k + 0]), float(u_flat[2 * k + 1])))

            min_v = float(np.min(u_flat[0::2])) if u_flat.size else 0.0
            max_v = float(np.max(u_flat[0::2])) if u_flat.size else 0.0
            min_w = float(np.min(u_flat[1::2])) if u_flat.size else 0.0
            max_w = float(np.max(u_flat[1::2])) if u_flat.size else 0.0

            u_star_txt = " ".join([f"({vk:+.2f},{wk:+.2f})" for (vk, wk) in pairs])

            self.get_logger().info(
                f"tick={self.tick_count} ok={rs} solve={solve_ms:.1f}ms | "
                f"pose=({self.rx:.2f},{self.ry:.2f},th={self.rth:.2f}) | "
                f"i_closest={info['i_closest']}/{len(self.path_xy)-1} i0={info['i0']} stride={info['stride']} d_path={info['d_path']:.2f} | "
                f"ref0=({info['ref0'][0]:.2f},{info['ref0'][1]:.2f},th={info['ref0'][2]:.2f}) d_ref0={info['d_ref0']:.2f} | "
                f"bearing={bearing:+.2f} e_th={e_th:+.2f} | "
                f"cmd(v={v:.3f},w={w:.3f}) | "
                f"u*[0:{show_k-1}]={u_star_txt} | "
                f"u_range v[{min_v:+.2f},{max_v:+.2f}] w[{min_w:+.2f},{max_w:+.2f}]"
            )


def main():
    rclpy.init()
    node = PathMPC()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
