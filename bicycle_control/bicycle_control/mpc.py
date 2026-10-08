"""
High-Level Lateral Steering Controller: Extended Kinematic Bicycle MPC.
Solves a constrained non-linear program over prediction horizon N using SciPy,
optimizing steering angle and longitudinal acceleration (mapped to throttle).
"""

import math
import warnings
import numpy as np
from scipy.optimize import minimize

warnings.filterwarnings('ignore', message='Values in x were outside bounds')


class KinematicBicycleMPC:
    """Nonlinear Model Predictive Control for an Extended Kinematic Bicycle Model.

    Optimizes future control sequences u = [delta_k, a_k] where steering angle delta_k
    and longitudinal acceleration a_k (mapped to throttle effort) are the control inputs,
    forward-simulating a 4-state extended kinematic bicycle model x = [x, y, theta, v]^T.
    """

    def __init__(self, wheelbase=1.25, dt=0.1, horizon=10,
                 max_steer_rad=math.radians(35.0), k_a=4.0,
                 max_accel=None, max_brake=None):
        self.L = wheelbase
        self.dt = dt
        self.N = horizon
        self.max_steer_rad = max_steer_rad
        self.k_a = float(max_accel if max_accel is not None else k_a)

        # Weights: heavily penalize lateral CTE, heading error, and steering rate
        self.w_lat = 20.0
        self.w_long = 0.1
        self.w_yaw = 5.0
        self.w_v = 3.0
        self.w_steer = 0.1
        self.w_dsteer = 15.0
        self.w_accel = 0.05

        self.last_u = np.zeros(2 * self.N)  # warm-start [delta_0, a_0, delta_1, a_1, ...]

    def solve(self, x0, ref_trajectory, current_steer=0.0):
        """Solves MPC optimization problem over horizon N.

        x0: [x, y, yaw, v]
        ref_trajectory: list of length N containing [x_ref, y_ref, yaw_ref, v_ref]
        current_steer: actual current steering angle in radians
        Returns: (steer_rad, throttle_cmd in [-1.0, 1.0])
        """
        # 1. Horizon & bounds
        N = min(self.N, len(ref_trajectory))
        if N < 2:
            return 0.0, 0.0

        ref = np.asarray(ref_trajectory[:N], dtype=np.float64)
        x_init, y_init, yaw_init, v_init = [float(c) for c in x0]
        dt = self.dt
        L = self.L

        bounds = []
        for _ in range(N):
            bounds.append((-self.max_steer_rad, self.max_steer_rad))
            bounds.append((-self.k_a, self.k_a))

        # 2. Objective
        def objective(u):
            x, y, yaw, v = x_init, y_init, yaw_init, v_init
            prev_delta = float(current_steer)
            cost = 0.0
            for k in range(N):
                delta = u[2 * k]
                a = u[2 * k + 1]

                # Forward simulate (extended kinematic bicycle, Euler)
                x = x + v * math.cos(yaw) * dt
                y = y + v * math.sin(yaw) * dt
                yaw = yaw + (v / L) * math.tan(delta) * dt
                v = max(0.0, v + a * dt)

                # Frenet-frame tracking error w.r.t. reference point k
                x_ref, y_ref, yaw_ref, v_ref = ref[k]
                dx = x - x_ref
                dy = y - y_ref
                e_long = math.cos(yaw_ref) * dx + math.sin(yaw_ref) * dy
                e_lat = -math.sin(yaw_ref) * dx + math.cos(yaw_ref) * dy
                e_yaw = math.atan2(math.sin(yaw - yaw_ref), math.cos(yaw - yaw_ref))

                cost += (self.w_lat * e_lat ** 2
                         + self.w_long * e_long ** 2
                         + self.w_yaw * e_yaw ** 2
                         + self.w_v * (v - v_ref) ** 2
                         + self.w_steer * delta ** 2
                         + self.w_dsteer * (delta - prev_delta) ** 2
                         + self.w_accel * a ** 2)

                prev_delta = delta
            return cost

        # 3. Warm start: shift previous solution forward by one step
        if len(self.last_u) != 2 * N:
            u_init = np.zeros(2 * N)
        else:
            u_init = np.zeros(2 * N)
            u_init[:-2] = self.last_u[2:]
            u_init[-2:] = self.last_u[-2:]
        u_init[0::2] = np.clip(u_init[0::2], -self.max_steer_rad, self.max_steer_rad)
        u_init[1::2] = np.clip(u_init[1::2], -self.k_a, self.k_a)

        # 4. Optimize
        res = minimize(objective, u_init, bounds=bounds, method='SLSQP',
                       options={'maxiter': 50, 'ftol': 1e-4})

        u_opt = res.x if np.all(np.isfinite(res.x)) else u_init
        self.last_u = np.array(u_opt, dtype=np.float64)

        delta_cmd = float(np.clip(u_opt[0], -self.max_steer_rad, self.max_steer_rad))
        accel_cmd = float(u_opt[1])
        throttle_cmd = float(np.clip(accel_cmd / self.k_a, -1.0, 1.0))

        return delta_cmd, throttle_cmd