"""
High-Level Lateral Steering Controller: Geometric Pure Pursuit.
Calculates steering curvature from lookahead arc geometry.
"""

import math
import numpy as np


class PurePursuitController:
    """Adaptive Pure Pursuit lateral controller."""

    def __init__(self, wheelbase=1.25, kv=0.25, l_min=0.8, l_max=2.5,
                 max_steer_rad=math.radians(35.0)):
        self.L = wheelbase
        self.kv = kv
        self.l_min = l_min
        self.l_max = l_max
        self.max_steer_rad = max_steer_rad

    def compute_lookahead(self, v):
        """Adaptive lookahead distance: Ld = clip(kv * v + l_min, l_min, l_max)."""
        ld = self.kv * float(v) + self.l_min
        return float(np.clip(ld, self.l_min, self.l_max))

    def find_target_waypoint(self, x, y, path_points, lookahead):
        """Searches along path for the target waypoint at lookahead distance."""
        n = len(path_points)
        if n == 0:
            return 0, (x, y)

        # Nearest waypoint to the vehicle
        nearest_idx = 0
        min_d_sq = float('inf')
        for i in range(n):
            dx = path_points[i][0] - x
            dy = path_points[i][1] - y
            d_sq = dx * dx + dy * dy
            if d_sq < min_d_sq:
                min_d_sq = d_sq
                nearest_idx = i

        # Walk forward (closed loop) until a waypoint is at least `lookahead` away
        idx = nearest_idx
        for _ in range(n):
            px, py = path_points[idx][0], path_points[idx][1]
            if math.hypot(px - x, py - y) >= lookahead:
                return idx, (px, py)
            idx = (idx + 1) % n

        # Fallback: no point far enough (very short path)
        return idx, (path_points[idx][0], path_points[idx][1])

    def compute_steering(self, x, y, yaw, target_pt, lookahead):
        """Computes steering angle in radians using Pure Pursuit geometry."""
        dx = target_pt[0] - x
        dy = target_pt[1] - y

        # Transform target into the vehicle's local frame
        x_local = math.cos(yaw) * dx + math.sin(yaw) * dy
        y_local = -math.sin(yaw) * dx + math.cos(yaw) * dy

        # Angle to the target in the vehicle frame
        alpha = math.atan2(y_local, x_local)

        # Pure Pursuit arc law: delta = atan(2 L sin(alpha) / Ld)
        ld = max(float(lookahead), 1e-3)
        delta = math.atan2(2.0 * self.L * math.sin(alpha), ld)

        return float(np.clip(delta, -self.max_steer_rad, self.max_steer_rad))