"""
Target Velocity Profiler based on track curvature.
Calculates maximum safe cornering speeds subject to lateral acceleration limits.
"""

import math


class VelocityProfiler:
    """Generates target speed profiles based on track curvature or precomputed data."""

    def __init__(self, default_speed=4.0, max_speed=8.0, max_lat_accel=5.0):
        self.default_speed = default_speed
        self.max_speed = max_speed
        self.max_lat_accel = max_lat_accel

    def compute_target_speed(self, kappa, fallback_speed=None):
        """Calculates curvature-limited velocity: v_max = sqrt(a_lat_max / |kappa|)."""
        # Guard against invalid input
        if kappa is None or math.isnan(kappa):
            return float(fallback_speed if fallback_speed is not None
                         else self.default_speed)

        abs_kappa = abs(float(kappa))

        # Near-straight segment: no cornering limit, use the maximum speed
        if abs_kappa < 1e-4:
            return float(self.max_speed)

        # Curvature-limited speed from a_lat = v^2 * |kappa|
        v_limit = math.sqrt(self.max_lat_accel / abs_kappa)

        return float(min(v_limit, self.max_speed))