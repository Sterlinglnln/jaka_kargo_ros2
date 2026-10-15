"""Measured startup readiness; never resets or commands a moving articulation."""
import math


class StartupMonitor:
    def __init__(self, position_tolerance=0.02, velocity_tolerance=0.03, settle_time=0.5):
        self.position_tolerance = position_tolerance
        self.velocity_tolerance = velocity_tolerance
        self.settle_time = settle_time
        self.stable_since = None
        self.ready = False
        self.position_error = math.inf
        self.speed = math.inf

    def update(self, positions, velocities, sim_time):
        if (not positions or len(positions) != len(velocities)
                or not math.isfinite(sim_time)
                or not all(math.isfinite(v) for v in list(positions) + list(velocities))):
            self.ready = False
            self.stable_since = None
            return False
        self.position_error = max(abs(v) for v in positions)
        self.speed = max(abs(v) for v in velocities)
        # Once commissioned, legitimate task motion must not clear readiness.
        if self.ready:
            return True
        if self.position_error > self.position_tolerance or self.speed > self.velocity_tolerance:
            self.stable_since = None
            return False
        if self.stable_since is None or sim_time < self.stable_since:
            self.stable_since = sim_time
        self.ready = sim_time - self.stable_since >= self.settle_time
        return self.ready
