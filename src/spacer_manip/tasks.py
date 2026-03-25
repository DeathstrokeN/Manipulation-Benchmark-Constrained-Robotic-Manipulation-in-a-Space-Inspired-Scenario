from __future__ import annotations
import numpy as np


class ConstrainedApproachTask:
    """
    2D constrained approach task inspired by spacecraft servicing / panel interaction.
    The end-effector must move to a target slot while remaining inside a corridor.
    A compliant wall model emulates fragile hardware around the capture interface.
    """
    def __init__(self):
        self.target = np.array([0.78, 0.42])
        self.wall_x = 0.82
        self.corridor_y = (0.30, 0.54)
        self.contact_k = 450.0
        self.contact_b = 30.0

    def desired_trajectory(self, t: float, x0: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        T = 3.0
        alpha = min(max(t / T, 0.0), 1.0)
        alpha = 3 * alpha**2 - 2 * alpha**3
        xd = x0 + alpha * (self.target - x0)
        vd = ((6 * (t / T) - 6 * (t / T) ** 2) / T) * (self.target - x0) if 0 <= t <= T else np.zeros(2)
        return xd, vd

    def contact_force(self, x: np.ndarray, dx: np.ndarray) -> np.ndarray:
        fx = 0.0
        fy = 0.0
        if x[0] > self.wall_x:
            penetration = x[0] - self.wall_x
            fx -= self.contact_k * penetration + self.contact_b * max(dx[0], 0.0)
        if x[1] < self.corridor_y[0]:
            penetration = self.corridor_y[0] - x[1]
            fy += self.contact_k * penetration + self.contact_b * max(-dx[1], 0.0)
        elif x[1] > self.corridor_y[1]:
            penetration = x[1] - self.corridor_y[1]
            fy -= self.contact_k * penetration + self.contact_b * max(dx[1], 0.0)
        return np.array([fx, fy])
