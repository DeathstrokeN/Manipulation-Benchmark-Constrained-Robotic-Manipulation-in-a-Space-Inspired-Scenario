from __future__ import annotations
import numpy as np
from .dynamics import fk, jacobian


class CartesianImpedanceController:
    def __init__(self, kp=120.0, kd=28.0, torque_limit=20.0):
        self.Kp = np.diag([kp, kp])
        self.Kd = np.diag([kd, kd])
        self.torque_limit = torque_limit

    def __call__(self, q, dq, xd, vd):
        x = fk(q)
        J = jacobian(q)
        dx = J @ dq
        f_cmd = self.Kp @ (xd - x) + self.Kd @ (vd - dx)
        tau = J.T @ f_cmd
        return np.clip(tau, -self.torque_limit, self.torque_limit)


class JointPDController:
    def __init__(self, q_goal, kp=16.0, kd=5.0, torque_limit=20.0):
        self.q_goal = np.array(q_goal, dtype=float)
        self.Kp = np.diag([kp, kp])
        self.Kd = np.diag([kd, kd])
        self.torque_limit = torque_limit

    def __call__(self, q, dq, xd=None, vd=None):
        tau = self.Kp @ (self.q_goal - q) - self.Kd @ dq
        return np.clip(tau, -self.torque_limit, self.torque_limit)
