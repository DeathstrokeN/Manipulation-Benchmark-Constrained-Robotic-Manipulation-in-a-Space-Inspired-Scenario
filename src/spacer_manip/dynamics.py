from __future__ import annotations
import numpy as np


def fk(q: np.ndarray, lengths=(0.7, 0.5)) -> np.ndarray:
    q1, q2 = q
    l1, l2 = lengths
    return np.array([
        l1 * np.cos(q1) + l2 * np.cos(q1 + q2),
        l1 * np.sin(q1) + l2 * np.sin(q1 + q2),
    ])


def jacobian(q: np.ndarray, lengths=(0.7, 0.5)) -> np.ndarray:
    q1, q2 = q
    l1, l2 = lengths
    s1, c1 = np.sin(q1), np.cos(q1)
    s12, c12 = np.sin(q1 + q2), np.cos(q1 + q2)
    return np.array([
        [-l1 * s1 - l2 * s12, -l2 * s12],
        [l1 * c1 + l2 * c12, l2 * c12],
    ])


def joint_space_dynamics(q: np.ndarray, dq: np.ndarray, tau: np.ndarray,
                         inertia=(1.6, 0.8), damping=(0.25, 0.15),
                         stiffness=(0.0, 0.0)) -> np.ndarray:
    M = np.diag(inertia)
    B = np.diag(damping)
    K = np.diag(stiffness)
    ddq = np.linalg.solve(M, tau - B @ dq - K @ q)
    return ddq


def step(q: np.ndarray, dq: np.ndarray, tau: np.ndarray, dt: float,
         inertia=(1.6, 0.8), damping=(0.25, 0.15), stiffness=(0.0, 0.0)):
    ddq = joint_space_dynamics(q, dq, tau, inertia=inertia, damping=damping, stiffness=stiffness)
    dq_next = dq + dt * ddq
    q_next = q + dt * dq_next
    return q_next, dq_next, ddq
