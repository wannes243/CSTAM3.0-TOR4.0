from __future__ import annotations

import math
import numpy as np


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def integrate_unicycle_state(state: np.ndarray, dt: float) -> tuple[np.ndarray, np.ndarray]:
    """Discrete EKF process model and Jacobian for [x,y,yaw,v,omega,bias]."""
    if state.shape != (6,):
        raise ValueError("Expected six-state vector")

    x, y, theta, v, omega, bias = state
    c = math.cos(theta)
    s = math.sin(theta)

    next_state = np.array([
        x + v * c * dt,
        y + v * s * dt,
        wrap_angle(theta + omega * dt),
        v,
        omega,
        bias,
    ], dtype=float)

    F = np.eye(6, dtype=float)
    F[0, 2] = -v * s * dt
    F[0, 3] = c * dt
    F[1, 2] = v * c * dt
    F[1, 3] = s * dt
    F[2, 4] = dt
    return next_state, F
