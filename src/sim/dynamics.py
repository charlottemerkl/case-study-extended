"""Differential-drive robot dynamics.

State:
    s = [x, y, theta, v, omega]

Action:
    a = [tau_L, tau_R]

Parameters:
    p = [m, J, L, r, b_v, b_omega]

Continuous-time model:
    x_dot     = v cos(theta)
    y_dot     = v sin(theta)
    theta_dot = omega
    v_dot     = (1 / m) * ((tau_R + tau_L) / r - b_v * v)
    omega_dot = (1 / J) * ((L / (2 r)) * (tau_R - tau_L) - b_omega * omega)

The module provides the dynamics for simulation
"""

from dataclasses import dataclass

import numpy as np

@dataclass
class RobotParams:
    """Physical parameters of the differential-drive robot.

    Attributes:
        m: Robot mass [kg].
        J: Robot yaw inertia [kg m^2].
        L: Distance between left and right wheels [m].
        r: Wheel radius [m].
        b_v: Linear viscous friction coefficient.
        b_omega: Angular viscous friction coefficient.
    """

    m: float
    J: float
    L: float
    r: float
    b_v: float
    b_omega: float

    def __init__(self, m: float = 10.0, J: float = 1.0, L: float = 0.5, r: float = 0.1, b_v: float = 0.1, b_omega: float = 0.1):
        self.m = m
        self.J = J
        self.L = L
        self.r = r
        self.b_v = b_v
        self.b_omega = b_omega
    
    def as_array(self) -> np.ndarray:
        """Return parameters as a NumPy array [m, J, L, r, b_v, b_omega]."""
        return np.array(
            [self.m, self.J, self.L, self.r, self.b_v, self.b_omega],
            dtype=float,
        )
    


def robot_dynamics(
    state: np.ndarray, action: np.ndarray, params: RobotParams
) -> np.ndarray:
    """Compute the time derivative of the state given the current state, action, and parameters.
    two-wheel differential drive robot dynamics in continuous time.

    Args:
        state: Robot state [x, y, theta, v, omega].
        action: Motor torques [tau_L, tau_R].
        params: Instance of the RobotParams class containing the physical parameters.

    Returns:
        State derivative [x_dot, y_dot, theta_dot, v_dot, omega_dot].
    """
    _, _, theta, v, omega = state
    tau_L, tau_R = action

    x_dot = v * np.cos(theta)
    y_dot = v * np.sin(theta)
    theta_dot = omega
    v_dot = ((tau_R + tau_L) / params.r - params.b_v * v) / params.m
    omega_dot = ((params.L / (2.0 * params.r)) * (tau_R - tau_L) - params.b_omega * omega) / params.J

    return np.array([x_dot, y_dot, theta_dot, v_dot, omega_dot], dtype=float)
