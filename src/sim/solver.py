import numpy as np
from src.sim.dynamics import RobotParams
from src.sim.dynamics import robot_dynamics
from src.sim.utils import wrap_angle

"""
Numerical solvers for the robot dynamics

NOTE: very non optimized code: params are passed as args at every step
class implementation could guarantee better performance
also: params should be updatable at every step so think of how
"""


def euler_step(
    state: np.ndarray,
    action: np.ndarray,
    dt: float,
    params: RobotParams,
    wrap_theta: bool = False,
) -> np.ndarray:
    """One explicit Euler integration step."""

    next_state = np.asarray(state, dtype=float) + dt * robot_dynamics(state, action, params)

    if wrap_theta:
        next_state[2] = wrap_angle(next_state[2])

    return next_state


def rk4_step(
    state: np.ndarray,
    action: np.ndarray,
    dt: float,
    params: RobotParams,
    wrap_theta: bool = False,
) -> np.ndarray:
    """One fourth-order Runge-Kutta explicit integration step."""

    k1 = robot_dynamics(state, action, params)
    k2 = robot_dynamics(state + 0.5 * dt * k1, action, params)
    k3 = robot_dynamics(state + 0.5 * dt * k2, action, params)
    k4 = robot_dynamics(state + dt * k3, action, params)

    next_state = state + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

    if wrap_theta:
        next_state[2] = wrap_angle(next_state[2])

    return next_state

