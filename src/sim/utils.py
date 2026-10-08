import numpy as np
import casadi as ca

def wrap_angle(theta: float | np.ndarray) -> float | np.ndarray:
    """Wrap angle(s) to [-pi, pi)."""
    return theta % (2.0 * np.pi)
