import numpy as np
from scipy.interpolate import CubicSpline


def build_full_ref_traj(
    dt,
    v_max: float,
    a_max: float,
    x_length: float = 10.0,
    spacing: float = 0.3,
    circle_turns: int = 1,
    traj_type: str = "circular",
    sine_cycles: float = 3.0,
    amplitude: float = 1.0,
    amplitude_modulation: float = 1.0,
    amplitude_modulation_cycles: float = 1.0,
) -> tuple[CubicSpline, CubicSpline]:
    """
    Build a reference trajectory for tracking scenario.

    The geometry and the speed profile are returned as continuous cubic
    splines parameterized by cumulative arc length s:

        xy_spline(s) = (x, y)
        v_spline(s)  = v

    theta and omega are derived analytically from the spline derivatives
    (see eval_ref_state).
    """

    n_points = max(2, int(round(x_length / spacing)))

    if traj_type == "straight":
        xy_ref = get_straight_traj(
            n_points=n_points,
            x_length=x_length,
        )
    elif traj_type == "circular":
        # Interpret x_length as the per-lap circumference, so spacing ~= x_length
        # / n_points; the full path is circle_turns laps of that many points.
        xy_ref = get_circular_traj(
            n_points=n_points * circle_turns,
            n_turns=circle_turns,
            radius=x_length / (2.0 * np.pi),
            center=(0.0, 0.0),
        )
    elif traj_type == "sinusoidal":
        xy_ref = get_sinusoidal_traj(
            n_points=n_points,
            x_length=x_length,
            sine_cycles=sine_cycles,
            amplitude=amplitude,
            amplitude_modulation=amplitude_modulation,
            amplitude_modulation_cycles=amplitude_modulation_cycles,
        )
    else:
        raise ValueError(f"Unknown traj_type '{traj_type}'.")

    xy_ref = np.asarray(xy_ref, dtype=float).copy()
    xy_ref -= xy_ref[0]

    dxy0 = xy_ref[1] - xy_ref[0]
    theta0 = np.arctan2(dxy0[1], dxy0[0])

    c = np.cos(theta0)
    s_rot = np.sin(theta0)
    R = np.array([[c, s_rot], [-s_rot, c]])

    xy_ref = xy_ref @ R.T

    dxy = np.diff(xy_ref, axis=0)

    theta_segments = np.unwrap(np.arctan2(dxy[:, 1], dxy[:, 0]))
    theta_segments -= theta_segments[0]

    distances = compute_path_distances(xy_ref)

    # Closed loops (e.g. circular) get periodic splines so the reference wraps
    # around: the MPC horizon never runs out of points and the loop closes.
    closed = bool(np.allclose(xy_ref[0], xy_ref[-1]))

    v_ref = get_velocity(
        xy_ref=xy_ref,
        theta_segments=theta_segments,
        v_max=v_max,
        a_max=a_max,
        v_start=None,
        v_end=None if closed else 0.0,
    )

    # Cumulative arc length along the points, used to parameterize the splines.
    s = np.concatenate([[0.0], np.cumsum(distances)])

    bc_type = "periodic" if closed else "not-a-knot"

    if closed:
        # bc_type="periodic" requires the endpoints to match to machine precision.
        xy_ref[-1] = xy_ref[0]
        v_ref[-1] = v_ref[0]

    xy_spline = CubicSpline(s, xy_ref, bc_type=bc_type)
    v_spline = CubicSpline(s, v_ref, bc_type=bc_type)

    return xy_spline, v_spline


def eval_ref_state(
    xy_spline: CubicSpline,
    v_spline: CubicSpline,
    s: float,
) -> np.ndarray:
    """
    Evaluate the reference state [x, y, theta, v, omega] at arc length s.

    theta is the path tangent; omega = curvature * v.
    """
    xy = xy_spline(s)
    d1 = xy_spline(s, 1)
    d2 = xy_spline(s, 2)

    theta = np.arctan2(d1[1], d1[0])
    v = float(v_spline(s))

    denom = (d1[0] ** 2 + d1[1] ** 2) ** 1.5
    curvature = (d1[0] * d2[1] - d1[1] * d2[0]) / denom if denom > 1e-9 else 0.0
    omega = curvature * v

    return np.array([xy[0], xy[1], theta, v, omega], dtype=float)


def project_to_arclength(
    xy_spline: CubicSpline,
    xy: np.ndarray,
    s_prev: float | None = None,
    s_back: float = 1.0,
    s_fwd: float = 5.0,
    n_samples: int = 400,
) -> float:
    """
    Closest-point projection of xy onto the path.

    Returns the arc length s minimizing the distance to xy. When s_prev is given
    the search is restricted to [s_prev - s_back, s_prev + s_fwd] so that the
    progress stays monotonic; this is what lets the robot advance through laps of
    a self-overlapping path (e.g. a multi-turn circle) instead of snapping back to
    the first lap. With s_prev=None the whole path is searched.
    """
    s_max = xy_spline.x[-1]

    if s_prev is None:
        s_lo, s_hi = 0.0, s_max
    else:
        s_lo = max(0.0, s_prev - s_back)
        s_hi = min(s_max, s_prev + s_fwd)

    s_grid = np.linspace(s_lo, s_hi, n_samples)
    xy_grid = xy_spline(s_grid)

    idx = np.argmin(np.linalg.norm(xy_grid - np.asarray(xy, dtype=float), axis=1))

    return float(s_grid[idx])


def sample_ref_window(
    xy_spline: CubicSpline,
    v_spline: CubicSpline,
    state: np.ndarray,
    N: int,
    dt: float,
    s_prev: float = 0.0,
) -> tuple[np.ndarray, float]:
    """
    Sample a reference window of shape (N + 1, 5).

    Starts at the projection of the current state onto the path and advances
    by the planned arc length v * dt at each step. For periodic (closed) paths
    the arc length wraps around; otherwise it is clamped at the path end.
    """
    periodic = xy_spline.extrapolate == "periodic"
    s_max = xy_spline.x[-1]

    step_len = max(float(v_spline(s_prev)) * dt, 1e-3)

    s = project_to_arclength(
        xy_spline,
        state[:2],
        s_prev=None if periodic else s_prev,
        s_back=2.0 * step_len,
        s_fwd=5.0 * step_len,
    )
    s_start = s

    window = np.zeros((N + 1, 5))

    for k in range(N + 1):
        window[k] = eval_ref_state(xy_spline, v_spline, s)
        v = window[k, 3]
        s = s + v * dt
        if not periodic:
            s = min(s, s_max)

    # Keep theta continuous across the window (per-point atan2 wraps at +-pi).
    window[:, 2] = np.unwrap(window[:, 2])

    return window, s_start


def sample_full_traj(
    xy_spline: CubicSpline,
    v_spline: CubicSpline,
    ds: float = 0.1,
) -> np.ndarray:
    """
    Densely sample the full reference trajectory for visualization.

    Returns shape (M, 5).
    """
    s_max = xy_spline.x[-1]
    n_points = max(2, int(round(s_max / ds)) + 1)

    s_grid = np.linspace(0.0, s_max, n_points)

    return np.array(
        [eval_ref_state(xy_spline, v_spline, s) for s in s_grid],
        dtype=float,
    )


def get_straight_traj(
    n_points: int,
    x_length: float,
) -> np.ndarray:
    """
    Straight line of length x_length along x (y = 0).
    """

    x = np.linspace(0.0, x_length, n_points)

    xy_ref = np.zeros((n_points, 2))
    xy_ref[:, 0] = x
    xy_ref[:, 1] = 0.0

    return xy_ref


def get_circular_traj(
    n_points: int,
    n_turns: int = 1,
    center: tuple[float, float] = (0.0, 0.0),
    radius: float = 2.0,
) -> np.ndarray:
    """
    Circular path of n_turns laps.

    endpoint=False keeps the path open (it does not snap back onto the start),
    so the velocity profile can decelerate to a stop after the last lap.
    """

    cx, cy = center
    phi = np.linspace(0.0, 2.0 * np.pi * n_turns, n_points, endpoint=False)

    xy_ref = np.zeros((n_points, 2))
    xy_ref[:, 0] = cx + radius * np.cos(phi)
    xy_ref[:, 1] = cy + radius * np.sin(phi)

    return xy_ref


def get_sinusoidal_traj(
    n_points: int,
    x_length: float,
    sine_cycles: float,
    amplitude: float,
    amplitude_modulation: float,
    amplitude_modulation_cycles: float,
) -> np.ndarray:

    p = np.linspace(0.0, 1.0, n_points)
    x = x_length * p

    amp_envelope = amplitude * (
        1.0
        + amplitude_modulation
        * np.sin(2.0 * np.pi * amplitude_modulation_cycles * p)
    )

    y = amp_envelope * np.sin(2.0 * np.pi * sine_cycles * p)

    xy_ref = np.zeros((len(x), 2))
    xy_ref[:, 0] = x
    xy_ref[:, 1] = y

    return xy_ref


def get_velocity(
    xy_ref: np.ndarray,
    theta_segments: np.ndarray,
    v_max: float,
    a_max: float,
    v_start: float | None = 0.0,
    v_end: float | None = 0.0,
) -> np.ndarray:
    """
    Build a velocity profile along a geometric path.

    Circular acceleration constraint:
        ax^2 + ay^2 <= a_max^2

    with:
        ay = v^2 * curvature

    v_start / v_end constrain the boundary speeds. Pass None to leave a boundary
    free, i.e. keep the curvature-limited cruise speed there (used for closed
    loops and to avoid a rest-to-rest start that would stall the horizon).

    Returns:
        v_ref: shape (n_points,)
    """

    xy_ref = np.asarray(xy_ref, dtype=float)

    n_points = xy_ref.shape[0]

    if n_points < 2:
        raise ValueError("xy_ref must contain at least two points.")

    if v_max <= 0.0:
        raise ValueError("v_max must be positive.")

    if a_max <= 0.0:
        raise ValueError("a_max must be positive.")

    distances = compute_path_distances(xy_ref)

    curvature = np.abs(
        compute_path_curvature(
            xy_ref=xy_ref,
            theta_segments=theta_segments,
            distances=distances,
        )
    )

    v_curve_limit = np.full(n_points, v_max)

    curved = curvature > 1e-9
    v_curve_limit[curved] = np.sqrt(a_max / curvature[curved])

    v_ref = np.minimum(v_curve_limit, v_max)

    if v_start is not None:
        v_ref[0] = min(v_start, v_ref[0])
    if v_end is not None:
        v_ref[-1] = min(v_end, v_ref[-1])

    # Forward pass: acceleration limit
    for i in range(n_points - 1):
        ds = distances[i]
        ax_max = available_ax(v_ref[i], curvature[i], a_max)

        v_reachable = np.sqrt(v_ref[i] ** 2 + 2.0 * ax_max * ds)
        v_ref[i + 1] = min(v_ref[i + 1], v_reachable)

    # Backward pass: deceleration limit
    for i in range(n_points - 1, 0, -1):
        ds = distances[i - 1]
        ax_max = available_ax(v_ref[i], curvature[i], a_max)

        v_reachable = np.sqrt(v_ref[i] ** 2 + 2.0 * ax_max * ds)
        v_ref[i - 1] = min(v_ref[i - 1], v_reachable)

    # Start and end limits
    if v_start is not None:
        v_ref[0] = v_start
    if v_end is not None:
        v_ref[-1] = v_end

    return v_ref


def available_ax(v: float, kappa: float, a_max: float) -> float:
    """
    Remaining longitudinal acceleration under circular acceleration limit.
    """
    ay = v * v * abs(kappa)
    remaining = a_max * a_max - ay * ay
    return np.sqrt(max(remaining, 0.0))


def compute_path_distances(xy_ref: np.ndarray) -> np.ndarray:
    """
    Distance between consecutive xy points.

    Returns:
        distances: shape (n_points - 1,)
    """
    dxy = np.diff(xy_ref, axis=0)
    return np.linalg.norm(dxy, axis=1)


def compute_path_curvature(
    xy_ref: np.ndarray,
    theta_segments: np.ndarray,
    distances: np.ndarray,
) -> np.ndarray:
    """
    Simple curvature estimate from heading changes along the path.

    Returns:
        curvature: shape (n_points,)
    """
    n_points = xy_ref.shape[0]

    curvature = np.zeros(n_points)

    if n_points < 3:
        return curvature

    dtheta = np.diff(theta_segments)

    ds_mid = 0.5 * (distances[:-1] + distances[1:])
    ds_mid = np.maximum(ds_mid, 1e-9)

    curvature[1:-1] = dtheta / ds_mid

    curvature[0] = curvature[1]
    curvature[-1] = curvature[-2]

    return curvature