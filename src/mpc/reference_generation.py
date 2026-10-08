import numpy as np
from scipy.interpolate import CubicSpline


def generate_ref_points(ref: dict) -> np.ndarray:
    """
    Generate world-coordinate points from ref (headings in radians).
    """
    traj_type = ref["traj_type"]
    if traj_type not in {"straight", "circular", "sinusoidal"}:
        raise ValueError(f"Unknown traj_type: {traj_type!r}.")

    def positive(value, name):
        value = float(value)
        if not np.isfinite(value) or value <= 0:
            raise ValueError(f"{name} must be finite and positive.")
        return value

    def point(value, name):
        value = np.asarray(value, dtype=float)
        if value.shape != (2,) or not np.all(np.isfinite(value)):
            raise ValueError(f"{name} must contain two finite coordinates.")
        return value

    spacing = positive(ref["spacing"], "spacing")
    cfg = ref[traj_type]
    start = point(ref["start_position"][:2], f"{traj_type}.start")

    if traj_type == "straight":
        end = point(cfg["end"], "straight.end")
        length = positive(np.linalg.norm(end - start), "straight.length")
        n = max(2, int(np.ceil(length / spacing)) + 1)
        return np.linspace(start, end, n)

    heading = float(cfg.get("heading", 0.0))
    if not np.isfinite(heading):
        raise ValueError("heading must be finite.")
    c, s = np.cos(heading), np.sin(heading)
    rotation = np.array([[c, -s], [s, c]])

    if traj_type == "circular":
        radius = positive(cfg["radius"], "circular.radius")
        turns_value = positive(cfg.get("turns", 1), "circular.turns")
        if not turns_value.is_integer():
            raise ValueError("circular.turns must be an integer.")
        turns = int(turns_value)
        clockwise = cfg.get("clockwise", False)
        if not isinstance(clockwise, bool):
            raise ValueError("circular.clockwise must be a boolean.")
        direction = -1.0 if clockwise else 1.0
        segments_per_lap = max(16, int(np.ceil(2.0 * np.pi * radius / spacing)))
        phi = np.linspace(0.0, 2.0 * np.pi * turns, segments_per_lap * turns + 1)
        # Start on the circle with tangent +x; its center is (0, direction*r).
        local = np.column_stack((
            radius * np.sin(phi),
            direction * radius * (1.0 - np.cos(phi)),
        ))
        local[-1] = local[0]
    else:
        length = positive(cfg["length"], "sinusoidal.length")
        cycles = positive(cfg.get("cycles", 1.0), "sinusoidal.cycles")
        amplitude = float(cfg["amplitude"])
        if not np.isfinite(amplitude) or amplitude < 0:
            raise ValueError("sinusoidal.amplitude must be finite and >= 0.")
        # spacing resolves the baseline; also resolve each oscillation.
        segments = max(2, int(np.ceil(length / spacing)), int(np.ceil(32.0 * cycles)))
        p = np.linspace(0.0, 1.0, segments + 1)
        local = np.column_stack((
            length * p,
            amplitude * np.sin(2.0 * np.pi * cycles * p),
        ))

    return start + local @ rotation.T


def validate_ref_bounds(xy_spline: CubicSpline, bounds: dict) -> None:
    """Check knots and all interior coordinate extrema, including overshoot."""
    limits = np.asarray([bounds["x"], bounds["y"]], dtype=float)
    if (limits.shape != (2, 2) or not np.all(np.isfinite(limits))
            or np.any(limits[:, 0] >= limits[:, 1])):
        raise ValueError("bounds must contain finite, increasing x/y intervals.")
    if not np.all(np.isfinite(xy_spline.c)):
        raise ValueError("Reference contains non-finite spline coefficients.")

    candidates = list(xy_spline.x)
    for i, width in enumerate(np.diff(xy_spline.x)):
        for axis in range(2):
            a, b, c, _ = xy_spline.c[:, i, axis]
            for root in np.roots([3.0 * a, 2.0 * b, c]):
                if abs(root.imag) < 1e-10 and 0.0 < root.real < width:
                    candidates.append(xy_spline.x[i] + float(root.real))

    xy = xy_spline(np.asarray(candidates))
    if not np.all(np.isfinite(xy)):
        raise ValueError("Reference contains non-finite coordinates.")
    actual_min, actual_max = xy.min(axis=0), xy.max(axis=0)
    tolerance = 1e-9
    if (np.any(actual_min < limits[:, 0] - tolerance)
            or np.any(actual_max > limits[:, 1] + tolerance)):
        raise ValueError(
            f"Generated reference: x=[{actual_min[0]:.3f}, {actual_max[0]:.3f}], "
            f"y=[{actual_min[1]:.3f}, {actual_max[1]:.3f}]. "
            f"Allowed bounds: x={bounds['x']}, y={bounds['y']}."
        )


def build_full_ref_traj(
    ref: dict,
    v_max: float,
    a_max: float,
) -> tuple[CubicSpline, CubicSpline]:
    """
    Build a finite reference and reject inadmissible geometry before use.

    Splines use cumulative chord length s. All paths, including multi-lap
    circles, end with zero reference speed and are not periodically repeated.
    """
    try:
        xy_ref = generate_ref_points(ref)
        distances = compute_path_distances(xy_ref)
        if not np.all(np.isfinite(distances)) or np.any(distances <= 0.0):
            raise ValueError("Consecutive reference points must be finite and differ.")
        dxy = np.diff(xy_ref, axis=0)
        theta_segments = np.unwrap(np.arctan2(dxy[:, 1], dxy[:, 0]))
        s = np.concatenate(([0.0], np.cumsum(distances)))
        xy_spline = CubicSpline(s, xy_ref, bc_type="not-a-knot", extrapolate=False)
        validate_ref_bounds(xy_spline, ref["bounds"])
    except (KeyError, TypeError, ValueError, OverflowError) as exc:
        raise ValueError(
            "Reference parameters are not admissible. "
            "Change parameters in experiment.yaml. "
            f"{exc}"
        ) from exc

    v_ref = get_velocity(
        xy_ref=xy_ref,
        theta_segments=theta_segments,
        v_max=v_max,
        a_max=a_max,
        v_start=None,
        v_end=0.0,
    )
    v_spline = CubicSpline(s, v_ref, bc_type="not-a-knot", extrapolate=False)
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