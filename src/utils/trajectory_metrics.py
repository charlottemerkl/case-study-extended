"""Geometric tracking errors against the sampled reference polyline."""
import numpy as np


def trajectory_distances(actual_xy, reference_xy):
    """
    Return the point-to-path distance for each actual sample.

    Project onto finite line segments, not only the reference vertices. This
    measures geometric tracking, not timing, lap progress or path completion.
    The reference polyline approximates the original continuous reference.
    Missing/empty trajectories return None rather than a misleading zero.
    """
    if actual_xy is None or reference_xy is None:
        return None
    actual = np.asarray(actual_xy, dtype=float)
    reference = np.asarray(reference_xy, dtype=float)
    if actual.size == 0 or reference.size == 0:
        return None
    for points in (actual, reference):
        if points.ndim != 2 or points.shape[1] != 2 or not np.isfinite(points).all():
            raise ValueError("Trajectories must be finite (N, 2) arrays.")

    # A single point (or duplicate vertices) is also a valid reference.
    starts = reference[:-1] if len(reference) > 1 else reference
    ends = reference[1:] if len(reference) > 1 else reference
    distances_squared = np.empty(len(actual))
    # Bound temporary memory independently of trajectory length.
    for i in range(0, len(actual), 256):
        points = actual[i:i + 256]
        best = np.full(len(points), np.inf)
        for j in range(0, len(starts), 256):
            origins = starts[j:j + 256]
            vectors = ends[j:j + 256] - origins
            lengths_squared = np.sum(vectors * vectors, axis=1)
            offsets = points[:, None, :] - origins[None, :, :]
            numerator = np.sum(offsets * vectors[None, :, :], axis=2)
            fraction = np.divide(numerator, lengths_squared,
                                 out=np.zeros_like(numerator), where=lengths_squared > 0)
            fraction = np.clip(fraction, 0.0, 1.0)
            residual = offsets - fraction[:, :, None] * vectors[None, :, :]
            best = np.minimum(best, np.sum(residual * residual, axis=2).min(axis=1))
        distances_squared[i:i + len(points)] = best
    return np.sqrt(distances_squared)


def trajectory_metrics(actual_xy, reference_xy):
    """Return MSE, MAE, RMSE and maximum absolute point-to-path error."""
    distances = trajectory_distances(actual_xy, reference_xy)
    if distances is None:
        return None
    return error_metrics(distances)


def error_metrics(error):
    """Sample-weighted metrics; max is the largest absolute error."""
    error = np.asarray(error, dtype=float)
    if error.ndim != 1 or not np.isfinite(error).all():
        raise ValueError("Errors must be a finite one-dimensional array.")
    if not error.size:
        return None
    mse = float(np.mean(error ** 2))
    return {"samples": len(error), "mse": mse,
            "mae": float(np.mean(np.abs(error))), "rmse": float(np.sqrt(mse)),
            "max": float(np.max(np.abs(error)))}


def orientation_metrics(actual_theta, reference_theta):
    """Evaluate shortest signed angle differences, in radians, modulo 2*pi."""
    actual = np.asarray(actual_theta, dtype=float)
    reference = np.asarray(reference_theta, dtype=float)
    if actual.shape != reference.shape or actual.ndim != 1:
        raise ValueError("Angles must be paired one-dimensional arrays.")
    if not np.isfinite(actual).all() or not np.isfinite(reference).all():
        raise ValueError("Angles must be finite.")
    difference = actual - reference
    return error_metrics(np.arctan2(np.sin(difference), np.cos(difference)))


def velocity_metrics(actual_v, reference_v):
    """Sample-weighted errors between paired linear or angular velocities."""
    actual, reference = np.asarray(actual_v, dtype=float), np.asarray(reference_v, dtype=float)
    if actual.shape != reference.shape or actual.ndim != 1:
        raise ValueError("Velocities must be paired one-dimensional arrays.")
    if not actual.size:
        return None
    if not np.isfinite(actual).all() or not np.isfinite(reference).all():
        raise ValueError("Velocities must be finite.")
    return error_metrics(actual - reference)
