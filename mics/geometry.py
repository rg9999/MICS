"""Small geometry / linear-algebra helpers used across the sim."""

from __future__ import annotations

import numpy as np


def unit(v: np.ndarray) -> np.ndarray:
    """Return the unit vector of v, or a zero vector if v is ~zero."""
    n = float(np.linalg.norm(v))
    if n < 1e-12:
        return np.zeros_like(v, dtype=float)
    return v / n


def norm(v: np.ndarray) -> float:
    return float(np.linalg.norm(v))


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def clamp_norm(v: np.ndarray, max_norm: float) -> np.ndarray:
    """Scale v down so its magnitude does not exceed max_norm."""
    n = norm(v)
    if n <= max_norm or n < 1e-12:
        return v
    return v * (max_norm / n)


def bearing_elevation(rel: np.ndarray) -> tuple[float, float]:
    """Azimuth (rad, from +x toward +y) and elevation (rad) of a relative vector."""
    az = float(np.arctan2(rel[1], rel[0]))
    horiz = float(np.hypot(rel[0], rel[1]))
    el = float(np.arctan2(rel[2], horiz))
    return az, el


def angle_between(a: np.ndarray, b: np.ndarray) -> float:
    """Angle (rad) between two vectors, robust to numerical drift."""
    na, nb = norm(a), norm(b)
    if na < 1e-12 or nb < 1e-12:
        return 0.0
    c = clamp(float(np.dot(a, b) / (na * nb)), -1.0, 1.0)
    return float(np.arccos(c))
