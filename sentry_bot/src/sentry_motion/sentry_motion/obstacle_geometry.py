"""Swept chassis collision checks in chassis coordinates, in metres."""
import numpy as np


def blocked_by_points(points, aligning=False):
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    if not len(points):
        return False
    x, y, z = points.T
    finite = np.isfinite(points).all(axis=1)
    height = (z > -0.05) & (z < 0.14)
    if aligning:
        # A 0.43 m disk covers rotation plus low-speed alignment corrections.
        # Points on/in the measured body box may be self returns; only the
        # external swept region is used for the rotation guard.
        swept = (x * x + y * y < 0.43 ** 2) & ((np.abs(x) > 0.29) | (np.abs(y) > 0.27))
    else:
        front = (x > 0.29) & (x < 0.60) & (np.abs(y) < 0.30)
        sides = (np.abs(x) < 0.29) & (np.abs(y) > 0.27) & (np.abs(y) < 0.34)
        swept = front | sides
    return bool(np.any(finite & height & swept))
