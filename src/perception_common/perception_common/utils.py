from __future__ import annotations

import time
from functools import wraps

import numpy as np


def timer(func):
    @wraps(func)
    def wrapper(*args, **kwargs):
        start_time = time.time()
        result = func(*args, **kwargs)
        end_time = time.time()
        print(f"{func.__name__} executed in {end_time - start_time:.4f} seconds")
        return result

    return wrapper


def inverse_rigid_transform(arr: np.ndarray) -> np.ndarray:
    Rt = arr[:3, :3].T
    tt = -np.dot(Rt, arr[:3, 3])
    return np.vstack((np.column_stack((Rt, tt)), [0, 0, 0, 1]))


def crop_pointcloud(pointcloud, lim_x, lim_y, lim_z):
    mask = (
        (pointcloud[:, 0] >= lim_x[0])
        & (pointcloud[:, 0] <= lim_x[1])
        & (pointcloud[:, 1] >= lim_y[0])
        & (pointcloud[:, 1] <= lim_y[1])
        & (pointcloud[:, 2] >= lim_z[0])
        & (pointcloud[:, 2] <= lim_z[1])
    )
    return pointcloud[mask]


def stamp_to_seconds(stamp) -> float:
    """Convert a builtin_interfaces/Time to float seconds."""
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def stamp_skew(header_a, header_b) -> float:
    """Absolute capture-time difference between two message headers, in seconds.

    Compares the two message stamps against each other and never against the node
    clock, so the result is independent of the clock domain: under bag replay both
    stamps come from the bag, and live both come from the sensor drivers.
    """
    return abs(stamp_to_seconds(header_a.stamp) - stamp_to_seconds(header_b.stamp))


def is_zero_stamp(header) -> bool:
    """True when a header carries an unset (0) timestamp."""
    return header.stamp.sec == 0 and header.stamp.nanosec == 0


# LatestStampedCache lived here until 2026-08-13. It gated a cached detection on being
# within max_age of the driving cloud's stamp -- a freshness *gate*, which can only trade
# dropped frames for misplaced ones: widen it and stale geometry gets through, tighten it
# and the node starves. Use perception_common.stamp_sync.StampMatchedBuffer instead, which
# buffers the fast stream and pairs each detection with the sample captured nearest to it.
