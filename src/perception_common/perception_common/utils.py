from __future__ import annotations

import threading
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


class LatestStampedCache:
    """Holds the most recent message from a slower stream for capture-time fusion.

    Detections arrive slower than the LiDAR they are fused against, so the cloud
    callback drives the output and pulls whatever detection is cached. ``get_fresh``
    releases the cached message only when its ``header.stamp`` is within ``max_age``
    of the driving message's stamp, which bounds how stale the fused geometry can be
    and makes a stalled upstream node fail closed instead of publishing forever.
    """

    def __init__(self, name: str = "detection"):
        self.name = name
        self._lock = threading.Lock()
        self._msg = None
        self.accepted_count = 0
        self.rejected_count = 0
        self.zero_stamp_count = 0

    def update(self, msg) -> None:
        with self._lock:
            self._msg = msg

    def get_fresh(self, reference_header, max_age: float):
        """Return ``(msg, skew)`` when the cached message is fresh, else ``(None, skew)``.

        ``skew`` is ``float('inf')`` when nothing has been cached yet, or when either
        header carries a zero stamp (which would otherwise compare as a ~decades-large
        difference and silently look like permanent staleness).
        """
        with self._lock:
            msg = self._msg

        if msg is None:
            return None, float("inf")

        if is_zero_stamp(reference_header) or is_zero_stamp(msg.header):
            with self._lock:
                self.zero_stamp_count += 1
                self.rejected_count += 1
            return None, float("inf")

        skew = stamp_skew(reference_header, msg.header)
        with self._lock:
            if skew <= max_age:
                self.accepted_count += 1
                return msg, skew
            self.rejected_count += 1
        return None, skew
