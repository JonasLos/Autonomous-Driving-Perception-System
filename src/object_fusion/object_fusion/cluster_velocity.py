"""Velocity from a track's own LiDAR clusters, over a window of about a second.

No rclpy, no message types, so the unit tests exercise this exact code.

WHY THIS EXISTS. Once the camera loses an object, cluster updates keep it alive but are
position-only, so nothing corrects its velocity. Radar still updates it, and the ESR's range-rate
lags the geometry while the bearing sweeps (see ``tracker.radar_rate_lag``), so a standing
pedestrian being passed picks up a velocity pointing at the car, which then freezes. On
2026-11-05 (run 145604) that braked the car from 9.7 to 4.2 m/s.

WHY THIS IS NOT THE THING CLUSTERS WERE KEPT AWAY FROM. Cluster updates are position-only because
the difference between CONSECUTIVE clusters is an offset, not motion. A median of slopes over a
window is a different estimator: measured with ``scripts/cluster_velocity_ab.py`` on 7 replays,
standing people read above 0.5 m/s 0% of the time at a 1 s window (the filter: 13%).

PEOPLE ONLY FOR NOW (2026-11-09). On standing VEHICLES the 1 s fit reads above 0.5 m/s 15% of the
time, and with the car above 5 m/s the overall rate (11%) failed the 10% limit set before measuring.
A car's cluster centre probably shifts as different faces come into view, but that is unconfirmed.
HANDOFF records what to investigate before widening ``DEFAULT_CLASSES``.
"""

from __future__ import annotations

import math
from collections import deque

import numpy as np

__all__ = ["DEFAULT_WINDOW_S", "DEFAULT_SIGMA", "DEFAULT_CLASSES", "CAMERA_STALE_S", "OBSERVER_MAX_AGE_S",
           "OdomFrame", "ClusterHistory", "median_slope_velocity", "velocity_measurement", "velocity_observed",
           "parse_classes"]

#: 1 s is the shortest window whose standing-person noise is clean; 0.5 s reads above 0.5 m/s 7% of the time.
DEFAULT_WINDOW_S = 1.0
#: Velocity sigma of one fit [m/s]. 0.23 m/s is the 90th percentile on standing people at 1 s, rounded up.
DEFAULT_SIGMA = 0.3
DEFAULT_CLASSES = "person"
#: The fit is applied only once the camera has not updated the track for this long.
CAMERA_STALE_S = 0.5
#: With no camera, radar or cluster-fit velocity update for this long, velocity_valid goes false.
OBSERVER_MAX_AGE_S = 0.5
#: How long the odometric pose history reaches back. Must cover the longest window plus lateness.
POSE_HISTORY_S = 4.0
#: History length per track: 10 Hz over the 4 s pose history.
_MAX_SAMPLES = 40


def parse_classes(text: str) -> frozenset:
    """``"person, cyclist"`` -> a set of class names. Empty text selects no class."""
    return frozenset(c.strip() for c in str(text).split(",") if c.strip())


class OdomFrame:
    """A ground-fixed frame integrated from the same increments ``tracker.predict`` consumes.

    Cluster positions from different sweeps can only be compared once ego motion is removed. Using
    the prediction's own increments means the fit cannot disagree with the filter about how the car
    moved. Poses are kept by stamp because a cluster can arrive older than the prediction clock.
    """

    def __init__(self, history_s: float = POSE_HISTORY_S):
        self.history_s = float(history_s)
        self._poses: deque = deque()

    def reset(self):
        self._poses.clear()

    def advance(self, t0: float, t1: float, dpsi: float, d_xy) -> None:
        """Add the step from ``t0`` to ``t1``. ``d_xy`` is expressed in the frame at ``t0``."""
        if not self._poses:
            self._poses.append((float(t0), 0.0, 0.0, 0.0))
        _, x, y, psi = self._poses[-1]
        c, s = math.cos(psi), math.sin(psi)
        dx, dy = float(d_xy[0]), float(d_xy[1])
        self._poses.append((float(t1), x + c * dx - s * dy, y + s * dx + c * dy, psi + float(dpsi)))
        cutoff = float(t1) - self.history_s
        while len(self._poses) > 2 and self._poses[1][0] < cutoff:
            self._poses.popleft()

    def pose_at(self, t: float):
        """``(x, y, psi)`` at ``t``, interpolated, or None outside the history."""
        if not self._poses or t < self._poses[0][0]:
            return None
        if t >= self._poses[-1][0]:
            return self._poses[-1][1:] if t - self._poses[-1][0] < 1e-3 else None
        stamps = [p[0] for p in self._poses]
        i = int(np.searchsorted(stamps, t, side="right"))
        (t0, x0, y0, p0), (t1, x1, y1, p1) = self._poses[i - 1], self._poses[i]
        a = (t - t0) / (t1 - t0) if t1 > t0 else 1.0
        return x0 + a * (x1 - x0), y0 + a * (y1 - y0), p0 + a * (p1 - p0)

    def to_fixed(self, t: float, xy):
        """A sensor-frame point observed at ``t``, in the fixed frame. None outside the history."""
        pose = self.pose_at(t)
        if pose is None:
            return None
        x, y, psi = pose
        c, s = math.cos(psi), math.sin(psi)
        return x + c * float(xy[0]) - s * float(xy[1]), y + s * float(xy[0]) + c * float(xy[1])

    def velocity_to_sensor(self, v_xy):
        """A fixed-frame velocity in the CURRENT sensor axes, the axes of the filter state."""
        psi = self._poses[-1][3]
        c, s = math.cos(psi), math.sin(psi)
        return np.array([c * float(v_xy[0]) + s * float(v_xy[1]), -s * float(v_xy[0]) + c * float(v_xy[1])])


class ClusterHistory:
    """Stamped fixed-frame cluster positions of one track, oldest first."""

    __slots__ = ("_q",)

    def __init__(self):
        self._q: deque = deque(maxlen=_MAX_SAMPLES)

    def add(self, t: float, xy) -> None:
        # A late cluster is dropped rather than inserted: a window does not need it.
        if not self._q or t > self._q[-1][0]:
            self._q.append((float(t), float(xy[0]), float(xy[1])))

    def __len__(self):
        return len(self._q)

    def array(self):
        return np.array(self._q, dtype=np.float64).reshape(-1, 3)


def median_slope_velocity(samples, now, window):
    """Median of pairwise slopes over the trailing ``window``, or None without enough support.

    Needs 60% of a 10 Hz sweep count and 70% of the window spanned. Pairs closer than a quarter of
    the window are skipped, because over a short baseline an offset between clusters reads as speed.
    A median ignores the occasional switch to a neighbouring cluster, which a least-squares line
    would follow. The offset is removed first: raw odometric coordinates are harmless here, but the
    same fit on UTM and epoch stamps was floating-point noise.
    """
    a = np.asarray(samples, dtype=np.float64)
    if a.size == 0:
        return None
    window = float(window)
    a = a[(a[:, 0] > now - window) & (a[:, 0] <= now + 1e-9)]
    if len(a) < max(3, int(0.6 * window * 10)) or a[-1, 0] - a[0, 0] < 0.7 * window:
        return None
    t = a[:, 0] - a[0, 0]
    xy = a[:, 1:3] - a[0, 1:3]
    i, j = np.triu_indices(len(t), k=1)
    dt = t[j] - t[i]
    keep = dt >= 0.25 * window
    if not keep.any():
        return None
    slopes = (xy[j[keep]] - xy[i[keep]]) / dt[keep][:, None]
    return np.median(slopes, axis=0)


def velocity_measurement(x, v_meas, sigma):
    """``(y, H, R)`` for ``kalman_update``: a 2-D velocity measurement of ``[px, py, vx, vy]``."""
    H = np.array([[0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
    return np.asarray(v_meas, dtype=np.float64) - np.asarray(x, dtype=np.float64)[2:], H, np.eye(2) * float(sigma) ** 2


def velocity_observed(now, stamps, max_age=OBSERVER_MAX_AGE_S) -> bool:
    """Has any velocity-observing update, among ``stamps`` (None = never), arrived recently?"""
    return any(s is not None and float(now) - float(s) <= float(max_age) for s in stamps)
