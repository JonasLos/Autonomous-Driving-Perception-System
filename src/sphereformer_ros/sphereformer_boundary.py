"""Road boundary from SphereFormer's labelled sweep, and fast PointCloud2 packing.

Free of rclpy and torch so it is unit-testable on any host; sphereformer_ros.py wraps it.

What the legacy rule (sphereformer_ros.ros_callback) does, and why each piece is replaced:

* Bins are ``np.linspace(min_x, max_x, 22)`` over ALL road points. Road reaches -30..92 m on
  the replays, so bins are ~5.8 m wide, put boundary points on the road BEHIND the vehicle, and
  change width every frame as the farthest road point moves.  -> FIXED bins from x_min to x_max
  in the VEHICLE frame (S3), so a station means the same distance ahead in every frame. (On a
  straight edge the lidar_tc-x strips still pick points on the edge; the yaw matters for where
  the stations are, not for whether the picked point is on the edge.)
* A road-labelled patch beside the road (a driveway, a lot) becomes the extreme.
  -> optional ego-corridor run selection (S4). A lateral gap separates runs only when non-road
  returns lie IN it (an empty gap is a hole between ring arcs, not a separation), and the run is
  chosen against a reference that cannot drift: the run containing the vehicle's own lateral
  position in the first bins, then the previous bin's run only within ``max_ref_jump``.
* One sweep holds only arcs of each ground ring (rings are 3-6 m apart beyond 20 m), so many
  bins are empty or hold a ring segment that stops inside the road.
  -> accumulate the last N labelled sweeps, carried into the current frame by the odometry
  twist (S7).
* The ego vehicle's roof and hood (0.8-0.9 m) come out labelled "car".
  -> crop them from the input before inference (S2).
"""

from __future__ import annotations

import math
from bisect import bisect_left
from collections import deque

import numpy as np

__all__ = ["ROAD", "OTHER_MIN_CLASS", "crop_ego", "lidar_to_vehicle", "vehicle_to_lidar", "fixed_bin_boundary",
           "legacy_boundary", "TwistIntegrator", "SweepAccumulator", "pack_cloud"]

ROAD = 9
#: Learned classes counted as NON-road evidence: parking .. traffic-sign. Vehicles and people
#: are excluded -- they hide road rather than bound it.
OTHER_MIN_CLASS = 10


def crop_ego(points: np.ndarray, radius: float) -> np.ndarray:
    """Drop points within ``radius`` (xy) of the sensor: the vehicle's own body."""
    if radius <= 0.0 or points.shape[0] == 0:
        return points
    return points[np.hypot(points[:, 0], points[:, 1]) >= radius]


def _rot(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s], [s, c]])


def lidar_to_vehicle(xy, ego_yaw_deg):
    return np.atleast_2d(np.asarray(xy, float)) @ _rot(-math.radians(ego_yaw_deg)).T


def vehicle_to_lidar(xy, ego_yaw_deg):
    return np.atleast_2d(np.asarray(xy, float)) @ _rot(math.radians(ego_yaw_deg)).T


def legacy_boundary(road_points: np.ndarray):
    """The original rule, unchanged: 21 bins between the road's min and max lidar_tc x."""
    left, right = [], []
    if road_points.size == 0:
        return np.empty((0, road_points.shape[1])), np.empty((0, road_points.shape[1]))
    bins = np.linspace(road_points[:, 0].min(), road_points[:, 0].max(), num=22)
    for i in range(len(bins) - 1):
        bp = road_points[(road_points[:, 0] >= bins[i]) & (road_points[:, 0] < bins[i + 1])]
        if bp.size == 0:
            continue
        left.append(bp[np.argmax(bp[:, 1])])
        right.append(bp[np.argmin(bp[:, 1])])
    return np.asarray(left), np.asarray(right)


def _select_run(ys_road, ys_other, ref, gap, min_other):
    """(lo, hi) of the road run overlapping ``ref`` best; gaps split only with evidence in them."""
    ys = np.sort(ys_road)
    breaks = np.flatnonzero(np.diff(ys) > gap)
    if breaks.size and ys_other.size:
        oth = np.sort(ys_other)
        n_in = (np.searchsorted(oth, ys[breaks + 1], side="left")
                - np.searchsorted(oth, ys[breaks], side="right"))
        breaks = breaks[n_in >= min_other]
    elif breaks.size:
        breaks = breaks[:0]
    lo_ref, hi_ref = ref
    best, best_key = None, None
    for run in np.split(ys, breaks + 1):
        lo, hi = run[0], run[-1]
        key = (min(hi, hi_ref) - max(lo, lo_ref), -abs(0.5 * (lo + hi) - 0.5 * (lo_ref + hi_ref)))
        if best_key is None or key > best_key:
            best, best_key = (lo, hi), key
    return best


def fixed_bin_boundary(road_xyz: np.ndarray, *, other_xy: np.ndarray | None = None,
                       ego_yaw_deg: float = -5.35, x_min: float = 5.0, x_max: float = 60.0,
                       bin_len: float = 2.0, min_bin_points: int = 3, y_max: float = 40.0,
                       connected: bool = False, run_gap: float = 2.0, min_other: int = 5,
                       seed_half_width: float = 1.5, max_ref_jump: float = 3.0,
                       require_evidence: bool = True, edge_evidence_m: float = 1.0):
    """Per fixed vehicle-frame x-bin, the lateral extremes of road. Returns lidar_tc (N,3) x2.

    With ``connected``, only the lateral run continuing the vehicle's road is used (see module
    docstring); ``other_xy`` (non-road returns, lidar_tc) is the evidence that splits runs.

    With ``require_evidence`` an extreme is kept only when a non-road return lies within
    ``edge_evidence_m`` beyond it in the same bin. A 2 m bin at range often holds only an arc of
    one ground ring, whose end is inside the road: measured at 40 m on the Sep 22 road replays,
    the right "edge" read -0.3 m against a real -2.9 m without this check.
    """
    road_xyz = np.asarray(road_xyz, dtype=np.float64).reshape(-1, 3)
    empty = np.empty((0, 3), np.float32)
    if road_xyz.shape[0] == 0:
        return empty, empty
    veh = lidar_to_vehicle(road_xyz[:, :2], ego_yaw_deg)
    keep = np.abs(veh[:, 1]) <= y_max
    veh, z = veh[keep], road_xyz[keep, 2]
    nb = int(math.floor((x_max - x_min) / bin_len))
    b = np.floor((veh[:, 0] - x_min) / bin_len).astype(np.int64)
    if (connected or require_evidence) and other_xy is not None and np.asarray(other_xy).size:
        ov = lidar_to_vehicle(np.asarray(other_xy)[:, :2], ego_yaw_deg)
        ob = np.floor((ov[:, 0] - x_min) / bin_len).astype(np.int64)
    else:
        ov, ob = np.empty((0, 2)), np.empty(0, np.int64)
    ref = (-seed_half_width, seed_half_width)
    left, right = [], []
    for i in range(nb):
        k = np.flatnonzero(b == i)
        if k.size < min_bin_points:
            continue
        ys = veh[k, 1]
        if connected:
            run = _select_run(ys, ov[ob == i, 1], ref, run_gap, min_other)
            centre = 0.5 * (run[0] + run[1])
            if abs(centre - 0.5 * (ref[0] + ref[1])) > max_ref_jump and \
                    min(run[1], ref[1]) - max(run[0], ref[0]) < 0.0:
                continue                      # a jump to a different surface: skip the bin
            sel = (ys >= run[0]) & (ys <= run[1])
            if sel.sum() < min_bin_points:
                continue
            k, ys = k[sel], ys[sel]
            ref = run
        # Without non-road returns at all (other_xy is None) there is nothing to test against.
        oth_y = ov[ob == i, 1] if (require_evidence and other_xy is not None) else None
        for out, j, sgn in ((left, k[np.argmax(ys)], 1.0), (right, k[np.argmin(ys)], -1.0)):
            if oth_y is not None:
                d = sgn * (oth_y - veh[j, 1])
                if not ((d > 0.0) & (d <= edge_evidence_m)).any():
                    continue
            out.append((veh[j, 0], veh[j, 1], z[j]))
    out = []
    for pts in (left, right):
        if not pts:
            out.append(empty)
            continue
        a = np.asarray(pts, dtype=np.float64)
        a[:, :2] = vehicle_to_lidar(a[:, :2], ego_yaw_deg)
        out.append(a.astype(np.float32))
    return out[0], out[1]


class TwistIntegrator:
    """Body-frame twist history -> how the vehicle frame moved between two stamps (SE2).

    The same contract as object_fusion.ego_motion.TwistBuffer (which this container does not
    have): stamps are only compared to each other, a stamp past the newest sample is served
    from it for up to ``max_hold`` s, and anything else returns None rather than inventing motion.
    """

    def __init__(self, duration: float = 3.0, max_hold: float = 0.2):
        self.duration, self.max_hold = duration, max_hold
        self._t: deque = deque()
        self._v: deque = deque()

    def add(self, t, vx, vy, wz):
        if self._t and t <= self._t[-1]:
            if t < self._t[-1] - 1.0:          # bag rewind: start over
                self._t.clear()
                self._v.clear()
            else:
                return
        self._t.append(t)
        self._v.append((vx, vy, wz))
        while self._t and self._t[0] < t - self.duration:
            self._t.popleft()
            self._v.popleft()

    def _at(self, t):
        if not self._t or t < self._t[0] - self.max_hold or t > self._t[-1] + self.max_hold:
            return None
        i = bisect_left(self._t, t)
        return self._v[min(i, len(self._v) - 1)]

    def motion(self, t0, t1):
        """(dpsi, dx, dy) of the vehicle frame from t0 to t1, in the t0 frame; None if unknown."""
        if t1 <= t0:
            return 0.0, 0.0, 0.0
        edges = [t0] + [t for t in self._t if t0 < t < t1] + [t1]
        psi, x, y = 0.0, 0.0, 0.0
        for a, b in zip(edges[:-1], edges[1:]):
            tw = self._at(0.5 * (a + b))
            if tw is None:
                return None
            dt = b - a
            c, s = math.cos(psi), math.sin(psi)
            x += (c * tw[0] - s * tw[1]) * dt
            y += (s * tw[0] + c * tw[1]) * dt
            psi += tw[2] * dt
        return psi, x, y


class SweepAccumulator:
    """The last N sweeps' road and non-road points, carried into the newest sweep's frame.

    Points are stored in the VEHICLE frame of their own sweep; at query time each older sweep is
    moved by the vehicle's motion since then (a world-fixed point p seen at t0 is
    R(dpsi)^T (p - d) at t1). Only static surfaces are accumulated -- road and the non-road
    evidence -- so moving traffic does not smear into the boundary.
    """

    def __init__(self, n: int, ego_yaw_deg: float = -5.35):
        self.n = max(1, int(n))
        self.ego_yaw_deg = ego_yaw_deg
        self._sweeps: deque = deque(maxlen=self.n)

    def add(self, t, road_xyz, other_xy):
        if self._sweeps and t < self._sweeps[-1][0] - 1e-6:
            self._sweeps.clear()               # bag rewind
        road = np.asarray(road_xyz, float).reshape(-1, 3).copy()
        road[:, :2] = lidar_to_vehicle(road[:, :2], self.ego_yaw_deg)
        other = lidar_to_vehicle(np.asarray(other_xy, float).reshape(-1, 2), self.ego_yaw_deg)
        self._sweeps.append((t, road, other))

    def merged(self, integrator: TwistIntegrator | None):
        """(road_xyz, other_xy) in the newest sweep's lidar_tc frame. Unmovable sweeps are skipped."""
        if not self._sweeps:
            return np.empty((0, 3)), np.empty((0, 2))
        t_new = self._sweeps[-1][0]
        roads, others = [], []
        for t, road, other in self._sweeps:
            if t != t_new:
                if integrator is None:
                    continue
                m = integrator.motion(t, t_new)
                if m is None:
                    continue
                dpsi, dx, dy = m
                c, s = math.cos(dpsi), math.sin(dpsi)
                rt = np.array([[c, s], [-s, c]])
                road = road.copy()
                road[:, :2] = (road[:, :2] - (dx, dy)) @ rt.T
                other = (other - (dx, dy)) @ rt.T
            roads.append(road)
            others.append(other)
        road = np.concatenate(roads)
        road[:, :2] = vehicle_to_lidar(road[:, :2], self.ego_yaw_deg)
        other = vehicle_to_lidar(np.concatenate(others), self.ego_yaw_deg)
        return road, other


def pack_cloud(header, columns: dict, datatypes: dict):
    """A PointCloud2 built straight from numpy columns -- no per-point Python tuples.

    ``columns`` maps field name -> 1-D array (all one length); ``datatypes`` maps field name ->
    a numpy dtype among float32 / uint8 / uint32. Returns (fields-as-(name, offset, numpy dtype),
    point_step, data bytes, width); the ROS wrapper turns that into a message, keeping this module
    free of sensor_msgs.
    """
    names = list(columns)
    n = len(next(iter(columns.values()))) if names else 0
    offsets, off = [], 0
    for name in names:
        dt = np.dtype(datatypes[name])
        align = dt.itemsize
        off = (off + align - 1) // align * align
        offsets.append(off)
        off += dt.itemsize
    step = (off + 3) // 4 * 4
    dtype = np.dtype({"names": names, "formats": [datatypes[k] for k in names],
                      "offsets": offsets, "itemsize": step})
    arr = np.zeros(n, dtype=dtype)
    for name in names:
        arr[name] = columns[name]
    fields = [(name, o, np.dtype(datatypes[name])) for name, o in zip(names, offsets)]
    return fields, step, arr.tobytes(), n
