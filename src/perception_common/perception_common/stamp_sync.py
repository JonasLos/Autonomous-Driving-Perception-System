"""Capture-time pairing of a slow detection stream against a fast sensor stream.

A detection describes an image captured some time before the message arrives — camera
transport plus inference latency. Pairing it with whatever projection is current at
*arrival* time therefore lifts stale pixels onto fresh geometry, and the result lands
wherever the vehicle was when the camera saw the scene rather than where the object is.
On this stack that was measured at ~1.1s of skew, roughly 5m of travel at 5m/s.

The fix is to buffer the fast stream and fuse each detection against the sample captured
closest to its own header stamp. Skew then drops to at most half a sensor period,
independent of how far behind the detection pipeline runs. The cost is that output appears
one pipeline latency after the scene it describes; carrying the matched sample's stamp on
the output lets consumers account for that.

Stamps are only ever compared to each other and never to the node clock, which keeps this
valid under bag replay without any ``use_sim_time`` configuration.

This module is deliberately free of ``rclpy``: :class:`StampMatchedBuffer` reads nothing but
``msg.header.stamp``, and reports unmatched cases as prose for the caller to log at whatever
severity and throttle it wants.
"""

from __future__ import annotations

from collections import deque
from threading import Lock

import numpy as np

from perception_common.utils import is_zero_stamp, stamp_to_seconds

__all__ = [
    "ProjectedCloud",
    "StampMatchedBuffer",
    "apply_bounded_parameters",
]

# Contour and lane pixels are matched to LiDAR returns within this radius, in pixels.
DEFAULT_PIXEL_LIM = 5


class ProjectedCloud:
    """A ``/lidar_2d_projection`` PointCloud2 parsed on first use.

    Consumers that share a cloud share the cost: the left and right contours of one frame,
    every detection in one array, and several camera frames falling inside one LiDAR period
    all reuse a single parse and a single KDTree. Buffered clouds that nothing ever matches
    cost only the memory to hold the message.
    """

    __slots__ = ("msg", "_parsed", "_xyz", "_u", "_v", "_tree", "_tree_built")

    def __init__(self, msg):
        self.msg = msg
        self._parsed = False
        self._xyz = None
        self._u = None
        self._v = None
        self._tree = None
        self._tree_built = False

    @property
    def header(self):
        return self.msg.header

    def arrays(self):
        """Return ``(xyz Nx3, u N, v N)``. All three are empty for an unusable cloud."""
        if not self._parsed:
            self._parsed = True
            self._xyz, self._u, self._v = self._parse()
        return self._xyz, self._u, self._v

    def _parse(self):
        empty = (
            np.empty((0, 3), dtype=np.float32),
            np.empty(0, dtype=np.float32),
            np.empty(0, dtype=np.float32),
        )
        # sensor_msgs_py rather than ros2_numpy: the latter is not on PyPI and has to be
        # hand-cloned into the SAM3 venv at container start, while this ships with Jazzy.
        from sensor_msgs_py import point_cloud2

        try:
            pts = point_cloud2.read_points_numpy(
                self.msg, field_names=["x", "y", "z", "u", "v"], skip_nans=True
            )
        except (AssertionError, KeyError, ValueError):
            # A cloud without u/v is not a projection; fail soft so the caller's
            # "no points" branch handles it like any other empty frame.
            return empty

        if pts.size == 0:
            return empty

        pts = np.atleast_2d(pts)
        if pts.shape[1] < 5:
            return empty

        # One parse for both the geometry and the pixel coordinates, so the two can never
        # disagree about which rows survived NaN filtering and index each other wrongly.
        return (
            np.ascontiguousarray(pts[:, :3], dtype=np.float32),
            np.ascontiguousarray(pts[:, 3], dtype=np.float32),
            np.ascontiguousarray(pts[:, 4], dtype=np.float32),
        )

    def pixel_tree(self):
        """Return ``(kdtree_over_uv, xyz_points)``, or ``(None, None)`` if empty."""
        if not self._tree_built:
            self._tree_built = True
            xyz, u, v = self.arrays()
            if xyz.shape[0]:
                # Imported here so consumers that only need arrays() -- fusion_node masks
                # u/v directly and never queries a tree -- do not pull in scipy.
                from scipy.spatial import KDTree

                self._tree = KDTree(np.column_stack((u, v)))
        return (self._tree, self._xyz) if self._tree is not None else (None, None)


class _Buffered:
    __slots__ = ("stamp", "value")

    def __init__(self, stamp, value):
        self.stamp = stamp
        self.value = value


class StampMatchedBuffer:
    """Buffers a fast stream so a slower one can pair against it by capture time.

    ``wrap`` is applied to each buffered message and the result is what :meth:`match` and
    :meth:`newest` hand back; it defaults to :class:`ProjectedCloud`. Pass ``lambda m: m``
    to buffer plain messages.
    """

    def __init__(
        self,
        name: str = "projection",
        *,
        buffer_duration: float = 2.0,
        max_skew: float = 0.08,
        stamp_offset: float = 0.0,
        wrap=ProjectedCloud,
    ):
        self.name = name
        self.buffer_duration = buffer_duration
        self.max_skew = max_skew
        self.stamp_offset = stamp_offset
        self._wrap = wrap

        self._lock = Lock()
        self._entries: deque[_Buffered] = deque()

        self.matched_count = 0
        self.unmatched_count = 0
        self.zero_stamp_count = 0
        self.reset_count = 0

    def add(self, msg) -> bool:
        """Buffer ``msg`` keyed on its header stamp. False when it carries a zero stamp."""
        if is_zero_stamp(msg.header):
            with self._lock:
                self.zero_stamp_count += 1
            return False

        stamp = stamp_to_seconds(msg.header.stamp) + self.stamp_offset
        entry = _Buffered(stamp, self._wrap(msg))

        with self._lock:
            # Bag replay can jump backwards; drop the buffer rather than carrying samples
            # from the previous run forward as match candidates.
            if self._entries and self._entries[0].stamp > stamp:
                self._entries.clear()
                self.reset_count += 1
            self._entries.append(entry)

            # Evict against the newest buffered stamp, never the node clock, so a paused
            # or replaying bag does not silently empty the buffer.
            cutoff = stamp - self.buffer_duration
            while self._entries and self._entries[0].stamp < cutoff:
                self._entries.popleft()
        return True

    def match(self, header):
        """Return ``(value, signed_skew)`` for the buffered sample nearest ``header``.

        ``skew`` is positive when the matched sample was captured after the reference, and
        ``float('inf')`` when there is nothing to match against or either stamp is zero
        (which would otherwise compare as a decades-large difference and read as permanent
        staleness). ``value`` is ``None`` when nothing falls within ``max_skew``.
        """
        if is_zero_stamp(header):
            with self._lock:
                self.zero_stamp_count += 1
                self.unmatched_count += 1
            return None, float("inf")

        stamp = stamp_to_seconds(header.stamp)

        with self._lock:
            candidates = list(self._entries)
        if not candidates:
            with self._lock:
                self.unmatched_count += 1
            return None, float("inf")

        best = min(candidates, key=lambda entry: abs(entry.stamp - stamp))
        skew = best.stamp - stamp

        with self._lock:
            if abs(skew) <= self.max_skew:
                self.matched_count += 1
                return best.value, skew
            self.unmatched_count += 1
        return None, skew

    def newest(self):
        """The most recently buffered value, or ``None`` when the buffer is empty."""
        with self._lock:
            return self._entries[-1].value if self._entries else None

    def clear(self) -> None:
        with self._lock:
            self._entries.clear()

    def describe_unmatched(self, skew: float, reference: str = "detection") -> str:
        """Explain an unmatched pairing, naming which bound was missed and in which direction."""
        if skew == float("inf"):
            return f"no {self.name} buffered (or zero-stamped header)"
        if skew > 0:
            # Everything still buffered was captured after this reference: the sample it
            # should have paired with has already been evicted.
            return (
                f"nearest {self.name} is {skew:.3f}s newer than the {reference} — "
                f"{reference} latency exceeds buffer_duration={self.buffer_duration:.3f}s"
            )
        # No sample as recent as the reference has arrived, so the fast stream is stalled
        # or running behind the slow one.
        return (
            f"nearest {self.name} is {-skew:.3f}s older than the {reference} — "
            f"no {self.name} was captured near it"
        )

    def status(self) -> str:
        return (
            f"max_skew={self.max_skew:.3f}s matched={self.matched_count} "
            f"unmatched={self.unmatched_count}"
        )


def apply_bounded_parameters(params, targets, *, minimum=0.0):
    """Validate every parameter before applying any, then apply.

    ``targets`` maps a parameter name to an ``(object, attribute)`` pair, so a node can
    point some names at itself and others at its buffer. Returns
    ``(ok, reason, [(name, value), ...])``; the caller builds its own ``SetParametersResult``
    and logs the applied values, which keeps ``rcl_interfaces`` out of this module.

    Validating everything first means a rejected request cannot leave a node half-updated.
    """
    for p in params:
        if p.name in targets and float(p.value) < minimum:
            return False, f"{p.name} must be >= {minimum}", []

    applied = []
    for p in params:
        if p.name in targets:
            obj, attr = targets[p.name]
            value = float(p.value)
            setattr(obj, attr, value)
            applied.append((p.name, value))
    return True, "", applied
