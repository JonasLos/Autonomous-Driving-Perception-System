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

That "half a sensor period" only holds if matching is *two-sided* -- if a sample captured
after the detection is available to match against. Measured on this stack it usually is not:
a detection is processed ~17ms before the cloud it should pair with has even been buffered,
so a purely backwards-looking search degrades to a full sensor period. :class:`StampMatchedBuffer`
therefore supports ``wait_for_newer``: a reference whose ideal partner has not arrived yet is
*deferred* for a bounded wait rather than matched backwards, and resolved as soon as a newer
sample lands. See :meth:`StampMatchedBuffer.match` and :meth:`StampMatchedBuffer.drain`.

Message stamps are only ever compared to each other and never to a clock, so pairing itself is
valid under bag replay regardless of ``use_sim_time``. Deferral expiry is the one exception: it
compares two readings of whatever clock the caller passes as ``now``, so under bag replay the
caller should be running on sim time for expiry to track playback.

This module is deliberately free of ``rclpy``: :class:`StampMatchedBuffer` reads nothing but
``msg.header.stamp``, takes ``now`` as an opaque float, and reports unmatched cases as prose for
the caller to log at whatever severity and throttle it wants. The caller owns the timer that
pumps :meth:`drain`. That split keeps this importable from the four separate Python
environments the nodes run in, and keeps the deferral state machine testable without a
spinning ROS context.
"""

from __future__ import annotations

from collections import deque
from threading import Lock

import numpy as np

from perception_common.utils import is_zero_stamp, stamp_to_seconds

__all__ = [
    "MATCHED",
    "DEFERRED",
    "UNMATCHED",
    "Pairing",
    "ProjectedCloud",
    "StampMatchedBuffer",
    "apply_bounded_parameters",
]

# Contour and lane pixels are matched to LiDAR returns within this radius, in pixels.
DEFAULT_PIXEL_LIM = 5

# Outcomes of a pairing attempt. Identity-comparable (``p.outcome is MATCHED``).
MATCHED = "matched"
DEFERRED = "deferred"
UNMATCHED = "unmatched"

# Sentinel for "every stream" in drain(); None is a legitimate stream key.
_ALL = object()


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


class _Pending:
    """A reference waiting for a sample captured at or after its own stamp."""

    __slots__ = ("stamp", "payload", "key", "deferred_at")

    def __init__(self, stamp, payload, key, deferred_at):
        self.stamp = stamp
        self.payload = payload
        self.key = key
        self.deferred_at = deferred_at


class Pairing:
    """The result of pairing one reference against the buffer.

    ``outcome`` is :data:`MATCHED`, :data:`DEFERRED` or :data:`UNMATCHED`. Only a MATCHED
    pairing carries a ``value``; a DEFERRED one carries nothing yet and will come back from
    :meth:`StampMatchedBuffer.drain` with its ``payload`` intact.

    ``skew`` is signed -- positive when the matched sample was captured after the reference --
    and ``float('inf')`` when there was nothing to compare against. ``reason`` names the
    non-obvious rejections (``"out-of-order"``, ``"expired"``, ``"non-monotonic"``) for
    :meth:`StampMatchedBuffer.describe_unmatched`.
    """

    __slots__ = ("outcome", "value", "skew", "payload", "key", "reference_stamp", "reason")

    def __init__(
        self,
        outcome,
        *,
        value=None,
        skew=float("inf"),
        payload=None,
        key=None,
        reference_stamp=0.0,
        reason=None,
    ):
        self.outcome = outcome
        self.value = value
        self.skew = skew
        self.payload = payload
        self.key = key
        self.reference_stamp = reference_stamp
        self.reason = reason

    def __repr__(self):  # pragma: no cover - diagnostics only
        return (
            f"Pairing({self.outcome}, key={self.key!r}, skew={self.skew:.4f}, "
            f"reason={self.reason!r})"
        )


class StampMatchedBuffer:
    """Buffers a fast stream so a slower one can pair against it by capture time.

    ``wrap`` is applied to each buffered message and the result is what :meth:`match` and
    :meth:`newest` hand back; it defaults to :class:`ProjectedCloud`. Pass ``lambda m: m``
    to buffer plain messages.

    ``wait_for_newer`` makes matching two-sided. At its default of ``0.0`` every reference
    resolves against whatever is already buffered, which is a purely backwards-looking search.
    Set it positive and a reference with no sample at or after its own stamp is held for up to
    that long, so the sample it should pair with gets a chance to arrive. Deferred references
    come back from :meth:`drain`, which the caller pumps after every :meth:`add` and from a
    timer.

    ``key`` partitions the pending queues so independent streams cannot block each other -- a
    silent left contour must not hold back the right. Buffered samples are always shared across
    keys, which is the point: two contours from one camera frame reuse a single parse and a
    single KDTree.

    Output stamps are non-decreasing per key by construction; see :meth:`_resolve_locked`.
    That guarantee holds within a single executor thread, which is how all callers spin.
    """

    def __init__(
        self,
        name: str = "projection",
        *,
        buffer_duration: float = 2.0,
        max_skew: float = 0.08,
        stamp_offset: float = 0.0,
        wait_for_newer: float = 0.0,
        wrap=ProjectedCloud,
    ):
        self.name = name
        self.buffer_duration = buffer_duration
        self.max_skew = max_skew
        self.stamp_offset = stamp_offset
        # 0.0 means "resolve immediately": one-sided, backwards-looking matching, which is what
        # every caller did before deferral existed. Keeping it the class default makes this a
        # drop-in for existing callers and gives every node a live rollback without a restart.
        # It is not quite bug-for-bug identical -- the monotonicity guards in _resolve_locked
        # stay on at any wait_for_newer, because publishing a backwards stamp is a defect
        # rather than a behaviour worth preserving. The nonmono counter reports the difference.
        self.wait_for_newer = wait_for_newer
        self._wrap = wrap

        self._lock = Lock()
        self._entries: deque[_Buffered] = deque()

        # Per-key: references awaiting a newer sample, and the two watermarks that keep
        # emission monotonic. Keyed by whatever the caller passes as ``key`` (None included).
        self._pending: dict = {}
        self._last_emitted_reference: dict = {}
        self._last_emitted_sample: dict = {}

        self.matched_count = 0
        self.unmatched_count = 0
        self.zero_stamp_count = 0
        self.reset_count = 0
        self.deferred_count = 0
        self.expired_count = 0
        self.stale_reference_count = 0
        self.non_monotonic_reject_count = 0
        self.flushed_on_reset_count = 0
        self.out_of_order_sample_count = 0

        # Rolling window of |skew| over matched pairings. Without this there is no way to
        # measure from outside the node whether a change to the pairing bound helped.
        self._skew_window: deque[float] = deque(maxlen=512)

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
            # from the previous run forward as match candidates. Pending references and the
            # watermarks belong to the previous pass too, so they go with it -- otherwise the
            # watermarks sit in the future and reject everything from the new pass.
            if self._entries and self._entries[0].stamp > stamp:
                self._entries.clear()
                self._reset_stream_state_locked()
                self.reset_count += 1
            self._insert_sorted_locked(entry)

            # Evict against the newest buffered stamp, never the node clock, so a paused
            # or replaying bag does not silently empty the buffer.
            cutoff = stamp - self.buffer_duration
            while self._entries and self._entries[0].stamp < cutoff:
                self._entries.popleft()
        return True

    def _insert_sorted_locked(self, entry) -> None:
        """Insert keeping ``_entries`` ascending by stamp. Caller holds the lock.

        Transport can deliver two samples out of order by a few milliseconds. A blind append
        would then leave the deque unsorted, which quietly breaks three things that all read
        ``_entries[-1]`` as the newest: the left-side eviction, :meth:`newest`, and the
        "is there a sample at or after this reference" test that decides whether to defer.
        The scan is from the right and almost always stops immediately.
        """
        if not self._entries or self._entries[-1].stamp <= entry.stamp:
            self._entries.append(entry)
            return

        self.out_of_order_sample_count += 1
        buffered = []
        while self._entries and self._entries[-1].stamp > entry.stamp:
            buffered.append(self._entries.pop())
        self._entries.append(entry)
        while buffered:
            self._entries.append(buffered.pop())

    def _reset_stream_state_locked(self) -> None:
        """Drop pending references and watermarks. Caller holds the lock."""
        flushed = sum(len(q) for q in self._pending.values())
        if flushed:
            self.flushed_on_reset_count += flushed
        self._pending.clear()
        self._last_emitted_reference.clear()
        self._last_emitted_sample.clear()

    def match(self, header, *, now=0.0, payload=None, key=None) -> Pairing:
        """Pair ``header`` against the buffer, deferring if a better sample may still arrive.

        Returns a :class:`Pairing`. A :data:`DEFERRED` result carries no value yet -- it will
        be handed back by :meth:`drain` once a newer sample arrives or ``wait_for_newer``
        elapses -- so the caller should do nothing with it beyond skipping this callback.
        ``payload`` is whatever the caller needs back at that point, normally the message
        itself.

        ``now`` is an opaque clock reading, only ever compared against other readings the
        caller supplies. Under bag replay it should come from the node clock on sim time.
        """
        if is_zero_stamp(header):
            with self._lock:
                self.zero_stamp_count += 1
                self.unmatched_count += 1
            return Pairing(UNMATCHED, payload=payload, key=key)

        stamp = stamp_to_seconds(header.stamp)

        with self._lock:
            # A reference at or behind one already emitted cannot produce a monotonic output,
            # and re-emitting it would publish the same scene twice. Transport reordering and
            # a duplicated message both land here.
            last_ref = self._last_emitted_reference.get(key)
            if last_ref is not None and stamp <= last_ref:
                self.stale_reference_count += 1
                self.unmatched_count += 1
                return Pairing(
                    UNMATCHED,
                    payload=payload,
                    key=key,
                    reference_stamp=stamp,
                    reason="out-of-order",
                )

            queue = self._pending.get(key)

            # Fast path: nothing is waiting and we are not asked to wait, so this is exactly
            # the pre-deferral behaviour.
            if self.wait_for_newer <= 0.0 and not queue:
                return self._resolve_locked(stamp, payload, key)

            # Something ahead of us is still waiting. Queue behind it rather than overtaking,
            # or output stamps would go backwards.
            if queue:
                queue.append(_Pending(stamp, payload, key, now))
                self.deferred_count += 1
                return Pairing(DEFERRED, payload=payload, key=key, reference_stamp=stamp)

            # Two-sided matching is only possible once a sample at or after this reference
            # exists. If one does, resolve now -- this is the common case and costs nothing.
            if self._has_sample_at_or_after_locked(stamp):
                return self._resolve_locked(stamp, payload, key)

            self._pending[key] = deque([_Pending(stamp, payload, key, now)])
            self.deferred_count += 1
            return Pairing(DEFERRED, payload=payload, key=key, reference_stamp=stamp)

    def drain(self, now=0.0, key=_ALL) -> list:
        """Resolve whatever deferred references can be resolved, oldest first.

        Call this after every :meth:`add` -- that is what resolves nearly all deferrals, in
        the same tick the awaited sample arrives -- and from a timer, which is what bounds the
        wait when the fast stream stalls. Returns a list of :class:`Pairing`, already ordered
        by reference stamp within each key, for the caller to act on outside the lock.
        """
        out: list = []
        with self._lock:
            keys = list(self._pending.keys()) if key is _ALL else [key]
            for k in keys:
                queue = self._pending.get(k)
                if not queue:
                    continue
                while queue:
                    pairing = self._try_resolve_head_locked(queue, now)
                    if pairing is None:
                        # Head-of-line: a later reference must not overtake this one.
                        break
                    out.append(pairing)
                if not queue:
                    self._pending.pop(k, None)
        return out

    def _try_resolve_head_locked(self, queue, now):
        """Resolve the head of ``queue`` if it is ready, else ``None``. Caller holds the lock."""
        head = queue[0]

        ready = self._has_sample_at_or_after_locked(head.stamp)
        expired = False
        if not ready:
            waited = now - head.deferred_at
            # A backwards clock step (sim time rewinding on a bag loop) must not park a
            # reference forever, so treat it as expiry rather than as "no time has passed".
            if waited >= self.wait_for_newer or waited < 0.0:
                ready = True
                expired = True

        if not ready:
            return None

        queue.popleft()
        if expired:
            self.expired_count += 1
        pairing = self._resolve_locked(head.stamp, head.payload, head.key)
        if expired and pairing.outcome is UNMATCHED and pairing.reason is None:
            pairing.reason = "expired"
        return pairing

    def _has_sample_at_or_after_locked(self, stamp) -> bool:
        """True when the buffer holds a sample captured at or after ``stamp``."""
        # The deque is append-ordered by stamp, so the newest is the only one worth checking.
        return bool(self._entries) and self._entries[-1].stamp >= stamp

    def _resolve_locked(self, stamp, payload, key):
        """Nearest-sample search plus bookkeeping. Caller holds the lock.

        The candidate set is restricted to samples at or after the last one already emitted on
        this key. Reference monotonicity alone does not give monotonic output: for references
        ``t1 < t2`` matching samples ``s1, s2``, ``s2 < s1`` is reachable whenever
        ``t2 - t1 < 2 * max_skew``, which at 10Hz with an 0.06 bound is the normal operating
        point. Equal is allowed -- two references legitimately sharing one sample -- so this
        makes output stamps non-decreasing rather than strictly increasing.

        The restriction normally costs nothing: it drops the frame only if it also empties the
        candidate set, and otherwise degrades to the nearest non-decreasing sample, which is
        still usually inside ``max_skew``. So the guard buys monotonic output without the
        dropped frames that rejecting outright would cause.
        """
        floor = self._last_emitted_sample.get(key)
        candidates = self._entries
        if floor is not None:
            candidates = [e for e in self._entries if e.stamp >= floor]

        if not candidates:
            self.unmatched_count += 1
            self._last_emitted_reference[key] = stamp
            reason = "non-monotonic" if floor is not None and self._entries else None
            if reason:
                self.non_monotonic_reject_count += 1
            return Pairing(
                UNMATCHED, payload=payload, key=key, reference_stamp=stamp, reason=reason
            )

        best = min(candidates, key=lambda entry: abs(entry.stamp - stamp))
        skew = best.stamp - stamp

        # The reference is retired either way: a rejected one must not be reconsidered later
        # under a different buffer state, or the same scene could publish twice.
        self._last_emitted_reference[key] = stamp

        if abs(skew) <= self.max_skew:
            self.matched_count += 1
            self._skew_window.append(abs(skew))
            self._last_emitted_sample[key] = best.stamp
            return Pairing(
                MATCHED,
                value=best.value,
                skew=skew,
                payload=payload,
                key=key,
                reference_stamp=stamp,
            )

        self.unmatched_count += 1
        return Pairing(
            UNMATCHED, skew=skew, payload=payload, key=key, reference_stamp=stamp
        )

    def newest(self):
        """The most recently buffered value, or ``None`` when the buffer is empty."""
        with self._lock:
            return self._entries[-1].value if self._entries else None

    def pending_count(self, key=_ALL) -> int:
        """How many references are currently deferred."""
        with self._lock:
            if key is _ALL:
                return sum(len(q) for q in self._pending.values())
            return len(self._pending.get(key, ()))

    def clear(self) -> None:
        with self._lock:
            self._entries.clear()
            self._reset_stream_state_locked()

    def describe_unmatched(
        self, skew: float, reference: str = "detection", *, reason=None
    ) -> str:
        """Explain an unmatched pairing, naming which bound was missed and in which direction."""
        if reason == "out-of-order":
            return f"{reference} arrived out of order, at or behind one already published"
        if reason == "non-monotonic":
            return (
                f"nearest {self.name} predates the one already published for the previous "
                f"{reference}; pairing it would walk output stamps backwards"
            )
        if reason == "expired":
            return (
                f"no {self.name} captured at or after the {reference} within "
                f"wait_for_newer={self.wait_for_newer:.3f}s"
            )
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
        """One-line health summary, including the skew figures a tuning pass is judged on."""
        with self._lock:
            window = list(self._skew_window)
            counters = (
                self.matched_count,
                self.unmatched_count,
                self.deferred_count,
                self.expired_count,
                self.stale_reference_count,
                self.non_monotonic_reject_count,
                self.reset_count,
            )
        matched, unmatched, deferred, expired, stale, nonmono, resets = counters

        if window:
            skew = f"mean|skew|={np.mean(window):.4f}s max|skew|={np.max(window):.4f}s"
        else:
            skew = "mean|skew|=n/a max|skew|=n/a"

        return (
            f"max_skew={self.max_skew:.3f}s wait_for_newer={self.wait_for_newer:.3f}s "
            f"matched={matched} unmatched={unmatched} deferred={deferred} "
            f"expired={expired} stale={stale} nonmono={nonmono} resets={resets} {skew}"
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
