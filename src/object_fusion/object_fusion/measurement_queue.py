"""Fixed-lag, capture-stamp-ordered release of measurements from several sensors.

``perception_common.stamp_sync.StampMatchedBuffer`` is a PAIRWISE matcher: one slow reference
against one fast stream. It is the right tool for ``fusion_node`` and for
``radar_fusion_node`` and it is used unchanged by both. A three-sensor tracker needs something
different -- every measurement must be applied in CAPTURE order regardless of which sensor it
came from or when it arrived, because applying a 10 Hz camera update after a 30 Hz radar
update that describes an earlier instant corrupts the state.

So measurements are buffered and released once ``now - stamp > lag``, in stamp order. The
tracker then predicts to each measurement's own stamp before updating it.

This keeps the repo's invariant -- COMPARE CAPTURE STAMPS, NEVER ARRIVAL STAMPS -- while
handling N streams. The cost is stated plainly rather than hidden: output is delayed by
``lag``. Do NOT "fix" that by predicting to wall-clock-now before publishing; that breaks the
contract that an output carries the stamp of the data it describes, and hides the latency from
every consumer. Publish at the capture stamp with the covariance and let consumers extrapolate.

``lag`` defaults to 0.12 s: the camera path is the slowest contributor and ``fusion_node``
already budgets 0.06 s of pairing skew on top of its inference latency.
"""

from __future__ import annotations

import heapq
import itertools

__all__ = ["Measurement", "MeasurementQueue", "DEFAULT_LAG_S"]

DEFAULT_LAG_S = 0.12


class Measurement:
    """One sensor observation, tagged with its capture time and origin."""

    __slots__ = ("stamp", "sensor", "payload")

    def __init__(self, stamp: float, sensor: str, payload):
        self.stamp = float(stamp)
        self.sensor = str(sensor)
        self.payload = payload

    def __repr__(self):  # pragma: no cover - diagnostics only
        return f"Measurement(t={self.stamp:.4f}, {self.sensor})"


class MeasurementQueue:
    """Stamp-ordered queue with a fixed release lag.

    Counters are part of the interface, not debug noise: ``late`` is how often a measurement
    arrived after its own release deadline (it is still released, immediately, but out of
    order), and a rising ``late`` is the signal that ``lag`` is too short for the pipeline it
    is bounding.
    """

    def __init__(self, lag: float = DEFAULT_LAG_S, max_depth: int = 4096):
        self.lag = float(lag)
        self.max_depth = int(max_depth)
        self._heap: list = []
        self._seq = itertools.count()
        self._last_released = None
        #: Released out of capture order because they arrived too late to be sequenced.
        self.late = 0
        #: Dropped because the queue was over ``max_depth`` -- a stalled consumer, not a
        #: timing problem.
        self.dropped = 0

    def __len__(self):
        return len(self._heap)

    def add(self, m: Measurement) -> None:
        if self._last_released is not None and m.stamp < self._last_released:
            self.late += 1
        # Sequence counter breaks stamp ties deterministically, so two measurements sharing a
        # stamp always release in arrival order rather than by payload comparison (which would
        # raise, payloads being arbitrary objects).
        heapq.heappush(self._heap, (m.stamp, next(self._seq), m))
        while len(self._heap) > self.max_depth:
            heapq.heappop(self._heap)
            self.dropped += 1

    def release(self, now: float):
        """Yield every measurement whose stamp is older than ``now - lag``, in stamp order."""
        cutoff = float(now) - self.lag
        out = []
        while self._heap and self._heap[0][0] <= cutoff:
            _, _, m = heapq.heappop(self._heap)
            self._last_released = m.stamp
            out.append(m)
        return out

    def drain(self):
        """Release everything regardless of lag. For shutdown and for offline replay."""
        out = []
        while self._heap:
            _, _, m = heapq.heappop(self._heap)
            self._last_released = m.stamp
            out.append(m)
        return out

    def reset(self) -> None:
        """Clear state after a backwards time jump.

        ``StampMatchedBuffer`` already resets its buffer and watermarks on a backwards jump and
        reports it through ``resets``. The track store must reset with it, or a looping bag
        resurrects tracks from the previous pass at positions from a different stretch of road.
        """
        self._heap.clear()
        self._last_released = None
