"""Reject a single camera-LiDAR measurement whose depth jumps away from its own track's motion.

ROS-free, so ``scripts/neighbour_ab.py`` scores this class rather than a copy of it.

WHY. On ``selfcal_loc_2026-09-08`` the adopted point selection still published 8.2% of
detections more than 2 m (in range) away from the same tracker id's neighbouring frames, and the
anatomy of those jumps rules out the obvious fixes:

* 82% are ALONG the ray (median 5.8 m along, 0.38 m across). A neighbour BESIDE the target would
  move the box sideways; these move it in depth, with the 2D box steady in 81% of them.
* They are one-frame spikes, about half nearer and half farther, and the frames either side agree
  with radar while the spike does not (nearer spikes sit 5.7 m short of radar, their neighbours
  0.3 m).
* 53% of detections on that drive are traffic cones, and cones carry them: a cone box holds ~3
  non-ground returns, so when the cone is missed for a frame the nearest thing left in the box is
  a road ring or the next cone in the line, several metres off in depth.
* No per-frame selection rule fixes it: centre-crop, 3D clustering (centre / largest / nearest the
  previous output) and a depth cluster chosen by prediction all left 11-13% big jumps.

So the measurement is refused instead of repaired. Scored on the same 4519 detections, whole-drive
spike rate (|range - neighbours| > 2 m) and range error spread against radar:

    rule                                     published  spikes   radar sd 0-25 / 25-40 / 40-60 / 60-80
    adopted selection                             4519    8.2%   1.77 / 0.69 / 1.58 / 2.76
    + drop boxes left empty by segmentation       4350    5.2%   1.77 / 0.68 / 1.42 / 1.91
    + this gate                                   4236    1.5%   1.66 / 0.68 / 1.45 / 1.75

Cones 12.9% -> 1.6%, vehicles 3.5% -> 1.5%. The gate width is not sensitive: every combination of
gate_min 0.75-2.5 m and range fraction 0.02-0.08 landed at 1.5-2.0% spikes with the same radar
spread, so the defaults sit on a plateau, not on a tuned optimum.

WHAT IT DOES NOT DO. It never moves a measurement, it only withholds one, and never two in a row:
a second consecutive disagreement is taken as the prediction being wrong (a ByteTrack id handed
to another object, a hard manoeuvre) and the history restarts from the new measurement. A track
with fewer than two recent measurements is never gated.
"""

from __future__ import annotations

#: Measurements further apart than this (seconds) are not extrapolated between. 10 Hz camera, so
#: this allows exactly one missing frame.
MAX_EXTRAPOLATION_S = 0.3


class DepthJumpGate:
    """Per tracker id, constant-velocity prediction of the forward coordinate; drop one outlier.

    ``x`` is forward depth in the LiDAR frame -- the axis the nearest-depth-cluster rule splits
    on, and the axis the jumps are in.
    """

    def __init__(self, gate_min_m: float = 1.5, gate_range_frac: float = 0.05,
                 forget_after_s: float = 2.0):
        self.gate_min_m = float(gate_min_m)
        self.gate_range_frac = float(gate_range_frac)
        self.forget_after_s = float(forget_after_s)
        self.reset()

    def reset(self) -> None:
        self._hist: dict[str, list[tuple[float, float, float]]] = {}
        self._miss: dict[str, int] = {}
        self._latest = None
        self.admitted = 0
        self.dropped = 0

    def predict(self, tracker_id: str, stamp: float):
        """Predicted (x, y) at ``stamp``, or None without two measurements close enough in time."""
        h = self._hist.get(tracker_id)
        if not h or len(h) < 2:
            return None
        (t0, x0, y0), (t1, x1, y1) = h
        if not (0.0 < t1 - t0 < MAX_EXTRAPOLATION_S and 0.0 < stamp - t1 < MAX_EXTRAPOLATION_S):
            return None
        k = (stamp - t1) / (t1 - t0)
        return x1 + k * (x1 - x0), y1 + k * (y1 - y0)

    def admit(self, tracker_id: str, stamp: float, x: float, y: float) -> bool:
        """True to publish this measurement; False to withhold it. Updates the history."""
        stamp = float(stamp)
        if self._latest is not None and stamp < self._latest - 1.0:
            self.reset()                                     # bag rewind
        if self._latest is None or stamp > self._latest:
            self._latest = stamp
            self._forget(stamp)
        if not tracker_id:
            self.admitted += 1
            return True

        pred = self.predict(tracker_id, stamp)
        if pred is not None:
            gate = max(self.gate_min_m, self.gate_range_frac * abs(pred[0]))
            if abs(float(x) - pred[0]) <= gate:
                self._miss[tracker_id] = 0
            elif self._miss.get(tracker_id, 0) == 0:
                self._miss[tracker_id] = 1
                self.dropped += 1
                return False
            else:
                self._miss[tracker_id] = 0
                self._hist[tracker_id] = []

        h = self._hist.setdefault(tracker_id, [])
        h.append((stamp, float(x), float(y)))
        del h[:-2]
        self.admitted += 1
        return True

    def _forget(self, now: float) -> None:
        stale = [k for k, h in self._hist.items() if h and now - h[-1][0] > self.forget_after_s]
        for k in stale:
            del self._hist[k]
            self._miss.pop(k, None)
