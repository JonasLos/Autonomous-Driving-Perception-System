"""Per tracker id, the class to use is the recent score-weighted majority, not the latest label.

ROS-free. YOLO relabels an object for a frame or two, and the camera-LiDAR detector sizes every
box it cannot measure from the class prior, so one relabel resizes the box. Seen on
adps_2026-08-25_11-58-32: a truck passing at ~9 m was labelled `train` for its last 2 of 41
frames, and the published box went from 5.3 x 1.9 m to 200 x 3.2 m -- a 200 m bar across the
scene. On selfcal_loc_2026-09-08 (4524 detections, 220 ids) a frame's label disagreed with its
id's 2 s score-weighted majority on 2.3% of detections, and 45 of those would have changed the
prior length by more than 2 m (car <-> truck 4.5 <-> 5.3 m, truck <-> bus 5.3 <-> 12 m).

The window is in seconds of capture time, so a genuine reclassification -- a far `car` that is
really a truck -- takes over once it holds the majority of the last ``window_s``.
"""

from __future__ import annotations


class ClassVote:
    def __init__(self, window_s: float = 2.0, forget_after_s: float = 2.0):
        self.window_s = float(window_s)
        self.forget_after_s = float(forget_after_s)
        self.reset()

    def reset(self) -> None:
        self._hist: dict[str, list[tuple[float, int, str, float]]] = {}
        self._latest = None
        self.overridden = 0

    def vote(self, tracker_id: str, stamp: float, class_id: int, class_name: str,
             score: float) -> tuple[int, str]:
        """Record this label and return the (class_id, class_name) to use for it."""
        stamp = float(stamp)
        if self._latest is not None and stamp < self._latest - 1.0:
            self.reset()                                     # bag rewind
        if self._latest is None or stamp > self._latest:
            self._latest = stamp
            self._forget(stamp)
        if not tracker_id:
            return int(class_id), str(class_name)

        h = self._hist.setdefault(tracker_id, [])
        h.append((stamp, int(class_id), str(class_name), max(float(score), 1e-3)))
        while h and stamp - h[0][0] > self.window_s:
            h.pop(0)

        weight: dict[tuple[int, str], float] = {}
        for _, cid, name, s in h:
            weight[(cid, name)] = weight.get((cid, name), 0.0) + s
        current = (int(class_id), str(class_name))
        best = max(weight, key=lambda k: (weight[k], k == current))   # ties keep the current label
        if best != current:
            self.overridden += 1
        return best

    def _forget(self, now: float) -> None:
        stale = [k for k, h in self._hist.items() if not h or now - h[-1][0] > self.forget_after_s]
        for k in stale:
            del self._hist[k]
