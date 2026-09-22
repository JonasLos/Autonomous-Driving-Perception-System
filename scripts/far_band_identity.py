#!/usr/bin/env python3
"""Far-band orphans with the TRACKS named: was the track never there, or did it disappear?

`live_orphans.py` says a >80 m measurement has no track on its bearing; it cannot say why. This
reads `/perception/objects` alongside the measurement markers, so every orphan can be put in one
of three classes:

  NEVER      no track was ever on that bearing in the 3 s before -- the detection never birthed
             a track, or an existing track claimed the detection and stayed where it was
  DIED       a track WAS on the bearing and is gone now. If another track sat within
             `merge_max_dist` of it in its last frame, merging is the obvious suspect and the
             report says so
  DISPLACED  a track on the bearing is still published but has moved off it

and separately reports FLICKER: an orphan that DOES have a track on its bearing in some published
frame within +-0.25 s, just not in the one frame the test matched it to. Those are a property of
the publish cadence, not of coverage -- the aggregator publishes on every input, so a frame
triggered by another sensor can land between the measurement and the track being born from it.
Read the orphan rate net of FLICKER before comparing two arms that publish at different rates.

WHAT THIS FOUND (2026-09-23): there was no far-band coverage loss to explain. Every candidate
mechanism was refuted -- merging (an arm with `MERGE_MAX_DIST=inf` left the far band where it was),
and separately the track populations are identical with the cluster path and without (117 vs 105
tracks born past 80 m, median life 0.25 vs 0.26 s). The whole effect was this audit and
`live_orphans.py` matching frames wrongly: see rule 4 in HANDOFF.md. The FLICKER line below is
what exposed it -- an "orphan" whose track sits in a frame milliseconds away is a matching
failure, not a coverage failure, and it should be near zero.

    python3 scripts/far_band_identity.py RECORDING_DIR/*.mcap
"""

import bisect
import math
import sys
from collections import defaultdict

import numpy as np
from mcap_ros2.reader import read_ros2_messages

sys.path.insert(0, "scripts")
from live_orphans import EGO_YAW_DEG, FAR_ALONG_FRAC, RANGE_TRUST_MAX_M, centres

MEAS = "/perception/measurements/camera_lidar_markers"
OBJECTS = "/perception/objects"
LOOKBACK_S = 3.0
MERGE_DIST = 2.5
FLICKER_S = 0.10


def _stamp(msg, fallback_ns):
    """Capture time. The aggregator stamps a published frame with the capture time of the
    measurement it just released, so a measurement and the frame carrying it share a stamp --
    while their RECORDING times differ by the pipeline's wall-clock latency, which is not a
    constant and is not the same in two arms."""
    h = getattr(msg, "header", None)
    if h is None:
        h = msg.markers[0].header if getattr(msg, "markers", None) else None
    if h is None:
        return fallback_ns * 1e-9
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def load(path):
    """(measurement centres in ego, published objects) both as time-sorted lists."""
    meas, objs = [], []
    k = math.radians(EGO_YAW_DEG)
    co, si = math.cos(k), math.sin(k)
    for m in read_ros2_messages(path, topics=[MEAS, OBJECTS]):
        t = _stamp(m.ros_msg, m.log_time_ns)
        if m.channel.topic == MEAS:
            c = [(x * co - y * si, x * si + y * co) for (x, y) in centres(m.ros_msg)]
            meas.append((t, c))
        else:
            objs.append((t, [(int(o.track_id), o.pose.position.x, o.pose.position.y,
                              float(o.age), int(o.track_status), int(o.contributions_this_frame))
                             for o in m.ros_msg.objects]))
    # Stable, by stamp only: frames sharing a stamp must keep their publication
    # order, because the last one is the only one that has seen everything
    # released at that instant (see live_orphans.py).
    meas.sort(key=lambda r: r[0]); objs.sort(key=lambda r: r[0])
    return meas, objs


def on_bearing(x, y, qx, qy, r):
    ux, uy = x / r, y / r
    a = (qx - x) * ux + (qy - y) * uy
    c = -(qx - x) * uy + (qy - y) * ux
    return abs(c) < 3.0 and abs(a) < FAR_ALONG_FRAC * r


def main(path):
    global FLICKER_S
    meas, objs = load(path)
    obj_t = [t for t, _ in objs]
    counts = defaultdict(int)
    merge_suspect = 0
    ages = []
    n = 0
    for t, cs in meas:
        i = bisect.bisect_left(obj_t, t)
        cand = [j for j in (i - 1, i) if 0 <= j < len(objs)]
        if not cand:
            continue
        j = min(cand, key=lambda k: abs(obj_t[k] - t))
        if abs(obj_t[j] - t) > 0.05:
            continue
        # The node publishes one frame per measurement released in a tick, all with the same
        # release stamp, so the first frame at a stamp need not carry this measurement's update.
        while j + 1 < len(objs) and obj_t[j + 1] == obj_t[j]:
            j += 1
        here = objs[j][1]
        for (x, y) in cs:
            r = math.hypot(x, y)
            if r < RANGE_TRUST_MAX_M:
                continue
            n += 1
            near = min((math.hypot(x - q[1], y - q[2]) for q in here), default=float("inf"))
            if near < 3.0 or any(on_bearing(x, y, q[1], q[2], r) for q in here):
                counts["covered"] += 1
                continue

            # Orphaned. First: is this the matched FRAME's fault? The aggregator publishes on
            # every input, so a frame from another sensor can fall between the measurement and
            # the track born from it.
            flicker = False
            for jj in range(len(objs)):
                if abs(obj_t[jj] - t) > FLICKER_S:
                    continue
                if any(on_bearing(x, y, q[1], q[2], r) for q in objs[jj][1]):
                    flicker = True
                    break
            counts["flicker"] += flicker

            # Walk back and see whether a track was ever on this bearing.
            was, last_frame = None, None
            for jj in range(j - 1, -1, -1):
                if obj_t[j] - obj_t[jj] > LOOKBACK_S:
                    break
                hit = [q for q in objs[jj][1] if on_bearing(x, y, q[1], q[2], r)]
                if hit:
                    was, last_frame = hit[0], jj
                    break
            if was is None:
                counts["NEVER"] += 1
                continue
            if any(q[0] == was[0] for q in here):
                counts["DISPLACED"] += 1
                continue
            counts["DIED"] += 1
            ages.append(was[3])
            # Was another track close enough to merge with it in its last frame?
            if any(q[0] != was[0] and math.hypot(q[1] - was[1], q[2] - was[2]) < MERGE_DIST
                   for q in objs[last_frame][1]):
                merge_suspect += 1

    orph = counts["NEVER"] + counts["DIED"] + counts["DISPLACED"]
    print(f"{path.split('/')[-2]:22s} far n={n:4d}  orphans {orph:3d} ({100*orph/max(n,1):4.1f}%)  "
          f"NEVER {counts['NEVER']:3d} | DIED {counts['DIED']:3d} "
          f"(a neighbour within {MERGE_DIST} m in its last frame: {merge_suspect:3d}"
          + (f", age median {np.median(ages):4.1f} s" if ages else "") + ")"
          f" | DISPLACED {counts['DISPLACED']:3d}")
    f = counts["flicker"]
    print(f"{'':22s}         of those, FLICKER {f:3d}: a track IS on the bearing within "
          f"+-{FLICKER_S:.2f} s, just not in the matched frame -> "
          f"{100*(orph-f)/max(n,1):4.1f}% net of the cadence")


if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if not a.startswith("--flicker-s=")]
    for a in sys.argv[1:]:
        if a.startswith("--flicker-s="):
            FLICKER_S = float(a.split("=", 1)[1])          # noqa: F841 - rebinds the module global
    for p in args:
        main(p)
