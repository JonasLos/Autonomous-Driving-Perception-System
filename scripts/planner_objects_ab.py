#!/usr/bin/env python3
"""Planner obstacle input, A/B: legacy tracker.py vs the fusion_object_bridge.

Both publish ava_local_planner/ObjectList (id + UTM-grid position history) from the same replay:

    legacy   /fused_bbox          -> tracker.py            -> /legacy/tracked_objects
    bridge   /perception/objects  -> fusion_object_bridge -> /planner/tracked_objects

Record them together with /novatel/oem7/odom_grid for >= 420 s (the bag is 398 s), then:

    python3 scripts/planner_objects_ab.py ~/fusion_data/recordings/planner_ab

What it scores, and why each is the right question for a planner:

1. LATENCY ERROR. A static object's along-track position error should not depend on how fast
   the vehicle is going. Placing a detection with the NEWEST odometry instead of the pose at
   capture time pushes it forward by v * latency, so the slope of along-track residual against
   ego speed IS the effective latency in seconds. The bridge should read ~0.

2. SPEED AS THE PLANNER SEES IT. Each object's history is run through the planner's own
   frenet_optimal_trajectory.kalman_predict, exactly as planner_main does. The FSM calls anything
   at or under 3 m/s a static obstacle to avoid; a parked car or cone read above that is not
   avoided, so the share of objects over 3 m/s on this mostly-static drive is the safety number.

3. POSITION STABILITY of each track over its life, and object/id bookkeeping.

4. THE LIDAR OFFSET. A wrong lever arm shifts every object by the same vector in the VEHICLE
   frame, which flips sign in the world when the heading flips. Static objects seen on opposite
   legs of the drive therefore disagree by twice the error.
"""

import argparse
import bisect
import glob
import math
import os
import sys
from collections import defaultdict

import numpy as np
from mcap_ros2.reader import read_ros2_messages

TOPICS = {"legacy": "/legacy/tracked_objects", "bridge": "/planner/tracked_objects"}
ODOM = "/novatel/oem7/odom_grid"
STATIC_SPEED = 3.0          # planner FSM: <= 3 m/s is an obstacle to avoid


def stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def load(bag):
    files = sorted(glob.glob(os.path.join(bag, "*.mcap"))) if os.path.isdir(bag) else [bag]
    odom, streams = [], {k: [] for k in TOPICS}
    by_topic = {v: k for k, v in TOPICS.items()}
    for f in files:
        for m in read_ros2_messages(f, topics=[ODOM, *TOPICS.values()]):
            msg = m.ros_msg
            if m.channel.topic == ODOM:
                q = msg.pose.pose.orientation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                v = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
                odom.append((stamp(msg.header), yaw, v))
            else:
                objs = [(o.id, list(o.x), list(o.y)) for o in msg.objects if len(o.x)]
                streams[by_topic[m.channel.topic]].append((m.log_time_ns * 1e-9, objs))
    odom.sort()
    for k in streams:
        streams[k].sort(key=lambda r: r[0])
    return odom, streams


class OdomLookup:
    def __init__(self, odom):
        self.t = [o[0] for o in odom]
        self.o = odom

    def at(self, t):
        i = bisect.bisect_left(self.t, t)
        cand = [j for j in (i - 1, i) if 0 <= j < len(self.t)]
        if not cand:
            return None
        j = min(cand, key=lambda k: abs(self.t[k] - t))
        return self.o[j] if abs(self.t[j] - t) < 0.2 else None


def segments(stream):
    """Split at bag rewinds; ids restart meaning after one."""
    seg, out = 0, []
    last = None
    for t, objs in stream:
        if last is not None and t < last - 1.0:
            seg += 1
        last = t
        out.append((seg, t, objs))
    return out


def score(name, stream, look, kalman_predict, predict_every):
    rows = segments(stream)
    tracks = defaultdict(list)            # (seg, id) -> [(t, x, y, yaw, v)]
    counts = []
    for seg, t, objs in rows:
        counts.append(len(objs))
        o = look.at(t)
        if o is None:
            continue
        for oid, xs, ys in objs:
            tracks[(seg, oid)].append((t, xs[-1], ys[-1], o[1], o[2]))

    # 1 + 3: residuals about each track's own median, split along / across the heading
    res_along, res_cross, speeds, per_track_p90, lifetimes = [], [], [], [], []
    for key, pts in tracks.items():
        if len(pts) < 10:
            continue
        a = np.array(pts)
        lifetimes.append(a[-1, 0] - a[0, 0])
        med = np.median(a[:, 1:3], axis=0)
        d = a[:, 1:3] - med
        per_track_p90.append(np.percentile(np.hypot(d[:, 0], d[:, 1]), 90))
        c, s = np.cos(a[:, 3]), np.sin(a[:, 3])
        along = d[:, 0] * c + d[:, 1] * s
        cross = -d[:, 0] * s + d[:, 1] * c
        v = a[:, 4]
        if np.ptp(v) > 1.0:               # the slope needs speed to vary inside the track
            res_along.append(along - along.mean())
            res_cross.append(cross - cross.mean())
            speeds.append(v - v.mean())

    print(f"\n=== {name} ({TOPICS[name]}) ===")
    print(f"  messages {len(rows)}   objects/message mean {np.mean(counts):.2f}   "
          f"tracks with >= 10 samples {len(per_track_p90)}   "
          f"median lifetime {np.median(lifetimes) if lifetimes else float('nan'):.1f} s")
    if per_track_p90:
        p = np.array(per_track_p90)
        print(f"  position spread over a track's life (p90 distance from its median): "
              f"median {np.median(p):.2f} m, tracks over 1 m {100 * np.mean(p > 1.0):.1f}%")
    if speeds:
        ra, rc, sv = np.concatenate(res_along), np.concatenate(res_cross), np.concatenate(speeds)
        k_along = float(sv @ ra / (sv @ sv))
        k_cross = float(sv @ rc / (sv @ sv))
        resid = ra - k_along * sv
        se = float(np.sqrt((resid @ resid) / max(len(sv) - 1, 1) / (sv @ sv)))
        print(f"  LATENCY ERROR: along-track residual vs ego speed = {1000 * k_along:+.0f} "
              f"+/- {1000 * se:.0f} ms  (cross-track {1000 * k_cross:+.0f} ms, n={len(sv)})")
        print(f"     i.e. {abs(k_along) * 10:.2f} m of along-track error at 10 m/s")

    # 2: speed exactly as the planner computes it
    if kalman_predict is not None:
        pred = []
        for idx, (seg, t, objs) in enumerate(rows):
            if idx % predict_every:
                continue
            ob = {oid: {"loc": [np.array(p) for p in zip(xs, ys)]}
                  for oid, xs, ys in objs if len(xs) >= 2}
            if not ob:
                continue
            _, vel = kalman_predict(ob)
            for i in range(len(vel)):
                pred.append(float(np.hypot(*vel[i, 1])))
        if pred:
            p = np.array(pred)
            print(f"  kalman_predict speed (the planner's own): median {np.median(p):.2f} m/s, "
                  f"p90 {np.percentile(p, 90):.2f}; READ AS MOVING (> {STATIC_SPEED:.0f} m/s, "
                  f"so NOT avoided) {100 * np.mean(p > STATIC_SPEED):.1f}%  (n={p.size})")
    return tracks


def offset_check(tracks, min_heading_diff_deg=150.0, radius=2.5):
    """Static tracks at the same place seen with opposite headings: their disagreement along /
    across the heading is twice the lever-arm error in that axis."""
    summ = []
    for key, pts in tracks.items():
        if len(pts) < 10:
            continue
        a = np.array(pts)
        h = math.atan2(np.sin(a[:, 3]).mean(), np.cos(a[:, 3]).mean())
        summ.append((np.median(a[:, 1]), np.median(a[:, 2]), h))
    along, cross = [], []
    for i in range(len(summ)):
        for j in range(i + 1, len(summ)):
            xi, yi, hi = summ[i]
            xj, yj, hj = summ[j]
            if math.hypot(xi - xj, yi - yj) > radius:
                continue
            dh = abs((hi - hj + math.pi) % (2 * math.pi) - math.pi)
            if math.degrees(dh) < min_heading_diff_deg:
                continue
            c, s = math.cos(hi), math.sin(hi)
            dx, dy = xi - xj, yi - yj                     # i minus j, in i's heading axes
            along.append(dx * c + dy * s)
            cross.append(-dx * s + dy * c)
    print("\n=== LiDAR offset check (bridge): static objects seen from opposite headings ===")
    if len(along) < 5:
        print(f"  only {len(along)} opposed-heading pairs within {radius} m -- not enough to say")
        return
    a, c = np.array(along), np.array(cross)
    print(f"  pairs {len(a)}: disagreement along heading median {np.median(a):+.2f} m, "
          f"across {np.median(c):+.2f} m")
    print(f"  -> implied offset error: along {np.median(a) / 2:+.2f} m, "
          f"across {np.median(c) / 2:+.2f} m  (0 means lidar_offset is right)")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag")
    ap.add_argument("--planner-scripts",
                    default=os.path.expanduser("~/planner/src/AVA_Local_Planner/scripts"))
    ap.add_argument("--predict-every", type=int, default=10,
                    help="run kalman_predict on every Nth message (it is slow on long histories)")
    args = ap.parse_args()

    kalman_predict = None
    sys.path.insert(0, args.planner_scripts)
    try:
        from frenet_optimal_trajectory import kalman_predict        # noqa: E402
    except Exception as exc:                                        # noqa: BLE001
        print(f"[warn] planner kalman_predict unavailable ({exc}); skipping speed scoring")

    odom, streams = load(args.bag)
    print(f"[load] odom_grid {len(odom)}  " +
          "  ".join(f"{k} {len(v)}" for k, v in streams.items()))
    look = OdomLookup(odom)
    tracks = {}
    for name in ("legacy", "bridge"):
        if streams[name]:
            tracks[name] = score(name, streams[name], look, kalman_predict, args.predict_every)
    if "bridge" in tracks:
        offset_check(tracks["bridge"])


if __name__ == "__main__":
    main()
