#!/usr/bin/env python3
"""Item 7, stage 2: would 360-degree LiDAR clusters actually keep objects alive?

A camera track dies ~0.5 s after the camera stops seeing it -- usually because the object left the
camera's ~33 deg view as the car drove past. Clusters are meant to SUSTAIN such tracks (never birth
them). So the question that decides whether the whole path is worth building:

    when a camera track dies, is there a LiDAR cluster where that object should still be?

For every track death this propagates the object's last position forward with the vehicle's own
motion (it is static in the world; production `predict` with zero velocity), clusters each
following sweep (Patchwork++ non-ground -> object_fusion.lidar_clusters), and counts how many
consecutive sweeps have a cluster inside the gate.

THE CONTROL IS THE POINT. A gate of a metre, in a scene full of cones, kerbs and bushes, will find
*something* a fair fraction of the time. The same test runs at the MIRROR position (y -> -y,
same range, other side of the car). Only the difference between the two is evidence.

    python3 scripts/lidar_cluster_ab.py [--source BAG] [--objects RECORDING]

`--objects` is any recording with /perception/objects that was made from `--source` (sim time =
bag time), e.g. ~/fusion_data/recordings/planner_ab.
"""

import argparse
import glob
import math
import os
import sys
import time
from collections import defaultdict

import numpy as np
from mcap_ros2.reader import read_ros2_messages

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import object_ab as oa  # noqa: E402  (puts the perception packages on the path)

from object_fusion import frames                                   # noqa: E402
from object_fusion.ego_motion import frame_increment                # noqa: E402
from object_fusion.ground_segmentation import GroundSegmenter       # noqa: E402
from object_fusion.lidar_clusters import cluster_nonground          # noqa: E402
from object_fusion.projection import project_to_pixels              # noqa: E402

LIDAR = "/lidar_tc/velodyne_points"
ODOM = "/novatel/oem7/odom"
OBJECTS = "/perception/objects"
FOLLOW_S = 3.0
SUSTAIN_SWEEPS = 5            # 0.5 s of support counts as "would have kept it alive"


def stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def decode(c):
    offs = {f.name: f.offset for f in c.fields}
    a = np.frombuffer(bytes(c.data), dtype=np.uint8).reshape(-1, c.point_step)
    g = lambda n: a[:, offs[n]:offs[n] + 4].copy().view(np.float32).ravel()
    p = np.stack([g("x"), g("y"), g("z"), g("intensity")], 1).astype(np.float64)
    return p[np.isfinite(p).all(1)]


def load_objects(rec):
    """First bag loop only: a replay rewind restarts every id."""
    msgs, last = [], None
    for f in sorted(glob.glob(os.path.join(rec, "*.mcap"))):
        for m in read_ros2_messages(f, topics=[OBJECTS]):
            t = stamp(m.ros_msg.header)
            if last is not None and t < last - 1.0:
                return msgs
            last = t
            rows = []
            for o in m.ros_msg.objects:
                p = frames.ego_to_lidar([np.array([o.pose.position.x, o.pose.position.y])],
                                        correction_deg=-5.35)[0]
                rows.append((o.track_id, p, o.update_count))
            msgs.append((t, rows))
    return msgs


def deaths_from(msgs, min_msgs=10):
    first, last, pos, count = {}, {}, {}, defaultdict(int)
    for t, rows in msgs:
        for tid, p, _ in rows:
            first.setdefault(tid, t)
            last[tid] = t
            pos[tid] = p
            count[tid] += 1
    t_end = msgs[-1][0]
    return [(last[k], pos[k], k) for k in last
            if count[k] >= min_msgs and last[k] < t_end - FOLLOW_S]


def in_view(p):
    kept, _, _ = project_to_pixels(np.array([[p[0], p[1], -1.5]]))
    return kept.shape[0] == 1


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--source", default="/home/avalocal/selfcal_loc_2026-09-08_11-47-43")
    ap.add_argument("--objects", default=os.path.expanduser("~/fusion_data/recordings/planner_ab"))
    args = ap.parse_args()

    msgs = load_objects(args.objects)
    deaths = deaths_from(msgs)
    t_lo, t_hi = msgs[0][0], msgs[-1][0]
    print(f"[objects] {len(msgs)} messages over {t_hi - t_lo:.0f} s, {len(deaths)} track deaths "
          f"(tracks seen in >= 10 messages, with {FOLLOW_S:.0f} s of drive after them)")

    odom = []
    for f in sorted(glob.glob(os.path.join(args.source, "*.mcap"))):
        for m in read_ros2_messages(f, topics=[ODOM]):
            tw = m.ros_msg.twist.twist
            odom.append((stamp(m.ros_msg.header), tw.linear.x, tw.linear.y, tw.angular.z))
    tw_buf, feed = oa._odom_feeder(odom)

    seg = GroundSegmenter(backend="patchworkpp", sensor_height=2.37, max_range=120.0)
    sweeps, spent, n_clusters, n_out = [], 0.0, [], []
    for f in sorted(glob.glob(os.path.join(args.source, "*.mcap"))):
        for m in read_ros2_messages(f, topics=[LIDAR]):
            t = stamp(m.ros_msg.header)
            if not (t_lo - 1.0 <= t <= t_hi):
                continue
            p = decode(m.ros_msg)
            t0 = time.perf_counter()
            ground = seg.segment(p[:, :3], p[:, 3] / 255.0)
            cl = cluster_nonground(p[~ground, :3])
            spent += time.perf_counter() - t0
            xy = np.array([[c.x, c.y] for c in cl]) if cl else np.empty((0, 2))
            sweeps.append((t, xy))
            n_clusters.append(len(cl))
            n_out.append(sum(not in_view((c.x, c.y)) for c in cl))
    sweeps.sort(key=lambda s: s[0])
    st = np.array([s[0] for s in sweeps])
    print(f"[lidar] {len(sweeps)} sweeps, segmentation + clustering {1000 * spent / len(sweeps):.1f}"
          f" ms/sweep, clusters/sweep median {np.median(n_clusters):.0f} "
          f"({np.median(n_out):.0f} outside the camera's view)\n")

    def follow(t_d, p0):
        """Consecutive sweeps after t_d with a cluster at the propagated position."""
        x = np.array([p0[0], p0[1], 0.0, 0.0])
        t_prev, run = t_d, 0
        i = int(np.searchsorted(st, t_d, side="right"))
        while i < len(st) and st[i] <= t_d + FOLLOW_S:
            dt = st[i] - t_prev
            feed(st[i])
            twist = tw_buf.at(st[i])
            if twist is None:
                break
            dpsi, d_xy = frame_increment(twist, dt)
            d_lidar = frames.ego_to_lidar([d_xy], correction_deg=-5.35)[0]
            c, s = math.cos(dpsi), math.sin(dpsi)
            Rt = np.array([[c, s], [-s, c]])
            x[:2] = Rt @ (x[:2] - d_lidar)        # a static point, re-expressed in the new frame
            t_prev = st[i]
            xy = sweeps[i][1]
            r = float(np.hypot(*x[:2]))
            if r > 60.0 or xy.shape[0] == 0:
                break
            if np.min(np.hypot(xy[:, 0] - x[0], xy[:, 1] - x[1])) > 1.0 + 0.02 * r:
                break
            run += 1
            i += 1
        return run

    rows = []
    for t_d, p, _tid in deaths:
        rows.append(dict(view=in_view(p), real=follow(t_d, p),
                         mirror=follow(t_d, (p[0], -p[1]))))

    print(f"  {'deaths':22s} {'n':>5s} {'kept alive >= 0.5 s':>20s} {'median sweeps':>14s}")
    for lab, sel in (("left the camera view", [x for x in rows if not x["view"]]),
                     ("died IN view", [x for x in rows if x["view"]]),
                     ("all", rows)):
        if not sel:
            continue
        real = np.array([x["real"] for x in sel])
        mir = np.array([x["mirror"] for x in sel])
        print(f"  {lab:22s} {len(sel):5d}   real {100 * np.mean(real >= SUSTAIN_SWEEPS):5.1f}%  "
              f"mirror {100 * np.mean(mir >= SUSTAIN_SWEEPS):5.1f}%   "
              f"real {np.median(real):3.0f} / mirror {np.median(mir):3.0f}")
    left = np.array([x["real"] for x in rows if not x["view"]])
    kept = left[left >= SUSTAIN_SWEEPS]
    if kept.size:
        print(f"\n  how long, for the {kept.size} tracks it would keep after they leave the view: "
              f"median {np.median(kept) / 10:.1f} s, p90 {np.percentile(kept, 90) / 10:.1f} s, "
              f"held the whole {FOLLOW_S:.0f} s window {100 * np.mean(kept >= FOLLOW_S * 10 - 1):.0f}%")
    print("\n  'mirror' is the same test at the other side of the car: chance support. Only the "
          "gap between the two columns is what clusters would genuinely add.")


if __name__ == "__main__":
    main()
