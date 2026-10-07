#!/usr/bin/env python3
"""Offline A/B: raw /sphereformer_* and /sam3_* boundaries vs. road_boundary's rule, per step.

Replays a recording of the node's INPUTS through the production ``BoundaryPipeline`` (the same
object the node wraps, so this measures the rule rather than a copy), in ARRIVAL order (mcap
log time) so fused objects show up with the same lag they have live. The raw arm is the raw
boundaries recorded alongside, from the same replay.

Record the inputs during a replay with the stack up (run_radar.sh WITHOUT --obstacle-only):

  ros2 bag record -s mcap -o OUT /sphereformer_road_segmentation \\
      /perception/lidar_2d_projection_ground /segmentation/mask_image/compressed \\
      /perception/objects /novatel/oem7/odom /camera_fl/camera_info \\
      /sphereformer_left_boundary /sphereformer_right_boundary \\
      /sam3_left_boundary /sam3_right_boundary

  python3 scripts/road_boundary_ab.py OUT/OUT_0.mcap [--arms all,raw,no_occlusion,...]
  python3 scripts/road_boundary_ab.py OUT/OUT_0.mcap --screen    # does this bag test anything?
  python3 scripts/road_boundary_ab.py OUT/OUT_0.mcap --arms raw,+occlusion+fit+tracking

Metrics, per source, at look-ahead stations x = 10/20/40 m in the ego frame, split by frames
WITH an obstacle near the road edge (a fused object 3 < x < 45 m, 1 < |y| < 7 m) and without:

  jump    |y(t) - y(t-1)| per side -- an obstacle pulling the edge in shows up here
  wdev    |width(t) - rolling median width| (+-5 s) -- one side pulled in narrows the road
  narrow  share of frames whose width is > 1 m below that rolling median: the failure itself
  empty   share of frames with no boundary at that station

Intervals are a block bootstrap over 5 s blocks (frames within a block are not independent).
A short bag gives few blocks; read the intervals before reading a difference.
"""

from __future__ import annotations

import argparse
import io
import math
import os
import sys
from collections import defaultdict

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, ".."))
sys.path[:0] = [os.path.join(REPO, "src", "object_fusion"), os.path.join(REPO, "src", "perception_common")]

from mcap_ros2.reader import read_ros2_messages  # noqa: E402
from PIL import Image  # noqa: E402

from object_fusion import frames  # noqa: E402
from object_fusion.flagged_cloud import decode_fields  # noqa: E402
from object_fusion.road_boundary import (  # noqa: E402
    STEPS, BoundaryParams, BoundaryPipeline, decode_semkitti_rgb, mask_road_pixels,
)

SEG = "/sphereformer_road_segmentation"
PROJ = "/perception/lidar_2d_projection_ground"
MASK = "/segmentation/mask_image/compressed"
OBJ = "/perception/objects"
ODOM = "/novatel/oem7/odom"
INFO = "/camera_fl/camera_info"
RAW = {"sphereformer": ("/sphereformer_left_boundary", "/sphereformer_right_boundary"),
       "sam3": ("/sam3_left_boundary", "/sam3_right_boundary")}
STATIONS = (10.0, 20.0, 40.0)
ROLL_S = 5.0
NARROW_M = 1.0
BLOCK_S = 5.0
MAX_PAIR_SKEW = 0.08


def stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def station_y(pts_lidar, x0, half=2.5):
    """Lateral offset of a boundary cloud at ego x0: median y of points within +-half m."""
    if pts_lidar is None or len(pts_lidar) == 0:
        return np.nan
    e = frames.lidar_to_ego(np.asarray(pts_lidar)[:, :2])
    k = np.abs(e[:, 0] - x0) <= half
    return float(np.median(e[k, 1])) if k.any() else np.nan


def xyz_of(msg):
    f = decode_fields(msg, ("x", "y", "z"))
    if f["x"] is None or f["x"].size == 0:
        return np.empty((0, 3))
    a = np.stack([f["x"], f["y"], f["z"]], 1)
    return a[np.isfinite(a).all(1)]


class Events:
    """Re-iterable stream of (log_time_ns, topic, msg) in arrival order, read per pass.

    A drive-length recording does not fit in memory deserialized (1.5 GB for 398 s), so each
    arm streams the file again rather than sharing a loaded list.
    """

    def __init__(self, path, topics=None):
        self.path, self.topics = path, topics

    def __iter__(self):
        for m in read_ros2_messages(self.path, topics=self.topics, log_time_order=True):
            yield m.log_time_ns, m.channel.topic, m.ros_msg


def load(path):
    return Events(path)


PARAM_OVERRIDES = {}


def run_arm(events, steps, sources=("sphereformer", "sam3")):
    """-> {source: [(t, left_pts, right_pts)]} from the production pipeline."""
    p = BoundaryParams()
    p.update(**PARAM_OVERRIDES)
    pl = BoundaryPipeline(p)
    for k in STEPS:
        pl.steps[k] = k in steps
    out = defaultdict(list)
    image_wh = None
    projections = []          # (stamp, msg), recent
    for _, topic, msg in events:
        if topic == ODOM:
            tw = msg.twist.twist
            pl.add_odom(stamp(msg.header), tw.linear.x, tw.linear.y, tw.angular.z)
        elif topic == OBJ:
            o = msg.objects
            pl.add_objects(stamp(msg.header), [(a.pose.position.x, a.pose.position.y) for a in o],
                           [(a.size.x, a.size.y) for a in o], [yaw(a.pose.orientation) for a in o],
                           [(a.velocity.x, a.velocity.y) if a.velocity_valid else (0, 0) for a in o])
        elif topic == INFO and image_wh is None and msg.width > 0:
            image_wh = (int(msg.width), int(msg.height))
        elif topic == SEG:
            f = decode_fields(msg, ("x", "y", "z", "rgb"))
            xyz = np.stack([f["x"], f["y"], f["z"]], 1)
            ok = np.isfinite(xyz).all(1)
            t = stamp(msg.header)
            l, r, _, _ = pl.sphereformer_frame(t, xyz[ok], decode_semkitti_rgb(f["rgb"][ok]))
            if "sphereformer" in sources:
                out["sphereformer"].append((t, l, r))
        elif topic == PROJ:
            projections.append((stamp(msg.header), msg))
            projections = projections[-40:]
        elif topic == MASK and "sam3" in sources and image_wh is not None:
            t = stamp(msg.header)
            if not projections:
                continue
            ts, pm = min(projections, key=lambda p: abs(p[0] - t))
            if abs(ts - t) > MAX_PAIR_SKEW:
                continue
            f = decode_fields(pm, ("x", "y", "z", "u", "v", "ground"))
            xyz = np.stack([f["x"], f["y"], f["z"]], 1)
            ok = np.isfinite(xyz).all(1) & np.isfinite(f["u"]) & np.isfinite(f["v"])
            g = None if f["ground"] is None else f["ground"][ok] > 0.5
            mask = np.asarray(Image.open(io.BytesIO(bytes(msg.data))).convert("L"))
            road_px = mask_road_pixels(mask, extraction=pl.steps["extraction"])
            l, r, _, _ = pl.sam3_frame(ts, xyz[ok], f["u"][ok], f["v"][ok], g, road_px, image_wh)
            out["sam3"].append((ts, l, r))
    return out


def raw_arm(events):
    out = {}
    for src, (lt, rt) in RAW.items():
        left, right = {}, {}
        for _, topic, msg in events:
            if topic == lt:
                left[stamp(msg.header)] = xyz_of(msg)
            elif topic == rt:
                right[stamp(msg.header)] = xyz_of(msg)
        ts = sorted(set(left) | set(right))
        out[src] = [(t, left.get(t), right.get(t)) for t in ts]
    return out


def object_track(events):
    ts, objs = [], []
    for _, topic, msg in events:
        if topic == OBJ:
            ts.append(stamp(msg.header))
            objs.append(np.array([(o.pose.position.x, o.pose.position.y) for o in msg.objects]).reshape(-1, 2))
    order = np.argsort(ts)
    return np.asarray(ts)[order], [objs[i] for i in order]


def near_edge(t, ots, oxy, max_age=0.3):
    if len(ots) == 0:
        return False
    i = int(np.argmin(np.abs(ots - t)))
    if abs(ots[i] - t) > max_age:
        return False
    o = oxy[i]
    return bool(((o[:, 0] > 3) & (o[:, 0] < 45) & (np.abs(o[:, 1]) > 1.0) & (np.abs(o[:, 1]) < 7.0)).any())


def frame_metrics(series, ots, oxy):
    """-> rows (t, near, station, jump_l, jump_r, width, empty_l, empty_r)."""
    rows = []
    prev = None
    for t, l, r in series:
        ys = {x: (station_y(l, x), station_y(r, x)) for x in STATIONS}
        ne = near_edge(t, ots, oxy)
        for x in STATIONS:
            yl, yr = ys[x]
            jl = abs(yl - prev[x][0]) if prev is not None else np.nan
            jr = abs(yr - prev[x][1]) if prev is not None else np.nan
            rows.append((t, ne, x, jl, jr, yl - yr, np.isnan(yl), np.isnan(yr)))
        prev = ys
    return rows


def block_ci(t, v, n_boot=2000, seed=0):
    t, v = np.asarray(t), np.asarray(v, float)
    k = np.isfinite(v)
    t, v = t[k], v[k]
    if v.size < 3:
        return np.nan, np.nan, np.nan
    blocks = np.floor((t - t.min()) / BLOCK_S).astype(int)
    ub = np.unique(blocks)
    groups = [v[blocks == b] for b in ub]
    rng = np.random.default_rng(seed)
    means = []
    for _ in range(n_boot):
        pick = rng.integers(0, len(groups), len(groups))
        means.append(np.concatenate([groups[i] for i in pick]).mean())
    return float(v.mean()), float(np.percentile(means, 2.5)), float(np.percentile(means, 97.5))


def rolling_median(t, v, half=ROLL_S):
    out = np.full(v.size, np.nan)
    for i in range(v.size):
        k = (np.abs(t - t[i]) <= half) & np.isfinite(v)
        if k.sum() >= 3:
            out[i] = np.median(v[k])
    return out


def report(name, rows):
    if not rows:
        print(f"  {name:14s} no frames")
        return
    a = np.array([(r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7]) for r in rows], dtype=float)
    wref = np.full(len(a), np.nan)
    for x in STATIONS:
        k = a[:, 2] == x
        wref[k] = rolling_median(a[k, 0], a[k, 5])
    for near in (1.0, 0.0):
        sel = a[:, 1] == near
        label = "obstacle-near-edge" if near else "clear"
        parts = []
        for x in STATIONS:
            s = sel & (a[:, 2] == x)
            if not s.any():
                continue
            jumps = np.r_[a[s, 3], a[s, 4]]
            tt = np.r_[a[s, 0], a[s, 0]]
            m, lo, hi = block_ci(tt, jumps)
            dw = a[s, 5] - wref[s]
            fin = np.isfinite(dw)
            wd = np.mean(np.abs(dw[fin])) if fin.any() else np.nan
            narrow = 100.0 * np.mean(dw[fin] < -NARROW_M) if fin.any() else np.nan
            empty = 100.0 * np.r_[a[s, 6], a[s, 7]].mean()
            parts.append(f"x={x:>2.0f}: jump {m:5.2f} [{lo:4.2f},{hi:4.2f}] wdev {wd:5.2f} "
                         f"narrow {narrow:4.1f}% empty {empty:4.1f}%")
        n = int(sel.sum() / len(STATIONS))
        print(f"  {name:14s} {label:19s} n={n:4d}  " + " | ".join(parts))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mcap")
    ap.add_argument("--arms", default="raw,all,no_occlusion,no_extraction,no_fit,no_tracking,none")
    ap.add_argument("--screen", action="store_true",
                    help="only report how much of the bag has obstacles near the edge")
    ap.add_argument("--param", action="append", default=[],
                    help="BoundaryParams override for every arm, e.g. --param run_gap=3.0")
    args = ap.parse_args()
    for kv in args.param:
        k, v = kv.split("=", 1)
        PARAM_OVERRIDES[k] = float(v)
    events = load(args.mcap)
    light = Events(args.mcap, topics=[OBJ, *RAW["sphereformer"], *RAW["sam3"]])
    ots, oxy = object_track(light)
    raw = raw_arm(light)
    if args.screen:
        for src, series in raw.items():
            rows = frame_metrics(series, ots, oxy)
            ne = sum(1 for r in rows if r[1] and r[2] == STATIONS[0])
            n = sum(1 for r in rows if r[2] == STATIONS[0])
            k = [r for r in rows if r[2] == 20.0]
            t = np.array([r[0] for r in k])
            w = np.array([r[5] for r in k])
            near = np.array([r[1] for r in k], bool)
            ref = rolling_median(t, w) if w.size else w
            narrow = np.isfinite(ref) & (w < ref - NARROW_M)
            print(f"{src}: {n} frames, {ne} with an obstacle near the edge; raw width at 20 m "
                  f"> {NARROW_M:.0f} m below its +-{ROLL_S:.0f} s median in {int(narrow.sum())} "
                  f"frames ({int((narrow & near).sum())} of them with an obstacle near the edge)")
        return
    arms = {}
    for name in args.arms.split(","):
        if name == "raw":
            arms[name] = raw
        elif name == "all":
            arms[name] = run_arm(events, set(STEPS))
        elif name == "none":
            arms[name] = run_arm(events, set())
        elif name.startswith("no_") and name[3:] in STEPS:
            arms[name] = run_arm(events, set(STEPS) - {name[3:]})
        elif name.startswith("+"):          # e.g. +occlusion+fit+tracking: exactly these steps
            steps = set(name[1:].split("+"))
            if not steps <= set(STEPS):
                raise SystemExit(f"unknown step in {name}")
            arms[name] = run_arm(events, steps)
        else:
            raise SystemExit(f"unknown arm {name}")
    for src in ("sphereformer", "sam3"):
        print(f"\n== {src}")
        for name, res in arms.items():
            report(name, frame_metrics(res.get(src, []), ots, oxy))


if __name__ == "__main__":
    main()
