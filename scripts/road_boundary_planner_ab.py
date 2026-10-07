#!/usr/bin/env python3
"""Planner-level A/B: what lmp_v10c makes of raw vs road_boundary inputs.

The fair comparison for road_boundary is not its output against the raw topics -- on unmarked
roads the planner never sees either directly. lmp_v10c already fits, fuses and temporally
filters the boundaries, so the question is whether its OUTPUT (/fused/road_left|right) is better
when fed road_boundary's topics. Run three unmodified lmp_v10c instances on the same live replay,
outputs remapped to /ab/<arm>/road_{left,right}, and record them (see docs/road_boundary.md):

  raw  lmp_v10c on /sphereformer_*_boundary + /sam3_*_boundary (what runs today)
  rb   lmp_v10c on /road_boundary/*            (road_boundary, default steps)
  rbo  lmp_v10c on /road_boundary_occl/*       (road_boundary, occlusion step only)

  python3 scripts/road_boundary_planner_ab.py REC/REC_0.mcap

Two references, chosen from the odometry:
* STATIONARY recording: the road does not move, so each arm's reference edge is its own median
  over frames with no obstacle on that side, and the error in obstacle frames is how far the edge
  was pulled INWARD from it -- the failure itself, independent of which arm is better calibrated.
* MOVING recording (p95 speed > 0.5 m/s, or --moving): the road changes, so the reference is the
  arm's own ROLLING median over +-ROLL_S seconds. A passing or parked vehicle hides an edge for a
  fraction of a second at a station, which shows up as a transient inward dip from that rolling
  reference; slow changes of the road itself (curves, widening) do not.

  inward   mean / p95 inward displacement of the side that has the obstacle [m]
  >0.5/>1  share of those frames pulled in by more than 0.5 / 1.0 m
  centre   mean / p95 |centreline - reference centreline| [m] -- what the planner drives on
  clear    the same inward mean on obstacle-free frames: each arm's noise floor
  empty    share of frames with no boundary at the station
  jitter   mean |y(t) - y(t-1)| over all frames, both sides [m]
"""

from __future__ import annotations

import argparse
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, ".."))
sys.path[:0] = [os.path.join(REPO, "src", "object_fusion"), os.path.join(REPO, "src", "perception_common")]

from mcap_ros2.reader import read_ros2_messages  # noqa: E402

from object_fusion import frames  # noqa: E402

ARMS = ("raw", "rb", "rbo")
STATIONS = (10.0, 20.0, 30.0, 40.0)
OBJ = "/perception/objects"
ODOM = "/novatel/oem7/odom"
BLOCK_S = 5.0
ROLL_S = 1.5


def rolling_median(t, y, half=ROLL_S):
    """Median of the finite y within +-half seconds of each t (t sorted)."""
    out = np.full(len(y), np.nan)
    lo = np.searchsorted(t, t - half, side="left")
    hi = np.searchsorted(t, t + half, side="right")
    for i in range(len(y)):
        w = y[lo[i]:hi[i]]
        w = w[np.isfinite(w)]
        if w.size >= 3:
            out[i] = np.median(w)
    return out


def stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def path_station_y(msg, xs):
    """y at each station (ego frame) by interpolation along the path; NaN outside it."""
    if not msg.poses:
        return np.full(len(xs), np.nan)
    xy = np.array([(p.pose.position.x, p.pose.position.y) for p in msg.poses])
    e = frames.lidar_to_ego(xy)
    o = np.argsort(e[:, 0])
    ex, ey = e[o, 0], e[o, 1]
    out = np.full(len(xs), np.nan)
    for i, x in enumerate(xs):
        if ex[0] <= x <= ex[-1]:
            out[i] = np.interp(x, ex, ey)
    return out


def discover_arms(path):
    """Arm names from the /ab/<arm>/road_left topics present in the recording."""
    from mcap.reader import make_reader
    with open(path, "rb") as fh:
        summary = make_reader(fh).get_summary()
    names = {ch.topic.split("/")[2] for ch in summary.channels.values()
             if ch.topic.startswith("/ab/") and ch.topic.endswith("/road_left")}
    return [a for a in ARMS if a in names] + sorted(names - set(ARMS))


def load(path, arms=None):
    arms = arms or discover_arms(path)
    series = {a: {"left": [], "right": []} for a in arms}
    objs, speed = [], []
    topics = [f"/ab/{a}/road_{s}" for a in arms for s in ("left", "right")] + [OBJ, ODOM]
    for m in read_ros2_messages(path, topics=topics):
        t, msg = m.channel.topic, m.ros_msg
        if t == ODOM:
            speed.append(math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y))
        elif t == OBJ:
            objs.append((stamp(msg.header),
                         np.array([(o.pose.position.x, o.pose.position.y) for o in msg.objects]).reshape(-1, 2)))
        else:
            _, _, arm, side = t.split("/")
            series[arm][side.split("_")[1]].append((stamp(msg.header), path_station_y(msg, STATIONS)))
    return series, objs, np.array(speed)


def obstacle_side(t, objs, max_age=0.3):
    """(left, right): is a fused object 3 < x < 45 m on that side of the road?

    Sides overlap across the centre (-1 < y < 1 counts for both): a vehicle in the ego lane hides
    road on whichever side it sits. The first version required |y| > 1 m, which left exactly the
    worst raw events -- a car at y = -1.0 pulling the right edge in by 1.1 m -- in the
    "obstacle-free" reference, as the time-series plot showed.
    """
    if not objs:
        return False, False
    ts = np.array([o[0] for o in objs])
    i = int(np.argmin(np.abs(ts - t)))
    if abs(ts[i] - t) > max_age:
        return False, False
    o = objs[i][1]
    k = (o[:, 0] > 3) & (o[:, 0] < 45)
    return (bool((k & (o[:, 1] > -1.0) & (o[:, 1] < 7.0)).any()),
            bool((k & (o[:, 1] < 1.0) & (o[:, 1] > -7.0)).any()))


def ci(t, v, n_boot=2000, seed=0):
    t, v = np.asarray(t), np.asarray(v, float)
    k = np.isfinite(v)
    t, v = t[k], v[k]
    if v.size < 3:
        return (np.nan, np.nan, np.nan)
    b = np.floor((t - t.min()) / BLOCK_S).astype(int)
    groups = [v[b == u] for u in np.unique(b)]
    rng = np.random.default_rng(seed)
    means = [np.concatenate([groups[j] for j in rng.integers(0, len(groups), len(groups))]).mean()
             for _ in range(n_boot)]
    return float(v.mean()), float(np.percentile(means, 2.5)), float(np.percentile(means, 97.5))


def analyse(series, objs, moving=False):
    rows = {}
    for arm in series:
        L = series[arm]["left"]
        R = series[arm]["right"]
        if not L or not R:
            rows[arm] = None
            continue
        # Pair left/right by stamp (atomic publish makes them equal); sides alone are enough
        # for inward displacement, the pair is needed for the centreline.
        rt = np.array([t for t, _ in R])
        res = {}
        for si, x in enumerate(STATIONS):
            tl = np.array([t for t, _ in L])
            yl = np.array([y[si] for _, y in L])
            yr_all = np.array([y[si] for _, y in R])
            yr = np.array([yr_all[np.argmin(np.abs(rt - t))] if np.min(np.abs(rt - t)) < 0.05 else np.nan
                           for t in tl])
            obs = np.array([obstacle_side(t, objs) for t in tl])
            ol, orr = obs[:, 0], obs[:, 1]
            clear_l, clear_r = ~ol, ~orr
            if moving:
                ref_l = rolling_median(tl, yl)
                ref_r = rolling_median(tl, yr)
            else:
                ref_l = np.nanmedian(yl[clear_l]) if np.isfinite(yl[clear_l]).any() else np.nan
                ref_r = np.nanmedian(yr[clear_r]) if np.isfinite(yr[clear_r]).any() else np.nan
            in_l = ref_l - yl                      # >0: left edge pulled toward the centre
            in_r = yr - ref_r                      # >0: right edge pulled toward the centre
            centre = 0.5 * (yl + yr) - 0.5 * (ref_l + ref_r)
            inward_obs = np.r_[in_l[ol], in_r[orr]]
            t_obs = np.r_[tl[ol], tl[orr]]
            inward_clear = np.r_[in_l[clear_l], in_r[clear_r]]
            t_clear = np.r_[tl[clear_l], tl[clear_r]]
            any_obs = ol | orr
            res[x] = dict(
                ref=(float(np.nanmedian(ref_l)) if np.isfinite(ref_l).any() else np.nan,
                     float(np.nanmedian(ref_r)) if np.isfinite(ref_r).any() else np.nan),
                n_obs=int(np.isfinite(inward_obs).sum()),
                inward=ci(t_obs, inward_obs),
                p95=float(np.nanpercentile(inward_obs, 95)) if np.isfinite(inward_obs).any() else np.nan,
                gt05=float(np.nanmean(inward_obs[np.isfinite(inward_obs)] > 0.5)) if np.isfinite(inward_obs).any() else np.nan,
                gt10=float(np.nanmean(inward_obs[np.isfinite(inward_obs)] > 1.0)) if np.isfinite(inward_obs).any() else np.nan,
                centre=ci(tl[any_obs], np.abs(centre[any_obs])),
                centre_p95=float(np.nanpercentile(np.abs(centre[any_obs]), 95)) if np.isfinite(centre[any_obs]).any() else np.nan,
                clear=ci(t_clear, inward_clear),
                empty=float(np.mean(~np.isfinite(yl) | ~np.isfinite(yr))),
                jitter=float(np.nanmean(np.r_[np.abs(np.diff(yl)), np.abs(np.diff(yr))])),
                frames=len(tl))
        rows[arm] = res
    return rows


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mcap")
    ap.add_argument("--moving", action="store_true", help="force the rolling reference")
    ap.add_argument("--arms", help="comma-separated arm names (default: all in the recording)")
    args = ap.parse_args()
    series, objs, speed = load(args.mcap, args.arms.split(",") if args.arms else None)
    moving = args.moving or (speed.size and np.percentile(speed, 95) > 0.5)
    rows = analyse(series, objs, moving=moving)
    kind = f"MOVING, rolling +-{ROLL_S:.1f} s reference" if moving else "stationary"
    print(f"{os.path.basename(args.mcap)}  ({kind}: median speed {np.median(speed):.1f} m/s, "
          f"p95 {np.percentile(speed, 95):.1f} m/s)")
    for x in STATIONS:
        print(f"\n  x = {x:.0f} m")
        print(f"    {'arm':4s} {'ref L/R':>13s} {'n_obs':>5s}  {'inward mean [95% CI]':>24s} {'p95':>5s} "
              f"{'>0.5m':>6s} {'>1m':>5s}  {'centre mean [CI]':>22s} {'p95':>5s}  {'clear inward':>12s} {'empty':>6s} {'jitter':>6s}")
        for arm in rows:
            r = rows[arm]
            if r is None:
                print(f"    {arm:4s} no output recorded")
                continue
            d = r[x]
            m, lo, hi = d["inward"]
            cm, clo, chi = d["centre"]
            print(f"    {arm:4s} {d['ref'][0]:6.2f}/{d['ref'][1]:6.2f} {d['n_obs']:5d}  "
                  f"{m:6.2f} [{lo:5.2f},{hi:5.2f}]      {d['p95']:5.2f} {100 * d['gt05']:5.1f}% {100 * d['gt10']:4.1f}%  "
                  f"{cm:5.2f} [{clo:4.2f},{chi:4.2f}]      {d['centre_p95']:5.2f}  {d['clear'][0]:12.2f} {100 * d['empty']:5.1f}% {d['jitter']:6.3f}")


if __name__ == "__main__":
    main()
