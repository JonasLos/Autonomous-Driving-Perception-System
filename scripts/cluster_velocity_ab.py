#!/usr/bin/env python3
"""Can a track's velocity be measured from its 360-degree LiDAR clusters?

Cluster updates are position-only because the difference between CONSECUTIVE clusters is an offset,
not motion. This asks the different question: over a window of about a second, is a robust fit
through the cluster positions (ego-compensated, in the world) a usable velocity?

Each recording needs /perception/objects, /perception/measurements/lidar and odometry. Each track
is labelled from its own whole-life world displacement, static or moving, and followed through the
clusters with the production gate (1.0 + 0.02 r, exclusive). Reported per window:

  * standing objects: the fit's speed, against what the filter publishes on the same samples;
  * movers: the vector error of the fit against the filter's velocity while the camera is fresh
    and the velocity is valid. That reference has its own error, so this is an upper bound.

    python3 scripts/cluster_velocity_ab.py BAG_DIR [BAG_DIR ...]

The positions are UTM and the stamps epoch seconds, so everything is taken relative to the first
sample of the track BEFORE any fit. Without that a least-squares slope is floating-point noise: it
read 16 m/s on a parked car. Needs PYTHONPATH with src/object_fusion and src/perception_common.
"""
import argparse
import collections
import glob
import itertools
import math
import multiprocessing
import os

import numpy as np
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory

from object_fusion import frames

CAM = 1
#: The planner bridge's LiDAR offset in the vehicle frame. A constant, so it cannot create a slope.
LIDAR_OFFSET = (0.67, -0.10)
WINDOWS = (0.5, 1.0, 1.5, 2.0)
OBJECTS, CLUSTERS, ODOMS = ("/perception/objects", "/perception/measurements/lidar",
                            ("/novatel/oem7/odom_grid", "/novatel/oem7/odom"))
#: Plan gates: stop before changing the tracker if either fails at the 1 s window.
GATE_MOVER_ERR, GATE_STATIC_ABOVE = 1.0, 0.10


def _stamp(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def _yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def _world(xy, pose):
    c, s = math.cos(pose[3]), math.sin(pose[3])
    x, y = xy[..., 0] + LIDAR_OFFSET[0], xy[..., 1] + LIDAR_OFFSET[1]
    return np.stack([pose[1] + c * x - s * y, pose[2] + s * x + c * y], -1)


def read_bag(path):
    files = glob.glob(os.path.join(path, "*.mcap"))
    if not files:
        return None
    odom, clusters, objects = collections.defaultdict(list), [], []
    with open(files[0], "rb") as fh:
        reader = make_reader(fh, decoder_factories=[DecoderFactory()])
        for _, ch, _, m in reader.iter_decoded_messages(topics=[OBJECTS, CLUSTERS, *ODOMS]):
            t = _stamp(m.header)
            if ch.topic in ODOMS:
                tw = m.twist.twist
                odom[ch.topic].append((t, m.pose.pose.position.x, m.pose.pose.position.y,
                                       _yaw(m.pose.pose.orientation),
                                       math.hypot(tw.linear.x, tw.linear.y)))
            elif ch.topic == CLUSTERS:
                clusters.append((t, [(d.pose.position.x, d.pose.position.y) for d in m.detections]))
            else:
                objects.append((t, [(o.track_id, o.pose.position.x, o.pose.position.y, o.velocity.x,
                                     o.velocity.y, o.velocity_valid, o.contributions_this_frame,
                                     o.class_name) for o in m.objects]))
    rows = next((odom[k] for k in ODOMS if odom[k]), None)
    return (np.array(sorted(rows)), clusters, objects) if rows and clusters and objects else None


def label_tracks(objects, pose_at):
    tracks = collections.defaultdict(list)
    for t, objs in objects:
        p = pose_at(t)
        c, s = math.cos(p[3]), math.sin(p[3])
        for k, x, y, vx, vy, valid, bits, name in objs:
            w = _world(np.array([x, y]), p)
            tracks[k].append((t, w[0], w[1], c * vx - s * vy, s * vx + c * vy, valid, bits,
                              name == "person", x, y, p[4]))
    out = {}
    for k, rows in tracks.items():
        rows.sort(key=lambda r: r[0])
        a = np.array(rows, float)
        dur = a[-1, 0] - a[0, 0]
        if dur < 1.0 or len(a) < 10:
            continue
        n = max(3, len(a) // 5)
        speed = float(np.hypot(*(np.median(a[-n:, 1:3], axis=0) - np.median(a[:n, 1:3], axis=0)))) / dur
        spread = np.percentile(np.hypot(*(a[:, 1:3] - np.median(a[:, 1:3], axis=0)).T), 90)
        if speed < 0.3 and spread < 2.5:
            out[k] = ("static", a, speed)
        elif speed >= 1.0:
            out[k] = ("moving", a, speed)
    return out


def follow_clusters(tracks, clusters, pose_at):
    """Per track, the world position of the cluster that the production gate would give it."""
    series = collections.defaultdict(list)
    for t, dets in clusters:
        if not dets:
            continue
        p = pose_at(t)
        cw = _world(np.asarray(frames.lidar_to_ego(np.array(dets))), p)
        cand = []
        for k, (_, a, _) in tracks.items():
            if a[0, 0] <= t <= a[-1, 0]:
                q = np.array([np.interp(t, a[:, 0], a[:, 1]), np.interp(t, a[:, 0], a[:, 2])])
                dd = np.hypot(*(cw - q).T)
                gate = 1.0 + 0.02 * float(np.hypot(*(q - p[1:3])))
                cand += [(dd[j], k, j) for j in np.flatnonzero(dd <= gate)]
        used_k, used_j = set(), set()
        for _, k, j in sorted(cand):
            if k not in used_k and j not in used_j:
                used_k.add(k)
                used_j.add(j)
                series[k].append((t, cw[j, 0], cw[j, 1], p[1], p[2]))
    return series


def fit(t, xy, now, window):
    """Trailing median-of-slopes velocity, or None without enough support. Inputs are relative."""
    m = (t > now - window) & (t <= now + 1e-9)
    if m.sum() < max(3, int(0.6 * window * 10)) or t[m][-1] - t[m][0] < 0.7 * window:
        return None
    idx = np.flatnonzero(m)
    slopes = [(xy[b] - xy[a]) / (t[b] - t[a]) for a, b in itertools.combinations(idx, 2)
              if t[b] - t[a] >= 0.25 * window]
    return np.median(np.array(slopes), axis=0) if slopes else None


def samples(path):
    data = read_bag(path)
    if data is None:
        return []
    odom, clusters, objects = data
    def pose_at(t):
        return odom[min(np.searchsorted(odom[:, 0], t), len(odom) - 1)]
    tracks = label_tracks(objects, pose_at)
    series = follow_clusters(tracks, clusters, pose_at)
    bag = path.rstrip("/").split("/")[-1]
    out = []
    for k, (label, a, life_speed) in tracks.items():
        if len(series.get(k, ())) < 5:
            continue
        s = np.array(sorted(series[k]), float)
        t0, xy0 = s[0, 0], s[0, 1:3].copy()
        t, xy = s[:, 0] - t0, s[:, 1:3] - xy0
        tcam = np.full(len(a), np.inf)
        last = None
        for i, row in enumerate(a):
            if int(row[6]) & CAM:
                last = row[0]
            if last is not None:
                tcam[i] = row[0] - last
        for i in range(len(t)):
            j = min(np.searchsorted(a[:, 0], s[i, 0]), len(a) - 1)
            u = s[i, 1:3] - s[i, 3:5]
            rng = float(np.hypot(*u))
            u = u / max(rng, 1e-3)
            rec = [label == "static", a[j, 7] > 0, rng, math.degrees(abs(math.atan2(a[j, 9], a[j, 8]))),
                   tcam[j], a[j, 10], float(np.hypot(a[j, 3], a[j, 4])), a[j, 5] > 0, life_speed,
                   a[j, 3], a[j, 4]]
            for w in WINDOWS:
                v = fit(t, xy, t[i], w)
                rec += [np.nan] * 3 if v is None else [float(v[0]), float(v[1]), float(v @ u)]
            out.append((bag, k, rec))
    return out


def report(name, z, m):
    if m.sum() < 30:
        print(f"\n{name}: too few samples ({int(m.sum())})")
        return None
    fv = z[:, 6]
    print(f"\n{name}  n={int(m.sum())}  filter today: median {np.median(fv[m]):.2f} m/s, "
          f"above 0.5: {100 * np.mean(fv[m] > .5):.0f}%, above 1: {100 * np.mean(fv[m] > 1):.0f}%")
    print("  window  avail   median   p90    p99   >0.5    >1   radial bias")
    above = None
    for wi, w in enumerate(WINDOWS):
        vx, vy, vr = z[:, 11 + 3 * wi], z[:, 12 + 3 * wi], z[:, 13 + 3 * wi]
        ok = m & np.isfinite(vx)
        if ok.sum() < 20:
            continue
        sp = np.hypot(vx[ok], vy[ok])
        a = float(np.mean(sp > .5))
        print(f"   {w:3.1f}s  {100 * ok.sum() / m.sum():4.0f}%  {np.median(sp):6.2f} "
              f"{np.percentile(sp, 90):6.2f} {np.percentile(sp, 99):6.2f}  {100 * a:4.0f}% "
              f"{100 * np.mean(sp > 1):4.0f}%  {np.median(vr[ok]):+.3f}")
        if w == 1.0:
            above = a
    return above


def report_movers(z, min_speed=2.0):
    m = (z[:, 0] == 0) & (z[:, 4] < 0.2) & (z[:, 7] > 0) & (z[:, 6] >= min_speed)
    print(f"\nMOVERS, camera fresh, filter valid, filter speed >= {min_speed:.0f} m/s  n={int(m.sum())}")
    if m.sum() < 30:
        print("  too few samples to judge")
        return None
    print("  window  avail   vector err median   p90   speed ratio (fit / filter)")
    err_1s = None
    for wi, w in enumerate(WINDOWS):
        vx, vy = z[:, 11 + 3 * wi], z[:, 12 + 3 * wi]
        ok = m & np.isfinite(vx)
        if ok.sum() < 20:
            continue
        e = np.hypot(vx[ok] - z[ok, 9], vy[ok] - z[ok, 10])
        print(f"   {w:3.1f}s  {100 * ok.sum() / m.sum():4.0f}%  {np.median(e):12.2f} "
              f"{np.percentile(e, 90):8.2f} {np.median(np.hypot(vx[ok], vy[ok]) / z[ok, 6]):10.2f}")
        if w == 1.0:
            err_1s = float(np.median(e))
    return err_1s


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("bags", nargs="+")
    args = ap.parse_args()
    with multiprocessing.Pool(min(8, len(args.bags))) as pool:
        got = [r for rs in pool.imap_unordered(samples, args.bags) for r in rs]
    if not got:
        raise SystemExit("no usable tracks in these recordings")
    print("tracks:", sorted(collections.Counter(
        (b, "static" if r[0] else "moving") for b, _, r in {(b, k): (b, k, r) for b, k, r in got}.values()).items()))
    z = np.array([r for _, _, r in got], float)
    still, person, rng, az, tc, ego = z[:, 0] > 0, z[:, 1] > 0, z[:, 2], z[:, 3], z[:, 4], z[:, 5]
    report("STANDING all", z, still)
    report("STANDING person", z, still & person)
    report("STANDING vehicle/other", z, still & ~person)
    report("STANDING, car below 0.5 m/s (noise floor)", z, still & (ego < .5))
    report("STANDING, car 0.5-5 m/s", z, still & (ego >= .5) & (ego < 5))
    fast = report("STANDING, car above 5 m/s", z, still & (ego >= 5))
    report("STANDING, the pass: camera stale, car above 2 m/s", z, still & (tc > .5) & (ego > 2))
    report("STANDING, beside: over 30 deg, under 20 m, car above 2 m/s", z, still & (az > 30) & (rng < 20) & (ego > 2))
    err = report_movers(z)
    print("\nGATES (1 s window)")
    print(f"  mover median error <= {GATE_MOVER_ERR} m/s: "
          + ("no data" if err is None else f"{err:.2f} -> {'pass' if err <= GATE_MOVER_ERR else 'FAIL'}"))
    print(f"  standing, car above 5 m/s, above 0.5 m/s <= {100 * GATE_STATIC_ABOVE:.0f}%: "
          + ("no data" if fast is None else f"{100 * fast:.0f}% -> {'pass' if fast <= GATE_STATIC_ABOVE else 'FAIL'}"))


if __name__ == "__main__":
    main()
