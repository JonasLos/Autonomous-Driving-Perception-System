#!/usr/bin/env python3
"""Compare replay arms of the false-velocity fix: does a standing obstacle stop having a velocity?

    python3 scripts/velocity_arms_ab.py off=DIR radar02=DIR clusters=DIR ...

Each DIR holds one recording per bag, with /perception/objects and odometry. Tracks are labelled
per arm from their own whole-life world displacement. Only the bags present in EVERY arm are
scored, so arms are compared on the same drives. X is how fast the true range-rate is changing,
v^2 sin^2(az) / r; radial velocity is negative towards the car. The acceleration table covers
standing objects dead ahead while the car brakes or speeds up, where the ESR lag also shows (2026-11-13).
"""
import argparse
import bisect
import collections
import glob
import math
import multiprocessing
import os

import numpy as np
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory

CAM = 1
OFFSET = (0.67, -0.10)
XBINS = ((0, .3), (.3, 1), (1, 2), (2, 4), (4, 30))
ABINS = ((-9, -2), (-2, -.7), (-.7, .7), (.7, 2), (2, 9))
ODOMS = ("/novatel/oem7/odom_grid", "/novatel/oem7/odom")


def score_bag(path):
    files = glob.glob(os.path.join(path, "*.mcap"))
    odom, objects = collections.defaultdict(list), []
    with open(files[0], "rb") as fh:
        reader = make_reader(fh, decoder_factories=[DecoderFactory()])
        for _, ch, _, m in reader.iter_decoded_messages(topics=["/perception/objects", *ODOMS]):
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            if ch.topic in ODOMS:
                q = m.pose.pose.orientation
                tw = m.twist.twist
                odom[ch.topic].append((t, m.pose.pose.position.x, m.pose.pose.position.y,
                                       math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)),
                                       math.hypot(tw.linear.x, tw.linear.y)))
            else:
                objects.append((t, [(o.track_id, o.pose.position.x, o.pose.position.y, o.velocity.x, o.velocity.y,
                                     o.velocity_valid, o.contributions_this_frame, o.class_name) for o in m.objects]))
    rows = sorted(next(odom[k] for k in ODOMS if odom[k]))
    stamps = [r[0] for r in rows]
    o = np.array(rows)
    acc = (o[:, 4] - np.interp(o[:, 0] - .2, o[:, 0], o[:, 4])) / .2
    tracks = collections.defaultdict(list)
    for t, objs in objects:
        i = min(bisect.bisect_left(stamps, t), len(rows) - 1)
        p, a_ego = rows[i], acc[i]
        c, s = math.cos(p[3]), math.sin(p[3])
        for k, x, y, vx, vy, valid, bits, name in objs:
            ax, ay = x + OFFSET[0], y + OFFSET[1]
            tracks[k].append((t, p[1] + c * ax - s * ay, p[2] + s * ax + c * ay, x, y, vx, vy, valid, bits,
                              p[4], name == "person", a_ego))
    out = []
    for rs in tracks.values():
        rs.sort(key=lambda r: r[0])
        a = np.array(rs, float)
        dur = a[-1, 0] - a[0, 0]
        if dur < 1.0 or len(a) < 10:
            continue
        n = max(3, len(a) // 5)
        life = float(np.hypot(*(np.median(a[-n:, 1:3], axis=0) - np.median(a[:n, 1:3], axis=0)))) / dur
        spread = np.percentile(np.hypot(*(a[:, 1:3] - np.median(a[:, 1:3], axis=0)).T), 90)
        label = 0 if life < .3 and spread < 2.5 else (1 if life >= 1.0 else -1)
        if label < 0:
            continue
        tc = np.full(len(a), np.inf)
        last = None
        for i in range(len(a)):
            if int(a[i, 8]) & CAM:
                last = a[i, 0]
            if last is not None:
                tc[i] = a[i, 0] - last
        r = np.hypot(a[:, 3], a[:, 4]).clip(.5)
        vr = (a[:, 5] * a[:, 3] + a[:, 6] * a[:, 4]) / r
        out.append(np.c_[np.full(len(a), label), a[:, 10], a[:, 7], tc, a[:, 9], r,
                         np.hypot(a[:, 5], a[:, 6]), vr,
                         a[:, 9] ** 2 * (a[:, 4] / r) ** 2 / r, a[:, 3] > 0, np.full(len(a), life),
                         a[:, 11], np.abs(np.degrees(np.arctan2(a[:, 4], a[:, 3])))])
    return np.vstack(out) if out else np.zeros((0, 13))


def report(name, z):
    label, person, valid, tc, ego, r, speed, vr, x, ahead, life, acc, az = z.T
    still, moving, valid = label == 0, label == 1, valid > 0
    def row(title, m):
        p = m & (tc > .5) & (ego > 2)
        print(f"  {title:24s} n={int(m.sum()):6d}  valid {100 * valid[m].mean():5.2f}%  pass: n={int(p.sum()):5d}"
              f" median speed {np.median(speed[p]) if p.any() else float('nan'):.2f}"
              f" valid {100 * valid[p].mean() if p.any() else float('nan'):5.2f}%")
    print(f"\n{name}")
    row("standing person", still & (person > 0))
    row("standing other", still & (person == 0))
    cells = []
    for lo, hi in XBINS:
        m = still & (person > 0) & (ahead > 0) & (ego > 2) & (x >= lo) & (x < hi)
        cells.append(f"{lo}-{hi}: {np.median(vr[m]):+.2f} ({int(m.sum())})" if m.sum() > 10 else f"{lo}-{hi}: -")
    print("  person radial velocity by X: " + "  ".join(cells))
    cells = []
    for lo, hi in ABINS:
        m = still & (ahead > 0) & (az < 20) & (r > 5) & (r < 45) & (ego > .3) & (acc >= lo) & (acc < hi)
        cells.append(f"{lo}..{hi}: {np.median(vr[m]):+.2f} {100 * valid[m].mean():4.1f}% ({int(m.sum())})"
                     if m.sum() > 10 else f"{lo}..{hi}: -")
    print("  standing ahead by ego accel (radial, valid%): " + "  ".join(cells))
    m = moving & valid
    print(f"  movers: frames {int(moving.sum())}, valid {100 * valid[moving].mean():.1f}%, "
          f"|speed - life speed| median {np.median(np.abs(speed[m] - life[m])) if m.any() else float('nan'):.2f} m/s")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("arms", nargs="+", help="name=DIR")
    args = ap.parse_args()
    arms = dict(a.split("=", 1) for a in args.arms)
    names = None
    for d in arms.values():
        got = {os.path.basename(os.path.dirname(p)) for p in glob.glob(os.path.join(d, "*", "*.mcap"))}
        names = got if names is None else names & got
    print("bags in every arm:", sorted(names))
    with multiprocessing.Pool(8) as pool:
        for arm, d in arms.items():
            parts = pool.map(score_bag, [os.path.join(d, b) for b in sorted(names)])
            report(arm, np.vstack([p for p in parts if len(p)]))


if __name__ == "__main__":
    main()
