#!/usr/bin/env python3
"""Compare the runs written by scripts/fusion_isolation_check.sh: A1 and A2 (existing stack alone)
and B (existing stack + object_fusion).

For /fused_bbox and /tracked_objects, per capture stamp common to two runs: are the same objects
there, and how far did they move? Plus the existing nodes' `pairing:` counters and container CPU.
The verdict rule: B may differ from A1 by no more than A2 does.

    python3 scripts/isolation_compare.py OUT_DIR
"""
from __future__ import annotations

import glob
import math
import os
import re
import sys
from collections import defaultdict

import numpy as np
from mcap_ros2.reader import read_ros2_messages

RUNS = ("A1", "B", "A2")


def stamp(h):
    return round(h.stamp.sec + h.stamp.nanosec * 1e-9, 3)


def load(out, run):
    fused, tracked = {}, {}
    for f in sorted(glob.glob(os.path.join(out, run, "*.mcap"))):
        for m in read_ros2_messages(f):
            o = m.ros_msg
            if m.channel.topic == "/fused_bbox":
                fused[stamp(o.header)] = [
                    (d.id, d.class_name, d.bbox.center.position.x, d.bbox.center.position.y,
                     d.bbox3d.center.position.x, d.bbox3d.center.position.y) for d in o.detections]
            elif m.channel.topic == "/tracked_objects":
                tracked[stamp(o.header)] = [
                    (ob.id, ob.class_name, None, None, ob.center.position.x, ob.center.position.y)
                    for ob in o.objects]
    return fused, tracked


def compare(a, b):
    """Frame-level agreement of two {stamp: [(id, class, u, v, x, y)]} streams."""
    common = sorted(set(a) & set(b))
    only_a, only_b = len(set(a) - set(b)), len(set(b) - set(a))
    same_count = same_set = 0
    d3 = []
    for t in common:
        da, db = a[t], b[t]
        same_count += len(da) == len(db)
        # match by nearest 3D position; ids are per-run tracker state and may legitimately differ
        used, ok = set(), len(da) == len(db)
        for x in da:
            best, bd = None, float("inf")
            for j, y in enumerate(db):
                if j in used or y[1] != x[1]:
                    continue
                dist = math.hypot(x[4] - y[4], x[5] - y[5])
                if dist < bd:
                    best, bd = j, dist
            if best is None:
                ok = False
                continue
            used.add(best)
            d3.append(bd)
        same_set += ok
    d3 = np.asarray(d3) if d3 else np.zeros(1)
    n = max(len(common), 1)
    return dict(frames_a=len(a), frames_b=len(b), common=len(common), only_a=only_a, only_b=only_b,
                same_count=100 * same_count / n, same_objects=100 * same_set / n,
                identical=100 * float(np.mean(d3 < 1e-6)), p99=float(np.percentile(d3, 99)),
                max=float(d3.max()))


def pairing(out, run):
    res = {}
    for f in glob.glob(os.path.join(out, f"{run}.perception_*.log")):
        name = f.split(f"{run}.")[1][:-4]
        lines = [ln for ln in open(f, errors="replace") if "pairing:" in ln]
        if not lines:
            continue
        last = lines[-1]
        kv = dict(re.findall(r"(matched|unmatched|expired|stale|resets)=(\d+)", last))
        res[name] = {k: int(v) for k, v in kv.items()}
    return res


def cpu(out, run):
    acc = defaultdict(list)
    p = os.path.join(out, f"{run}.cpu.csv")
    if os.path.exists(p):
        for ln in open(p):
            parts = ln.strip().split(",")
            if len(parts) >= 2 and parts[1].endswith("%"):
                acc[parts[0]].append(float(parts[1][:-1]))
    return {k: (float(np.mean(v)), float(np.max(v))) for k, v in acc.items()}


def main():
    out = sys.argv[1]
    data = {r: load(out, r) for r in RUNS}
    for k, topic in enumerate(("/fused_bbox", "/tracked_objects")):
        print(f"\n{topic}")
        print(f"  {'pair':8s} {'frames':>13s} {'common':>7s} {'only':>9s} {'same count':>11s} "
              f"{'same objs':>10s} {'identical':>10s} {'p99 move':>9s} {'max move':>9s}")
        for x, y in (("A1", "A2"), ("A1", "B"), ("A2", "B")):
            c = compare(data[x][k], data[y][k])
            print(f"  {x+' vs '+y:8s} {c['frames_a']:6d}/{c['frames_b']:<6d} {c['common']:7d} "
                  f"{c['only_a']:4d}/{c['only_b']:<4d} {c['same_count']:10.1f}% {c['same_objects']:9.1f}% "
                  f"{c['identical']:9.1f}% {c['p99']:8.3f}m {c['max']:8.3f}m")
    print("\npairing counters at the end of each run (existing nodes)")
    for r in RUNS:
        for name, kv in sorted(pairing(out, r).items()):
            print(f"  {r:3s} {name:34s} {kv}")
    print("\ncontainer CPU %, mean / max over the run")
    for r in RUNS:
        for name, (mean, mx) in sorted(cpu(out, r).items()):
            print(f"  {r:3s} {name:34s} {mean:6.1f} / {mx:6.1f}")


if __name__ == "__main__":
    main()
