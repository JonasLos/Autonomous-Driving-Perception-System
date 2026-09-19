#!/usr/bin/env python3
"""Item 10: measure the coast budgets that track_store.MAX_COAST_S still marks INVENTED.

A track survives a detection gap for its coast budget (camera 0.5 s, radar 0.3 s, both 1.0 s) and
is deleted after it. If an object's own detections routinely come back after a LONGER gap, the
track dies and is re-born with a new id -- churn a planner sees as one object vanishing and
another appearing. The budget's own comment names the measurement: the distribution of
re-acquisition gaps in the sensors' OWN identity -- ByteTrack id for the camera, ESR track_id for
the radar -- and its 95th percentile.

A gap only counts as a re-acquisition when the same id comes back within `--max-gap` seconds;
longer absences are an object leaving and (for recycled ESR ids) something else arriving. For the
radar a return must also be within a plausible range of where the id was.

    python3 scripts/coast_budget_ab.py [--measurements rows.pkl ...] [--source BAG]
"""

import argparse
import os
import pickle
import sys
from collections import defaultdict

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from radar_false_alarm_ab import load_radar  # noqa: E402  (ESR tracks with their ids)

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..",
                                "src", "object_fusion"))
from object_fusion.track_store import MAX_COAST_S  # noqa: E402


def camera_gaps(pkl, arm, max_gap):
    """Gaps between consecutive published measurements of the same ByteTrack id. DropNF is what
    the aggregator actually receives, so the depth gate's withheld frames count as gaps."""
    by_id = defaultdict(list)
    with open(pkl, "rb") as fh:
        for r in pickle.load(fh):
            if r.get(arm) is not None and r.get("id"):
                by_id[r["id"]].append(r["t"])
    gaps = []
    for ts in by_id.values():
        ts = np.sort(np.asarray(ts))
        d = np.diff(ts)
        gaps.extend(d[(d > 0.15) & (d <= max_gap)])        # > 1.5 frames: a real miss
    return np.asarray(gaps), len(by_id)


def radar_gaps(bag, max_gap):
    radar, _odom, _R, _t = load_radar(bag)
    last = {}
    gaps = []
    for t, ids, rng, _ang, _rr in radar:
        for i, tid in enumerate(ids):
            prev = last.get(tid)
            if prev is not None:
                dt = t - prev[0]
                plausible = abs(rng[i] - prev[1]) <= 5.0 + 40.0 * dt
                if 0.05 < dt <= max_gap and plausible:
                    gaps.append(dt)
            last[tid] = (t, float(rng[i]))
    return np.asarray(gaps)


def describe(label, gaps, budget):
    if gaps.size == 0:
        print(f"  {label:34s} no re-acquisitions")
        return
    p50, p90, p95 = np.percentile(gaps, [50, 90, 95])
    over = 100 * np.mean(gaps > budget)
    print(f"  {label:34s} n={gaps.size:6d}  median {p50:.2f} s  p90 {p90:.2f}  p95 {p95:.2f}   "
          f"budget {budget:.2f} s -> {over:4.1f}% of re-acquisitions come back TOO LATE")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--measurements", nargs="+",
                    default=[os.path.expanduser("~/fusion_data/measurements/rows_gate2.pkl")])
    ap.add_argument("--source", default="/home/avalocal/selfcal_loc_2026-09-08_11-47-43")
    ap.add_argument("--arm", default="DropNF")
    ap.add_argument("--max-gap", type=float, default=3.0)
    args = ap.parse_args()

    print(f"=== coast budgets (track_store.MAX_COAST_S = {MAX_COAST_S}) ===")
    print("  Only gaps longer than 1.5 frames, where the SAME id comes back within "
          f"{args.max_gap:.0f} s.\n")
    for pkl in args.measurements:
        g, n_ids = camera_gaps(pkl, args.arm, args.max_gap)
        describe(f"camera, {os.path.basename(pkl)} ({n_ids} ids)", g, MAX_COAST_S["camera"])
    if args.source:
        describe(f"radar, {os.path.basename(args.source.rstrip('/'))}",
                 radar_gaps(args.source, args.max_gap), MAX_COAST_S["radar"])


if __name__ == "__main__":
    main()
