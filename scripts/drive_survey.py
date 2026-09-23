#!/usr/bin/env python3
"""Every scored drive, side by side: do the live rules hold, and can the drive referee at all?

`neighbour_ab.py --dump` writes one pickle per drive. This reads them together and answers three
questions that only make sense across drives:

  DOES THE RULE HOLD     spike rate (second difference of one tracker id > 2 m) for the old
                         fusion_node rule (A25) and the live one (DropNF). The live rule should
                         win on every drive, not just the one it was tuned on.
  CAN THIS DRIVE JUDGE   radar is the ruler for range, and on a crowded drive one return gets
                         matched to several detections -- then a range number from that drive
                         says as much about the matching as about the stack. Reported as
                         detections per DISTINCT matched return, with the bearing offset.
  WHAT IS IN THE BOX     share of detections whose box holds real support behind the kept depth
                         cluster (the occlusion flag, HANDOFF item 4), and how much of the box
                         the kept cluster holds.

    python3 scripts/drive_survey.py ~/fusion_data/measurements/rows_*.pkl
"""

import glob
import pickle
import sys
from collections import defaultdict

import numpy as np

sys.path.insert(0, "scripts")
from ground_ab import _rng as radar_frame_range
from neighbour_ab import windows as neighbour_windows


def spikes(rows, arm, thresh=2.0):
    """Spike rate on neighbour_ab's OWN definition, so these numbers sit beside the ones the
    harness has already recorded: deviation of a detection's range from its same-id neighbours
    interpolated in time, measured in the radar frame, neighbours within 0.25 s.

    Do not substitute a plain second difference here. It is a different statistic (it does not
    interpolate, and in the LiDAR frame it is a different range), and it reads 2-4 points apart
    on these drives -- close enough to look like the same number and not be one.
    """
    w = neighbour_windows(rows, arm)
    if not w:
        return float("nan"), 0
    dev = np.abs(np.array([x[1] for x in w]))
    return 100.0 * float((dev > thresh).mean()), dev.size


def main(paths):
    print(f"  {'drive':16s} {'dets':>6s} {'A25':>7s} {'DropNF':>7s} | {'dets/return':>11s} "
          f"{'bearing':>8s} | {'occluded':>8s} {'kept frac':>9s} | {'range err vs radar':>19s}")
    for p in paths:
        try:
            rows = pickle.load(open(p, "rb"))
        except Exception as e:
            print(f"  {p}: {e}")
            continue
        name = p.split("rows_")[-1].replace(".pkl", "")
        a25, _ = spikes(rows, "A25")
        drop, _ = spikes(rows, "DropNF")

        sc = [r for r in rows if r.get("radar") is not None]
        per, dets = defaultdict(set), defaultdict(int)
        for r in sc:
            per[r["t"]].add(round(r["radar"], 3))
            dets[r["t"]] += 1
        reuse = np.median([dets[t] / max(len(per[t]), 1) for t in per]) if per else float("nan")
        daz = [abs(r["radar_az"] - r["obj_az"]) for r in sc if r.get("radar_az") is not None]

        occ = [r for r in rows if r.get("n_clusters")]
        flagged = sum(1 for r in occ if (r.get("bg_frac") or 0) >= 0.15
                      and (r.get("bg_gap") or 0) >= 2.0)
        fg = np.median([r["fg_frac"] for r in occ]) if occ else float("nan")

        err = np.array([radar_frame_range(r["DropNF"]) - r["radar"] for r in sc
                        if r.get("DropNF")])
        e = (f"{np.median(err):+5.2f} m, {100 * np.mean(np.abs(err) > 2):4.1f}% > 2 m"
             if err.size else "-")
        print(f"  {name:16s} {len(rows):6d} {a25:6.1f}% {drop:6.1f}% | {reuse:11.2f} "
              f"{np.median(daz) if daz else float('nan'):7.2f}d | "
              f"{100 * flagged / max(len(occ), 1):7.1f}% {fg:9.2f} | {e:>19s}")
    print("\n  dets/return at 1.00 means every detection had its own radar return; above ~1.2 the")
    print("  drive cannot referee range on its own. kept frac is how much of the box the nearest")
    print("  depth cluster holds -- near 1.0 the box is one object, near 0 it is fragmented.")


if __name__ == "__main__":
    args = sys.argv[1:]
    main(sorted(args if args else glob.glob(
        "/home/avalocal/fusion_data/measurements/rows_*.pkl")))
