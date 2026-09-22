#!/usr/bin/env python3
"""Does a box holding more than one depth cluster mean the range is wrong? (items 4 and 12)

The production rule keeps the NEAREST depth cluster in a 2D box. That is right when the box holds
one object and wrong when a nearer object overlaps it in image space -- the range then belongs to
the occluder. The car-park drive's remaining spikes are all along the ray and concentrate on cars
at 40-60 m (11%) and 60+ m (30%), which is the signature.

`scripts/neighbour_ab.py --dump` now records what each box contained (`n_clusters`, `fg_frac`,
`bg_gap`, `bg_frac`). This scores the candidate flag against radar range:

    python3 scripts/occlusion_ab.py ~/fusion_data/measurements/rows_adps115045_occ.pkl

A flag is only worth having if the flagged population is measurably worse than the unflagged one.
If it is not, say so and stop -- the cheapest outcome here is a refutation.
"""

import argparse
import math
import pickle
import sys

import numpy as np

sys.path.insert(0, "scripts")
from ground_ab import _rng as radar_frame_range   # range from the RADAR's origin, not the LiDAR's

ARM = "DropNF"                      # the live rule
BANDS = ((0, 15), (15, 25), (25, 40), (40, 60), (60, 80), (80, 200))


def rng(p):
    """Range from the vehicle, for BANDING only."""
    return math.hypot(p[0], p[1])


def err_of(r, arm=ARM):
    """Signed range error against the radar return matched to this detection, or None.

    The comparison has to happen in the RADAR's frame: it sits about 3 m ahead of the LiDAR, so
    comparing a LiDAR-frame range against a radar range charges that offset to every detection
    (it read a +1.11 m median and 79.5% of detections beyond 2 m before this was fixed).
    """
    if r.get(arm) is None or r.get("radar") is None:
        return None
    return radar_frame_range(r[arm]) - r["radar"]


def describe(label, errs):
    e = np.asarray(errs, dtype=float)
    if e.size < 5:
        return f"  {label:34s} n={e.size:4d}   (too few)"
    med = float(np.median(e))
    sd = 1.4826 * float(np.median(np.abs(e - med)))
    return (f"  {label:34s} n={e.size:4d}  median {med:+6.2f} m  robust sd {sd:5.2f}  "
            f"|err|>2 m {100 * float(np.mean(np.abs(e) > 2.0)):5.1f}%  "
            f"|err|>5 m {100 * float(np.mean(np.abs(e) > 5.0)):5.1f}%")


def flagged(r, min_bg_frac, min_gap):
    return (int(r.get("n_clusters") or 0) > 1
            and (r.get("bg_frac") or 0.0) >= min_bg_frac
            and (r.get("bg_gap") or 0.0) >= min_gap)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dump")
    ap.add_argument("--bg-frac", type=float, default=0.15,
                    help="a background cluster smaller than this fraction of the box is noise")
    ap.add_argument("--gap", type=float, default=2.0,
                    help="metres the background cluster must sit behind the kept one")
    args = ap.parse_args()
    rows = pickle.load(open(args.dump, "rb"))
    scored = [r for r in rows if err_of(r) is not None]
    print(f"{args.dump}: {len(rows)} detections, {len(scored)} with a radar match on the "
          f"{ARM} arm\n")

    multi = [r for r in scored if int(r.get("n_clusters") or 0) > 1]
    print(f"  boxes with more than one depth cluster: {len(multi)} of {len(scored)} "
          f"({100 * len(multi) / max(len(scored), 1):.1f}%)\n")

    print("SPLIT BY THE RAW STRUCTURE (no thresholds):")
    print(describe("one cluster", [err_of(r) for r in scored
                                   if int(r.get("n_clusters") or 0) <= 1]))
    print(describe("two or more clusters", [err_of(r) for r in multi]))

    print(f"\nSPLIT BY THE CANDIDATE FLAG (bg_frac >= {args.bg_frac}, gap >= {args.gap} m):")
    yes = [r for r in scored if flagged(r, args.bg_frac, args.gap)]
    no = [r for r in scored if not flagged(r, args.bg_frac, args.gap)]
    print(describe("flagged (occluded)", [err_of(r) for r in yes]))
    print(describe("not flagged", [err_of(r) for r in no]))

    print("\nBY BAND, flagged vs not (median / |err|>2 m / n):")
    for lo, hi in BANDS:
        band = [r for r in scored if lo <= rng(r["pub"]) < hi]
        if not band:
            continue
        cells = []
        for name, sel in (("flag", [r for r in band if flagged(r, args.bg_frac, args.gap)]),
                          ("no", [r for r in band if not flagged(r, args.bg_frac, args.gap)])):
            e = np.asarray([err_of(r) for r in sel], dtype=float)
            cells.append(f"{name} " + (f"{np.median(e):+5.2f} / {100*np.mean(np.abs(e)>2):4.1f}% "
                                       f"/ {e.size:3d}" if e.size >= 5 else f"({e.size:3d})"))
        print(f"  {lo:3d}-{hi:3d} m   " + "   ".join(cells))

    print("\nTHRESHOLD SWEEP -- share of the >2 m errors the flag catches, and what it costs:")
    big = [r for r in scored if abs(err_of(r)) > 2.0]
    print(f"  {len(big)} detections are more than 2 m from radar "
          f"({100 * len(big) / max(len(scored), 1):.1f}% of the scored ones)")
    print(f"  {'bg_frac':>8s} {'gap':>5s} {'flagged':>8s} {'of the >2 m':>12s} "
          f"{'of the rest':>12s} {'precision':>10s}")
    for bf in (0.05, 0.10, 0.15, 0.25, 0.40):
        for gap in (1.0, 2.0, 4.0, 8.0):
            f_big = sum(1 for r in big if flagged(r, bf, gap))
            f_all = sum(1 for r in scored if flagged(r, bf, gap))
            f_ok = f_all - f_big
            rest = len(scored) - len(big)
            print(f"  {bf:8.2f} {gap:5.1f} {f_all:8d} "
                  f"{100 * f_big / max(len(big), 1):11.1f}% "
                  f"{100 * f_ok / max(rest, 1):11.1f}% "
                  f"{100 * f_big / max(f_all, 1):9.1f}%")


if __name__ == "__main__":
    main()
