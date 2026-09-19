#!/usr/bin/env python3
"""Item 10: is a longer camera coast budget worth it?

scripts/coast_budget_ab.py measured that 13-28% of camera re-acquisitions (the same ByteTrack id
coming back) arrive after the 0.5 s camera budget has already deleted the track -- so the object
vanishes and re-appears under a new id. A longer budget keeps the id; the price is a track that
lingers after its object is really gone, and more room for a re-birth beside a coasting track.
This scores both sides on the reference drive, with the production rules and radar held out:

    births          fewer for the same measurements = better continuity
    orphaned        measurements with no track within 3 m (objects lost)
    dup             published tracks within 2 m of another (includes genuine close cone pairs, so
                    read it as RELATIVE between arms)
    held-out range error, jumps

    python3 scripts/coast_sweep_ab.py [--measurements rows.pkl]
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import object_ab as oa  # noqa: E402
from object_fusion import track_store  # noqa: E402

ARMS = (("LIVE: camera 0.5 s, both 1.0 s", 0.5, 1.0),
        ("camera 1.0 s", 1.0, 1.0),
        ("camera 1.5 s, both 1.5 s", 1.5, 1.5),
        ("camera 2.0 s, both 2.0 s", 2.0, 2.0))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--replay", default="/home/avalocal/fused_replay_selfcal_2026-09-08")
    ap.add_argument("--measurements",
                    default=os.path.expanduser("~/fusion_data/measurements/rows_gate2.pkl"))
    args = ap.parse_args()
    fused, radar, odom, R_sl, t_sl = oa.load(args.replay)
    assert oa.self_check(R_sl, t_sl), "self-check failed"
    fused = oa.load_measurements(args.measurements, "DropNF")
    base = dict(radar_birth=False, cam_gate=oa.CAMERA_GATE_CHI2, ego_yaw_deg=-5.35)
    live = dict(track_store.MAX_COAST_S)

    print(f"\n  {'arm':30s} {'births':>7s} {'orphaned':>9s} {'dup':>6s} {'shared':>7s} | "
          f"{'held-out med':>12s} {'p90':>6s} {'jumps>2m':>9s}")
    try:
        for label, cam, both in ARMS:
            track_store.MAX_COAST_S.update(camera=cam, both=both)
            d = oa.run(fused, radar, odom, R_sl, t_sl, count_objects=True, **base)
            pf = np.array(d["counts"]["per_frame"])
            orph = pf[:, 2].sum() / max(pf[:, 0].sum(), 1)
            dup = pf[:, 3].sum() / max(pf[:, 1].sum(), 1)
            shr = pf[:, 4].sum() / max(pf[:, 1].sum(), 1)
            h = oa.run(fused, radar, odom, R_sl, t_sl, collect_ab=True, radar_holdout=4, **base)
            f, j = np.asarray(h["ab"]["filt"]), np.asarray(h["ab"]["jump_filt"])
            print(f"  {label:30s} {d['counts']['births']:7d} {100 * orph:8.1f}% {100 * dup:5.1f}% "
                  f"{100 * shr:6.1f}% | {np.median(f):12.2f} {np.percentile(f, 90):6.2f} "
                  f"{100 * np.mean(j > 2):8.1f}%")
    finally:
        track_store.MAX_COAST_S.clear()
        track_store.MAX_COAST_S.update(live)


if __name__ == "__main__":
    main()
