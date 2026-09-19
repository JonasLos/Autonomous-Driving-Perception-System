#!/usr/bin/env python3
"""Item 2's remainder: the camera NIS is inconsistent -- would trusting the camera more help?

On the current measurements the camera innovation is a heavy-tailed mixture: NIS median 0.18
against the 1.39 a consistent 2-DOF filter would show (the BULK is far smaller than the filter
assumes), but mean 9.0 against 2 (the TAIL is far heavier). No single sigma fits both. The
chi-square gate already handles the tail, so the practical question is whether scaling the
camera's measurement sigma DOWN -- trusting the bulk -- makes the published position better.

Scored with radar held out of one track in four, so the reference stays independent of the state:

    python3 scripts/filter_scale_ab.py [--measurements rows.pkl]
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import object_ab as oa  # noqa: E402

ARMS = (
    ("LIVE: sigma_along x1.0, sigma_cross x1.0", dict()),
    ("sigma_along x0.7", dict(sigma_along_scale=0.7)),
    ("sigma_along x0.5", dict(sigma_along_scale=0.5)),
    ("sigma_along x0.35", dict(sigma_along_scale=0.35)),
    ("sigma_along x0.5, sigma_cross x0.7", dict(sigma_along_scale=0.5, sigma_cross_scale=0.7)),
)


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
    base = dict(radar_birth=False, cam_gate=oa.CAMERA_GATE_CHI2, ego_yaw_deg=-5.35,
                collect_ab=True, radar_holdout=4, collect_nis=True)
    print(f"\n  {'arm':40s} {'n':>4s} {'med':>6s} {'p90':>6s} {'jumps>2m':>9s} {'lag':>6s} | "
          f"{'cam NIS med':>11s} {'mean':>6s} {'gated':>6s}")
    for label, kw in ARMS:
        d = oa.run(fused, radar, odom, R_sl, t_sl, **{**base, **kw})
        ab, c = d["ab"], d["counts"]
        f = np.asarray(ab["filt"])
        j = np.asarray(ab["jump_filt"])
        lag = np.asarray(ab.get("lag", []))
        nis = np.asarray(d["nis"].get("camera_lidar", []))
        gated = c.get("gated_out", 0) / max(c.get("gated_out", 0) + c.get("cam_updates", 0), 1)
        print(f"  {label:40s} {f.size:4d} {np.median(f):6.2f} {np.percentile(f, 90):6.2f} "
              f"{100 * np.mean(j > 2):8.1f}% {np.median(lag) if lag.size else float('nan'):+6.2f} | "
              f"{np.median(nis):11.3f} {nis.mean():6.2f} {100 * gated:5.1f}%")
    print("\n  Consistent 2-DOF: NIS median 1.39, mean 2.0. Accuracy columns are the ones that "
          "matter; NIS says whether the filter's own uncertainty can be believed.")


if __name__ == "__main__":
    main()
