"""Does a merge/association change cost accuracy? Radar is held out of one track in four, so
the reference is independent of the state being scored -- without that the filter is fitted to
the very measurement it is graded against and every arm looks good.

    python3 scripts/merge_accuracy.py <measurements.pkl> [fused_replay_bag]

The pkl is a `scripts/neighbour_ab.py --dump` file (the position rule the detector runs today);
the bag supplies radar + odometry. Edit ARMS to compare different settings.
"""
import sys, os, numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__))))
import object_ab as oa

BAG = sys.argv[2] if len(sys.argv) > 2 else "/home/avalocal/fused_replay_selfcal_2026-09-08"
fused, radar, odom, R_sl, t_sl = oa.load(BAG)
assert oa.self_check(R_sl, t_sl), "self-check failed; not scoring an unvalidated pipeline"
fused = oa.load_measurements(sys.argv[1], "DropNF")
base = dict(radar_birth=False, cam_gate=oa.CAMERA_GATE_CHI2, ego_yaw_deg=-5.35,
            collect_ab=True, radar_holdout=4)
inf = float("inf")
ARMS = (("as first shipped: unbounded, assoc 6 m", dict(merge_max_dist=inf)),
        ("live now: chi2 4.0 unbounded, assoc 4 m", dict(merge_chi2=4.0, merge_max_dist=inf,
                                                         assoc_max_dist=4.0)),
        ("bound 2.5 m, chi2 9.21, assoc 4 m", dict(assoc_max_dist=4.0)),
        ("bound 2.5 m, chi2 9.21, assoc 6 m (bound only)", dict()),
        ("merge OFF (reference)", dict(merge=False)))
print(f"  {'arm':44s} {'n':>6s} {'raw med':>8s} {'filt med':>9s} {'filt p90':>9s}"
      f" {'jumps>2m':>9s} {'lag':>7s}")
for label, kw in ARMS:
    d = oa.run(fused, radar, odom, R_sl, t_sl, **{**base, **kw})
    ab = d["ab"]
    raw, filt = np.asarray(ab["raw"]), np.asarray(ab["filt"])
    j = np.asarray(ab["jump_filt"]); lag = np.asarray(ab.get("lag", []))
    print(f"  {label:44s} {filt.size:6d} {np.median(raw):8.2f} {np.median(filt):9.2f}"
          f" {np.percentile(filt, 90):9.2f} {100*np.mean(j > 2):8.1f}%"
          f" {np.median(lag) if lag.size else float('nan'):+7.2f}")
