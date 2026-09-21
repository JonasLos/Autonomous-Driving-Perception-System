#!/usr/bin/env python3
"""Why do static objects appear to move when the vehicle turns? (the false velocity arrows)

The reference drive is ground truth: every object on it is static, so ANY track speed is the
filter's own error, and an arrow in RViz (drawn above 1 m/s) is always false. Measured on the
published output, the error tracks the ego YAW RATE and not ego speed:

    ego yaw rate  < 0.02 rad/s   arrows  4-6%      correlation of speed with yaw rate  +0.36
                  > 0.10 rad/s   arrows 34-41%                        with ego speed   -0.07

This runs the production filter over the same measurements with one candidate cause changed at a
time, and reports apparent speed binned by yaw rate. The cause is whatever flattens the bins.

    python3 scripts/velocity_truth_ab.py [--measurements rows.pkl]

Candidates it can switch (a third, an un-deskewed LiDAR sweep, needs a re-dump and is not here):
  * the IMU->LiDAR lever arm, which the aggregator never passes to TwistBuffer.increment, so the
    sensor's own omega x r motion is missing from every prediction;
  * a timing offset between the odometry and the sensors, which aliases yaw rate into position.
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import object_ab as oa  # noqa: E402

#: Vendor geometry: the IMU sits at (-0.658, +0.159) in lidar_tc, so the LiDAR is this far from the
#: IMU in vehicle axes. Same quantity as HANDOFF to-do 13, which is still open -- the sweep below
#: reports how sensitive the answer is to it.
LEVER_IMU_TO_LIDAR = (0.670, -0.097)

BINS = ((0.0, 0.02, "straight"), (0.02, 0.05, ""), (0.05, 0.10, ""), (0.10, 0.50, "turning"))


def summarise(label, rows):
    sp, omega = rows[:, 0], rows[:, 3]
    cells = []
    for lo, hi, _ in BINS:
        m = (omega >= lo) & (omega < hi)
        cells.append(f"{100 * np.mean(sp[m] > 1.0):5.1f}%" if m.sum() > 50 else "    -  ")
    corr = float(np.corrcoef(sp, omega)[0, 1])
    print(f"  {label:46s} {rows.shape[0]:6d} " + " ".join(cells)
          + f"  {np.median(sp):6.2f} {corr:+7.3f}")


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
                collect_speed=True)

    arms = [("LIVE (no lever arm, no offset)", dict())]
    arms += [(f"IMU->LiDAR lever arm {LEVER_IMU_TO_LIDAR}", dict(pred_lever=LEVER_IMU_TO_LIDAR))]
    arms += [(f"lever arm x{k:g} (sensitivity)",
              dict(pred_lever=(LEVER_IMU_TO_LIDAR[0] * k, LEVER_IMU_TO_LIDAR[1] * k)))
             for k in (2.0, 4.0)]
    arms += [(f"odometry stamp offset {o * 1e3:+.0f} ms", dict(odom_offset=o))
             for o in (-0.04, 0.02)]
    # The measurement itself jitters while the bearing sweeps (0.77 -> 3.0 m/s straight to
    # turning, mostly cross-ray). Widen the cross-ray sigma by k * omega * r * frame period.
    arms += [(f"turn-aware cross-ray sigma, k={k:g}", dict(turn_sigma_k=k))
             for k in (0.5, 1.0, 2.0)]

    print(f"\n  every object on this drive is STATIC: any speed is error, any arrow is false\n")
    print(f"  {'arm':46s} {'n':>6s} " + " ".join(f"{lo:.2f}-{hi:.2f}" for lo, hi, _ in BINS)
          + f"  {'median':>6s} {'corr':>7s}")
    print(f"  {'':46s} {'':>6s} " + " ".join(f"{lab:>9s}" for _, _, lab in BINS))
    for label, kw in arms:
        d = oa.run(fused, radar, odom, R_sl, t_sl, **{**base, **kw})
        rows = np.concatenate([np.asarray(x) for x in d["speed"].values()])
        summarise(label, rows)
    print("\n  Columns are the share of published static objects an arrow would be drawn on "
          "(> 1 m/s), by ego yaw rate. The cause is whatever flattens 'turning' toward 'straight'.")


if __name__ == "__main__":
    main()
