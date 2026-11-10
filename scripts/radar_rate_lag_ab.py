#!/usr/bin/env python3
"""Does the ESR's range-rate lag the geometry? Measured on static targets, from raw radar.

For a target standing still, ``s = range_rate + u . v_ego`` should be 0. If the reading is late
by tau it is ``-tau * X`` instead, where ``X = v^2 sin^2(az) / r`` is how fast the true range-rate is
changing. So ``s`` against X has slope ``-tau``, and it is only visible while the bearing sweeps,
which is exactly while the car is passing something.

    python3 scripts/radar_rate_lag_ab.py BAG_DIR [BAG_DIR ...]

Each bag needs /delphi_esr_interface/radar/tracks and /novatel/oem7/odom_grid or /odom. A track is
treated as static if its median |s| is under 0.9 m/s, which excludes the worst lagged samples too,
so the numbers lean low. Three things are printed:

  * median s by X: the size of the effect;
  * s after shifting the range-rate by tau, against the bearing at t: where the lag goes to zero;
  * s at a fixed bearing by range: a lag grows as range shrinks, an azimuth scale error does not.

Needs PYTHONPATH with src/object_fusion, src/radar_ros and src/perception_common.
"""
import argparse
import collections
import glob
import math
import multiprocessing
import os

import numpy as np
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory

from object_fusion import frames
from radar_ros.radar_geometry import polar_to_cartesian

RADAR_TOPIC = "/delphi_esr_interface/radar/tracks"
ODOM_TOPICS = ("/novatel/oem7/odom_grid", "/novatel/oem7/odom")
#: tf_static lidar_tc -> delphi_esr_radar yaw; the measured value is -5.47 and this is not sensitive to it.
RADAR_YAW_DEG = -5.44
BINS = ((0, .1), (.1, .3), (.3, 1), (1, 2), (2, 4), (4, 8), (8, 30))


def load(path):
    files = glob.glob(os.path.join(path, "*.mcap"))
    if not files:
        return None
    odom, radar = collections.defaultdict(list), []
    with open(files[0], "rb") as fh:
        reader = make_reader(fh, decoder_factories=[DecoderFactory()])
        for _, ch, _, m in reader.iter_decoded_messages(topics=[RADAR_TOPIC, *ODOM_TOPICS]):
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            if ch.topic == RADAR_TOPIC:
                radar += [(t, x.track_id, x.range, x.angle, x.range_rate)
                          for x in m.tracks if x.track_status > 0 and x.range > 1]
            else:
                tw = m.twist.twist
                odom[ch.topic].append((t, tw.linear.x, tw.linear.y, tw.angular.z))
    rows = next((odom[k] for k in ODOM_TOPICS if odom[k]), None)
    return (sorted(rows), radar) if rows and radar else None


def segments(odom, radar):
    """Per ESR track: stretches without a gap or range jump, static ones only."""
    yaw = math.radians(RADAR_YAW_DEG)
    r_ls = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
    o = np.array(odom)
    by_id = collections.defaultdict(list)
    for row in radar:
        by_id[row[1]].append(row)
    out = []
    for rows in by_id.values():
        a = np.array(sorted(rows))
        cuts = np.flatnonzero((np.diff(a[:, 0]) > .3) | (np.abs(np.diff(a[:, 2])) > 4)) + 1
        for g in np.split(a, cuts):
            if len(g) < 12:
                continue
            t, rng, ang, rr = g[:, 0], g[:, 2], g[:, 3], g[:, 4]
            xs, ys = polar_to_cartesian(rng, ang)
            az = np.arctan2(ys, xs)
            i = np.clip(np.searchsorted(o[:, 0], t), 0, len(o) - 1)
            ve = np.array(frames.ego_to_lidar(o[i, 1:3])) @ r_ls
            if np.median(np.abs(rr + np.cos(az) * ve[:, 0] + np.sin(az) * ve[:, 1])) <= .9:
                out.append((t, rng, az, rr, ve, np.hypot(ve[:, 0], ve[:, 1]), o[i, 3]))
    return out


def collect(segs, tau):
    out = []
    for t, rng, az, rr, ve, v, w in segs:
        q = t + tau
        ok = (q >= t[0]) & (q <= t[-1])
        if ok.sum() < 3:
            continue
        s = np.interp(q, t, rr) + np.cos(az) * ve[:, 0] + np.sin(az) * ve[:, 1]
        m = ok & (np.abs(w) < .03) & (v > 3) & (rng > 2) & (rng < 60)
        x = v ** 2 * np.sin(az) ** 2 / rng.clip(1)
        out.append(np.c_[x[m], s[m], v[m], rng[m], np.degrees(np.abs(az[m]))])
    return np.vstack(out)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("bags", nargs="+")
    args = ap.parse_args()
    with multiprocessing.Pool(min(8, len(args.bags))) as pool:
        data = [d for d in pool.imap_unordered(load, args.bags) if d]
    segs = [s for odom, radar in data for s in segments(odom, radar)]
    z = collect(segs, 0.0)
    x, s = z[:, 0], z[:, 1]
    print(f"bags {len(data)}  samples {len(z)}")
    print("X [m/s^2]      n     median s [m/s]")
    for lo, hi in BINS:
        m = (x >= lo) & (x < hi)
        if m.sum() > 20:
            print(f"{lo:4}-{hi:<4} {m.sum():7d}   {np.median(s[m]):+.3f}")
    print("\nshift tau [s]   slope of s on X   median |s| for X > 1")
    for tau in (0, .05, .1, .15, .2, .25, .3, .4):
        q = collect(segs, tau)
        m = (q[:, 0] > .3) & (q[:, 0] < 20)
        print(f"  {tau:4.2f}        {(q[m, 0] @ q[m, 1]) / (q[m, 0] @ q[m, 0]):+.3f}          "
              f"{np.median(np.abs(q[q[:, 0] > 1, 1])):.3f}")
    print("\nfixed |azimuth|, by range (5-12 m/s)")
    for a0, a1 in ((10, 20), (20, 35), (35, 60)):
        for lo, hi in ((3, 8), (8, 15), (15, 30), (30, 60)):
            m = (z[:, 4] >= a0) & (z[:, 4] < a1) & (z[:, 3] >= lo) & (z[:, 3] < hi) & (z[:, 2] > 5) & (z[:, 2] < 12)
            if m.sum() > 25:
                print(f"  az {a0}-{a1}  r {lo}-{hi}  n={m.sum():5d}  median s {np.median(s[m]):+.3f}")


if __name__ == "__main__":
    main()
