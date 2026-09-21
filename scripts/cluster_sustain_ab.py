#!/usr/bin/env python3
"""Does the 360-degree cluster path keep passed objects alive -- and does it invent ghosts?

Compares two recordings of /perception/objects from the SAME drive, one with
ENABLE_LIDAR_CLUSTERS=true and one without. The gain and the risk are two sides of the same
mechanism, so both are reported:

  GAIN   a track the camera has passed keeps being updated by clusters, so it lives on into the
         side and rear sectors where the camera has nothing. Read the lifetime of tracks that
         reach past 90 degrees of bearing.
  RISK   a cluster has no class. If it latches onto a bush, the track lives on with nothing real
         under it. The tell is the CLUSTER-ONLY TAIL: how long a track survives after its last
         camera update. A few seconds is the point of the feature; tens of seconds is a ghost.

    python3 scripts/cluster_sustain_ab.py OFF_RECORDING ON_RECORDING
"""

import argparse
import glob
import math
import os
from collections import defaultdict

import numpy as np
from mcap_ros2.reader import read_ros2_messages

OBJECTS = "/perception/objects"
CONTRIB_CAMERA, CONTRIB_LIDAR = 1, 2


def load(rec):
    """track_id -> per-observation (t, bearing deg, range, saw_camera_this_frame)."""
    tracks = defaultdict(list)
    for f in sorted(glob.glob(os.path.join(os.path.expanduser(rec), "*.mcap"))):
        for m in read_ros2_messages(f, topics=[OBJECTS]):
            t = m.ros_msg.header.stamp.sec + m.ros_msg.header.stamp.nanosec * 1e-9
            for o in m.ros_msg.objects:
                x, y = o.pose.position.x, o.pose.position.y
                tracks[o.track_id].append(
                    (t, abs(math.degrees(math.atan2(y, x))), math.hypot(x, y),
                     bool(o.contributions_this_frame & CONTRIB_CAMERA)))
    return tracks


def describe(label, tracks):
    life, tails, passed_life, long_tail = [], [], [], 0
    for obs in tracks.values():
        obs.sort()
        # NO minimum-observation filter. Requiring >= 5 observations makes the feature look like a
        # regression: sustaining a weak track past the threshold ADDS it to the sample as a
        # short-lived one, so the count rises and the median falls while the truth is the
        # opposite. Count every track, and read the count together with the lifetime.
        life.append(obs[-1][0] - obs[0][0])
        cam = [o[0] for o in obs if o[3]]
        tail = obs[-1][0] - cam[-1] if cam else obs[-1][0] - obs[0][0]
        tails.append(tail)
        long_tail += tail > 10.0
        if max(o[1] for o in obs) > 90.0:
            passed_life.append(obs[-1][0] - obs[0][0])
    life, tails = np.asarray(life), np.asarray(tails)
    print(f"  {label:14s} tracks {life.size:4d} | lifetime median {np.median(life):5.2f} s "
          f"p90 {np.percentile(life, 90):5.2f} max {life.max():6.2f} | "
          f"reached past 90 deg {len(passed_life):3d}"
          + (f", their lifetime median {np.median(passed_life):5.2f} s" if passed_life else "")
          + f" | cluster-only tail median {np.median(tails):4.2f} s p90 "
            f"{np.percentile(tails, 90):5.2f}, over 10 s: {long_tail}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("off")
    ap.add_argument("on")
    args = ap.parse_args()
    for label, rec in (("clusters OFF", args.off), ("clusters ON", args.on)):
        describe(label, load(rec))
    print("\n  The cluster-only tail is the ghost check: it is the time a track kept being "
          "published after the camera last saw it.")


if __name__ == "__main__":
    main()
