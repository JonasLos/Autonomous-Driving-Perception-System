#!/usr/bin/env python3
"""Item 12's open question: when radar puts a VEHICLE 3+ m farther than the camera does, who is right?

At 40-80 m, 38.9% of radar updates on non-cone tracks put the object more than 3 m behind that
track's own camera range. Radar ranges the body to ~0.1 m but may be seeing another object further
down the bearing; the camera+LiDAR rule takes the nearest depth cluster in the box and may catch
something in front of the car. Neither can referee the other, so this asks the LiDAR directly:
what physically sits on that bearing, at what ranges?

For each disagreement, non-ground LiDAR points within +-1.5 deg of the radar's azimuth (in the
radar frame) are split into range segments (gaps > 1.5 m), and the event is classified:

    ONE OBJECT SPANS BOTH   a segment covers the camera range AND the radar range -- one long object
                            (a truck, a bus): both are right, radar just ranges deeper into it
    TWO OBJECTS             separate segments at both ranges: radar is on something BEHIND the
                            target -> a wrong association, and the track is being pulled away
    CAMERA ONLY             something at the camera range, nothing at the radar range: radar ghost
    RADAR ONLY              something at the radar range, nothing at the camera range: the camera
                            range is wrong and the radar pull is a correction

    python3 scripts/radar_vehicle_truth.py [--measurements rows.pkl]
"""

import argparse
import glob
import os
import sys
from collections import Counter

import numpy as np
from mcap_ros2.reader import read_ros2_messages

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import object_ab as oa  # noqa: E402

from object_fusion.ground_segmentation import GroundSegmenter    # noqa: E402

LIDAR = "/lidar_tc/velodyne_points"
HALF_BEAM_DEG = 1.5
MATCH_M = 1.5


def decode(c):
    offs = {f.name: f.offset for f in c.fields}
    a = np.frombuffer(bytes(c.data), dtype=np.uint8).reshape(-1, c.point_step)
    g = lambda n: a[:, offs[n]:offs[n] + 4].copy().view(np.float32).ravel()
    p = np.stack([g("x"), g("y"), g("z"), g("intensity")], 1).astype(np.float64)
    return p[np.isfinite(p).all(1)]


def segments(r_sorted, gap=1.5):
    if r_sorted.size == 0:
        return []
    cuts = np.flatnonzero(np.diff(r_sorted) > gap)
    starts = np.r_[0, cuts + 1]
    ends = np.r_[cuts, r_sorted.size - 1]
    return [(float(r_sorted[a]), float(r_sorted[b]), int(b - a + 1)) for a, b in zip(starts, ends)]


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--source", default="/home/avalocal/selfcal_loc_2026-09-08_11-47-43")
    ap.add_argument("--replay", default="/home/avalocal/fused_replay_selfcal_2026-09-08")
    ap.add_argument("--measurements",
                    default=os.path.expanduser("~/fusion_data/measurements/rows_gate2.pkl"))
    ap.add_argument("--camera-gate", action="store_true",
                    help="run with the camera-referenced radar range gate on")
    args = ap.parse_args()

    fused, radar, odom, R_sl, t_sl = oa.load(args.replay)
    assert oa.self_check(R_sl, t_sl), "self-check failed"
    fused = oa.load_measurements(args.measurements, "DropNF")
    d = oa.run(fused, radar, odom, R_sl, t_sl, ego_yaw_deg=-5.35, radar_birth=False,
               cam_gate=oa.CAMERA_GATE_CHI2, radar_camera_gate=args.camera_gate)
    events = []
    for t, r_rad, az_rad, cam_l, cls in d["counts"].get("radar_vs_cam_events", []):
        if "cone" in cls:
            continue
        r_cam = float(np.hypot(*(R_sl @ cam_l + t_sl)))
        if 40.0 <= r_cam < 80.0 and r_rad - r_cam > 3.0:
            events.append((t, r_rad, az_rad, r_cam, cls))
    print(f"[events] {len(events)} radar updates on non-cone tracks at 40-80 m with radar > 3 m "
          f"behind the camera")
    if not events:
        return

    need = sorted({round(e[0], 2) for e in events})
    sweeps = {}
    seg = GroundSegmenter(backend="patchworkpp", sensor_height=2.37, max_range=120.0)
    for f in sorted(glob.glob(os.path.join(args.source, "*.mcap"))):
        for m in read_ros2_messages(f, topics=[LIDAR]):
            h = m.ros_msg.header
            t = h.stamp.sec + h.stamp.nanosec * 1e-9
            i = np.searchsorted(need, t)
            near = [need[j] for j in (i - 1, i) if 0 <= j < len(need)]
            if not near or min(abs(t - n) for n in near) > 0.06:
                continue
            p = decode(m.ros_msg)
            ground = seg.segment(p[:, :3], p[:, 3] / 255.0)
            q = p[~ground, :3]
            q = q[(q[:, 2] > -2.2) & (q[:, 2] < 0.9)]
            ps = q[:, :2] @ R_sl.T + t_sl                         # lidar -> radar frame
            sweeps[t] = (np.hypot(ps[:, 0], ps[:, 1]), np.degrees(np.arctan2(ps[:, 1], ps[:, 0])))
    st = np.array(sorted(sweeps))

    verdict = Counter()
    by_class = Counter()
    for t, r_rad, az_rad, r_cam, cls in events:
        j = int(np.argmin(np.abs(st - t)))
        if abs(st[j] - t) > 0.06:
            verdict["no LiDAR sweep near the event"] += 1
            continue
        rng, az = sweeps[st[j]]
        on_beam = np.sort(rng[np.abs(az - az_rad) <= HALF_BEAM_DEG])
        segs = segments(on_beam)
        at_cam = [s for s in segs if s[0] - MATCH_M <= r_cam <= s[1] + MATCH_M]
        at_rad = [s for s in segs if s[0] - MATCH_M <= r_rad <= s[1] + MATCH_M]
        if at_cam and at_rad and set(at_cam) & set(at_rad):
            v = "ONE OBJECT SPANS BOTH (radar ranges deeper into it)"
        elif at_cam and at_rad:
            v = "TWO OBJECTS (radar on something behind -> wrong association)"
        elif at_cam:
            v = "CAMERA ONLY (nothing at the radar range -> radar ghost)"
        elif at_rad:
            v = "RADAR ONLY (nothing at the camera range -> the camera is wrong)"
        else:
            v = "NEITHER (the LiDAR sees nothing on the bearing at either range)"
        verdict[v] += 1
        by_class[(cls, v.split(" (")[0])] += 1

    n = sum(verdict.values())
    print(f"\n  {'what the LiDAR sees on that bearing':68s} {'events':>7s}")
    for v, c in verdict.most_common():
        print(f"  {v:68s} {c:5d}  {100 * c / n:5.1f}%")
    print("\n  by class:")
    for (cls, v), c in sorted(by_class.items(), key=lambda kv: -kv[1])[:10]:
        print(f"    {cls:12s} {v:24s} {c:4d}")


if __name__ == "__main__":
    main()
