#!/usr/bin/env python3
"""To-do item 8: would radar-only track birth publish false alarms, and how many?

Radar-only birth is gated off because nobody has measured its false-alarm rate. This measures a
proxy for it. Where the camera and radar overlap, the camera+LiDAR detector is a reasonable
referee: a radar object the camera NEVER sees over its whole life is most likely clutter
(guardrail, sign, manhole cover). Outside the overlap there is no referee, so this also counts how
many objects radar-only birth would ADD there -- the benefit side of the same decision.

    python3 scripts/radar_false_alarm_ab.py [--source BAG] [--measurements rows.pkl]

A radar object is followed by its ESR track_id. A new life starts after a 0.5 s gap or a range jump
the object could not make, because the ESR recycles ids. A life "would be born" if it passes the
production radar-only confirmation (track_store.may_confirm): at least 8 observations, at least
80% of them moving over the ground (compensated range rate >= RADAR_ONLY_MIN_SPEED). The track
store's other conditions -- slot continuity and log-odds >= 3 -- follow from those.

THE NUMBER IS AN UPPER BOUND on the false-alarm rate. "Never confirmed by the camera" also counts
real objects YOLO has no class for, objects hidden from the camera but not the radar, and
anything the depth gate withheld. It cannot count a false alarm the camera happens to agree with.
"""

import argparse
import glob
import math
import os
import pickle
import sys
from collections import defaultdict

import numpy as np
from mcap_ros2.reader import read_ros2_messages

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import object_ab as oa  # noqa: E402  (also puts the perception packages on the path)

from object_fusion import frames                                  # noqa: E402
from object_fusion.projection import project_to_pixels             # noqa: E402
from object_fusion.track_store import RADAR_ONLY_MIN_SPEED          # noqa: E402
from object_fusion.tracker import compensated_range_rate           # noqa: E402
from radar_ros.radar_geometry import gate_tracks, polar_to_cartesian  # noqa: E402

RADAR = "/delphi_esr_interface/radar/tracks"
ODOM = "/novatel/oem7/odom"
TFS = "/tf_static"

MIN_OBS = 8                 # track_store.may_confirm, radar-only branch
MIN_MOVING = 0.8
GAP_S = 0.5
OBJECT_Z = -1.5             # object mid-height in lidar_tc (LiDAR 2.37 m above the road)


def load_radar(bag):
    """ESR tracks WITH their ids, which object_ab.load drops, plus odometry and the tf."""
    radar, odom, R_lr, t_lr = [], [], None, None
    st = lambda h: h.stamp.sec + h.stamp.nanosec * 1e-9
    for f in sorted(glob.glob(os.path.join(bag, "*.mcap"))):
        for m in read_ros2_messages(f, topics=[RADAR, ODOM, TFS]):
            o, tp = m.ros_msg, m.channel.topic
            if tp == TFS and R_lr is None:
                for tr in o.transforms:
                    if tr.child_frame_id == "delphi_esr_radar":
                        q, t = tr.transform.rotation, tr.transform.translation
                        y = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                        c, s = math.cos(y), math.sin(y)
                        R_lr, t_lr = np.array([[c, -s], [s, c]]), np.array([t.x, t.y])
            elif tp == ODOM:
                tw = o.twist.twist
                odom.append((st(o.header), tw.linear.x, tw.linear.y, tw.angular.z))
            elif tp == RADAR and len(o.tracks):
                n = len(o.tracks)
                arr = lambda a_, d=float: np.fromiter((getattr(k, a_) for k in o.tracks), d, n)
                rng, ang = arr("range"), arr("angle")
                g = gate_tracks(rng, ang, arr("amplitude"), arr("track_status", int),
                                arr("update_count", int), min_range=1.0, max_range=175.0,
                                min_amplitude=-1e9, min_update_count=0)
                if g.any():
                    radar.append((st(o.header), arr("track_id", int)[g], rng[g], ang[g],
                                  arr("range_rate")[g]))
    if R_lr is None:
        sys.exit("no lidar_tc -> delphi_esr_radar in tf_static")
    return radar, odom, R_lr.T, -R_lr.T @ t_lr        # node convention: lidar -> radar


def load_camera(pkl, arm="DropNF"):
    by_t = defaultdict(list)
    with open(pkl, "rb") as fh:
        for r in pickle.load(fh):
            p = r.get(arm)
            if p is not None:
                by_t[round(r["t"], 3)].append((p[0], p[1], r["cls"]))
    ts = sorted(by_t)
    return ts, by_t


def build_lives(radar, odom, R_sl, t_sl):
    tw, feed = oa._odom_feeder(odom)
    R_ls, t_ls = R_sl.T, -R_sl.T @ t_sl
    lives, open_life = [], {}
    for t, ids, rng, ang, rr in radar:
        feed(t)
        twist = tw.at(t)
        if twist is None:
            continue
        v_lidar = frames.ego_to_lidar([np.array([twist.vx, twist.vy])], correction_deg=-5.35)[0]
        lx, ly = oa.LEVER_IMU_TO_RADAR
        v_s = R_sl @ (v_lidar + twist.omega * np.array([-ly, lx]))
        comp = compensated_range_rate(rr, ang, v_s)
        xs, ys = polar_to_cartesian(rng, ang)
        p_l = (np.stack([xs, ys], axis=1) - t_sl) @ R_sl       # radar -> lidar (R_ls^T = R_sl)
        for i, tid in enumerate(ids):
            obs = (t, float(rng[i]), float(ang[i]), float(comp[i]), p_l[i], float(twist.omega))
            life = open_life.get(tid)
            if life is not None:
                pt, pr = life[-1][0], life[-1][1]
                if t - pt > GAP_S or abs(rng[i] - pr) > max(5.0, 0.2 * pr + 30.0 * (t - pt)):
                    lives.append(life)
                    life = None
            if life is None:
                life = open_life[tid] = []
            life.append(obs)
    lives.extend(v for v in open_life.values() if v)
    del t_ls, R_ls
    return lives


def in_camera_view(p_l):
    xyz = np.array([[p_l[0], p_l[1], OBJECT_Z]])
    kept, _, _ = project_to_pixels(xyz)
    return kept.shape[0] == 1 and 0.0 <= p_l[0] <= 150.0 and abs(p_l[1]) <= 20.0


def camera_confirms(obs, cam_ts, cam_by_t, R_sl, t_sl, window=0.06):
    t, r_r, az_r = obs[0], obs[1], obs[2]
    i = np.searchsorted(cam_ts, t - window)
    while i < len(cam_ts) and cam_ts[i] <= t + window:
        for cx, cy, cls in cam_by_t[cam_ts[i]]:
            ps = R_sl @ np.array([cx, cy]) + t_sl                 # camera object in radar frame
            r_c = float(np.hypot(*ps))
            az_c = math.degrees(math.atan2(ps[1], ps[0]))
            if (abs(az_c - az_r) <= oa.match_gate_deg(r_r, cls)
                    and abs(r_c - r_r) <= oa.match_range_window(r_r)):
                return True
        i += 1
    return False


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--source", default="/home/avalocal/selfcal_loc_2026-09-08_11-47-43")
    ap.add_argument("--measurements",
                    default=os.path.expanduser("~/fusion_data/measurements/rows_gate2.pkl"))
    ap.add_argument("--confirm-hits", type=int, default=3,
                    help="camera matches needed to call a life confirmed; 1 lets a single chance "
                         "alignment 'confirm' a guardrail, which would UNDER-state false alarms")
    args = ap.parse_args()

    radar, odom, R_sl, t_sl = load_radar(args.source)
    cam_ts, cam_by_t = load_camera(args.measurements)
    cam_ts = np.asarray(cam_ts)
    t0, t1 = radar[0][0], radar[-1][0]
    minutes = (t1 - t0) / 60.0
    print(f"[load] radar frames {len(radar)} over {minutes:.1f} min, camera frames {len(cam_ts)}")
    lives = build_lives(radar, odom, R_sl, t_sl)
    print(f"[lives] {len(lives)} radar object lives (id continuity, 0.5 s gap, range-jump split)\n")

    rows = []
    for life in lives:
        if len(life) < 2:
            continue
        mid = life[len(life) // 2]
        view = np.mean([in_camera_view(o[4]) for o in life]) >= 0.5
        moving = np.mean([abs(o[3]) >= RADAR_ONLY_MIN_SPEED for o in life])
        would_birth = len(life) >= MIN_OBS and moving >= MIN_MOVING
        hits = 0
        if view:
            for o in life:
                if camera_confirms(o, cam_ts, cam_by_t, R_sl, t_sl):
                    hits += 1
                    if hits >= args.confirm_hits:
                        break
        rows.append(dict(view=view, moving=moving >= MIN_MOVING, birth=would_birth,
                         confirmed=hits >= args.confirm_hits, r=mid[1], n=len(life),
                         dur=life[-1][0] - life[0][0],
                         speed=float(np.median([abs(o[3]) for o in life])),
                         yaw=float(np.max([abs(o[5]) for o in life]))))

    def table(sel, title):
        print(title)
        print(f"  {'band':>9s} {'lives':>6s} {'never seen by camera':>21s}")
        for lo, hi in ((0, 25), (25, 50), (50, 80), (80, 120), (120, 180)):
            b = [x for x in sel if lo <= x["r"] < hi]
            if b:
                un = sum(not x["confirmed"] for x in b)
                print(f"  {lo:4d}-{hi:<4d} {len(b):6d} {100 * un / len(b):20.1f}%")
        if sel:
            un = sum(not x["confirmed"] for x in sel)
            print(f"  {'all':>9s} {len(sel):6d} {100 * un / len(sel):20.1f}%   "
                  f"({un} lives, {un / minutes:.1f} per minute)\n")

    inview = [x for x in rows if x["view"]]
    table([x for x in inview if not x["moving"]],
          "IN THE CAMERA'S VIEW, STATIONARY radar lives (the clutter class; the birth gate "
          "already refuses these):")
    table([x for x in inview if x["birth"]],
          "IN THE CAMERA'S VIEW, lives that WOULD BE BORN (>= 8 obs, >= 80% moving) -- the "
          "false-alarm proxy:")
    born = [x for x in inview if x["birth"]]
    if born:
        print("WHAT THE UNCONFIRMED WOULD-BE-BORN LIVES LOOK LIKE -- real movers, or static clutter "
              "the ego-motion compensation leaked into 'moving'?")
        print(f"  {'':12s} {'lives':>6s} {'median |speed|':>15s} {'max yaw rate':>13s} "
              f"{'turning (>0.1 rad/s)':>21s} {'duration':>9s}")
        for lab, sel in (("confirmed", [x for x in born if x["confirmed"]]),
                         ("never seen", [x for x in born if not x["confirmed"]])):
            if sel:
                sp = np.array([x["speed"] for x in sel])
                yw = np.array([x["yaw"] for x in sel])
                du = np.array([x["dur"] for x in sel])
                print(f"  {lab:12s} {len(sel):6d} {np.median(sp):13.1f} m/s "
                      f"{np.median(yw):9.3f} rad/s {100 * np.mean(yw > 0.1):20.0f}% "
                      f"{np.median(du):7.1f} s")
        print("  Clutter that leaks through compensation moves at just over the 1.5 m/s gate and "
              "only while turning; a real vehicle does neither.\n")

    out = [x for x in rows if not x["view"] and x["birth"]]
    print(f"OUTSIDE the camera's view, lives that would be born (what radar-only birth ADDS, with "
          f"no referee): {len(out)} ({len(out) / minutes:.1f} per minute)")
    print("\nUpper bound: 'never seen' also counts real objects YOLO has no class for and objects "
          "the camera cannot see. See the docstring.")


if __name__ == "__main__":
    main()
