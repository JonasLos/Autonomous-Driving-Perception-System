#!/usr/bin/env python3
"""Offline A/B of LiDAR point selection for the camera+LiDAR path. No ROS graph.

Every arm sees IDENTICAL inputs: the raw sweep from the source bag and the 2D boxes that
fusion_node actually published in /fused_bbox for that same sweep (fusion_node copies the
projection's header, and transform.py copies the cloud's, so the stamps are equal). The only
thing that differs between arms is how 3D points are chosen for each box.

  A25    fusion_node's rule at its default: crop, project, box mask, ground cut only past 25 m,
         nearest depth cluster, median. MUST reproduce the published /fused_bbox positions --
         that is the self-check, and no metric prints unless it passes.
  A10    the same rule with the ground cut from 10 m, object_fusion's current default.
  G      ground segmentation on the FULL sweep first, then crop, project, box mask, nearest
         depth cluster, median. The percentile ground cut is not applied: the ground is gone.
  G+     G, with the percentile cut kept as well.

Scoring is PAIRED. Every arm is banded by the published (A25) range and scored against the same
radar return, chosen once per detection by azimuth from the published position. So a
difference between arms is a difference in selection, never a difference in which detections
or which radar returns were compared.

Imports the production rules from object_fusion rather than copying them, as radar_ab.py,
patch_ab.py and lane_ab.py do.
"""
from __future__ import annotations

import argparse
import glob
import math
import os
import sys
import time

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (os.path.join(REPO, "src", "object_fusion"),
           os.path.join(REPO, "src", "perception_common"),
           os.path.join(REPO, "src", "radar_ros")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from mcap_ros2.reader import read_ros2_messages                            # noqa: E402

from perception_common.utils import crop_pointcloud                         # noqa: E402
from object_fusion.detection_geometry import (                              # noqa: E402
    nearest_depth_cluster, reject_ground_returns,
)
from object_fusion.ground_segmentation import GroundSegmenter               # noqa: E402
from object_fusion.projection import CAMERA_INFO_WH, project_to_pixels      # noqa: E402
from radar_ros.radar_geometry import cartesian_to_polar, gate_tracks        # noqa: E402

# Defaults: the reference drive and its replay through the existing stack. Override with --source /
# --replay (both harnesses); a replay is the existing stack's /fused_bbox + radar tracks recorded
# while that source bag played, and the self-check refuses to score if it does not match.
SOURCE = "/home/avalocal/selfcal_loc_2026-09-08_11-47-43"
REPLAY = "/home/avalocal/fused_replay_selfcal_2026-09-08"


def add_bag_args(ap):
    ap.add_argument("--source", default=SOURCE, help="raw sensor bag (LiDAR, odometry)")
    ap.add_argument("--replay", default=REPLAY,
                    help="that bag replayed through the existing stack (/fused_bbox, radar tracks)")


def use_bags(args):
    global SOURCE, REPLAY
    SOURCE, REPLAY = args.source, args.replay

# transform.py's defaults on the image that produced the replay (voxel off).
CROP_X, CROP_Y, CROP_Z = [0.0, 150.0], [-20.0, 20.0], [-3.5, 1.0]
GROUND_MARGIN, GROUND_MIN_POINTS = 0.4, 2

YAW = math.radians(5.443)
R_SL = np.array([[math.cos(YAW), -math.sin(YAW)], [math.sin(YAW), math.cos(YAW)]])
T_SL = np.array([-2.964, 0.371])
BANDS = [(0, 15), (15, 25), (25, 40), (40, 60), (60, 80), (80, 200)]
SELF_CHECK_TOL = 1e-3      # metres


def stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def decode(cloud, names=("x", "y", "z")):
    """mcap_ros2 yields its own dynamic class, which sensor_msgs_py rejects; decode directly."""
    offs = {f.name: f.offset for f in cloud.fields}
    a = np.frombuffer(bytes(cloud.data), dtype=np.uint8).reshape(-1, cloud.point_step)
    return np.stack([a[:, offs[n]:offs[n] + 4].copy().view(np.float32).ravel()
                     for n in names], axis=1)


#: Focal length in pixels (PROJ[0,0]); an object of pixel width w subtends w/FX radians.
FX_PX = 3461.179

#: Radar azimuth noise, degrees (ESR datasheet 0.5 deg), added to the object's own extent.
RADAR_AZIMUTH_SIGMA_DEG = 0.5


def azimuth_gate_deg(box_w_px=None):
    """How far in azimuth a radar return may sit from the object's CENTRE and still be it.

    A fixed +-1 deg is wrong close in: at 10 m a 1.8 m car spans ~10 deg, so its bumper return --
    the one the radar actually sees -- is nowhere near the box centre's bearing and the nearest
    return is often a different object. That is the 0-15 m "error" of -5.9 m in the old tables.
    The gate is therefore the object's own half-extent plus the radar's azimuth noise, floored at
    the historical 1 deg so nothing gets tighter than before.
    """
    if not box_w_px:
        return 1.0
    half = math.degrees(math.atan(0.5 * float(box_w_px) / FX_PX))
    return max(1.0, half + RADAR_AZIMUTH_SIGMA_DEG)


def plausible_range_window(obj_range_m):
    """How far in RANGE a radar return may sit from the object and still plausibly be it.

    Needed because the angular gate alone is not enough where the object is at the edge of the
    camera's field of view: at 5-15 m the objects on this drive sit at 15-26 deg azimuth, radar
    coverage there is sparse, and the nearest-in-azimuth return is routinely a different object --
    one pair had a 6.7 m truck matched to a 69.9 m return on the same bearing, which is where the
    old "-5.9 m at 0-15 m" came from.

    DELIBERATELY WIDE (3 m + 50% of range: +-8 m at 10 m, +-43 m at 80 m). The Phase 0 trap was a
    +-3 m gate that truncated the very distribution being measured; this window is far wider than
    any error under test -- it still admits the -7.6 m bias at 80-100 m and the -32 m one beyond --
    and only rejects matches that cannot be the same object at all.
    """
    return 3.0 + 0.5 * float(obj_range_m)


def match_radar_range(obj_range_m, obj_az_deg, box_w_px, radar_range, radar_az):
    """Nearest-in-azimuth radar return, inside the object's angular extent AND a plausible range.

    Returns ``(range, azimuth)`` or None. Range is the quantity under test, so the window is only
    a sanity bound -- see plausible_range_window.
    """
    daz = np.abs(np.asarray(radar_az) - float(obj_az_deg))
    ok = (daz <= azimuth_gate_deg(box_w_px)) & (
        np.abs(np.asarray(radar_range) - float(obj_range_m)) <= plausible_range_window(obj_range_m))
    if not np.any(ok):
        return None
    idx = np.flatnonzero(ok)
    k = idx[int(np.argmin(daz[idx]))]
    return float(radar_range[k]), float(radar_az[k])


def band_of(r):
    for lo, hi in BANDS:
        if lo <= r < hi:
            return (lo, hi)
    return None


def select(px, py, pz, *, ground_min_range):
    """Box points -> (x, y), through the production rules. None if nothing survives."""
    if ground_min_range is not None:
        px, py, pz = reject_ground_returns(px, py, pz, min_range=ground_min_range,
                                           margin=GROUND_MARGIN,
                                           min_points=GROUND_MIN_POINTS)
    px, py, pz = nearest_depth_cluster(px, py, pz)
    if px.size == 0:
        return None
    return float(np.median(px)), float(np.median(py))


def load_boxes_and_radar():
    fused, radar = {}, []
    for f in sorted(glob.glob(os.path.join(REPLAY, "*.mcap"))):
        for m in read_ros2_messages(f, topics=["/fused_bbox",
                                               "/delphi_esr_interface/radar/tracks"]):
            o = m.ros_msg
            if m.channel.topic == "/fused_bbox":
                if o.detections:
                    fused[round(stamp(o.header), 3)] = [
                        (d.bbox.center.position.x, d.bbox.center.position.y,
                         d.bbox.size.x, d.bbox.size.y, d.id,
                         d.bbox3d.center.position.x, d.bbox3d.center.position.y)
                        for d in o.detections]
            else:
                n = len(o.tracks)
                if not n:
                    continue
                arr = lambda a_, dt=float: np.fromiter((getattr(k, a_) for k in o.tracks), dt, n)
                g = gate_tracks(arr("range"), arr("angle"), arr("amplitude"),
                                arr("track_status", int), arr("update_count", int),
                                min_range=1.0, max_range=175.0, min_amplitude=-1e9,
                                min_update_count=0)
                if g.any():
                    radar.append((stamp(o.header), arr("range")[g], arr("angle")[g]))
    return fused, radar


def run(args):
    t0 = time.perf_counter()
    fused, radar = load_boxes_and_radar()
    rts = np.array([r[0] for r in radar])
    print(f"[load] box frames {len(fused)}  radar sweeps {len(radar)}  "
          f"({time.perf_counter() - t0:.0f} s)")

    arms = {"A25": dict(ground=None, gmr=25.0), "A10": dict(ground=None, gmr=10.0),
            "G": dict(ground=True, gmr=None), "G+": dict(ground=True, gmr=10.0),
            # G+ that never loses a detection: when ground removal empties the box, fall back
            # to the unfiltered points under A10's rule. Measured separately because flipping
            # between two selection rules frame to frame could itself add jitter.
            "G+f": dict(ground=True, gmr=10.0, fallback=True)}
    seg = GroundSegmenter(backend=args.backend, max_range=args.max_range)
    print(f"[ground] backend = {seg.backend}  max_range = {args.max_range or 'library default'}")

    # per detection: arm -> (x, y) ; plus reference band, radar range, tracker id, stamp
    rows = []
    worst_self = 0.0
    n_self = 0
    emptied = {a: 0 for a in arms}
    seg_ms = []
    done = 0
    for f in sorted(glob.glob(os.path.join(SOURCE, "*.mcap"))):
        for m in read_ros2_messages(f, topics=["/lidar_tc/velodyne_points"]):
            key = round(stamp(m.ros_msg.header), 3)
            dets = fused.get(key)
            if dets is None:
                continue
            xyz = decode(m.ros_msg).astype(np.float64)
            xyz = xyz[np.isfinite(xyz).all(axis=1)]

            ts = time.perf_counter()
            ground = seg.segment(xyz)
            seg_ms.append(1000 * (time.perf_counter() - ts))

            views = {}
            for use_ground in (False, True):
                pts = xyz[~ground] if use_ground else xyz
                cropped = np.asarray(crop_pointcloud(pts, CROP_X, CROP_Y, CROP_Z))[:, :3]
                # camera_info's bounds, as transform.py uses -- NOT the 1-px-short
                # principal-point fallback, which is what failed the first self-check.
                views[use_ground] = project_to_pixels(cropped, image_wh=CAMERA_INFO_WH)

            ri = int(np.argmin(np.abs(rts - key))) if rts.size else None
            radar_ok = ri is not None and abs(rts[ri] - key) <= 0.05

            for (bu, bv, bw, bh, oid, pubx, puby) in dets:
                row = {"t": key, "id": oid, "pub": (pubx, puby)}
                for arm, cfg in arms.items():
                    kept, u, v = views[bool(cfg["ground"])]
                    mask = ((u >= bu - bw / 2) & (u <= bu + bw / 2)
                            & (v >= bv - bh / 2) & (v <= bv + bh / 2))
                    if not np.any(mask):
                        if cfg.get("fallback"):
                            k0, u0, v0 = views[False]
                            m0 = ((u0 >= bu - bw / 2) & (u0 <= bu + bw / 2)
                                  & (v0 >= bv - bh / 2) & (v0 <= bv + bh / 2))
                            if np.any(m0):
                                row[arm] = select(k0[m0, 0].astype(np.float64),
                                                  k0[m0, 1].astype(np.float64),
                                                  k0[m0, 2].astype(np.float64),
                                                  ground_min_range=10.0)
                                continue
                        row[arm] = None
                        emptied[arm] += 1
                        continue
                    row[arm] = select(kept[mask, 0].astype(np.float64),
                                      kept[mask, 1].astype(np.float64),
                                      kept[mask, 2].astype(np.float64),
                                      ground_min_range=cfg["gmr"])
                if row["A25"] is not None:
                    d = math.hypot(row["A25"][0] - pubx, row["A25"][1] - puby)
                    worst_self = max(worst_self, d)
                    n_self += 1
                r_pub = math.hypot(pubx, puby)
                row["band"] = band_of(r_pub)
                row["radar"] = None
                if radar_ok:
                    _, rr, raz = radar[ri]
                    p_s = R_SL @ np.array([pubx, puby]) + T_SL
                    _, oaz = cartesian_to_polar(p_s[0], p_s[1])
                    hit = match_radar_range(_rng(row["A25"]) if row["A25"] else r_pub,
                                            float(oaz), bw, rr, raz)
                    if hit is not None:
                        row["radar"] = hit[0]
                rows.append(row)
            done += 1
            if args.max_frames and done >= args.max_frames:
                break
        if args.max_frames and done >= args.max_frames:
            break

    print(f"[run] sweeps {done}  detections {len(rows)}  "
          f"segmentation {np.median(seg_ms):.0f} ms median / {np.percentile(seg_ms, 95):.0f} p95  "
          f"({time.perf_counter() - t0:.0f} s)")

    ok = n_self > 0 and worst_self <= SELF_CHECK_TOL
    print(f"\n[self-check] A25 vs published /fused_bbox over {n_self} detections: "
          f"worst |Δ| = {worst_self:.2e} m  (tol {SELF_CHECK_TOL:.0e})  -> "
          f"{'PASS' if ok else 'FAIL'}")
    if not ok and not args.skip_self_check:
        sys.exit("self-check failed: not scoring arms whose baseline does not reproduce the node")

    print("\nDetections left with no LiDAR point inside the box:")
    for a in arms:
        print(f"  {a:4s} {emptied[a]:6d}  ({100 * emptied[a] / max(len(rows), 1):.2f}%)")

    report(rows, list(arms))

    # The table above pairs only detections where EVERY arm produced a position, so it can never
    # show what happens on the detections an arm lost -- or, for a fallback arm, on the frames
    # where it switches rule. Re-score using only the arms that never lose a detection.
    complete = [a for a in arms if emptied[a] == 0]
    if len(complete) < len(arms):
        print("\n================ ALL DETECTIONS (arms that never lose one) ================")
        report(rows, complete)


def report(rows, arms):
    print("\nRANGE ERROR vs radar, paired (same detections, same radar return), by published range")
    hdr = "  ".join(f"{a:>13s}" for a in arms)
    print(f"  {'band':>9s} {'n':>5s}  {hdr}")
    print(f"  {'':>9s} {'':>5s}  " + "  ".join(f"{'median / sd':>13s}" for _ in arms))
    for b in BANDS:
        sel = [r for r in rows if r["band"] == b and r["radar"] is not None
               and all(r[a] is not None for a in arms)]
        if len(sel) < 10:
            continue
        cells = []
        for a in arms:
            e = np.array([_rng(r[a]) - r["radar"] for r in sel])
            med = np.median(e)
            sd = 1.4826 * np.median(np.abs(e - med))
            cells.append(f"{med:+6.2f} / {sd:4.2f}")
        print(f"  {b[0]:3d}-{b[1]:<5d} {len(sel):5d}  " + "  ".join(f"{c:>13s}" for c in cells))

    print("\nJITTER of the same tracker id, by published range   (median / p90 of |p[t+1] - 2 p[t] + p[t-1]|)")
    print("  A first difference includes the ego's real relative motion (~1.2 m per frame at")
    print("  12 m/s); the second difference cancels constant-velocity motion and leaves noise.")
    print(f"  {'band':>9s} {'n':>5s}  " + "  ".join(f"{a:>13s}" for a in arms))
    for b in BANDS:
        per_arm = {a: [] for a in arms}
        hist = {}
        for r in sorted(rows, key=lambda x: x["t"]):
            if not r["id"] or r["band"] != b or any(r[a] is None for a in arms):
                continue
            h = hist.setdefault(r["id"], [])
            if h and not (0 < r["t"] - h[-1]["t"] < 0.3):
                h.clear()
            h.append(r)
            if len(h) >= 3:
                p0, p1, p2 = h[-3], h[-2], h[-1]
                for a in arms:
                    per_arm[a].append(math.hypot(p2[a][0] - 2 * p1[a][0] + p0[a][0],
                                                 p2[a][1] - 2 * p1[a][1] + p0[a][1]))
        n = len(per_arm[arms[0]])
        if n < 10:
            continue
        cells = [f"{np.median(np.asarray(per_arm[a])):5.2f} /{np.percentile(np.asarray(per_arm[a]), 90):5.2f}"
                 for a in arms]
        print(f"  {b[0]:3d}-{b[1]:<5d} {n:5d}  " + "  ".join(f"{c:>13s}" for c in cells))

    print("\nDETECTIONS LEFT WITH NO POINT, by published range")
    print(f"  {'band':>9s} {'n':>5s}  " + "  ".join(f"{a:>7s}" for a in arms))
    for b in BANDS:
        sel = [r for r in rows if r["band"] == b]
        if not sel:
            continue
        print(f"  {b[0]:3d}-{b[1]:<5d} {len(sel):5d}  "
              + "  ".join(f"{100 * sum(r[a] is None for r in sel) / len(sel):6.1f}%" for a in arms))


def _rng(p):
    p_s = R_SL @ np.array(p) + T_SL
    return float(math.hypot(p_s[0], p_s[1]))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--backend", default="fallback", choices=("fallback", "patchworkpp", "auto"))
    ap.add_argument("--max-range", type=float, default=None,
                    help="Patchwork++ max_range; the library default is 80 m")
    ap.add_argument("--max-frames", type=int, default=None)
    ap.add_argument("--skip-self-check", action="store_true")
    add_bag_args(ap)
    args = ap.parse_args()
    use_bags(args)
    run(args)


if __name__ == "__main__":
    main()
