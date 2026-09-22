#!/usr/bin/env python3
"""Why does a camera-LiDAR object's position jump between frames, and what stops it?

Scores point-selection and measurement-gating rules on the reference replay
(selfcal_loc_2026-09-08, 4519 detections), importing the production rules, behind the same
self-check as ground_ab.py (fusion_node's rule must reproduce the published /fused_bbox).

WHAT WAS FOUND (so it is not re-derived):

* The hypothesis that prompted this -- a NEIGHBOURING object inside the whole 2D box drags the
  median -- is only a minor contributor. Of the live rule's >2 m second differences, 82% are
  ALONG the ray (median 5.8 m along vs 0.38 m across), and the 2D box itself is steady in 81% of
  them. A neighbour beside the target would move the box sideways.
* They are one-frame spikes, half nearer and half farther; the neighbouring frames agree with
  radar and the spike does not.
* 53% of detections on this drive are traffic cones. A cone box holds ~3 non-ground returns, so
  when the cone is missed for a frame the nearest thing left is a road ring or the next cone in
  the line, metres away in depth. When segmentation leaves the box EMPTY (3.7%, 165/169 cones),
  the old fallback to every point placed the cone on a road ring: 27% of all spikes.
* No per-frame selection rule fixes it (centre crop, 3D clustering by centre / size / previous
  output, depth cluster chosen by prediction: 11-13% big jumps each). Refusing the outlier does.

ARMS
  never drop a detection (scored paired):
    A25     fusion_node's rule. SELF-CHECK against /fused_bbox.
    A10     fusion_node's rule with the 10 m percentile gate, no segmentation (the detector
            on /lidar_2d_projection).
    G+f     segmentation + percentile cut + nearest depth cluster, empty box -> every point.
    Gmed    G+f without the depth cut.                       (isolates the SLIVER flag)
    C50     G+f on the central 50% of the box.               (neighbours enter at the edges)
    Kctr / Ksize / Kprev   voxel connected components; the cluster nearest the box centre /
            the largest / nearest this id's previous output.
    Dprev   the depth cluster nearest a constant-velocity prediction of this id.
  may drop a detection (scored on their own outputs):
    NF       G+f, but an empty box publishes nothing.        (select_object_points default)
    DropNF_nopct   DropNF without the 10 m percentile height cut after segmentation.
    DropNF_cam     DropNF, but a ground-only box is ranged by camera_ground_position (box bottom
                   edge ray vs local LiDAR ground), through the same depth gate.
    CAM      camera_ground_position for every detection: the independent reference.
    DropNF   NF + DepthJumpGate.                              <- PRODUCTION, asserted equal
    A10gate  A10 + DepthJumpGate.
    Dhold / DholdNF  Dprev-style cluster switching + one-frame drop, with / without fallback.

    PYTHONPATH=<pypatchworkpp dir> python3 scripts/neighbour_ab.py [--gate-sweep] [--dump rows.pkl]
"""
from __future__ import annotations

import argparse
import glob
import math
import os
import pickle
import sys
import time

import numpy as np
from scipy import ndimage

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ground_ab as G                                                       # noqa: E402

from mcap_ros2.reader import read_ros2_messages                              # noqa: E402
from perception_common.utils import crop_pointcloud                          # noqa: E402
from object_fusion import frames                                            # noqa: E402
from object_fusion.detection_geometry import nearest_depth_cluster           # noqa: E402
from object_fusion.ground_segmentation import (                               # noqa: E402
    GroundSegmenter, LevelledGroundSegmenter, levelling_rotation, quaternion_roll_pitch_deg,
    slopes_from_attitude,
)
from object_fusion.measurement_gate import DepthJumpGate                     # noqa: E402
from object_fusion.projection import (                                       # noqa: E402
    CAMERA_INFO_WH, camera_ground_position, project_to_pixels,
)

RING_PITCH = math.tan(math.radians(0.333))
BIG = 2.0                          # metres; a "big jump" / "spike"
PAIRED_ARMS = ["A25", "A10", "G+f", "Gmed", "C50", "Kctr", "Ksize", "Kprev", "Dprev"]
DROP_ARMS = ["NF", "DropNF", "A10gate", "Dhold", "DholdNF", "DropNF_nopct", "DropNF_cam", "CAM"]
REPORT_BANDS = [(0, 25), (25, 40), (40, 60), (60, 80), (80, 200)]


def percentile_keep(P):
    """reject_ground_returns as a mask (min_range 10, margin 0.4, min_points 2)."""
    keep = np.ones(P.shape[0], bool)
    if P.shape[0] < 2 or float(P[:, 0].min()) < 10.0:
        return keep
    k = P[:, 2] > float(np.percentile(P[:, 2], 10.0)) + G.GROUND_MARGIN
    return k if k.sum() >= G.GROUND_MIN_POINTS else keep


def voxel_clusters(P):
    """26-connected components on a voxel grid sized to exceed the ring spacing at this range."""
    r = float(np.median(np.hypot(P[:, 0], P[:, 1])))
    vox = max(0.5, 1.2 * r * RING_PITCH)
    ijk = np.floor((P - P.min(axis=0)) / vox).astype(np.int64)
    grid = np.zeros(tuple(ijk.max(axis=0) + 1), bool)
    grid[tuple(ijk.T)] = True
    lab, _ = ndimage.label(grid, structure=np.ones((3, 3, 3)))
    return lab[tuple(ijk.T)]


def depth_clusters(px):
    """Every depth cluster, nearest first, cut with nearest_depth_cluster's own gap threshold."""
    order = np.argsort(px)
    if px.size < 2:
        return [order]
    diffs = np.diff(px[order])
    med = float(np.median(diffs))
    mad = float(np.median(np.abs(diffs - med)))
    cuts = np.where(diffs > max(med + 3.0 * mad, 0.05))[0]
    return np.split(order, cuts + 1)


def predict(hist, t):
    """Constant-velocity extrapolation of an id's last two outputs; None without two recent ones."""
    if not hist or len(hist) < 2:
        return None
    (t0, p0), (t1, p1) = hist
    if not (0 < t1 - t0 < 0.3 and 0 < t - t1 < 0.3):
        return None
    k = (t - t1) / (t1 - t0)
    return (p1[0] + k * (p1[0] - p0[0]), p1[1] + k * (p1[1] - p0[1]))


class DepthGate:
    """Reference implementation over depth CLUSTERS, so it can also switch cluster (Dhold).

    Used for the Dhold arms only: they may SWITCH to the cluster nearest the prediction, which
    the production gate never does. The DropNF arm calls the production gate itself.
    ``Q=None``: no usable points, emit nothing, state untouched.
    """

    def __init__(self, gate_min=1.5, gate_frac=0.05, switch=True):
        self.hist, self.miss, self.dropped = {}, {}, 0
        self.gate_min, self.gate_frac, self.switch = gate_min, gate_frac, switch

    def step(self, oid, t, Q):
        if Q is None or Q.shape[0] == 0:
            return None
        groups = depth_clusters(Q[:, 0])
        choice = groups[0]
        pred = predict(self.hist.get(oid), t) if oid else None
        if pred is not None:
            d = [abs(float(np.median(Q[g, 0])) - pred[0]) for g in groups]
            best = int(np.argmin(d)) if self.switch else 0
            if d[best] <= max(self.gate_min, self.gate_frac * abs(pred[0])):
                choice = groups[best]
                self.miss[oid] = 0
            elif self.miss.get(oid, 0) == 0:
                self.miss[oid] = 1
                self.dropped += 1
                return None
            else:
                self.miss[oid] = 0
                self.hist[oid] = []
        out = (float(np.median(Q[choice, 0])), float(np.median(Q[choice, 1])))
        if oid:
            h = self.hist.setdefault(oid, [])
            h.append((t, out))
            del h[:-2]
        return out


def box_view(view, bu, bv, bw, bh, frac=1.0):
    kept, u, v = view
    m = ((u >= bu - frac * bw / 2) & (u <= bu + frac * bw / 2)
         & (v >= bv - frac * bh / 2) & (v <= bv + frac * bh / 2))
    return kept[m].astype(np.float64), u[m].astype(np.float64), v[m].astype(np.float64)


def load_classes():
    cls = {}
    for f in sorted(glob.glob(os.path.join(G.REPLAY, "*.mcap"))):
        for m in read_ros2_messages(f, topics=["/fused_bbox"]):
            o = m.ros_msg
            for d in o.detections:
                cls[(round(G.stamp(o.header), 3), d.id, round(d.bbox.center.position.x, 3))] = \
                    d.class_name
    return cls


def cluster_arms(row, Q, qu, qv, bu, bv, oid, last_out):
    lab = voxel_clusters(Q)
    ids, counts = np.unique(lab, return_counts=True)
    big = ids[counts >= max(3, 0.05 * Q.shape[0])]
    if big.size == 0:
        big = ids[[int(np.argmax(counts))]]
    info = []
    for c in big:
        s = lab == c
        info.append((int(s.sum()), np.median(Q[s, :2], axis=0),
                     math.hypot(np.median(qu[s]) - bu, np.median(qv[s]) - bv)))
    ctr = min(info, key=lambda e: e[2])
    row["Kctr"] = tuple(map(float, ctr[1]))
    row["Ksize"] = tuple(map(float, max(info, key=lambda e: e[0])[1]))
    prev = last_out.get(oid) if oid else None
    pick = min(info, key=lambda e: float(np.hypot(*(e[1] - prev)))) if prev is not None else ctr
    row["Kprev"] = tuple(map(float, pick[1]))
    if oid:
        last_out[oid] = np.asarray(row["Kprev"])
    # MULTI: two substantial clusters (>= 20% of the points each) more than 2 m apart.
    subst = [e[1] for e in info if e[0] >= 0.2 * Q.shape[0]]
    if len(subst) >= 2:
        cs = np.array(subst)
        row["multi"] = bool(np.linalg.norm(cs[:, None] - cs[None], axis=2).max() > 2.0)


def deskew_sweep(raw, stamp_s, odom):
    """Undo the ego motion WITHIN one sweep: every point to the frame at the sweep's own stamp.

    The VLP-32C spins for 100 ms per sweep and its `time` field says when each point was really
    captured (-0.0995..0 s here), so the cloud as published mixes 100 ms of vehicle motion. While
    turning that is a bearing-dependent position error: at 0.2 rad/s a sweep smears 1.15 deg,
    0.6 m at 30 m, and an object's share of it changes as its bearing changes -- read as motion by
    anything differencing consecutive frames.

    A static point seen at t_pt sits, in the frame at t_ref, at R(dpsi)^T (p - d): the same
    convention as tracker.predict, with dpsi and d the frame's motion from t_pt to t_ref.
    """
    xyz = raw[:, :3].copy()
    dt = -raw[:, 3]                                   # seconds BEFORE the sweep stamp
    vx = float(np.interp(stamp_s, odom[:, 0], odom[:, 3]))
    vy = float(np.interp(stamp_s, odom[:, 0], odom[:, 4]))
    w = float(np.interp(stamp_s, odom[:, 0], odom[:, 5]))
    v_l = frames.ego_to_lidar([np.array([vx, vy])], correction_deg=-5.35)[0]
    dpsi = w * dt
    c, s_ = np.cos(dpsi), np.sin(dpsi)
    px, py = xyz[:, 0] - v_l[0] * dt, xyz[:, 1] - v_l[1] * dt
    xyz[:, 0] = c * px + s_ * py
    xyz[:, 1] = -s_ * px + c * py
    return xyz


def run(args):
    t0 = time.perf_counter()
    fused, radar = G.load_boxes_and_radar()
    classes = load_classes()
    rts = np.array([r[0] for r in radar])
    seg = GroundSegmenter(backend="patchworkpp", max_range=120.0)
    hybrid = LevelledGroundSegmenter(backend="patchworkpp", max_range=120.0,
                                     near_range=args.near_range)            # PRODUCTION class
    odom = None
    if args.level in ("odom", "odom-nomount", "odom-near") or args.deskew:
        rows = []
        for f in sorted(glob.glob(os.path.join(G.SOURCE, "*.mcap"))):
            for m in read_ros2_messages(f, topics=["/novatel/oem7/odom"]):
                q = m.ros_msg.pose.pose.orientation
                tw = m.ros_msg.twist.twist
                rows.append((G.stamp(m.ros_msg.header),
                             *quaternion_roll_pitch_deg(q.x, q.y, q.z, q.w),
                             tw.linear.x, tw.linear.y, tw.angular.z))
        odom = np.array(rows)
        print(f"[level] odometry from {len(odom)} odom messages"
              + (" (also driving --deskew)" if args.deskew else ""))

    last_out, dprev_hist = {}, {}
    ref = {"Dhold": DepthGate(), "DholdNF": DepthGate()}
    production, production_a10 = DepthJumpGate(), DepthJumpGate()
    gate_nopct, gate_cam = DepthJumpGate(), DepthJumpGate()
    sweep = {}
    if args.gate_sweep:
        for gm in (0.75, 1.0, 1.5, 2.5):
            for gf in (0.02, 0.05, 0.08):
                sweep[f"gate {gm:.2f} m / {gf:.2f}"] = DepthJumpGate(gate_min_m=gm,
                                                                     gate_range_frac=gf)
    rows, worst_self, n_self, done = [], 0.0, 0, 0

    for f in sorted(glob.glob(os.path.join(G.SOURCE, "*.mcap"))):
        for m in read_ros2_messages(f, topics=["/lidar_tc/velodyne_points"]):
            key = round(G.stamp(m.ros_msg.header), 3)
            dets = fused.get(key)
            if dets is None:
                continue
            if args.deskew:
                raw = G.decode(m.ros_msg, names=("x", "y", "z", "time")).astype(np.float64)
                xyz = deskew_sweep(raw[np.isfinite(raw).all(axis=1)], key, odom)
            else:
                xyz = G.decode(m.ros_msg).astype(np.float64)
                xyz = xyz[np.isfinite(xyz).all(axis=1)]
            level = roll = pitch = None
            if odom is not None:
                roll = float(np.interp(key, odom[:, 0], odom[:, 1]))
                pitch = float(np.interp(key, odom[:, 0], odom[:, 2]))
                level = levelling_rotation(*slopes_from_attitude(
                    roll, pitch, include_mount=(args.level != "odom-nomount")))
            if args.level == "odom-near":
                ground = hybrid.segment(xyz, roll, pitch)
            else:
                ground = seg.segment(xyz, level=level)
            views = {}
            for use_ground in (False, True):
                pts = xyz[~ground] if use_ground else xyz
                c = np.asarray(crop_pointcloud(pts, G.CROP_X, G.CROP_Y, G.CROP_Z))[:, :3]
                views[use_ground] = project_to_pixels(c, image_wh=CAMERA_INFO_WH)
            # The ground returns the detector would have: flagged, cropped and inside the image.
            gcrop = np.asarray(crop_pointcloud(xyz[ground], G.CROP_X, G.CROP_Y, G.CROP_Z))[:, :3]
            ground_fov = project_to_pixels(gcrop, image_wh=CAMERA_INFO_WH)[0] if gcrop.size else gcrop
            ri = int(np.argmin(np.abs(rts - key))) if rts.size else None
            radar_ok = ri is not None and abs(rts[ri] - key) <= 0.05

            for (bu, bv, bw, bh, oid, pubx, puby) in dets:
                row = {a: None for a in PAIRED_ARMS + DROP_ARMS + list(sweep)}
                row.update(n_clusters=0, fg_frac=None, bg_gap=None, bg_frac=None,
                           bg_frac_all=None, cluster_depths=None, cluster_sizes=None,
                           cluster_xy=None)
                row.update(t=key, id=oid, multi=False, sliver=False, fallback=False,
                           box=(bu, bv, bw, bh), pub=(pubx, puby),
                           cls=classes.get((key, oid, round(bu, 3)), "?"))

                P0, _, _ = box_view(views[False], bu, bv, bw, bh)
                if P0.shape[0]:
                    row["A25"] = G.select(P0[:, 0], P0[:, 1], P0[:, 2], ground_min_range=25.0)
                    row["A10"] = G.select(P0[:, 0], P0[:, 1], P0[:, 2], ground_min_range=10.0)
                    if row["A25"] is not None:
                        worst_self = max(worst_self, math.hypot(row["A25"][0] - pubx,
                                                                row["A25"][1] - puby))
                        n_self += 1
                    if row["A10"] is not None and production_a10.admit(oid, key, *row["A10"]):
                        row["A10gate"] = row["A10"]

                cam = camera_ground_position(bu, bv + bh / 2.0, ground_fov)
                row["CAM"] = (float(cam[0][0]), float(cam[0][1])) if cam is not None else None

                P, pu, pv = box_view(views[True], bu, bv, bw, bh)
                row["fallback"] = P.shape[0] == 0
                if row["fallback"]:
                    P, pu, pv = box_view(views[False], bu, bv, bw, bh)
                if P.shape[0]:
                    k = percentile_keep(P)
                    Q, qu, qv = P[k], pu[k], pv[k]
                    fx, fy, _ = nearest_depth_cluster(Q[:, 0], Q[:, 1], Q[:, 2])
                    row["G+f"] = (float(np.median(fx)), float(np.median(fy)))
                    row["Gmed"] = (float(np.median(Q[:, 0])), float(np.median(Q[:, 1])))
                    row["sliver"] = fx.size < 0.25 * Q.shape[0]

                    Pc, _, _ = box_view(views[True], bu, bv, bw, bh, frac=0.5)
                    Qc = Pc if Pc.shape[0] >= 5 else P
                    Qc = Qc[percentile_keep(Qc)]
                    cx, cy, _ = nearest_depth_cluster(Qc[:, 0], Qc[:, 1], Qc[:, 2])
                    row["C50"] = (float(np.median(cx)), float(np.median(cy)))

                    cluster_arms(row, Q, qu, qv, bu, bv, oid, last_out)

                    groups = depth_clusters(Q[:, 0])
                    # Occlusion evidence (item 4 / item 12 remainder). The production rule keeps
                    # the NEAREST depth cluster, which is the right answer when the box holds one
                    # object and the wrong one when a nearer object overlaps it in image space.
                    # These record what the box actually contained, so the flag can be designed
                    # from data instead of guessed: how many clusters, how much of the box the
                    # kept one holds, and how far behind the next one sits.
                    row["n_clusters"] = len(groups)
                    # Every cluster's depth and support, so the question "is the object in a
                    # LATER cluster?" can be asked of the data instead of assumed.
                    row["cluster_depths"] = [float(np.median(Q[g, 0])) for g in groups]
                    row["cluster_sizes"] = [int(g.size) for g in groups]
                    row["cluster_xy"] = [(float(np.median(Q[g, 0])), float(np.median(Q[g, 1])))
                                         for g in groups]
                    row["fg_frac"] = float(groups[0].size) / float(Q.shape[0])
                    if len(groups) > 1:
                        d0 = float(np.median(Q[groups[0], 0]))
                        d1 = float(np.median(Q[groups[1], 0]))
                        row["bg_gap"] = d1 - d0
                        row["bg_frac"] = float(groups[1].size) / float(Q.shape[0])
                        row["bg_frac_all"] = 1.0 - row["fg_frac"]
                    pick = groups[0]
                    pred = predict(dprev_hist.get(oid), key) if oid else None
                    if pred is not None:
                        d = [abs(float(np.median(Q[g, 0])) - pred[0]) for g in groups]
                        best = int(np.argmin(d))
                        if d[best] <= max(1.5, 0.05 * abs(pred[0])):
                            pick = groups[best]
                    row["Dprev"] = (float(np.median(Q[pick, 0])), float(np.median(Q[pick, 1])))
                    if oid:
                        dprev_hist.setdefault(oid, []).append((key, row["Dprev"]))
                        del dprev_hist[oid][:-2]

                    usable = None if row["fallback"] else Q
                    row["NF"] = None if row["fallback"] else row["G+f"]
                    row["Dhold"] = ref["Dhold"].step(oid, key, Q)
                    row["DholdNF"] = ref["DholdNF"].step(oid, key, usable)
                    # The DropNF arm IS the production gate, not a copy of it. There used to be
                    # a second implementation here with an assert that the two agreed; on
                    # adps_2026-08-25_11-55-43 they diverged, because a dropped frame leaves the
                    # two gates' miss counters and histories in different states and nothing
                    # resynchronises them. A reference that can drift from the rule it checks is
                    # not a check. DepthGate stays for the Dhold arms, which are a DIFFERENT rule
                    # (it may switch cluster); only that arm needs it.
                    if row["NF"] is not None and production.admit(oid, key, *row["NF"]):
                        row["DropNF"] = row["NF"]
                    # item 1: no percentile cut after segmentation
                    if not row["fallback"]:
                        nx, ny, _ = nearest_depth_cluster(P[:, 0], P[:, 1], P[:, 2])
                        cand = (float(np.median(nx)), float(np.median(ny)))
                        if gate_nopct.admit(oid, key, *cand):
                            row["DropNF_nopct"] = cand
                    # item 2: a ground-only box is ranged from the camera instead of dropped
                    cand = row["NF"] if not row["fallback"] else row["CAM"]
                    if cand is not None and gate_cam.admit(oid, key, *cand):
                        row["DropNF_cam"] = cand
                    for name, g in sweep.items():
                        if row["NF"] is not None and g.admit(oid, key, *row["NF"]):
                            row[name] = row["NF"]

                row["band"] = G.band_of(math.hypot(pubx, puby))
                row["radar"] = None
                if radar_ok:
                    _, rr, raz = radar[ri]
                    p_s = G.R_SL @ np.array([pubx, puby]) + G.T_SL
                    _, oaz = G.cartesian_to_polar(p_s[0], p_s[1])
                    hit = G.match_radar_range(math.hypot(pubx, puby), float(oaz), bw, rr, raz)
                    if hit is not None:
                        row["radar"], row["radar_az"] = hit
                        row["obj_az"] = float(oaz)
                rows.append(row)
            done += 1
            if args.max_frames and done >= args.max_frames:
                break
        if args.max_frames and done >= args.max_frames:
            break

    print(f"[run] sweeps {done}  detections {len(rows)}  ({time.perf_counter() - t0:.0f} s)")
    ok = n_self > 0 and worst_self <= G.SELF_CHECK_TOL
    print(f"[self-check] A25 vs published /fused_bbox over {n_self}: worst {worst_self:.2e} m -> "
          f"{'PASS' if ok else 'FAIL'}")
    if not ok and args.deskew:
        # EXPECTED here, and only here. The check exists to prove the harness reproduces the node
        # on the SAME input; --deskew deliberately changes the input points, so /fused_bbox (which
        # was recorded without it) is no longer the right reference. Every other arm still has to
        # pass. Compare a deskewed dump against a non-deskewed one, never against the replay.
        print("[self-check] ignored because --deskew changes the input cloud on purpose; this "
              "dump is only comparable with another dump, not with the recorded /fused_bbox")
    elif not ok:
        sys.exit("self-check failed: not scoring arms whose baseline does not reproduce the node")
    print("[self-check] the DropNF arm calls the production DepthJumpGate directly")
    if args.dump:
        with open(args.dump, "wb") as fh:
            pickle.dump(rows, fh)

    seen = {}
    for r in rows:
        seen[r["cls"]] = seen.get(r["cls"], 0) + 1
    print(f"\n[classes] {dict(sorted(seen.items(), key=lambda kv: -kv[1]))}")
    fb = [r["cls"] for r in rows if r["fallback"]]
    print(f"[empty after segmentation] {len(fb)} ({100 * len(fb) / len(rows):.1f}%), "
          f"of which cones {fb.count('cone')}")
    print(f"[gate drops] production {production.dropped}  A10 {production_a10.dropped}")

    diagnose(rows)
    G.report(rows, PAIRED_ARMS)
    print("\n========== ARMS THAT MAY DROP A DETECTION (each scored on its own outputs) ==========")
    for title, sub in (("ALL", None), ("CONES", lambda r: r["cls"] == "cone"),
                       ("VEHICLES", lambda r: r["cls"] in ("car", "truck", "bus"))):
        spike_table(rows, ["A10", "G+f"] + DROP_ARMS, title, sub)
    radar_table(rows, ["A10", "G+f"] + DROP_ARMS)
    camera_reference(rows)
    if sweep:
        print("\n========== GATE SWEEP (DepthJumpGate on NF) ==========")
        spike_table(rows, ["NF"] + list(sweep), "ALL", None)
        radar_table(rows, ["NF"] + list(sweep))


def windows(rows, arm, subset=None, max_gap=0.25):
    """(range, deviation from time-interpolated same-id neighbours, row) per detection of ``arm``."""
    tracks = {}
    for r in sorted(rows, key=lambda x: x["t"]):
        if r["id"] and r[arm] is not None:
            tracks.setdefault(r["id"], []).append(r)
    out = []
    for h in tracks.values():
        for i in range(1, len(h) - 1):
            a, b, c = h[i - 1], h[i], h[i + 1]
            if not (0 < b["t"] - a["t"] <= max_gap and 0 < c["t"] - b["t"] <= max_gap):
                continue
            if subset and not subset(b):
                continue
            ra, rb, rc = (G._rng(x[arm]) for x in (a, b, c))
            w = (b["t"] - a["t"]) / (c["t"] - a["t"])
            out.append((rb, rb - (ra + w * (rc - ra)), b))
    return out


def diagnose(rows):
    print(f"\n[anatomy] the G+f rule's spikes: |range - same-id neighbours| > {BIG:.0f} m")
    w = windows(rows, "G+f", max_gap=0.15)
    dev = np.array([x[1] for x in w])
    spike = np.abs(dev) > BIG
    err = np.array([np.nan if x[2]["radar"] is None else G._rng(x[2]["G+f"]) - x[2]["radar"]
                    for x in w])
    print(f"  {100 * spike.mean():.1f}% of detections; nearer {100 * (dev[spike] < 0).mean():.0f}%"
          f" / farther {100 * (dev[spike] > 0).mean():.0f}%")
    for name, sel in (("nearer", spike & (dev < 0)), ("farther", spike & (dev > 0))):
        e = err[sel & np.isfinite(err)]
        if e.size:
            print(f"  {name:7s} spike frames vs radar: median {np.median(e):+.2f} m (n={e.size})")
    for flag in ("multi", "sliver", "fallback"):
        f = np.array([x[2][flag] for x in w])
        print(f"  {flag.upper():8s} on {100 * f.mean():4.1f}% of detections, {100 * f[spike].mean():4.1f}%"
              f" of spikes;  P(spike | flag) {100 * spike[f].mean() if f.any() else 0:4.1f}%"
              f"  P(spike | no flag) {100 * spike[~f].mean():4.1f}%")


def camera_reference(rows):
    """Cone and vehicle range from LiDAR arms vs the camera ground intercept (independent of the
    LiDAR point selection), and how camera fill-ins sit against their LiDAR neighbours."""
    print("\nLIDAR RANGE - CAMERA GROUND-INTERCEPT RANGE, same detection: median / robust sd (n)")
    groups = (("cones", lambda r: r["cls"] == "cone"),
              ("vehicles", lambda r: r["cls"] in ("car", "truck", "bus")))
    for title, sub in groups:
        print(f"  {title}")
        for a in ("A10", "G+f", "DropNF", "DropNF_nopct"):
            cells = []
            for lo, hi in ((10, 25), (25, 40), (40, 60)):
                e = np.array([G._rng(r[a]) - G._rng(r["CAM"]) for r in rows
                              if sub(r) and r[a] is not None and r["CAM"] is not None
                              and lo <= math.hypot(*r["pub"]) < hi])
                if e.size < 10:
                    cells.append(f"{'-':>20s}")
                    continue
                med = np.median(e)
                cells.append(f"{med:+6.2f} / {1.4826 * np.median(np.abs(e - med)):4.2f} ({e.size:4d})")
            print(f"    {a:14s} " + " ".join(f"{c:>20s}" for c in cells) + "    [10-25 | 25-40 | 40-60 m]")
    fb = [r for r in rows if r["fallback"]]
    print(f"\n[camera fill-in] ground-only boxes {len(fb)}; with a camera ground position "
          f"{sum(r['CAM'] is not None for r in fb)}; published by DropNF_cam "
          f"{sum(r['DropNF_cam'] is not None for r in fb)}")
    dev = []
    for rng_, d, r in windows(rows, "DropNF_cam"):
        if r["fallback"]:
            dev.append(abs(d))
    if dev:
        dev = np.array(dev)
        print(f"  camera fill-ins vs their LiDAR neighbours: median |dev| {np.median(dev):.2f} m, "
              f"p90 {np.percentile(dev, 90):.2f}, > 2 m {100 * np.mean(dev > 2):.1f}% (n={dev.size})")


def spike_table(rows, arms, title, subset):
    print(f"\n{title}: SPIKE RATE (|range - neighbours| > {BIG:.0f} m), % of scored detections")
    print(f"  {'arm':22s} {'published':>9s} "
          + "".join(f"{f'{lo}-{hi}':>9s}" for lo, hi in REPORT_BANDS) + f"{'all':>8s}")
    for a in arms:
        w = windows(rows, a, subset)
        if not w:
            continue
        rng = np.array([x[0] for x in w])
        dev = np.abs(np.array([x[1] for x in w]))
        pub = sum(r[a] is not None for r in rows if subset is None or subset(r))
        cells = []
        for lo, hi in REPORT_BANDS:
            sel = (rng >= lo) & (rng < hi)
            cells.append(f"{100 * (dev[sel] > BIG).mean():8.1f}%" if sel.sum() >= 10 else f"{'-':>9s}")
        print(f"  {a:22s} {pub:9d} " + "".join(cells) + f"{100 * (dev > BIG).mean():7.1f}%")


def radar_table(rows, arms):
    print("\nRANGE ERROR vs radar on each arm's own outputs: median / robust sd (n)")
    print(f"  {'arm':22s} " + " ".join(f"{f'{lo}-{hi} m':>18s}" for lo, hi in REPORT_BANDS))
    for a in arms:
        cells = []
        for lo, hi in REPORT_BANDS:
            e = np.array([G._rng(r[a]) - r["radar"] for r in rows
                          if r[a] is not None and r["radar"] is not None
                          and lo <= math.hypot(*r["pub"]) < hi])
            if e.size < 10:
                cells.append(f"{'-':>18s}")
                continue
            med = np.median(e)
            cells.append(f"{med:+6.2f} / {1.4826 * np.median(np.abs(e - med)):4.2f} ({e.size:3d})")
        print(f"  {a:22s} " + " ".join(f"{c:>18s}" for c in cells))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--max-frames", type=int, default=None)
    ap.add_argument("--dump", default=None, help="pickle the per-detection rows here")
    ap.add_argument("--gate-sweep", action="store_true", help="also score a grid of gate widths")
    ap.add_argument("--level", choices=["off", "odom", "odom-nomount", "odom-near"], default="off",
                    help="level the sweep by odometry attitude before segmenting")
    ap.add_argument("--near-range", type=float, default=25.0,
                    help="odom-near: levelled labels inside this range, unlevelled beyond")
    ap.add_argument("--deskew", action="store_true",
                    help="undo ego motion WITHIN each sweep from the per-point `time` field before "
                         "segmenting and projecting -- the candidate cause of the yaw-rate-scaled "
                         "measurement jitter (scripts/velocity_truth_ab.py)")
    G.add_bag_args(ap)
    args = ap.parse_args()
    G.use_bags(args)
    run(args)


if __name__ == "__main__":
    main()
