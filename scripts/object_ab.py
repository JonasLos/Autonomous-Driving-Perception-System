#!/usr/bin/env python3
"""Offline A/B for the track-level object fusion estimator. No ROS graph.

Replays a recorded bag through the PRODUCTION modules in ``src/object_fusion`` -- imported,
never copied, the same rule ``radar_ab.py``/``patch_ab.py``/``lane_ab.py`` follow -- so what
is scored here is literally what the node will run.

Two things Phase 2 needs before a node is worth building:

  --ego-yaw      The ego-yaw A/B. The INS twist is body-referenced and the tracker's state
                 lives in lidar_tc; those frames differ by the measured -5.35 deg. Arm A
                 ignores it (today's behaviour); arm B rotates the twist properly.

                 MEASURED ON THIS BAG, and it settles how the diagnostic must be built: the
                 statistic works FILTER-FREE and does not work inside the filter.

                   filter-free  k(vs sin az)  A +0.875 +/- 0.002  (435 sigma)
                                              B +0.024 +/- 0.002  ( 14 sigma)   -- 37x better
                   in-filter    k(vs sin az)  A +0.020 +/- 0.010
                                              B +0.016 +/- 0.010   -- no separation at all

                 A CONSISTENT FRAME ERROR IS UNOBSERVABLE FROM INSIDE THE FILTER. Both the
                 prediction and the state carry the same wrong ego velocity, so the filter is
                 self-consistent in its own wrong frame and the innovation goes to zero. The
                 track's estimated velocity absorbs the bias; nothing residual is left to see.
                 Detecting it requires comparing against something OUTSIDE the filter -- raw
                 range rate against independently-known ego velocity, which is exactly what
                 the Phase 0 boresight fit did.

                 The bias is also ODD in azimuth, so any pooled mean or median cancels it
                 exactly. This is the same blindness radar_ab.py documents for the cos term.
                 Regress on sin(az) or see nothing.

  --filter-ab    Does the filter actually improve position accuracy? Scored against the one
                 independent reference available: radar range. For every object the radar also
                 sees, compare |obj_range - radar_range| for the RAW camera+LiDAR measurement
                 (what passthrough publishes) against the FILTERED track state. Also reports
                 frame-to-frame position jump, which is the stability half of the question.

                 This was not measurable until publish_mode became a real passthrough -- before
                 that both modes published the filtered state and only the covariance differed.

  --rejection    Camera innovation-gate rejection rate, by range band. Explains the 36%
                 measured offline against 2.9% seen on a short live window.

  --nis          Normalised innovation squared per sensor per range band. This is the only
                 tuning loop here with a defined right answer: the mean should match the
                 measurement dimension and ~95% of samples should fall under the chi-square
                 95th percentile. High means the model is too stiff; low means Q is inflated
                 and information is being thrown away.

Self-check first, always: the harness must reproduce the transform the shipped node logs
before any metric prints. Anything else is scoring an unvalidated pipeline.
"""
from __future__ import annotations

import argparse
import glob
import hashlib
import pickle
import math
import os
import sys

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (os.path.join(REPO, "src", "object_fusion"),
           os.path.join(REPO, "src", "perception_common"),
           os.path.join(REPO, "src", "radar_ros")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from mcap_ros2.reader import read_ros2_messages                      # noqa: E402

from object_fusion import frames                                      # noqa: E402
from object_fusion.ego_motion import EgoTwist, TwistBuffer            # noqa: E402
from object_fusion.measurement_queue import Measurement, MeasurementQueue  # noqa: E402
from object_fusion.tracker import sigma_along as _sigma_along        # noqa: E402
from object_fusion.tracker import (                                   # noqa: E402
    CAMERA_GATE_CHI2, RADAR_GATE_CHI2, compensated_range_rate, gated_update, sigma_cross,
    init_from_radar,
    kalman_update, lidar_measurement, predict, process_noise, radar_R, radar_h_and_H,
    range_is_trustworthy, wrap_deg,
)
from object_fusion.association import associate_radar, solve_assignment  # noqa: E402
from object_fusion.track_store import (                               # noqa: E402
    SENSOR_CAMERA, SENSOR_RADAR, Track, TrackStore, radar_expected,
)
from radar_ros.radar_geometry import cartesian_to_polar, gate_tracks  # noqa: E402

# IMU -> radar lever arm in lidar_tc, from tf_static: radar (2.915, -0.650), imu
# (-0.658, 0.159). Phase 0 fitted the LONGITUDINAL arm at 3.63 +/- 0.02 m, matching 3.573
# here; the lateral term did NOT fit (-0.111 measured vs -0.809 expected) and is unresolved,
# so it is carried from the survey and flagged rather than trusted.
LEVER_IMU_TO_RADAR = (3.573, -0.809)

FUSED = "/fused_bbox"
RADAR = "/delphi_esr_interface/radar/tracks"
ODOM = "/novatel/oem7/odom"
TFS = "/tf_static"

# The values the shipped radar node logs for this transform. The harness must reproduce them.
EXPECTED_TF_YAW = 5.443
EXPECTED_TF_T = (-2.964, 0.371)


class Sweep:
    __slots__ = ("range", "azimuth", "range_rate")

    def __init__(self, r, a, rr):
        self.range, self.azimuth, self.range_rate = r, a, rr


def load(bag):
    """Read the bag once into plain arrays. Returns (fused, radar, odom, R_sl, t_sl)."""
    fused, radar, odom, R_lr, t_lr = [], [], [], None, None
    st = lambda h: h.stamp.sec + h.stamp.nanosec * 1e-9
    for f in sorted(glob.glob(os.path.join(bag, "*.mcap"))):
        for m in read_ros2_messages(f, topics=[FUSED, RADAR, ODOM, TFS]):
            o, tp = m.ros_msg, m.channel.topic
            if tp == TFS and R_lr is None:
                for tr in o.transforms:
                    if tr.child_frame_id == "delphi_esr_radar":
                        q, t = tr.transform.rotation, tr.transform.translation
                        y = math.atan2(2 * (q.w * q.z + q.x * q.y),
                                       1 - 2 * (q.y * q.y + q.z * q.z))
                        c, s = math.cos(y), math.sin(y)
                        R_lr = np.array([[c, -s], [s, c]])
                        t_lr = np.array([t.x, t.y])
            elif tp == ODOM:
                tw = o.twist.twist
                odom.append((st(o.header), tw.linear.x, tw.linear.y, tw.angular.z))
            elif tp == FUSED:
                if o.detections:
                    fused.append((st(o.header),
                                  np.array([[d.bbox3d.center.position.x,
                                             d.bbox3d.center.position.y]
                                            for d in o.detections]),
                                  [d.id for d in o.detections],
                                  [d.class_name for d in o.detections]))
            elif tp == RADAR and len(o.tracks):
                n = len(o.tracks)
                arr = lambda a_, d=float: np.fromiter((getattr(k, a_) for k in o.tracks), d, n)
                g = gate_tracks(arr("range"), arr("angle"), arr("amplitude"),
                                arr("track_status", int), arr("update_count", int),
                                min_range=1.0, max_range=175.0,
                                min_amplitude=-1e9, min_update_count=0)
                if g.any():
                    radar.append((st(o.header), arr("range")[g], arr("angle")[g],
                                  arr("range_rate")[g]))
    if R_lr is None:
        sys.exit("no lidar_tc -> delphi_esr_radar in tf_static")
    # Node convention: map points lidar_tc -> radar, the inverse of the tf_static pose.
    return fused, radar, odom, R_lr.T, -R_lr.T @ t_lr


#: Object width by class, metres, for the azimuth match gate (radar_ros/config/class_averages).
_CLASS_WIDTH = {"car": 1.8, "truck": 1.9, "bus": 2.5, "train": 3.2, "cone": 0.5,
                "person": 0.6, "stop sign": 0.75, "fire hydrant": 0.5, "bench": 1.5}


def match_gate_deg(range_m, class_name=None):
    """Azimuth tolerance for calling a radar return the same object: the object's own half-extent
    plus the ESR's ~0.5 deg, floored at 1 deg. A car at 10 m subtends ~10 deg, so a fixed +-1 deg
    gate matches the wrong return -- see scripts/ground_ab.py:azimuth_gate_deg."""
    w = _CLASS_WIDTH.get(class_name or "", 1.8)
    return max(1.0, math.degrees(math.atan(0.5 * w / max(float(range_m), 1.0))) + 0.5)


def match_range_window(range_m):
    """Wide sanity bound on the range difference (3 m + 50%): excludes a 6.7 m object matched to a
    69.9 m return on the same bearing, while still admitting every error under test."""
    return 3.0 + 0.5 * float(range_m)


def _odom_feeder(odom):
    """A TwistBuffer plus a `feed(t)` that adds odometry up to t, as the live node receives it.

    Pre-loading a whole drive into an 8 s buffer keeps only its LAST 8 s, and every earlier query
    is then served the oldest retained sample -- a twist from minutes later. That is what made
    static objects look like they were moving at 31% of ego speed in an earlier run of this
    harness. Feeding it in stamp order reproduces the node and keeps the buffer small.
    """
    buf = TwistBuffer(duration=8.0)
    rows = sorted(odom)
    idx = {"i": 0}

    def feed(t):
        while idx["i"] < len(rows) and rows[idx["i"]][0] <= t + 0.5:
            st, vx, vy, w = rows[idx["i"]]
            buf.add(EgoTwist(st, vx, vy, w))
            idx["i"] += 1

    feed(rows[0][0] if rows else 0.0)
    return buf, feed


def load_measurements(pkl, arm="DropNF"):
    """A `scripts/neighbour_ab.py --dump` file -> the same stream shape as /fused_bbox.

    /fused_bbox is what the OLD rule published; the detector now runs segmentation + the
    percentile cut + the nearest depth cluster, drops boxes segmentation empties, and withholds
    depth outliers (DepthJumpGate). Scoring the filter on /fused_bbox therefore tunes it against an
    input it no longer receives. neighbour_ab already computes the production position per
    detection, behind a self-check that reproduces the shipped node, so its dump is that stream.
    """
    with open(pkl, "rb") as fh:
        rows = pickle.load(fh)
    by_t = {}
    for r in rows:
        p = r.get(arm)
        if p is None:
            continue
        xy, ids, names = by_t.setdefault(round(r["t"], 3), ([], [], []))
        xy.append([p[0], p[1]])
        ids.append(r["id"])
        names.append(r["cls"])
    out = [(t, np.asarray(v[0], dtype=float), v[1], v[2]) for t, v in sorted(by_t.items())]
    kept = sum(len(v[1]) for v in by_t.values())
    print(f"[measurements] {pkl} arm={arm}: {len(out)} frames, {kept} detections "
          f"(of {len(rows)} rows)")
    return out


def self_check(R_sl, t_sl):
    yaw = math.degrees(math.atan2(R_sl[1, 0], R_sl[0, 0]))
    ok = (abs(yaw - EXPECTED_TF_YAW) < 1e-2
          and abs(t_sl[0] - EXPECTED_TF_T[0]) < 1e-2
          and abs(t_sl[1] - EXPECTED_TF_T[1]) < 1e-2)
    print(f"[self-check] lidar_tc -> delphi_esr_radar: yaw {yaw:+.3f} deg "
          f"t=({t_sl[0]:+.3f}, {t_sl[1]:+.3f})   "
          f"node logs {EXPECTED_TF_YAW:+.3f} / ({EXPECTED_TF_T[0]:+.3f}, {EXPECTED_TF_T[1]:+.3f})"
          f"  -> {'PASS' if ok else 'FAIL'}")
    return ok


def run(fused, radar, odom, R_sl, t_sl, *, ego_yaw_deg, collect_nis=False,
        sigma_long=2.0, sigma_lat=1.0, lag=0.12, max_frames=None,
        sigma_along_scale=1.0, range_trust=None, cam_gate=None, collect_ab=False,
        radar_holdout=0, lever=True, radar_range_gate=60.0, collect_speed=False,
        merge=True, count_objects=False, merge_range_gap=20.0, merge_bearing_deg=1.5,
        radar_birth=True, assoc_max_dist=6.0, sigma_cross_scale=1.0, merge_chi2=9.21,
        merge_max_dist=2.5, radar_camera_gate=False):
    """One arm. Returns a dict of diagnostics.

    ``collect_ab`` scores the RAW camera measurement (what publish_mode=passthrough publishes)
    against the FILTERED state, both versus radar range, inside this full pipeline -- i.e. WITH
    radar updates. ``filter_ab()`` answers the narrower question (camera updates only, no radar),
    which is the motion model on its own.

    ``radar_holdout=n`` (n > 1) is what makes that scoring HONEST: one track in every ``n`` never
    receives a radar update, and only those tracks are scored. Without it the filtered state has
    been fitted to the very radar ranges it is compared against, and "wins" by construction.
    """
    tw, feed_odom = _odom_feeder(odom)

    q = MeasurementQueue(lag=lag)
    for i, (t, xy, ids, names) in enumerate(fused):
        if max_frames and i >= max_frames:
            break
        q.add(Measurement(t, "camera_lidar", (xy, ids, names)))
    tmax = fused[min(len(fused), max_frames or len(fused)) - 1][0]
    for t, r, a, rr in radar:
        if t <= tmax:
            q.add(Measurement(t, "radar", Sweep(r, a, rr)))

    # Radar-only birth is OFF in the node by default. The harness turns it ON because
    # the ego-yaw diagnostic is measured on STATIC objects, and static objects --
    # guardrails, signs, manhole covers -- are exactly the radar-originated class the
    # camera never sees. Without them there is nothing stationary to measure.
    store = TrackStore(enable_radar_only_birth=radar_birth)
    last_t = None
    nis = {"camera_lidar": [], "radar": []}
    nis_banded = {}
    ab = {"raw": [], "filt": [], "jump_raw": [], "jump_filt": [], "banded": {}}
    speed_log = {}
    ab_prev = {"raw": {}, "filt": {}}
    ab_rts = np.array([r[0] for r in radar]) if radar else np.zeros(0)
    # Three metrics, because the obvious one does not work. A radar-only track's CROSS-RAY
    # velocity is never observed -- the radar measures range rate along the ray and nothing
    # else -- so |v| is dominated by process-noise wander and swamps the ~1.4 m/s ego-yaw
    # signal. The RADIAL component is observed, and the range-rate innovation is the most
    # direct read of all.
    static_speeds, all_speeds = [], []
    static_radial, static_rr_innov = [], []
    counts = {"cam_updates": 0, "radar_updates": 0, "births": 0, "gated_out": 0}
    cam_seen = {}      # track id -> (t, lidar xy) of its latest camera measurement

    for meas in q.drain():
        t = meas.stamp
        feed_odom(t)
        if last_t is None:
            last_t = t
        dt = t - last_t
        if dt > 0:
            inc = tw.increment(last_t, t)
            if inc is None:
                last_t = t
                continue
            dpsi, d_body = inc
            # The twist is BODY-referenced; the state lives in lidar_tc. Rotating between
            # them is the whole ego-yaw question. correction 0.0 reproduces today's
            # behaviour, where base_link is treated as lidar_tc.
            d_lidar = frames.ego_to_lidar([d_body], correction_deg=ego_yaw_deg)[0]
            Q = process_noise(dt, sigma_long, sigma_lat)
            for tr in store.tracks:
                tr.x, tr.P = predict(tr.x, tr.P, dt, dpsi, d_lidar, Q)
        last_t = max(last_t, t)          # never rewind: see object_aggregator_node._apply

        twist_now = tw.at(t)
        if twist_now is None:
            continue
        v_body = np.array([twist_now.vx, twist_now.vy])
        v_lidar = frames.ego_to_lidar([v_body], correction_deg=ego_yaw_deg)[0]
        # Velocity of the RADAR ORIGIN, not of the twist's reference point: omega x r.
        # The twist is reported at the IMU; the radar sits LEVER_IMU_TO_RADAR away, so its own
        # velocity carries omega x r. `lever=False` reproduces the shipped aggregator, which omits
        # this term -- the ablation that shows what the omission costs.
        lx, ly = LEVER_IMU_TO_RADAR if lever else (0.0, 0.0)
        v_lidar = v_lidar + twist_now.omega * np.array([-ly, lx])
        v_ego_s = R_sl @ v_lidar
        # Arm-INDEPENDENT staticness: classify with the measured-correct yaw in BOTH arms.
        # Selecting the sample with the arm's own ego estimate is a selection bias -- the
        # wrong arm simply never finds anything it calls static, which is what happened.
        v_ref = frames.ego_to_lidar([v_body])[0] + twist_now.omega * np.array([-ly, lx])
        v_ego_ref_s = R_sl @ v_ref

        if meas.sensor == "camera_lidar":
            xy, ids, names = meas.payload
            # Associate by nearest predicted position, with the ByteTrack id as a hard claim.
            used = set()
            for j, p in enumerate(xy):
                hit = None
                for k, tr in enumerate(store.tracks):
                    if k in used:
                        continue
                    if ids[j] and tr.bytetrack_id == ids[j]:
                        hit = k
                        break
                if hit is None:
                    best, bd = None, 1e9
                    for k, tr in enumerate(store.tracks):
                        if k in used:
                            continue
                        d = float(np.linalg.norm(tr.x[:2] - p))
                        if d < bd:
                            best, bd = k, d
                    if best is not None and bd < assoc_max_dist:
                        hit = best
                if hit is None:
                    trk = Track(np.array([p[0], p[1], 0.0, 0.0]),
                                np.diag([4.0, 4.0, 400.0, 400.0]), t, SENSOR_CAMERA)
                    trk.bytetrack_id = ids[j]
                    trk.vote_class(names[j])
                    store.add(trk)
                    counts["births"] += 1
                    continue
                used.add(hit)
                tr = store.tracks[hit]
                r = float(np.linalg.norm(tr.x[:2]))
                _trust = range_trust if range_trust is not None else 80.0
                _sa = None if sigma_along_scale == 1.0 else _sigma_along(r) * sigma_along_scale
                _sc = None if sigma_cross_scale == 1.0 else sigma_cross(r) * sigma_cross_scale
                y, H, R = lidar_measurement(tr.x[:2], p, sigma_a=_sa, sigma_c=_sc,
                                            drop_range=not range_is_trustworthy(r, _trust))
                tr.x, tr.P, n_, applied = kalman_update(tr.x, tr.P, y, H, R,
                                                        gate_chi2=cam_gate)
                tr.last_update = t
                tr.sensors_ever |= SENSOR_CAMERA
                tr.bytetrack_id = ids[j] or tr.bytetrack_id
                tr.vote_class(names[j])
                tr.hits["camera_lidar"] += 1
                if not applied:
                    counts["cam_gated"] = counts.get("cam_gated", 0) + 1
                if applied:
                    counts["cam_updates"] += 1
                    if collect_nis:
                        nis["camera_lidar"].append(n_)
                        band = int(min(r, 170) // 20) * 20
                        nis_banded.setdefault(("camera_lidar", band), []).append(n_)
                # passthrough publishes this measurement whether or not the gate applied it
                tr.last_cam_xy = np.asarray(p, dtype=float).copy()
                cam_seen[tr.id] = (t, tr.last_cam_xy)
            if collect_ab and ab_rts.size:
                _score_ab(store, t, radar, ab_rts, R_sl, t_sl, ab, ab_prev,
                          holdout=radar_holdout)
        else:
            sweep = meas.payload
            cref = None
            if radar_camera_gate:
                cref = []
                for tr_ in store.tracks:
                    seen_ = cam_seen.get(tr_.id)
                    cref.append(float(np.hypot(*(R_sl @ seen_[1] + t_sl)))
                                if seen_ is not None and t - seen_[0] <= 0.3 else None)
            pairs, _info = associate_radar(store.tracks, sweep, R_sl, t_sl, v_ego_s,
                                           max_azimuth_err_deg=1.0,
                                           max_range_err=radar_range_gate, camera_ref=cref)
            matched_det = {di for _, di in pairs}
            for ti, di in pairs:
                tr = store.tracks[ti]
                if _held_out(tr, radar_holdout):
                    continue                      # held out: scored, never radar-corrected
                z_pred, H = radar_h_and_H(tr.x, R_sl, t_sl, v_ego_s)
                y = np.array([sweep.range[di] - z_pred[0],
                              sweep.range_rate[di] - z_pred[1],
                              float(wrap_deg(sweep.azimuth[di] - z_pred[2]))])
                tr.x, tr.P, n_, applied = kalman_update(tr.x, tr.P, y, H, radar_R())
                tr.last_update = t
                tr.sensors_ever |= SENSOR_RADAR
                tr.hits["radar"] += 1
                tr.radar_hits += 1
                s = float(compensated_range_rate(sweep.range_rate[di], sweep.azimuth[di],
                                                 v_ego_ref_s))
                if abs(s) >= store.radar_only_min_speed:
                    tr.moving_hits += 1
                if applied:
                    counts["radar_updates"] += 1
                    # Item 12: does the radar return agree in RANGE with this track's own camera
                    # measurement from the same moment? A return from a different object behind
                    # the target shows up as a large positive difference.
                    seen = cam_seen.get(tr.id)
                    if seen is not None and t - seen[0] <= 0.3:
                        ps = R_sl @ seen[1] + t_sl
                        counts.setdefault("radar_vs_cam", []).append(
                            (float(np.hypot(*ps)), float(sweep.range[di] - np.hypot(*ps)),
                             # how far the update left the track from its own camera range
                             float(np.hypot(*tr.x[:2]) - np.hypot(*seen[1])),
                             1.0 if "cone" in (tr.class_name() or "") else 0.0))
                        counts.setdefault("radar_vs_cam_events", []).append(
                            (t, float(sweep.range[di]), float(sweep.azimuth[di]),
                             seen[1].copy(), tr.class_name() or ""))
                    if collect_nis:
                        nis["radar"].append(n_)
                        band = int(min(sweep.range[di], 170) // 20) * 20
                        nis_banded.setdefault(("radar", band), []).append(n_)
                else:
                    counts["gated_out"] += 1
                # Ground-referenced speed of a track the radar says is static over ground.
                if abs(s) < 0.8:
                    # Innovation in range rate, before this update was applied. For a target
                    # static over ground this must be ~0; a mis-rotated ego velocity biases it.
                    static_rr_innov.append((float(y[1]), float(sweep.azimuth[di])))
                    if tr.hits["radar"] >= 3:
                        static_speeds.append(tr.speed)
                        p = tr.x[:2]
                        nrm = float(np.linalg.norm(p))
                        if nrm > 1e-6:
                            static_radial.append(abs(float(tr.x[2:] @ (p / nrm))))
                all_speeds.append(tr.speed)

            R_ls, t_ls = R_sl.T, -R_sl.T @ t_sl
            for di in range(sweep.range.size):
                if di in matched_det:
                    continue
                x0, P0 = init_from_radar(sweep.range[di], sweep.azimuth[di],
                                         sweep.range_rate[di], v_ego_s, R_ls, t_ls)
                trk = Track(x0, P0, t, SENSOR_RADAR)
                trk.radar_slot = di
                store.add(trk)
                counts["births"] += 1

        if collect_speed:
            ego_sp = float(np.hypot(*v_lidar))
            for tr in store.tracks:
                if tr.hits.get("camera_lidar", 0) >= 3 and getattr(tr, "last_cam_xy", None) is not None:
                    v = np.array([tr.x[2], tr.x[3]])
                    sp = float(np.hypot(*v))
                    # cos of the angle between the track velocity and the EGO velocity: +1 means
                    # the track is drifting the way we are driving, which is what an
                    # under-compensated ego motion looks like on a static object.
                    cos = float(v @ v_lidar / (sp * ego_sp)) if sp > 1e-3 and ego_sp > 1e-3 else 0.0
                    speed_log.setdefault("all", []).append((sp, ego_sp, cos))
        store.prune(t)
        if merge:
            store.merge_pass(chi2=merge_chi2, max_merge_dist=merge_max_dist,
                             max_range_gap=merge_range_gap,
                             max_bearing_deg=merge_bearing_deg)
        store.promote()
        if count_objects and meas.sensor == "camera_lidar":
            xy = meas.payload[0]
            shown = [tr.x[:2] for tr in store.tracks
                     if store.may_publish(tr) and tr.hits.get("camera_lidar", 0) > 0]
            # the user's metric: a measurement with no published box on it is a lost object
            orphan = sum(1 for p in xy
                         if not any(float(np.hypot(p[0]-q[0], p[1]-q[1])) < 3.0 for q in shown))
            # what merging exists to prevent: two published boxes on one object. Counting
            # tracks with nothing under them instead would mostly count objects this camera
            # frame simply did not detect, which is not a duplicate.
            dup = sum(1 for i, q in enumerate(shown)
                      if any(float(np.hypot(q[0]-r[0], q[1]-r[1])) < 2.0
                             for j, r in enumerate(shown) if j != i))
            # and the failure the user saw in RViz: one box standing in for two cones
            shared = sum(1 for q in shown
                         if sum(1 for p in xy
                                if float(np.hypot(p[0]-q[0], p[1]-q[1])) < 3.0) >= 2)
            counts.setdefault("per_frame", []).append(
                (len(xy), len(shown), orphan, dup, shared))

    return {"speed": speed_log, "nis": nis, "nis_banded": nis_banded, "static_speeds": static_speeds,
            "static_radial": static_radial, "static_rr_innov": static_rr_innov,
            "all_speeds": all_speeds, "counts": counts, "shadow": store.shadow, "ab": ab}


def _held_out(tr, holdout):
    """Hold radar out of one CAMERA track in ``holdout``, keyed on the ByteTrack id.

    Keyed on the id string, not on the internal track id: internal ids depend on birth order,
    which changes with Q, so the held-out set would differ between arms and the comparison would
    not be paired.
    """
    if holdout <= 1:
        return False
    bid = getattr(tr, "bytetrack_id", None)
    if not bid:
        return False
    return (int(hashlib.md5(str(bid).encode()).hexdigest(), 16) % holdout) == 0


def _score_ab(store, t, radar, rts, R_sl, t_sl, ab, prev, holdout=0):
    """Raw camera measurement vs filtered state, both against the nearest radar sweep in time.

    Matched on AZIMUTH alone: range is the quantity under test, so gating on range would decide
    the answer in advance (the trap Phase 0 documents).
    """
    ri = int(np.argmin(np.abs(rts - t)))
    if abs(rts[ri] - t) > 0.05:
        return
    _, rrng, raz, _rrate = radar[ri]
    for tr in store.tracks:
        cam = getattr(tr, "last_cam_xy", None)
        if cam is None:
            continue
        if holdout > 1 and not _held_out(tr, holdout):
            continue                              # only radar-free tracks are a fair test
        for key, pos in (("raw", cam), ("filt", tr.x[:2])):
            p_s = R_sl @ np.asarray(pos, dtype=float) + t_sl
            orng, oaz = cartesian_to_polar(p_s[0], p_s[1])
            d_az = np.abs(raz - float(oaz))
            ok = ((d_az <= match_gate_deg(float(orng), getattr(tr, "class_name", None)))
                  & (np.abs(rrng - float(orng)) <= match_range_window(float(orng))))
            if np.any(ok):
                idx = np.flatnonzero(ok)
                k = idx[int(np.argmin(d_az[idx]))]
                err = abs(float(orng) - float(rrng[k]))
                ab[key].append(err)
                band = int(min(float(rrng[k]), 199) // 20) * 20
                ab["banded"].setdefault((key, band), []).append(err)
            if tr.id in prev[key]:
                ab["jump_" + key].append(float(np.linalg.norm(np.asarray(pos) - prev[key][tr.id])))
            prev[key][tr.id] = np.asarray(pos, dtype=float).copy()
        # LAG: a smoother that trails is not a win. Positive = the filtered range sits BEHIND the
        # raw one in the direction the object is actually moving.
        r_cam = float(np.linalg.norm(cam))
        r_filt = float(np.linalg.norm(tr.x[:2]))
        hist = prev.setdefault("rhist", {}).setdefault(tr.id, [])
        hist.append((t, r_cam, r_filt))
        del hist[:-3]
        if len(hist) == 3 and hist[2][0] > hist[0][0]:
            drdt = (hist[2][1] - hist[0][1]) / (hist[2][0] - hist[0][0])
            if abs(drdt) > 3.0:                   # only while the range is actually changing
                ab.setdefault("lag", []).append(math.copysign(1.0, drdt) * (r_cam - r_filt))


def ego_yaw_filter_free(radar, odom, R_sl):
    """The ego-yaw statistic with NO tracker in the loop. This is the one that works.

    For a target static over ground the compensated range rate must be ~0. A wrong ego yaw
    injects a term odd in azimuth, so it is recovered by regressing the residual on sin(az)
    -- never by a pooled mean, which cancels it.
    """
    od = np.array(odom)
    od = od[np.argsort(od[:, 0])]

    def ego_s(vb, w, yaw):
        v = frames.ego_to_lidar([vb], correction_deg=yaw)[0]
        v = v + w * np.array([-LEVER_IMU_TO_RADAR[1], LEVER_IMU_TO_RADAR[0]])
        return R_sl @ v

    acc = {"A": [[], []], "B": [[], []]}
    for t, rng, az, rr in radar:
        vx, vy, w = (np.interp(t, od[:, 0], od[:, i]) for i in (1, 2, 3))
        if vx < 3.0:
            continue
        vb = np.array([vx, vy])
        # Staticness classified once, with the measured-correct yaw, for BOTH arms.
        keep = np.abs(compensated_range_rate(rr, az, ego_s(vb, w, None))) < 0.8
        if not keep.any():
            continue
        for tag, yaw in (("A", 0.0), ("B", None)):
            acc[tag][0].append(compensated_range_rate(rr[keep], az[keep], ego_s(vb, w, yaw)))
            acc[tag][1].append(az[keep])

    out = {}
    for tag in ("A", "B"):
        if not acc[tag][0]:
            continue
        r = np.concatenate(acc[tag][0])
        a = np.concatenate(acc[tag][1])
        x = np.sin(np.radians(a))
        k = float(x @ r / (x @ x))
        res = r - k * x
        se = float(np.sqrt((res @ res) / max(r.size - 1, 1) / (x @ x)))
        out[tag] = (r.size, k, se, float(np.median(r)))
    return out


def filter_ab(fused, radar, odom, R_sl, t_sl, max_frames=None,
              sigma_long=2.0, sigma_lat=1.0):
    """Raw measurement vs filtered state, both scored against radar range.

    Radar range is ~0.1 m accurate, so for an object the radar also sees it is the closest
    thing to ground truth available. The question the filter has to answer is whether its
    state sits closer to that than the raw measurement does.
    """
    tw, feed_odom = _odom_feeder(odom)
    rts = np.array([r[0] for r in radar])
    store = TrackStore()
    last_t = None
    err_raw, err_filt, jump_raw, jump_filt = [], [], [], []
    prev_raw, prev_filt = {}, {}

    for fi, (t, xy, ids, names) in enumerate(fused):
        if max_frames and fi >= max_frames:
            break
        feed_odom(t)
        twist = tw.at(t)
        if twist is None:
            continue
        if last_t is not None and t > last_t:
            inc = tw.increment(last_t, t)
            if inc is None:
                last_t = max(last_t, t)
                continue
            dpsi, d_body = inc
            d_l = frames.ego_to_lidar([d_body], correction_deg=None)[0]
            Q = process_noise(t - last_t, sigma_long, sigma_lat)
            for tr in store.tracks:
                tr.x, tr.P = predict(tr.x, tr.P, t - last_t, dpsi, d_l, Q)
        last_t = t if last_t is None else max(last_t, t)

        # camera update, keeping the RAW measurement alongside the filtered state
        used = set()
        for j, p in enumerate(xy):
            hit = None
            for k, tr in enumerate(store.tracks):
                if k in used:
                    continue
                if ids[j] and tr.bytetrack_id == ids[j]:
                    hit = k
                    break
            if hit is None:
                best, bd = None, 1e9
                for k, tr in enumerate(store.tracks):
                    if k in used:
                        continue
                    d = float(np.linalg.norm(tr.x[:2] - p))
                    if d < bd:
                        best, bd = k, d
                if best is not None and bd < 6.0:
                    hit = best
            if hit is None:
                trk = Track(np.array([p[0], p[1], 0.0, 0.0]),
                            np.diag([4.0, 4.0, 400.0, 400.0]), t, SENSOR_CAMERA)
                trk.bytetrack_id = ids[j]
                trk.last_cam_xy = np.asarray(p, dtype=float).copy()
                store.add(trk)
                continue
            used.add(hit)
            tr = store.tracks[hit]
            r = float(np.linalg.norm(tr.x[:2]))
            y, H, R = lidar_measurement(tr.x[:2], p, drop_range=not range_is_trustworthy(r))
            tr.x, tr.P, _, ok, _f = gated_update(
                tr.x, tr.P, y, H, R, gate_chi2=CAMERA_GATE_CHI2,
                consecutive_rejects=tr.consecutive_rejects)
            tr.consecutive_rejects = 0 if ok else tr.consecutive_rejects + 1
            tr.last_cam_xy = np.asarray(p, dtype=float).copy()
            tr.last_update = t
            tr.sensors_ever |= SENSOR_CAMERA

        # score against the radar sweep nearest this frame
        ri = int(np.argmin(np.abs(rts - t)))
        if abs(rts[ri] - t) > 0.05:
            continue
        _, rrng, raz, rrate = radar[ri]
        for tr in store.tracks:
            if tr.last_cam_xy is None:
                continue
            for label, pos, errs, jumps, prev in (
                    ("raw", tr.last_cam_xy, err_raw, jump_raw, prev_raw),
                    ("filt", tr.x[:2], err_filt, jump_filt, prev_filt)):
                p_s = R_sl @ np.asarray(pos) + t_sl
                orng, oaz = cartesian_to_polar(p_s[0], p_s[1])
                # nearest radar return in AZIMUTH -- range is the quantity under test, so
                # gating on it would decide the answer in advance.
                d_az = np.abs(raz - float(oaz))
                ok = ((d_az <= match_gate_deg(float(orng), getattr(tr, "class_name", None)))
                      & (np.abs(rrng - float(orng)) <= match_range_window(float(orng))))
                if np.any(ok):
                    idx = np.flatnonzero(ok)
                    k = idx[int(np.argmin(d_az[idx]))]
                    errs.append(abs(float(orng) - float(rrng[k])))
                if tr.id in prev:
                    jumps.append(float(np.linalg.norm(np.asarray(pos) - prev[tr.id])))
                prev[tr.id] = np.asarray(pos, dtype=float).copy()
        store.prune(t)
    return err_raw, err_filt, jump_raw, jump_filt


def rejection_by_band(fused, radar, odom, R_sl, t_sl, max_frames=None):
    """Camera gate rejection rate, binned by range. Explains 36% offline vs 2.9% live."""
    tw, feed_odom = _odom_feeder(odom)
    store = TrackStore()
    last_t = None
    stats = {}
    for fi, (t, xy, ids, names) in enumerate(fused):
        if max_frames and fi >= max_frames:
            break
        feed_odom(t)
        if tw.at(t) is None:
            continue
        if last_t is not None and t > last_t:
            inc = tw.increment(last_t, t)
            if inc is None:
                last_t = max(last_t, t)
                continue
            dpsi, d_body = inc
            d_l = frames.ego_to_lidar([d_body], correction_deg=None)[0]
            Q = process_noise(t - last_t, 2.0, 1.0)
            for tr in store.tracks:
                tr.x, tr.P = predict(tr.x, tr.P, t - last_t, dpsi, d_l, Q)
        last_t = t if last_t is None else max(last_t, t)
        used = set()
        for j, p in enumerate(xy):
            hit = None
            for k, tr in enumerate(store.tracks):
                if k in used:
                    continue
                if ids[j] and tr.bytetrack_id == ids[j]:
                    hit = k
                    break
            if hit is None:
                best, bd = None, 1e9
                for k, tr in enumerate(store.tracks):
                    if k in used:
                        continue
                    d = float(np.linalg.norm(tr.x[:2] - p))
                    if d < bd:
                        best, bd = k, d
                if best is not None and bd < 6.0:
                    hit = best
            if hit is None:
                trk = Track(np.array([p[0], p[1], 0.0, 0.0]),
                            np.diag([4.0, 4.0, 400.0, 400.0]), t, SENSOR_CAMERA)
                trk.bytetrack_id = ids[j]
                store.add(trk)
                continue
            used.add(hit)
            tr = store.tracks[hit]
            r = float(np.linalg.norm(tr.x[:2]))
            y, H, R = lidar_measurement(tr.x[:2], p, drop_range=not range_is_trustworthy(r))
            tr.x, tr.P, _, ok, _f = gated_update(
                tr.x, tr.P, y, H, R, gate_chi2=CAMERA_GATE_CHI2,
                consecutive_rejects=tr.consecutive_rejects)
            tr.consecutive_rejects = 0 if ok else tr.consecutive_rejects + 1
            band = int(min(r, 170) // 20) * 20
            a, b = stats.get(band, (0, 0))
            stats[band] = (a + (0 if ok else 1), b + 1)
            tr.last_update = t
        store.prune(t)
    return stats


def rejection_runs(fused, radar, odom, R_sl, t_sl, max_frames=None):
    """Are rejections independent, or do they lock a track out?

    A hard gate with no fallback can be self-reinforcing: a rejected measurement does not
    update the track, so the state drifts further from the measurements, so the next
    innovation is larger, so it is rejected too. The signature is rejections clustering into
    long consecutive RUNS on the same track rather than scattering.

    If runs are short the tail is simply fat and the gate is doing its job. If runs are long
    the gate is locking tracks out, and it needs a forced-update escape after N misses.
    """
    tw, feed_odom = _odom_feeder(odom)
    store = TrackStore()
    last_t = None
    runs, cur = [], {}
    for fi, (t, xy, ids, names) in enumerate(fused):
        if max_frames and fi >= max_frames:
            break
        feed_odom(t)
        if tw.at(t) is None:
            continue
        if last_t is not None and t > last_t:
            inc = tw.increment(last_t, t)
            if inc is None:
                last_t = max(last_t, t)
                continue
            dpsi, d_body = inc
            d_l = frames.ego_to_lidar([d_body], correction_deg=None)[0]
            Q = process_noise(t - last_t, 2.0, 1.0)
            for tr in store.tracks:
                tr.x, tr.P = predict(tr.x, tr.P, t - last_t, dpsi, d_l, Q)
        last_t = t if last_t is None else max(last_t, t)
        used = set()
        for j, p in enumerate(xy):
            hit = None
            for k, tr in enumerate(store.tracks):
                if k in used:
                    continue
                if ids[j] and tr.bytetrack_id == ids[j]:
                    hit = k
                    break
            if hit is None:
                best, bd = None, 1e9
                for k, tr in enumerate(store.tracks):
                    if k in used:
                        continue
                    d = float(np.linalg.norm(tr.x[:2] - p))
                    if d < bd:
                        best, bd = k, d
                if best is not None and bd < 6.0:
                    hit = best
            if hit is None:
                trk = Track(np.array([p[0], p[1], 0.0, 0.0]),
                            np.diag([4.0, 4.0, 400.0, 400.0]), t, SENSOR_CAMERA)
                trk.bytetrack_id = ids[j]
                store.add(trk)
                continue
            used.add(hit)
            tr = store.tracks[hit]
            r = float(np.linalg.norm(tr.x[:2]))
            y, H, R = lidar_measurement(tr.x[:2], p, drop_range=not range_is_trustworthy(r))
            tr.x, tr.P, _, ok, _f = gated_update(
                tr.x, tr.P, y, H, R, gate_chi2=CAMERA_GATE_CHI2,
                consecutive_rejects=tr.consecutive_rejects)
            tr.consecutive_rejects = 0 if ok else tr.consecutive_rejects + 1
            if ok:
                if cur.get(tr.id):
                    runs.append(cur[tr.id])
                cur[tr.id] = 0
            else:
                cur[tr.id] = cur.get(tr.id, 0) + 1
            tr.last_update = t
        store.prune(t)
    runs.extend(v for v in cur.values() if v)
    return runs


CHI2_2DOF_Q = {25: 0.5754, 50: 1.3863, 75: 2.7726, 90: 4.6052, 95: 5.9915, 99: 9.2103}


def innovation_shape(fused, radar, odom, R_sl, t_sl, max_frames=None):
    """Empirical NIS quantiles per range band, against the chi-square(2) they should follow.

    This is the diagnostic that separates the two possible causes of a high rejection rate,
    which call for opposite fixes:

      * every quantile inflated by the SAME factor  -> sigma is uniformly too tight. Fix sigma;
        gating would then be throwing away good measurements.
      * low quantiles fine, high quantiles blown out -> genuinely heavy-tailed. Keep sigma and
        keep gating; inflating sigma would ruin the bulk to chase the tail.

    Ungated on purpose -- gating first would truncate the very tail under examination.
    """
    tw, feed_odom = _odom_feeder(odom)
    store = TrackStore()
    last_t = None
    bands = {}
    for fi, (t, xy, ids, names) in enumerate(fused):
        if max_frames and fi >= max_frames:
            break
        feed_odom(t)
        if tw.at(t) is None:
            continue
        if last_t is not None and t > last_t:
            inc = tw.increment(last_t, t)
            if inc is None:
                last_t = max(last_t, t)
                continue
            dpsi, d_body = inc
            d_l = frames.ego_to_lidar([d_body], correction_deg=None)[0]
            Q = process_noise(t - last_t, 2.0, 1.0)
            for tr in store.tracks:
                tr.x, tr.P = predict(tr.x, tr.P, t - last_t, dpsi, d_l, Q)
        last_t = t if last_t is None else max(last_t, t)
        used = set()
        for j, p in enumerate(xy):
            hit = None
            for k, tr in enumerate(store.tracks):
                if k in used:
                    continue
                if ids[j] and tr.bytetrack_id == ids[j]:
                    hit = k
                    break
            if hit is None:
                best, bd = None, 1e9
                for k, tr in enumerate(store.tracks):
                    if k in used:
                        continue
                    d = float(np.linalg.norm(tr.x[:2] - p))
                    if d < bd:
                        best, bd = k, d
                if best is not None and bd < 6.0:
                    hit = best
            if hit is None:
                trk = Track(np.array([p[0], p[1], 0.0, 0.0]),
                            np.diag([4.0, 4.0, 400.0, 400.0]), t, SENSOR_CAMERA)
                trk.bytetrack_id = ids[j]
                store.add(trk)
                continue
            used.add(hit)
            tr = store.tracks[hit]
            r = float(np.linalg.norm(tr.x[:2]))
            if not range_is_trustworthy(r):
                continue                      # 1-D lateral there; not comparable to chi2(2)
            y, H, R = lidar_measurement(tr.x[:2], p, drop_range=False)
            # UNGATED, so the tail survives to be measured.
            tr.x, tr.P, nis, _ = kalman_update(tr.x, tr.P, y, H, R)
            bands.setdefault(int(min(r, 79) // 20) * 20, []).append(nis)
            tr.last_update = t
        store.prune(t)
    return bands


def _stat(v):
    if not v:
        return "n/a"
    a = np.asarray(v)
    return f"n={a.size:6d} mean={a.mean():7.3f} median={np.median(a):7.3f} p95={np.percentile(a,95):8.3f}"


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag", nargs="?",
                    default="/home/avalocal/fused_replay_selfcal_2026-09-08")
    ap.add_argument("--ego-yaw", action="store_true", help="ego-yaw correction A/B")
    ap.add_argument("--nis", action="store_true", help="NIS per sensor per range band")
    ap.add_argument("--filter-ab", action="store_true",
                    help="raw measurement vs filtered state, scored against radar range")
    ap.add_argument("--rejection", action="store_true",
                    help="camera gate rejection rate by range band")
    ap.add_argument("--full-ab", action="store_true",
                    help="raw vs filtered inside the FULL pipeline (radar updates included), "
                         "overall and by radar-range band")
    ap.add_argument("--count-objects", action="store_true",
                    help="published tracks vs measurements per frame, with and without the "
                         "track merge -- does the aggregator lose objects?")
    ap.add_argument("--track-speed", action="store_true",
                    help="ground-referenced speed of camera-backed tracks; most objects on this "
                         "drive are static, so anything far from 0 is the filter's own error")
    ap.add_argument("--radar-holdout", type=int, default=0,
                    help="--full-ab: hold radar out of one track in N and score only those")
    ap.add_argument("--radar-assoc", action="store_true",
                    help="item 12: do applied radar updates agree in range with the same track's "
                         "camera measurement? Large disagreement = a different object on the bearing")
    ap.add_argument("--q-sweep", action="store_true",
                    help="can ANY process noise make the filter beat the raw measurement?")
    ap.add_argument("--runs", action="store_true",
                    help="consecutive-rejection run lengths: fat tail or gate lockout?")
    ap.add_argument("--sigma", action="store_true",
                    help="empirical NIS quantiles vs chi-square(2): is sigma wrong, or is the "
                         "distribution heavy-tailed?")
    ap.add_argument("--max-frames", type=int, default=None)
    ap.add_argument("--measurements", default=None,
                    help="score the filter on a scripts/neighbour_ab.py --dump file (the rule the "
                         "detector runs today) instead of the recorded /fused_bbox")
    ap.add_argument("--arm", default="DropNF", help="which neighbour_ab arm to use")
    args = ap.parse_args()

    print(f"[load] {args.bag}")
    fused, radar, odom, R_sl, t_sl = load(args.bag)
    print(f"[load] fused={len(fused)}  radar={len(radar)}  odom={len(odom)}")
    if not self_check(R_sl, t_sl):
        sys.exit("self-check failed; not scoring an unvalidated pipeline")
    if args.measurements:
        fused = load_measurements(args.measurements, args.arm)

    if args.ego_yaw or not (args.ego_yaw or args.nis or args.filter_ab or args.rejection):
        print("\n=== EGO-YAW A/B -- FILTER-FREE (the instrument that works) ===")
        ff = ego_yaw_filter_free(radar, odom, R_sl)
        for tag, lab in (("A", "correction  0.00 deg (today)   "),
                         ("B", "correction -5.35 deg (measured)")):
            if tag not in ff:
                continue
            n, k, se, med = ff[tag]
            print(f"  {tag}  {lab}  n={n:7d}  pooled median {med:+.4f} m/s")
            print(f"     k vs sin(az) = {k:+.4f} +/- {se:.4f} m/s   ({abs(k/se):6.1f} sigma)")
        if "A" in ff and "B" in ff and abs(ff["B"][1]) > 1e-9:
            print(f"  -> correction reduces the odd-in-azimuth bias "
                  f"{abs(ff['A'][1] / ff['B'][1]):.0f}x")

        print("\n=== EGO-YAW A/B -- IN-FILTER (documented NEGATIVE result) ===")
        print("  A consistent frame error is unobservable from inside the filter: the state")
        print("  absorbs it and the innovation goes to zero. Kept as a regression so nobody")
        print("  rebuilds this diagnostic expecting it to work.\n")
        for label, yaw in (("A  correction  0.00 deg (today) ", 0.0),
                           ("B  correction -5.35 deg (measured)", None)):
            out = run(fused, radar, odom, R_sl, t_sl, ego_yaw_deg=yaw,
                      max_frames=args.max_frames)
            sp = np.asarray(out["static_speeds"]) if out["static_speeds"] else np.array([])
            rad = np.asarray(out["static_radial"]) if out["static_radial"] else np.array([])
            innp = out["static_rr_innov"]
            inn = np.asarray([v for v, _ in innp]) if innp else np.array([])
            azs = np.asarray([a for _, a in innp]) if innp else np.array([])
            def med(a):
                return float(np.median(a)) if a.size else float("nan")
            print(f"  {label}")
            print(f"     |v| of static tracks      n={sp.size:6d}  median {med(sp):6.3f} m/s"
                  "   <- NOT sensitive: cross-ray velocity is unobserved")
            print(f"     radial |v| of static      n={rad.size:6d}  median {med(rad):6.3f} m/s"
                  "   <- observed by the radar")
            print(f"     range-rate innovation     n={inn.size:6d}  median {med(inn):+6.3f} m/s"
                  f"  mean {float(inn.mean()) if inn.size else float('nan'):+6.3f}"
                  "   <- POOLED: blind, the bias is odd in azimuth")
            if inn.size > 50:
                # A yaw error puts a term k*sin(az) into the static-target innovation. It is
                # ODD in azimuth, so any pooled statistic cancels it -- the same blindness
                # radar_ab.py documents for the cos term. Regress on sin(az) instead.
                x = np.sin(np.radians(azs))
                k = float(x @ inn / (x @ x))
                resid = inn - k * x
                se = float(np.sqrt((resid @ resid) / max(inn.size - 1, 1) / (x @ x)))
                print(f"     innovation vs sin(az)     k = {k:+6.3f} +/- {se:.3f} m/s"
                      f"  ({abs(k / se) if se else 0:5.1f} sigma)"
                      "   <- THE sensitive statistic")
            print()

    if args.filter_ab:
        _report_filter_ab(args, fused, radar, odom, R_sl, t_sl)

    if args.rejection:
        _report_rejection(args, fused, radar, odom, R_sl, t_sl)

    if args.count_objects:
        print("\n=== OBJECTS PUBLISHED vs MEASURED ===")
        print("  orphaned = a measurement with no track within 3 m (objects lost)")
        print("  dup      = a published track within 2 m of another one (two boxes, one object)")
        print("  shared   = a published track covering 2+ measurements (one box, two objects)")
        print(f"  {'arm':40s} {'meas/frame':>11s} {'published':>10s} {'orphaned':>10s}"
              f" {'dup':>9s} {'shared':>9s}")
        base = dict(radar_birth=False, cam_gate=CAMERA_GATE_CHI2)   # what the node runs
        # The merge gate's reach is bounded two ways: in units of covariance (chi2) and in
        # metres (merge_max_dist). Only the second tells a duplicate from the next cone.
        for label, kw in (("LIVE NOW: bound 2.5 m, chi2 9.21, assoc 6 m", dict()),
                          ("  as first shipped: no bound, assoc 6 m",
                           dict(merge_max_dist=float("inf"))),
                          ("  merge OFF entirely (the floor)", dict(merge=False)),
                          ("  chi2 -> 4.0, still unbounded", dict(merge_chi2=4.0,
                                                                  merge_max_dist=float("inf"))),
                          ("  chi2 -> 4.0, unbounded, assoc 4 m (rejected: merging never fires)",
                           dict(merge_chi2=4.0, merge_max_dist=float("inf"),
                                assoc_max_dist=4.0)),
                          ("  bound 2.5 m, chi2 9.21, assoc 4 m (rejected: doubles boxes)",
                           dict(assoc_max_dist=4.0)),
                          ("  bound 1.5 m, chi2 9.21, assoc 4 m", dict(merge_max_dist=1.5,
                                                                       assoc_max_dist=4.0)),
                          ("  bound 3.5 m, chi2 9.21, assoc 4 m", dict(merge_max_dist=3.5,
                                                                       assoc_max_dist=4.0)),
                          ("  bound 2.5 m, chi2 9.21, assoc 5 m", dict(assoc_max_dist=5.0))):
            kw = {**base, **kw}
            d = run(fused, radar, odom, R_sl, t_sl, ego_yaw_deg=-5.35, max_frames=args.max_frames,
                    count_objects=True, **kw)
            pf = np.array(d["counts"].get("per_frame", []))
            if pf.size:
                m, p = pf[:, 0].mean(), pf[:, 1].mean()
                orph = pf[:, 2].sum() / max(pf[:, 0].sum(), 1)
                dup = pf[:, 3].sum() / max(pf[:, 1].sum(), 1)
                shr = pf[:, 4].sum() / max(pf[:, 1].sum(), 1)
                print(f"  {label:40s} {m:11.2f} {p:10.2f} {100*orph:9.1f}%"
                      f" {100*dup:8.1f}% {100*shr:8.1f}%")

    if args.track_speed:
        print("\n=== TRACK SPEED -- what the filter thinks objects are doing ===")
        print("  Most detections on this drive are cones and roadside furniture: STATIC.")
        print(f"  {'arm':44s} {'n':>6s} {'median':>8s} {'p90':>7s} {'>2 m/s':>8s}")
        arms = (("shipped: Q 2.0/1.0, no lever arm", dict(lever=False, sigma_long=2.0, sigma_lat=1.0)),
                ("no radar at all (camera only), Q 2.0/1.0", dict(lever=False, sigma_long=2.0,
                                                                   sigma_lat=1.0,
                                                                   radar_range_gate=0.001)),
                ("Q 1.0/0.5", dict(lever=False, sigma_long=1.0, sigma_lat=0.5)),
                ("Q 0.5/0.25", dict(lever=False, sigma_long=0.5, sigma_lat=0.25)),
                ("Q 0.25/0.1", dict(lever=False, sigma_long=0.25, sigma_lat=0.1)),
                ("Q 0.1/0.05", dict(lever=False, sigma_long=0.1, sigma_lat=0.05)),
                ("Q 0.25/0.1 + lever arm", dict(lever=True, sigma_long=0.25, sigma_lat=0.1)))
        for label, kw in arms:
            d = run(fused, radar, odom, R_sl, t_sl, ego_yaw_deg=-5.35,
                    max_frames=args.max_frames, collect_speed=True, **kw)
            v = np.concatenate([np.asarray(x) for x in d["speed"].values()]) if d["speed"] else np.zeros(0)
            if v.size:
                print(f"  {label:44s} {v.size:6d} {np.median(v):7.2f}  {np.percentile(v,90):6.2f}"
                      f" {100*np.mean(v>2):7.0f}%")
            else:
                print(f"  {label:44s} {'no tracks':>6s}")

    if args.full_ab:
        print("\n=== FULL-PIPELINE A/B -- raw measurement vs filtered state (radar updates ON) ===")
        if args.radar_holdout > 1:
            print(f"  radar HELD OUT for one track in {args.radar_holdout}; only those are scored,"
                  f" so the reference is independent of the state.")
        else:
            print("  WARNING: radar updates are applied to the scored tracks, so the filtered "
                  "state is fitted to the reference. Use --radar-holdout 5 for an honest number.")
        for sl, sa in ((2.0, 1.0), (1.0, 0.5), (0.5, 0.25), (0.25, 0.1)):
            d = run(fused, radar, odom, R_sl, t_sl, ego_yaw_deg=-5.35, max_frames=args.max_frames,
                    sigma_long=sl, sigma_lat=sa, collect_ab=True,
                    radar_holdout=args.radar_holdout)
            ab, c = d["ab"], d["counts"]
            print(f"\n  Q: sigma_long={sl} sigma_lat={sa}   camera updates {c.get('cam_updates',0)}"
                  f"  radar updates {c.get('radar_updates',0)}  births {c.get('births',0)}")
            lag = np.asarray(ab.get("lag", []))
            if lag.size:
                print(f"    lag of the filtered range behind raw, while |dr/dt| > 3 m/s: "
                      f"median {np.median(lag):+.2f} m (n={lag.size})")
            for key, lab in (("raw", "raw measurement"), ("filt", "filtered state ")):
                a = np.asarray(ab[key])
                j = np.asarray(ab["jump_" + key])
                if a.size:
                    print(f"    {lab}  n={a.size:6d}  |range err| median {np.median(a):6.2f} "
                          f"mean {a.mean():6.2f} p90 {np.percentile(a, 90):6.2f}"
                          + (f"   jump median {np.median(j):5.2f} p90 {np.percentile(j, 90):5.2f}"
                             f" >2 m {100 * np.mean(j > 2):4.1f}%" if j.size else ""))
            bands = sorted({b for (_k, b) in ab["banded"]})
            print("    by radar range:  " + "  ".join(f"{b}-{b+20}" for b in bands))
            for key, lab in (("raw", "raw   "), ("filt", "filt  ")):
                cells = []
                for b in bands:
                    v = np.asarray(ab["banded"].get((key, b), []))
                    cells.append(f"{np.median(v):5.2f}({v.size:4d})" if v.size >= 10 else "    -    ")
                print(f"      {lab} " + " ".join(cells))

    if args.radar_assoc:
        print("\n=== RADAR ASSOCIATION: applied radar range minus the same track's camera range ===")
        print("  Only updates with a camera measurement from the same track within 0.3 s. Under")
        print("  80 m the camera range is trusted, so a big difference there is a wrong association;")
        print("  past 80 m the camera range is biased and the difference is expected.\n")
        # A per-track "trusted range cap" (3 sigma_along under 80 m) was tried here and REMOVED:
        # 29239 -> 29227 radar updates, 60-80 m unchanged. The disagreement sits within 3 sigma,
        # and once a track has been pulled toward a return, later returns agree with the TRACK.
        for label, gate in (("LIVE", False),
                            ("camera-referenced radar gate (3 sigma_along under 80 m)", True)):
            d = run(fused, radar, odom, R_sl, t_sl, ego_yaw_deg=-5.35,
                    max_frames=args.max_frames, radar_birth=False, cam_gate=CAMERA_GATE_CHI2,
                    radar_camera_gate=gate)
            rows = np.array(d["counts"].get("radar_vs_cam", []))
            print(f"  {label}   (radar updates applied: {d['counts']['radar_updates']})")
            print(f"  {'camera range':>13s} {'updates':>8s} {'|diff|>max(3m,10%)':>19s}"
                  f" {'radar BEHIND >3m':>17s} {'track pulled >3m off camera':>28s}")
            for lo, hi in ((0, 25), (25, 40), (40, 60), (60, 80), (80, 120), (120, 200)):
                m = (rows[:, 0] >= lo) & (rows[:, 0] < hi) if rows.size else np.array([], bool)
                if m.sum() < 10:
                    continue
                r, dd, pull = rows[m, 0], rows[m, 1], rows[m, 2]
                bad = np.abs(dd) > np.maximum(3.0, 0.1 * r)
                print(f"  {lo:5d}-{hi:<5d}   {m.sum():8d} {100 * bad.mean():18.1f}%"
                      f" {100 * np.mean(dd > 3.0):16.1f}% {100 * np.mean(np.abs(pull) > 3.0):27.1f}%")
            if rows.size and rows.shape[1] > 3:
                inb = (rows[:, 0] >= 40) & (rows[:, 0] < 80)
                for lab, sel in (("cones", inb & (rows[:, 3] > 0.5)),
                                 ("everything else", inb & (rows[:, 3] < 0.5))):
                    if sel.sum():
                        print(f"    40-80 m {lab:16s} updates {sel.sum():4d}   radar BEHIND >3 m "
                              f"{100 * np.mean(rows[sel, 1] > 3.0):5.1f}%   track pulled >3 m off "
                              f"its camera {100 * np.mean(np.abs(rows[sel, 2]) > 3.0):5.1f}%")
            print()

    if args.q_sweep:
        print("\n=== PROCESS-NOISE SWEEP ===")
        print("  the filter has to beat the raw measurement against radar range to be worth")
        print("  switching on. Raw is the same in every row -- only Q changes.\n")
        print(f"  {'sig_long':>9s} {'sig_lat':>8s} {'med err':>9s} {'mean':>8s} "
              f"{'p90':>8s} {'jump med':>9s} {'>2m':>6s}")
        base = None
        for sl, sa in ((0.25, 0.12), (0.5, 0.25), (1.0, 0.5), (2.0, 1.0), (4.0, 2.0)):
            er, ef, jr, jf = filter_ab(fused, radar, odom, R_sl, t_sl,
                                       max_frames=args.max_frames,
                                       sigma_long=sl, sigma_lat=sa)
            if base is None:
                a = np.asarray(er)
                base = (np.median(a), a.mean(), np.percentile(a, 90),
                        np.median(jr), 100 * np.mean(np.asarray(jr) > 2))
            f = np.asarray(ef); j = np.asarray(jf)
            print(f"  {sl:9.2f} {sa:8.2f} {np.median(f):9.2f} {f.mean():8.2f} "
                  f"{np.percentile(f,90):8.2f} {np.median(j):9.2f} "
                  f"{100*np.mean(j>2):5.1f}%")
        print(f"  {'RAW':>9s} {'--':>8s} {base[0]:9.2f} {base[1]:8.2f} {base[2]:8.2f} "
              f"{base[3]:9.2f} {base[4]:5.1f}%")

    if args.runs:
        print("\n=== CONSECUTIVE REJECTION RUNS ===")
        r = np.asarray(rejection_runs(fused, radar, odom, R_sl, t_sl,
                                      max_frames=args.max_frames))
        if r.size:
            print(f"  runs={r.size}  rejections={int(r.sum())}  "
                  f"median {np.median(r):.1f}  p90 {np.percentile(r,90):.0f}  max {r.max()}")
            for k in (1, 2, 3, 5, 10):
                print(f"    runs of >= {k:2d}: {int((r>=k).sum()):5d}  "
                      f"({100*float((r>=k).mean()):5.1f}% of runs, "
                      f"{100*float(r[r>=k].sum()/r.sum()):5.1f}% of all rejections)")
            print("\n  short runs -> fat tail, the gate is working.")
            print("  long runs  -> LOCKOUT: the gate is starving tracks it already starved.")
        else:
            print("  no rejections")

    if args.sigma:
        print("\n=== INNOVATION SHAPE vs CHI-SQUARE(2) ===")
        print("  ratio = empirical quantile / chi2(2) quantile.")
        print("  flat across quantiles  -> sigma uniformly too tight (scale it)")
        print("  rising with quantile   -> heavy-tailed (keep sigma, keep gating)\n")
        bands = innovation_shape(fused, radar, odom, R_sl, t_sl, max_frames=args.max_frames)
        qs = sorted(CHI2_2DOF_Q)
        print(f"  {'band':>10s} {'n':>6s} " + " ".join(f"q{q:<2d}".rjust(7) for q in qs))
        for band in sorted(bands):
            v = np.asarray(bands[band])
            if v.size < 60:
                continue
            ratios = [np.percentile(v, q) / CHI2_2DOF_Q[q] for q in qs]
            print(f"  {band:4d}-{band+20:<5d} {v.size:6d} "
                  + " ".join(f"{r:7.2f}" for r in ratios))
        print("\n  (a ratio of 1.00 everywhere is a perfectly calibrated R)")

    if args.nis:
        print("\n=== NIS ===")
        print("  target: mean ~ measurement dimension (camera 2, radar 3);")
        print("          ~95% under the chi-square 95th percentile (5.99 / 7.81).\n")
        out = run(fused, radar, odom, R_sl, t_sl, ego_yaw_deg=None, collect_nis=True,
                  max_frames=args.max_frames)
        for s in ("camera_lidar", "radar"):
            print(f"  {s:14s} {_stat(out['nis'][s])}")
        print("\n  by range band:")
        for (s, band) in sorted(out["nis_banded"]):
            v = out["nis_banded"][(s, band)]
            if len(v) < 30:
                continue
            thr = 5.99 if s == "camera_lidar" else 7.81
            under = 100.0 * float(np.mean(np.asarray(v) <= thr))
            print(f"    {s:14s} {band:3d}-{band+20:3d} m  {_stat(v)}  under-gate {under:5.1f}%")


def _report_filter_ab(args, fused, radar, odom, R_sl, t_sl):
    print("\n=== FILTER A/B -- raw measurement vs filtered state ===")
    print("  scored against radar range, the only ~0.1 m reference available.")
    print("  matched on AZIMUTH only: range is the quantity under test.\n")
    er, ef, jr, jf = filter_ab(fused, radar, odom, R_sl, t_sl, max_frames=args.max_frames)
    for lab, v in (("raw measurement (passthrough)", er), ("filtered state    (filtered)", ef)):
        a = np.asarray(v)
        if a.size == 0:
            print(f"  {lab}  no samples"); continue
        print(f"  {lab}  n={a.size:6d}  |range err| median {np.median(a):6.2f} m  "
              f"mean {a.mean():6.2f}  p90 {np.percentile(a,90):6.2f}")
    for lab, v in (("raw measurement", jr), ("filtered state ", jf)):
        a = np.asarray(v)
        if a.size == 0:
            continue
        print(f"  {lab}  frame-to-frame jump median {np.median(a):5.2f} m  "
              f"p90 {np.percentile(a,90):5.2f}  >2 m {100*np.mean(a>2):4.1f}%")


def _report_rejection(args, fused, radar, odom, R_sl, t_sl):
    print("\n=== CAMERA GATE REJECTION BY RANGE BAND ===")
    st = rejection_by_band(fused, radar, odom, R_sl, t_sl, max_frames=args.max_frames)
    tot_r = sum(v[0] for v in st.values()); tot_n = sum(v[1] for v in st.values())
    print(f"  {'band':>12s} {'updates':>9s} {'rejected':>9s} {'rate':>7s}")
    for band in sorted(st):
        rej, n = st[band]
        print(f"  {band:5d}-{band+20:<6d} {n:9d} {rej:9d} {100*rej/max(n,1):6.1f}%")
    print(f"  {'ALL':>12s} {tot_n:9d} {tot_r:9d} {100*tot_r/max(tot_n,1):6.1f}%")


if __name__ == "__main__":
    main()
