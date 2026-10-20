#!/usr/bin/env python3
"""Score one LIVE vehicle pass: is the 360-degree cluster path holding tracks, and if not, why.

    scripts/fusion_live_check.py PASS_DIR            # a pass recorded by fusion_live_capture.sh
    scripts/fusion_live_check.py BAG_DIR --lag 0.12  # any bag with the same topics

The question it exists for (2026-10-06): on replay, clusters keep a track alive after it leaves
the camera view; on the Sep 24 vehicle bags they updated 0-0.2% of frames and the person dummy
died at the FOV edge. Three explanations, and the sections below separate them:

  1. PRODUCTION -- the chain (ground_projection -> /perception/nonground ->
     lidar_cluster_detector -> /perception/measurements/lidar) is not producing live.
     Read section A: rates and gaps of those topics.
  2. TIMING -- clusters arrive later than measurement_lag. The aggregator then applies them
     with NO prediction (dt <= 0) against tracks already advanced to a newer stamp, so a static
     object's cluster sits v * dt away from its track and misses the 1.0 + 0.02 r m gate.
     Read section B: "exposure", replayed from recorded arrival times.
  3. COVERAGE -- clusters arrive on time but no cluster lies on the object (a person-sized
     target in the side sector). Read section C: per death, was a cluster AVAILABLE within the
     gate at the same capture stamp, and was it USED.

"available but unused" points at 2, "none available" at 1 or 3. Arrival times are the
recorder's receive times; the recorder runs on the same host as the aggregator, so they stand
in for the aggregator's own (worth knowing if a number looks off by a few ms).
"""
from __future__ import annotations

import argparse
import bisect
import csv
import glob
import math
import os
import re
import struct
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory

REPO = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(REPO / "src/object_fusion"), str(REPO / "src/perception_common")]
from object_fusion.frames import lidar_to_ego  # noqa: E402
from object_fusion.lidar_clusters import CLUSTER_GATE_BASE_M, CLUSTER_GATE_PER_M  # noqa: E402

T_OBJ = "/perception/objects"
T_CL = "/perception/measurements/lidar"
T_CAM = "/perception/measurements/camera_lidar"
T_RAD = "/perception/measurements/radar"
T_NG = "/perception/nonground"
T_ODOM = "/novatel/oem7/odom"
T_ODOM_GRID = "/novatel/oem7/odom_grid"   # twist bit-identical to /odom; the fallback
MEAS = (T_CAM, T_RAD, T_CL)
#: Topics in section A, with the rate below which the stream counts as unhealthy.
HEALTH = [("/lidar_tc/velodyne_points", 9.0), (T_NG, 9.0), (T_CL, 9.0), (T_CAM, 5.0),
          (T_RAD, 15.0), (T_OBJ, 10.0), (T_ODOM, 50.0), ("/novatel/oem7/odom_grid", 50.0)]
CONTRIB_CAMERA, CONTRIB_LIDAR = 1, 2


def header_stamp(data: bytes) -> float:
    """Capture stamp of any message whose first field is std_msgs/Header, without decoding it.

    Every topic scored here starts with a Header, and the clouds are megabytes; reading 8 bytes
    after the 4-byte CDR encapsulation is what keeps a 20-minute pass quick to score.
    """
    little = data[1] & 1
    sec, nsec = struct.unpack_from("<iI" if little else ">iI", data, 4)
    return sec + nsec * 1e-9


def read_pass(bag_dir):
    files = sorted(glob.glob(os.path.join(bag_dir, "**", "*.mcap"), recursive=True))
    if not files:
        sys.exit(f"no .mcap under {bag_dir}")
    arr = defaultdict(list)          # topic -> [(arrival, stamp, nbytes)]
    clusters = []                    # (stamp, arrival, frame_id, Nx2 xy in ego)
    objects = []                     # (stamp, arrival, [(id, x, y, exist, contrib_frame, status, cls)])
    odom = defaultdict(list)         # topic -> [(stamp, speed)]
    dec = DecoderFactory()
    for f in files:
        with open(f, "rb") as fh:
            reader = make_reader(fh, decoder_factories=[dec])
            for schema, ch, msg in reader.iter_messages():
                t = ch.topic
                a = msg.log_time * 1e-9
                try:
                    s = header_stamp(msg.data)
                except struct.error:
                    continue
                arr[t].append((a, s, len(msg.data)))
                if t not in (T_CL, T_OBJ, T_ODOM, T_ODOM_GRID):
                    continue
                m = dec.decoder_for(ch.message_encoding, schema)(msg.data)
                if t in (T_ODOM, T_ODOM_GRID):
                    v = m.twist.twist.linear
                    odom[t].append((s, math.hypot(v.x, v.y)))
                elif t == T_CL:
                    xy = np.array([[d.pose.position.x, d.pose.position.y] for d in m.detections]
                                  ).reshape(-1, 2)
                    if m.header.frame_id != "ego" and len(xy):
                        xy = lidar_to_ego(xy)
                    clusters.append((s, a, m.header.frame_id, xy))
                else:
                    objects.append((s, a, [(o.track_id, o.pose.position.x, o.pose.position.y,
                                            o.existence_probability, o.contributions_this_frame,
                                            o.track_status, o.class_name) for o in m.objects]))
    for v in arr.values():
        v.sort()
    clusters.sort(key=lambda c: c[1])
    objects.sort(key=lambda o: o[1])
    odom = sorted(odom.get(T_ODOM) or odom.get(T_ODOM_GRID) or [])
    # A replay recording can hold several loops of one bag, and every stamp repeats per loop.
    # Live data never rewinds; for a recording that does, score the first loop only.
    stamps = [c[0] for c in clusters] or [o[0] for o in objects]
    if stamps and len(set(stamps)) < 0.9 * len(stamps):
        print(f"   WARNING: {len(stamps) - len(set(stamps))} repeated capture stamps -- a looped "
              "replay recorded on sim time. Arrival order is meaningless; B and C are INVALID.")
    cut = next((objects[i][1] for i in range(1, len(objects))
                if objects[i][0] < objects[i - 1][0] - 5.0), None)
    if cut is not None:
        print(f"   NOTE: capture stamps rewind {cut - objects[0][1]:.0f} s in (bag loop); "
              "scoring the first loop only")
        arr = {t: [r for r in v if r[0] < cut] for t, v in arr.items()}
        clusters = [c for c in clusters if c[1] < cut]
        objects = [o for o in objects if o[1] < cut]
        odom = sorted(o for o in odom if o[0] <= max((x[0] for x in objects), default=0))
    return arr, clusters, objects, odom


def pct(x, q):
    return float(np.percentile(x, q)) if len(x) else float("nan")


def speed_at(odom, t):
    if not odom:
        return float("nan")
    i = bisect.bisect_left(odom, (t,))
    i = min(max(i, 0), len(odom) - 1)
    return odom[i][1]


def section_a(arr, duration):
    print("\nA. STREAM HEALTH  (latency = receive time - capture stamp)")
    print(f"   {'topic':44s} {'Hz':>6s} {'max gap':>8s} {'lat med':>8s} {'p90':>7s} {'max':>7s}"
          f" {'MB/s':>6s}")
    verdict = []
    for topic, min_hz in HEALTH:
        rows = arr.get(topic, [])
        if not rows:
            print(f"   {topic:44s}   -- not recorded / no messages")
            if topic != "/lidar_tc/velodyne_points":
                verdict.append(f"{topic} SILENT")
            continue
        a = np.array([r[0] for r in rows])
        lat = a - np.array([r[1] for r in rows])
        hz = (len(a) - 1) / max(a[-1] - a[0], 1e-9)
        gap = float(np.max(np.diff(a))) if len(a) > 1 else float("nan")
        mbs = sum(r[2] for r in rows) / 1e6 / max(duration, 1e-9)
        flag = "  <-- LOW" if hz < min_hz else ""
        print(f"   {topic:44s} {hz:6.1f} {gap:7.2f}s {1e3 * np.median(lat):6.0f}ms "
              f"{1e3 * pct(lat, 90):5.0f}ms {1e3 * lat.max():5.0f}ms {mbs:6.2f}{flag}")
        if flag:
            verdict.append(f"{topic} at {hz:.1f} Hz")
    return verdict


def section_b(arr, clusters, odom, lag):
    """Replay the aggregator's fixed-lag queue from recorded arrivals.

    A measurement is released at max(arrival, stamp + lag); the clock (_last_t) is the newest
    stamp released so far and never runs back. A cluster released when _last_t is already past
    its own stamp is applied with no prediction, against tracks dt = _last_t - stamp ahead.
    """
    print(f"\nB. TIMING EXPOSURE  (measurement_lag = {lag:.3f} s)")
    ev = []
    for topic in MEAS:
        for a, s, _ in arr.get(topic, []):
            ev.append((max(a, s + lag), s, topic))
    ev.sort()
    last_t = -math.inf
    expo = defaultdict(list)            # topic -> [(dt_behind, speed)]
    late = defaultdict(int)
    for r, s, topic in ev:
        if r - s > lag + 1e-6:
            late[topic] += 1
        behind = max(0.0, last_t - s)
        expo[topic].append((behind, speed_at(odom, s)))
        last_t = max(last_t, s)
    gate10 = CLUSTER_GATE_BASE_M + CLUSTER_GATE_PER_M * 10.0
    print(f"   {'stream':38s} {'n':>6s} {'late':>6s} {'applied behind':>15s} {'med':>6s}"
          f" {'p90':>6s}  ego shift > {gate10:.1f} m gate")
    out = {}
    for topic in MEAS:
        e = expo.get(topic, [])
        if not e:
            print(f"   {topic:38s} {0:6d}")
            continue
        b = np.array([x[0] for x in e])
        shift = np.array([x[0] * x[1] for x in e])
        frac_behind = 100.0 * np.mean(b > 1e-6)
        frac_gate = 100.0 * np.nanmean(shift > gate10)
        print(f"   {topic:38s} {len(e):6d} {late[topic]:6d} {frac_behind:13.1f}% "
              f"{1e3 * np.median(b):4.0f}ms {1e3 * pct(b, 90):4.0f}ms  {frac_gate:5.1f}%")
        out[topic] = frac_gate
    cl_lat = np.array([a - s for a, s, _ in arr.get(T_CL, [])])
    if len(cl_lat):
        print(f"   cluster arrival latency p95 {1e3 * pct(cl_lat, 95):.0f} ms -> a measurement_lag "
              f"of {pct(cl_lat, 95) + 0.05:.2f} s would sequence 95% of clusters in order")
    if clusters:
        n = np.array([len(c[3]) for c in clusters])
        print(f"   clusters per sweep: median {np.median(n):.0f}, frame_id "
              f"{sorted({c[2] for c in clusters})}")
    return out


def section_c(objects, clusters, odom, have_clusters, horizon_s=2.0, max_range=40.0, cls_filter=None):
    print("\nC. TRACK USE OF CLUSTERS")
    n_frames = len(objects)
    n_cl_frames = sum(1 for _, _, objs in objects if any(o[4] & CONTRIB_LIDAR for o in objs))
    hist = defaultdict(list)       # id -> [(stamp, x, y, exist, contrib, status, cls)]
    for s, _, objs in objects:
        for o in objs:
            hist[o[0]].append((s,) + o[1:])
    fed = sum(1 for h in hist.values() if any(r[4] & CONTRIB_LIDAR for r in h))
    print(f"   /perception/objects frames with a cluster update: {n_cl_frames}/{n_frames} "
          f"({100.0 * n_cl_frames / max(n_frames, 1):.1f}%; replay ~6%)   tracks ever "
          f"cluster-fed: {fed}/{len(hist)} (replay 35-76%)")
    if not objects:
        return
    t_end = objects[-1][0]
    cl_by_stamp = sorted(clusters, key=lambda c: c[0])
    cl_stamps = [c[0] for c in cl_by_stamp]
    print(f"\n   Deaths ahead of / beside the car (x > 0, range < {max_range:.0f} m), last "
          f"{horizon_s:.0f} s of each life:")
    print(f"   {'t':>6s} {'id':>6s} {'class':10s} {'x':>5s} {'y':>5s} {'brg':>5s} {'v_ego':>5s} "
          f"{'ex':>4s} {'cam age':>7s} {'sweeps':>6s} {'avail':>5s} {'used':>4s} verdict")
    t0 = objects[0][0]
    rows = 0
    for tid, h in sorted(hist.items(), key=lambda kv: kv[1][-1][0]):
        s_d, x, y, ex, _, _, cls = h[-1]
        if t_end - s_d < 1.0 or x <= 0.0 or math.hypot(x, y) > max_range or len(h) < 5:
            continue
        if cls_filter and cls not in cls_filter:
            continue
        cam = [r[0] for r in h if r[4] & CONTRIB_CAMERA]
        cam_age = s_d - cam[-1] if cam else float("nan")
        lo = s_d - horizon_s
        used = sum(1 for r in h if r[0] >= lo and r[4] & CONTRIB_LIDAR)
        hs = [r[0] for r in h]
        sweeps = avail = 0
        for i in range(bisect.bisect_left(cl_stamps, lo), bisect.bisect_right(cl_stamps, s_d)):
            s_c, _, _, xy = cl_by_stamp[i]
            j = bisect.bisect_left(hs, s_c)
            cand = [k for k in (j - 1, j) if 0 <= k < len(h)]
            if not cand:
                continue
            k = min(cand, key=lambda k: abs(hs[k] - s_c))
            if abs(hs[k] - s_c) > 0.06:
                continue
            sweeps += 1
            tx, ty = h[k][1], h[k][2]
            if len(xy):
                d = np.min(np.hypot(xy[:, 0] - tx, xy[:, 1] - ty))
                if d <= CLUSTER_GATE_BASE_M + CLUSTER_GATE_PER_M * math.hypot(tx, ty):
                    avail += 1
        if not have_clusters:
            verdict = "clusters not recorded"
        elif sweeps == 0:
            verdict = "no cluster sweeps -> PRODUCTION"
        elif avail == 0:
            verdict = "none on object -> COVERAGE"
        elif used < 0.5 * avail:
            verdict = "available, unused -> TIMING/gate"
        else:
            verdict = "used"
        print(f"   {s_d - t0:6.1f} {tid:6d} {cls[:10]:10s} {x:5.1f} {y:5.1f} "
              f"{math.degrees(math.atan2(y, x)):+5.0f} {speed_at(odom, s_d):5.1f} {ex:4.2f} "
              f"{(f'{cam_age:6.2f}s' if cam else '      -'):>7s} {sweeps:6d} {avail:5d} {used:4d} {verdict}")
        rows += 1
    if not rows:
        print("   (none)")


def section_d(pass_dir):
    p = Path(pass_dir)
    stats, gpu, log = p / "docker_stats.csv", p / "gpu.csv", p / "fusion_node.log"
    if not (stats.exists() or gpu.exists() or log.exists()):
        return
    print("\nD. LOAD AND NODE STATUS")
    if stats.exists():
        cpu = defaultdict(list)
        with open(stats) as f:
            for row in csv.reader(f):
                if len(row) >= 3:
                    try:
                        cpu[row[1]].append(float(row[2].rstrip("%")))
                    except ValueError:
                        pass
        for name in sorted(cpu, key=lambda n: -np.median(cpu[n])):
            print(f"   cpu {name:36s} median {np.median(cpu[name]):6.0f}%  max "
                  f"{max(cpu[name]):6.0f}%")
    if gpu.exists():
        util = defaultdict(list)
        with open(gpu) as f:
            for row in csv.reader(f):
                if len(row) >= 4:
                    try:
                        util[row[1].strip()].append(float(row[2].strip().split()[0]))
                    except (ValueError, IndexError):
                        pass
        for idx in sorted(util):
            print(f"   gpu {idx}  util median {np.median(util[idx]):3.0f}%  max {max(util[idx]):3.0f}%")
    if log.exists():
        text = log.read_text(errors="replace").splitlines()
        for node in ("object_aggregator", "lidar_cluster_detector", "ground_projection"):
            lines = [ln for ln in text if f"[{node}]" in ln]
            if lines:
                print(f"   {node}: {lines[-1].split(']: ', 1)[-1][:220]}")
        warn = [ln for ln in text if re.search(r"\[(WARN|ERROR)\]", ln)]
        if warn:
            print(f"   {len(warn)} WARN/ERROR lines in fusion_node.log, last: "
                  f"{warn[-1].split(']: ', 1)[-1][:160]}")


def read_meta(pass_dir):
    meta = {}
    p = Path(pass_dir) / "meta.env"
    if p.exists():
        for ln in p.read_text().splitlines():
            if "=" in ln:
                k, v = ln.split("=", 1)
                meta[k.strip()] = v.strip().strip('"')
    return meta


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("pass_dir")
    ap.add_argument("--lag", type=float, default=None,
                    help="measurement_lag in force (default: from meta.env, else 0.12)")
    ap.add_argument("--class", dest="cls", action="append",
                    help="only list deaths of this class (repeatable), e.g. --class person")
    ap.add_argument("--brief", action="store_true", help="sections A and B only (preflight)")
    args = ap.parse_args()
    meta = read_meta(args.pass_dir)
    lag = args.lag
    if lag is None:
        try:
            lag = float(meta.get("MEASUREMENT_LAG", ""))
        except ValueError:
            lag = 0.12
            print("   measurement_lag was not read back for this pass; ASSUMING the default 0.12 s "
                  "(override with --lag)")
    bag = os.path.join(args.pass_dir, "bag")
    bag = bag if os.path.isdir(bag) else args.pass_dir
    arr, clusters, objects, odom = read_pass(bag)
    all_a = [r[0] for v in arr.values() for r in v]
    duration = (max(all_a) - min(all_a)) if all_a else 0.0
    v = [x[1] for x in odom]
    print(f"== {meta.get('LABEL', os.path.basename(os.path.normpath(args.pass_dir)))}: "
          f"{duration:.0f} s, ego speed median {np.median(v) if v else float('nan'):.1f} "
          f"max {max(v) if v else float('nan'):.1f} m/s")
    if meta.get("NOTE"):
        print(f"   note: {meta['NOTE']}")
    verdict = section_a(arr, duration)
    section_b(arr, clusters, odom, lag)
    if args.brief:
        print("\nPREFLIGHT: " + ("GO" if not verdict else "NO-GO -- " + "; ".join(verdict)))
        return 0 if not verdict else 1
    section_c(objects, clusters, odom, T_CL in arr, cls_filter=args.cls)
    section_d(args.pass_dir)
    return 0


if __name__ == "__main__":
    sys.exit(main())
