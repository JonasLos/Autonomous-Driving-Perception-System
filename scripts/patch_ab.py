#!/usr/bin/env python3
"""Offline A/B of pixel->point association rules for fusion_node, against a probe bag.

fusion_node places objects systematically too near. The ROS 1 stack did not, and the one
material difference is the pixel->point association rule: ROS 1 took a +-20px window around
the bbox *centre* (objects_transform.py:141-146, removed 2026-09-09 -- read it at
`git show pre-radar-known-good:src/yolov9_ros/objects_transform.py`), while fusion_node takes every point in the
full box and then tries to claw the road back out with reject_ground. The road in front of a
distant vehicle falls in the box's bottom rows, is genuinely nearer, and foreground_points
adopts it.

This script replays a recorded probe bag -- /yolo/tracking, /lidar_2d_projection and the
node's own /fused_bbox -- and scores several association rules against each other on the
same detections, with no GPU, no containers, no YOLO and no ROS graph. It runs in seconds,
so the sweep is cheap to repeat.

It is only worth trusting because of two self-checks that run before any metric is printed:
the pairing it reproduces must select the same projections the node did, and arm A must
reproduce the node's own published positions. Both are exact comparisons against the
recorded /fused_bbox. If either fails, nothing downstream of it means anything and the
script stops.

reject_ground and foreground_points are imported from fusion_node itself rather than copied,
so the baseline arm cannot drift away from the node it is meant to represent.

Arms
----
A       the rule in the node today: full box -> reject_ground -> foreground_points -> median
B_p     centre patch: |u-cx| <= p and |v-cy| <= p, intersected with the box. No
        reject_ground -- the patch is meant to supersede it. p in --patch-px.
F_q     box fraction: the top q of the box by height (v <= y_min + q*h), full width. Also
        without reject_ground. q in --box-frac.

B and F are each scored two ways: *drop*, where a detection whose mask catches fewer than
--min-points returns is not published at all (what ROS 1 did, via `if idx.size > 0`), and
*fallback*, where it falls back to arm A instead. The gap between the two columns is what
the drop-on-empty choice actually costs.

Why a box-fraction arm exists
-----------------------------
The centre-patch idea was originally justified by "1.29deg ring pitch at f=3461 -> 78px
constant ring spacing, so a 40px patch can only hold one ring band". That is wrong. The
LiDAR is a VLP-32C with a deliberately non-uniform beam pattern: measured off the ring field
of the 2026-08-20 bag, the pitch is 0.333deg across rings 9-25 -- the band where anything
past ~30m is imaged -- which is 20.1px at f=3461, not 78px. 1.29deg is 40deg/31, the average
over the full FOV, and it is the wrong statistic. Run with --lidar-bag to reprint that
measurement from the raw sweeps.

So the patch does not exclude the road by ring geometry. It excludes it by staying away from
the box's *bottom rows*, which is where the road returns were measured to sit (fusion_node.py
records them "pinned to the bottom 8% of the box"). That is a box-height effect, so the
protection weakens with range: a 1.5m car is 519px tall at 10m but 52px at 100m, where +-20px
clips only ~6px off each end. A fraction of box height is the scale-free way to say the same
thing, so it is swept alongside, and every metric is broken down by range band -- that is
where the two parameterisations should diverge if the difference is real.

Usage
-----
    source /opt/ros/jazzy/setup.bash
    source ~/.local/opt/adps_custom_msgs/setup.bash
    scripts/patch_ab.py /home/avalocal/probe_bag \
        --lidar-bag /home/avalocal/rosbag2_2026_08_20-13_11_07
"""

from __future__ import annotations

import argparse
import importlib.util
import os
import sys
from collections import defaultdict

import numpy as np
import yaml

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# perception_common is not installed on the host overlay (that builds only the message
# packages), so point at the source tree the containers build from.
sys.path.insert(0, os.path.join(REPO_ROOT, "src", "perception_common"))

import rosbag2_py  # noqa: E402
from rclpy.serialization import deserialize_message  # noqa: E402
from rosidl_runtime_py.utilities import get_message  # noqa: E402

from perception_common.stamp_sync import DEFERRED, StampMatchedBuffer  # noqa: E402
from perception_common.utils import stamp_to_seconds  # noqa: E402

FUSION_NODE_PY = os.path.join(
    REPO_ROOT, "src", "Custom_YOLO_ROS", "src", "yolo_ros",
    "yolo_ros", "yolo_ros", "fusion_node.py",
)

TRACKING_TOPIC = "/yolo/tracking"
PROJECTION_TOPIC = "/lidar_2d_projection"
FUSED_TOPIC = "/fused_bbox"

# fusion_node's own declared defaults. Overridable so a probe bag recorded under different
# settings can still be reproduced.
DEFAULTS = dict(
    max_pairing_skew=0.06,
    projection_buffer_duration=2.0,
    projection_stamp_offset=0.0,
    wait_for_newer=0.06,
    ground_rejection_min_range=25.0,
    ground_margin=0.4,
    ground_min_points=2,
)

RANGE_BANDS = [(0.0, 25.0), (25.0, 50.0), (50.0, 75.0), (75.0, 1e9)]


def load_fusion_module():
    """Import fusion_node.py by path, so the harness runs the node's own functions.

    Importing the module is side-effect free: everything that needs a ROS context lives
    inside FusionNode.__init__ or main().
    """
    spec = importlib.util.spec_from_file_location("fusion_node_under_test", FUSION_NODE_PY)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    for name in ("reject_ground", "foreground_points"):
        if not hasattr(module, name):
            raise SystemExit(
                f"{FUSION_NODE_PY} has no module-level {name}(). Step 2 of the plan -- "
                f"lifting the two pure functions out of FusionNode -- has to land first, "
                f"or this harness would be scoring a copy that can drift from the node."
            )
    return module


def open_bag(path):
    """Return a SequentialReader plus {topic: type_name}, honouring the bag's own storage id."""
    meta_path = os.path.join(path, "metadata.yaml")
    if not os.path.isfile(meta_path):
        raise SystemExit(f"{path} has no metadata.yaml -- point at the bag directory.")
    with open(meta_path, "r", encoding="utf-8") as f:
        meta = yaml.safe_load(f)
    storage_id = meta["rosbag2_bagfile_information"].get("storage_identifier", "mcap")

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=path, storage_id=storage_id),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    return reader, types


# --------------------------------------------------------------------------------------
# association rules
# --------------------------------------------------------------------------------------


class Box:
    """The pixel geometry of one detection, plus the point mask it selects."""

    __slots__ = ("cx", "cy", "w", "h", "x_min", "x_max", "y_min", "y_max", "mask")

    def __init__(self, det, u, v):
        self.cx = float(det.bbox.center.position.x)
        self.cy = float(det.bbox.center.position.y)
        self.w = float(det.bbox.size.x)
        self.h = float(det.bbox.size.y)
        self.x_min = self.cx - self.w / 2.0
        self.x_max = self.cx + self.w / 2.0
        self.y_min = self.cy - self.h / 2.0
        self.y_max = self.cy + self.h / 2.0
        self.mask = (
            (u >= self.x_min) & (u <= self.x_max) & (v >= self.y_min) & (v <= self.y_max)
        )


def decide(px, py, pz, fusion, *, ground):
    """Run the tail of the node's rule and return the published point plus the deciders.

    ``ground`` selects whether reject_ground runs first. The patch arms skip it: the patch
    is meant to supersede it, and leaving both on would confuse which one did the work.
    """
    if ground:
        px, py, pz = fusion.reject_ground(
            px, py, pz,
            min_range=DEFAULTS["ground_rejection_min_range"],
            margin=DEFAULTS["ground_margin"],
            min_points=DEFAULTS["ground_min_points"],
        )
    px, py, pz = fusion.foreground_points(px, py, pz)
    return (
        float(np.median(px)),
        float(np.median(py)),
        float(np.median(pz)),
        px,
        py,
        pz,
    )


def deciding_indices(px, pz, fusion, *, ground):
    """Which of the given points survive to decide the output, as indices into them.

    Used by the RViz replay to draw the exact returns each arm's median was taken over --
    that is the thing you actually want to look at, since it shows at a glance whether an
    arm settled on the vehicle body or on the road in front of it.

    The trick is that reject_ground and foreground_points both only ever *carry* their second
    argument -- neither reads ``py`` for any decision, they just index it alongside the other
    two -- so passing indices through that slot recovers the surviving rows exactly, without
    duplicating either rule or reaching inside the node's code to instrument it.
    """
    idx = np.arange(px.size, dtype=np.float64)
    if ground:
        px, idx, pz = fusion.reject_ground(
            px, idx, pz,
            min_range=DEFAULTS["ground_rejection_min_range"],
            margin=DEFAULTS["ground_margin"],
            min_points=DEFAULTS["ground_min_points"],
        )
    _, idx, _ = fusion.foreground_points(px, idx, pz)
    return idx.astype(int)


def fit_local_ground(bx, bz):
    """Fit the road under a box as a sloped line ``z = a*x + b`` through its lowest quartile.

    A flat threshold flatters the result: these roads fall ~1.4cm per metre, so over the
    depth a box spans the ground is measurably not level. Returns ``(a, b)``, or ``None``
    when the box holds too few points, or too little depth spread, to fit anything -- in
    which case a flat median is used instead of pretending to a slope.
    """
    if bx.size < 4:
        return None
    cut = float(np.percentile(bz, 25.0))
    low = bz <= cut
    if int(np.count_nonzero(low)) < 4:
        return None
    xq, zq = bx[low], bz[low]
    if float(np.ptp(xq)) < 1.0:
        return 0.0, float(np.median(zq))
    a, b = np.polyfit(xq, zq, 1)
    return float(a), float(b)


# --------------------------------------------------------------------------------------
# replay
# --------------------------------------------------------------------------------------


class Arm:
    """One association rule, scored both drop-on-empty and fallback-to-A.

    ``ground`` is deliberately a per-arm switch rather than a global one. Restricting which
    pixels are considered and switching reject_ground off are two separate changes, and the
    original proposal bundled them -- the patch was to "supersede" reject_ground. Scoring
    both variants of every mask is the only way to see which of the two did the work.
    """

    def __init__(self, name, kind, value, ground):
        self.name = name
        self.kind = kind          # "patch", "frac" or "box"
        self.value = value
        self.ground = ground
        self.offered = 0
        self.answered = 0
        self.rows = []            # dicts, one per detection where this arm answered
        self.fallback_rows = []   # same, but arm A's row substituted where it did not

    def select(self, box, u, v):
        if self.kind == "box":
            return box.mask
        if self.kind == "patch":
            p = self.value
            return box.mask & (np.abs(u - box.cx) <= p) & (np.abs(v - box.cy) <= p)
        return box.mask & (v <= box.y_min + self.value * box.h)


def build_arms(patch_px, box_frac):
    """Every mask, twice: once with reject_ground and once without."""
    arms = []
    for ground in (False, True):
        suffix = "+G" if ground else ""
        arms.append(Arm(f"box{suffix}", "box", None, ground))
        arms += [Arm(f"B_{p:g}px{suffix}", "patch", float(p), ground) for p in patch_px]
        arms += [Arm(f"F_{q:g}{suffix}", "frac", float(q), ground) for q in box_frac]
    return arms


def arm_from_name(name):
    """Build a single Arm from the name it is printed under, e.g. ``A``, ``B_20px``, ``F_0.4+G``.

    So the RViz replay shows exactly the arm this script scored, named the same way, rather
    than a second parameterisation that has to be kept in sync by hand.
    """
    base, ground = (name[:-2], True) if name.endswith("+G") else (name, False)
    if base == "A":
        # "A" is the node as it ships: the full box with reject_ground on.
        return Arm("A", "box", None, True)
    if base == "box":
        return Arm(name, "box", None, ground)
    if base.startswith("B_") and base.endswith("px"):
        return Arm(name, "patch", float(base[2:-2]), ground)
    if base.startswith("F_"):
        return Arm(name, "frac", float(base[2:]), ground)
    raise ValueError(
        f"unknown arm {name!r}; expected A, box, B_<px>px or F_<frac>, optionally with +G"
    )


class ProbeReplay:
    """Walks a probe bag, reproducing fusion_node's pairing exactly.

    Both the batch scorer and the RViz replay drive off this, so the pictures on screen and
    the numbers in the table can never come from two different pairings.

    ``events()`` yields, in bag order:
      ``("array", detections_msg)``   a tracking message was read
      ``("pair", (detections_msg, entry))``  the node fused these two together
      ``("unmatched", pairing)``      a detection array the node could not pair
      ``("fused", msg)``              a recorded /fused_bbox, for the self-check
    """

    def __init__(self, bag, args):
        self.reader, types = open_bag(bag)
        for topic in (TRACKING_TOPIC, PROJECTION_TOPIC, FUSED_TOPIC):
            if topic not in types:
                raise SystemExit(f"probe bag is missing {topic}; it has {sorted(types)}")
        self.msg_types = {
            t: get_message(types[t])
            for t in (TRACKING_TOPIC, PROJECTION_TOPIC, FUSED_TOPIC)
        }
        self.projections = StampMatchedBuffer(
            "projection",
            buffer_duration=args.projection_buffer_duration,
            max_skew=args.max_pairing_skew,
            stamp_offset=args.projection_stamp_offset,
            wait_for_newer=args.wait_for_newer,
        )

    def _complete(self, pairing):
        if pairing.value is None:
            return ("unmatched", pairing)
        return ("pair", (pairing.payload, pairing.value))

    def events(self):
        while self.reader.has_next():
            topic, data, recv_ns = self.reader.read_next()
            if topic not in self.msg_types:
                continue
            now = recv_ns * 1e-9
            # Stands in for the node's 0.02s pump timer. The node drains after every cloud
            # and on that timer; draining before every message is the closest thing here.
            for pairing in self.projections.drain(now):
                yield self._complete(pairing)

            msg = deserialize_message(data, self.msg_types[topic])
            if topic == PROJECTION_TOPIC:
                self.projections.add(msg)
                for pairing in self.projections.drain(now):
                    yield self._complete(pairing)
            elif topic == TRACKING_TOPIC:
                yield ("array", msg)
                pairing = self.projections.match(msg.header, now=now, payload=msg)
                if pairing.outcome is not DEFERRED:
                    yield self._complete(pairing)
            elif topic == FUSED_TOPIC:
                yield ("fused", msg)

        for pairing in self.projections.drain(float("inf")):
            yield self._complete(pairing)


def replay(bag, fusion, args):
    """Walk the probe bag, reproducing the node's pairing, and score every arm per detection."""
    source = ProbeReplay(bag, args)
    arms = build_arms(args.patch_px, args.box_frac)
    a_rows = []               # arm A, one row per detection it answered
    harness_outputs = []      # (projection stamp ns, [(det id, x, y, z), ...]) in publish order
    recorded_outputs = []
    n_detections = 0
    n_arrays = 0
    n_unmatched = 0
    n_box_smaller_than_patch = 0

    def fuse(detections_msg, entry):
        nonlocal n_detections, n_box_smaller_than_patch
        xyz, u, v = entry.arrays()
        published = []
        if xyz.shape[0]:
            x, y, z = xyz[:, 0], xyz[:, 1], xyz[:, 2]
            for det in detections_msg.detections:
                box = Box(det, u, v)
                if not np.any(box.mask):
                    continue        # the node's own `continue`: no box points, no output
                n_detections += 1
                bx, by, bz = x[box.mask], y[box.mask], z[box.mask]

                ax, ay, az, apx, _apy, apz = decide(bx, by, bz, fusion, ground=True)
                published.append((det.id, ax, ay, az))
                ground = fit_local_ground(bx, bz)
                a_row = row_for(det, ax, ay, az, apx, apz, ground, arm="A")
                a_rows.append(a_row)

                if min(box.w, box.h) < 2.0 * max(args.patch_px):
                    n_box_smaller_than_patch += 1

                for arm in arms:
                    arm.offered += 1
                    sel = arm.select(box, u, v)
                    n_sel = int(np.count_nonzero(sel))
                    if n_sel < args.min_points:
                        # ROS 1's `if idx.size > 0`: nothing to place the object on.
                        arm.fallback_rows.append(dict(a_row, arm=arm.name, fell_back=True))
                        continue
                    arm.answered += 1
                    sx, sy, sz, spx, _spy, spz = decide(
                        x[sel], y[sel], z[sel], fusion, ground=arm.ground
                    )
                    r = row_for(det, sx, sy, sz, spx, spz, ground, arm=arm.name)
                    r["range_A"] = a_row["range"]
                    arm.rows.append(r)
                    arm.fallback_rows.append(dict(r, fell_back=False))
        harness_outputs.append((stamp_ns(entry.header.stamp), published))

    for kind, payload in source.events():
        if kind == "array":
            n_arrays += 1
        elif kind == "unmatched":
            n_unmatched += 1
        elif kind == "pair":
            fuse(*payload)
        elif kind == "fused":
            recorded_outputs.append(
                (
                    stamp_ns(payload.header.stamp),
                    [
                        (
                            d.id,
                            float(d.bbox3d.center.position.x),
                            float(d.bbox3d.center.position.y),
                            float(d.bbox3d.center.position.z),
                        )
                        for d in payload.detections
                    ],
                )
            )

    return dict(
        arms=arms,
        a_rows=a_rows,
        harness_outputs=harness_outputs,
        recorded_outputs=recorded_outputs,
        n_arrays=n_arrays,
        n_detections=n_detections,
        n_unmatched=n_unmatched,
        n_box_smaller_than_patch=n_box_smaller_than_patch,
        status=source.projections.status(),
    )


def stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def row_for(det, x, y, z, px, pz, ground, *, arm):
    """One scored detection: where it was placed, and how far above the road that is."""
    if ground is None:
        height = float("nan")
    else:
        a, b = ground
        height = float(np.median(pz)) - (a * float(np.median(px)) + b)
    return dict(
        arm=arm,
        track=det.id,
        cls=det.class_name,
        range=float(x),
        y=float(y),
        z=float(z),
        height=height,
        n_points=int(px.size),
    )


# --------------------------------------------------------------------------------------
# self-checks
# --------------------------------------------------------------------------------------


def self_check(result, tol):
    """Prove the harness reproduces the node before any of its numbers are believed.

    Two independent things can be wrong: the pairing (did we pick the projection the node
    picked?) and the arithmetic (given that projection, did we place the object where the
    node placed it?). Both are checked against the recorded /fused_bbox, and both have to
    pass -- a pairing error would silently shift every arm by one frame of ego motion.
    """
    harness = result["harness_outputs"]
    recorded = result["recorded_outputs"]

    stamp_mismatch = 0
    position_mismatch = 0
    worst = 0.0
    compared = 0
    missing_ids = 0

    i = j = 0
    while i < len(harness) and j < len(recorded):
        h_stamp, h_dets = harness[i]
        r_stamp, r_dets = recorded[j]
        if h_stamp != r_stamp:
            # A watchdog publish carries the newest cloud's stamp and no detections; it has
            # no harness counterpart, so skip it rather than counting it as a mismatch.
            if not r_dets:
                j += 1
                continue
            stamp_mismatch += 1
            i += 1
            j += 1
            continue
        r_by_id = {d[0]: d[1:] for d in r_dets}
        for det_id, x, y, z in h_dets:
            if det_id not in r_by_id:
                missing_ids += 1
                continue
            rx, ry, rz = r_by_id[det_id]
            err = max(abs(x - rx), abs(y - ry), abs(z - rz))
            worst = max(worst, err)
            compared += 1
            if err > tol:
                position_mismatch += 1
        i += 1
        j += 1

    return dict(
        stamp_mismatch=stamp_mismatch,
        position_mismatch=position_mismatch,
        missing_ids=missing_ids,
        compared=compared,
        worst=worst,
        n_harness=len(harness),
        n_recorded=len(recorded),
        leftover_harness=len(harness) - i,
        leftover_recorded=len(recorded) - j,
    )


# --------------------------------------------------------------------------------------
# metrics
# --------------------------------------------------------------------------------------


def band_of(r):
    for lo, hi in RANGE_BANDS:
        if lo <= r < hi:
            return f"{lo:g}-{hi:g}m" if hi < 1e8 else f"{lo:g}m+"
    return "?"


def summarise(rows):
    if not rows:
        return None
    rng = np.array([r["range"] for r in rows])
    hgt = np.array([r["height"] for r in rows], dtype=float)
    hgt = hgt[~np.isnan(hgt)]
    npts = np.array([r["n_points"] for r in rows])
    return dict(
        n=len(rows),
        range_med=float(np.median(rng)),
        height_med=float(np.median(hgt)) if hgt.size else float("nan"),
        height_frac_elevated=float(np.mean(hgt > 0.4)) if hgt.size else float("nan"),
        points_med=float(np.median(npts)),
    )


def paired_delta(rows):
    d = np.array([r["range"] - r["range_A"] for r in rows if "range_A" in r])
    if d.size == 0:
        return None
    return dict(
        n=int(d.size),
        med=float(np.median(d)),
        q1=float(np.percentile(d, 25.0)),
        q3=float(np.percentile(d, 75.0)),
        frac_positive=float(np.mean(d > 0.0)),
    )


def track_jumps(rows, threshold):
    """Frame-to-frame range jumps per track: the road->vehicle 'snap' this rule should remove."""
    by_track = defaultdict(list)
    for r in rows:
        by_track[r["track"]].append(r["range"])
    jumps = steps = 0
    for seq in by_track.values():
        for a, b in zip(seq, seq[1:]):
            steps += 1
            if abs(b - a) > threshold:
                jumps += 1
    return jumps, steps, len(by_track)


# --------------------------------------------------------------------------------------
# ring pitch, measured rather than assumed
# --------------------------------------------------------------------------------------


def measure_ring_pitch(lidar_bag, focal):
    """Print the real per-ring beam pitch, in degrees and in pixels at the camera's focal length.

    This exists because the number this whole idea was originally justified with -- 78px
    constant ring spacing from a 1.29deg pitch -- is not what the sensor does.
    """
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2

    reader, types = open_bag(lidar_bag)
    topic_name = "/lidar_tc/velodyne_points"
    if topic_name not in types:
        print(f"  {lidar_bag} has no {topic_name}; skipping")
        return
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic_name]))
    if not reader.has_next():
        return
    _t, data, _ns = reader.read_next()
    msg = deserialize_message(data, PointCloud2)
    if "ring" not in [f.name for f in msg.fields]:
        print("  sweep carries no ring field; skipping")
        return

    pts = point_cloud2.read_points(msg, field_names=["x", "y", "z", "ring"], skip_nans=True)
    arr = np.array([[p[0], p[1], p[2], p[3]] for p in pts])
    xyz, ring = arr[:, :3], arr[:, 3].astype(int)
    rho = np.linalg.norm(xyz, axis=1)
    elev = np.degrees(np.arcsin(np.clip(xyz[:, 2] / np.maximum(rho, 1e-6), -1.0, 1.0)))

    per_ring = {k: float(np.median(elev[ring == k])) for k in np.unique(ring)
                if int(np.count_nonzero(ring == k)) > 50}
    keys = sorted(per_ring)
    pitches = np.abs(np.diff([per_ring[k] for k in keys]))
    px = focal * np.tan(np.radians(pitches))
    dense = pitches[pitches < 0.5]
    print(f"  {len(keys)} rings, elevation {per_ring[keys[0]]:+.2f} .. {per_ring[keys[-1]]:+.2f} deg")
    print(f"  pitch  min={pitches.min():.3f} median={np.median(pitches):.3f} max={pitches.max():.3f} deg")
    print(f"  px@f={focal:.0f}  min={px.min():.1f} median={np.median(px):.1f} max={px.max():.1f}")
    if dense.size:
        dense_px = focal * np.tan(np.radians(dense))
        print(f"  dense band ({dense.size} intervals below 0.5deg): "
              f"pitch {np.median(dense):.3f} deg -> {np.median(dense_px):.1f} px")
    print("  (the 78px figure this experiment was justified with assumed a uniform 1.29deg "
          "pitch, i.e. 40deg/31 -- the FOV average, not the pitch near the horizon)")


# --------------------------------------------------------------------------------------
# reporting
# --------------------------------------------------------------------------------------


def print_arm_table(title, per_arm, jump_threshold):
    print(f"\n{title}")
    print(f"  {'arm':>10} {'yield':>8} {'n':>6} {'range':>8} {'height':>8} "
          f"{'>0.4m':>7} {'pts':>5} {'dmed':>8} {'IQR':>15} {'d>0':>6} {'jumps':>12}")
    for name, rows, offered in per_arm:
        s = summarise(rows)
        if s is None:
            print(f"  {name:>10}      -- no answers")
            continue
        d = paired_delta(rows)
        jumps, steps, ntracks = track_jumps(rows, jump_threshold)
        yld = f"{100.0 * s['n'] / offered:.1f}%" if offered else "n/a"
        dtxt = f"{d['med']:+.2f}" if d else "   n/a"
        iqr = f"[{d['q1']:+.2f},{d['q3']:+.2f}]" if d else "n/a"
        dpos = f"{100 * d['frac_positive']:.0f}%" if d else "n/a"
        jtxt = f"{jumps}/{steps} ({ntracks}t)"
        print(f"  {name:>10} {yld:>8} {s['n']:>6} {s['range_med']:>8.2f} "
              f"{s['height_med']:>8.2f} {100 * s['height_frac_elevated']:>6.0f}% "
              f"{s['points_med']:>5.0f} {dtxt:>8} {iqr:>15} {dpos:>6} {jtxt:>12}")


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("bag", help="probe bag directory (/yolo/tracking, /lidar_2d_projection, /fused_bbox)")
    ap.add_argument("--lidar-bag", default=None,
                    help="source bag with raw /lidar_tc/velodyne_points, to reprint the ring pitch")
    ap.add_argument("--patch-px", type=float, nargs="+", default=[10, 15, 20, 25, 30, 39])
    ap.add_argument("--box-frac", type=float, nargs="+", default=[0.4, 0.5, 0.6])
    ap.add_argument("--min-points", type=int, default=1,
                    help="returns a patch must hold before it is allowed to place the object")
    ap.add_argument("--jump-threshold", type=float, default=5.0,
                    help="metres of frame-to-frame range change counted as a track snap")
    ap.add_argument("--tolerance", type=float, default=1e-4,
                    help="metres of disagreement allowed between arm A and the recorded /fused_bbox")
    ap.add_argument("--focal", type=float, default=3461.18)
    ap.add_argument("--skip-self-check", action="store_true",
                    help="report the self-check but do not stop on it (diagnostics only)")
    for name, value in DEFAULTS.items():
        if name.startswith("ground"):
            continue
        ap.add_argument(f"--{name.replace('_', '-')}", type=float, default=value)
    args = ap.parse_args()

    fusion = load_fusion_module()

    print(f"probe bag : {args.bag}")
    print(f"pairing   : max_skew={args.max_pairing_skew} wait_for_newer={args.wait_for_newer} "
          f"buffer={args.projection_buffer_duration} offset={args.projection_stamp_offset}")
    print(f"arm A     : reject_ground(min_range={DEFAULTS['ground_rejection_min_range']}, "
          f"margin={DEFAULTS['ground_margin']}, min_points={DEFAULTS['ground_min_points']}) "
          f"-> foreground_points -> median")

    result = replay(args.bag, fusion, args)

    print(f"\nreplayed  : {result['n_arrays']} detection arrays, "
          f"{result['n_detections']} detections with box points, "
          f"{result['n_unmatched']} unmatched")
    print(f"pairing   : {result['status']}")

    check = self_check(result, args.tolerance)
    print("\nself-check")
    print(f"  published arrays  harness={check['n_harness']} recorded={check['n_recorded']} "
          f"(leftover {check['leftover_harness']}/{check['leftover_recorded']})")
    print(f"  projection-stamp mismatches : {check['stamp_mismatch']}")
    print(f"  position mismatches         : {check['position_mismatch']} "
          f"of {check['compared']} compared (worst {check['worst']:.2e} m, tol {args.tolerance:g})")
    print(f"  detections absent from recording : {check['missing_ids']}")
    failed = check["stamp_mismatch"] or check["position_mismatch"] or not check["compared"]
    if failed and not args.skip_self_check:
        raise SystemExit(
            "\nSELF-CHECK FAILED. The harness is not reproducing the node, so no arm "
            "comparison below it would mean anything. Fix this before reading any metric."
        )
    print("  => arm A reproduces the recorded /fused_bbox" if not failed else "  => FAILED (continuing anyway)")

    if args.lidar_bag:
        print("\nmeasured ring geometry")
        measure_ring_pitch(args.lidar_bag, args.focal)

    a_rows = result["a_rows"]
    arms = result["arms"]
    offered = len(a_rows)

    no_ground = [a for a in arms if not a.ground]
    with_ground = [a for a in arms if a.ground]

    print("\n" + "=" * 118)
    print("ALL RANGES".center(118))
    print("=" * 118)
    print("  A = the node today (full box + reject_ground). 'box' isolates reject_ground's own")
    print("  contribution; the +G arms keep it under the restricted mask.")
    print_arm_table(
        "mask only, reject_ground OFF -- what the original proposal specified",
        [("A", a_rows, offered)] + [(a.name, a.rows, a.offered) for a in no_ground],
        args.jump_threshold,
    )
    print_arm_table(
        "mask + reject_ground ON -- separates 'restrict the pixels' from 'stop rejecting ground'",
        [("A", a_rows, offered)] + [(a.name, a.rows, a.offered) for a in with_ground],
        args.jump_threshold,
    )
    print_arm_table(
        "fallback-to-A instead of dropping (mask only, reject_ground OFF)",
        [(a.name, a.fallback_rows, a.offered) for a in no_ground],
        args.jump_threshold,
    )

    # Class matters more than range here. The whole hypothesis is about the road *in front of
    # a vehicle* being adopted instead of the vehicle body, and "the deciding points should be
    # elevated" is only the right answer for something with a body. A traffic cone stands
    # ~0.5m and sits on the road, so its returns are legitimately at road height -- an arm
    # that lifts a cone off the road has made it worse, not better, and pooling the two
    # classes hides that in whichever direction the class mix happens to lean.
    classes = sorted({r["cls"] for r in a_rows}, key=lambda c: -sum(
        1 for r in a_rows if r["cls"] == c))
    for cls in classes:
        cls_a = [r for r in a_rows if r["cls"] == cls]
        tracks = len({r["track"] for r in cls_a})
        print("\n" + "=" * 118)
        print(f"CLASS {cls}  ({len(cls_a)} detections, {tracks} distinct tracks)".center(118))
        if cls in ("cone", "traffic_cone"):
            print("  a cone is ~0.5m and sits ON the road: for this class, deciding points at "
                  "road height are CORRECT".center(118))
        print("=" * 118)
        per_arm = [("A", cls_a, len(cls_a))]
        for a in no_ground:
            per_arm.append((a.name, [r for r in a.rows if r["cls"] == cls], len(cls_a)))
        print_arm_table("mask only", per_arm, args.jump_threshold)

    for lo, hi in RANGE_BANDS:
        label = f"{lo:g}-{hi:g}m" if hi < 1e8 else f"{lo:g}m+"
        # Band on arm A's range so a detection stays in the same band across every arm,
        # which is what makes the paired delta within a band meaningful.
        band_a = [r for r in a_rows if band_of(r["range"]) == label]
        if not band_a:
            continue
        tracks = len({r["track"] for r in band_a})
        print("\n" + "=" * 118)
        print(f"RANGE BAND {label}  ({len(band_a)} detections, {tracks} distinct tracks)".center(118))
        print("=" * 118)
        for variant, group in (("mask only", no_ground), ("mask + reject_ground", with_ground)):
            per_arm = [("A", band_a, len(band_a))]
            for a in group:
                rows = [r for r in a.rows if band_of(r.get("range_A", r["range"])) == label]
                per_arm.append((a.name, rows, len(band_a)))
            print_arm_table(variant, per_arm, args.jump_threshold)

    print(f"\nnote: {result['n_box_smaller_than_patch']} of {offered} detections had a box "
          f"smaller than the widest patch ({2 * max(args.patch_px):g}px) in some dimension. "
          f"Patches are intersected with the box here, so for those the patch arm degenerates "
          f"towards arm A without reject_ground.")
    print("legend: height = median height of the deciding points above the box's own fitted "
          "ground line; >0.4m = share of detections decided by elevated returns rather than road; "
          "dmed/IQR = paired range change vs arm A; jumps = frame-to-frame range changes over "
          f"{args.jump_threshold:g}m, per track.")


if __name__ == "__main__":
    main()
