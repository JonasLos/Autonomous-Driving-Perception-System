#!/usr/bin/env python3
"""Replay a probe bag into RViz with several association rules published side by side.

scripts/patch_ab.py scores the rules against each other on ground-truth-free proxies. Those
proxies can be fooled -- on the 2026-08-20 bag the 50-75m band shows one arm gaining +11.8m
of range with 100% of detections moving outwards, which reads as a triumph until you notice
the deciding points never left road height. It moved to a farther road ring. This script is
how you catch that kind of thing by eye instead of by metric.

Every arm is computed from the *same* paired (detections, projection) stream as patch_ab.py
-- both drive off ProbeReplay -- so what you see here is what the table scored, frame for
frame. No GPU, no containers, no YOLO; it replays a 55MB probe bag.

Published, all on sim time (this script owns /clock, so nothing else should publish it):

  /lidar_2d_projection    the cloud fusion actually consumed, frame lidar_tc
  /ab/<arm>/points        the returns that arm's median was taken over -- the thing to look at
  /ab/<arm>/markers       sphere at the published position, text label with the range, and
                          for non-baseline arms a line back to the baseline's position, so
                          the disagreement is a literal line segment you can see the length of
  /ab/image               camera frame with the 2D box, each arm's mask window, and each arm's
                          deciding pixels drawn on it (needs --source-bag)
  /tf_static              passed through from the source bag

Controls on stdin: ENTER pauses/resumes, "n" steps one frame while paused, "q" quits.

Usage
-----
    source /opt/ros/jazzy/setup.bash
    source ~/.local/opt/adps_custom_msgs/setup.bash

    scripts/patch_ab_rviz.py /home/avalocal/probe_bag \
        --source-bag /home/avalocal/rosbag2_2026_08_20-13_11_07 \
        --arms A F_0.4 B_20px --rate 0.25 --skip-empty

    rviz2 -d config/patch_ab.rviz          # written on startup

Set the RViz fixed frame to lidar_tc. The camera and LiDAR frames are not related by
/tf_static on this vehicle (it carries lidar_tc -> camera_tl_arena_camera_node, not
camera_fl), so the Image display stands on its own rather than being projected into 3D.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import threading
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import (  # noqa: E402
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from builtin_interfaces.msg import Time as TimeMsg  # noqa: E402
from rosgraph_msgs.msg import Clock  # noqa: E402
from sensor_msgs.msg import Image, PointCloud2  # noqa: E402
from sensor_msgs_py import point_cloud2  # noqa: E402
from std_msgs.msg import Header  # noqa: E402
from tf2_msgs.msg import TFMessage  # noqa: E402
from visualization_msgs.msg import Marker, MarkerArray  # noqa: E402

from patch_ab import (  # noqa: E402
    Box,
    ProbeReplay,
    arm_from_name,
    deciding_indices,
    load_fusion_module,
    open_bag,
    stamp_ns,
)
import rosbag2_py  # noqa: E402
from rclpy.serialization import deserialize_message  # noqa: E402
from rosidl_runtime_py.utilities import get_message  # noqa: E402

IMAGE_TOPIC = "/camera_fl/image"
TF_STATIC_TOPIC = "/tf_static"

# Distinguishable at a glance against a grey point cloud, and distinguishable from each
# other for viewers with the common forms of colour blindness.
PALETTE = [
    (0.95, 0.25, 0.20),   # red      -- the baseline, listed first
    (0.20, 0.80, 0.35),   # green
    (0.35, 0.55, 1.00),   # blue
    (1.00, 0.80, 0.10),   # amber
    (0.85, 0.35, 0.90),   # magenta
]


# --------------------------------------------------------------------------------------
# pass 1: score every arm on the same pairing patch_ab.py uses
# --------------------------------------------------------------------------------------


def precompute(bag, fusion, arms, args):
    """Return per-frame arm results, plus the projections to republish.

    Holds the projection clouds in memory (~63MB for a 127s bag) so pass 2 can merge them
    against a streamed image topic on header stamps without walking the probe bag twice.
    """
    source = ProbeReplay(bag, args)
    frames = []          # one per fused (detections, projection) pair, in publish order
    projections = []     # (stamp_ns, PointCloud2)
    seen_projections = set()

    for kind, payload in source.events():
        if kind != "pair":
            continue
        detections_msg, entry = payload
        proj_ns = stamp_ns(entry.header.stamp)
        if proj_ns not in seen_projections:
            seen_projections.add(proj_ns)
            projections.append((proj_ns, entry.msg))

        xyz, u, v = entry.arrays()
        results = []
        if xyz.shape[0]:
            x, y, z = xyz[:, 0], xyz[:, 1], xyz[:, 2]
            for det in detections_msg.detections:
                if args.classes and det.class_name not in args.classes:
                    continue
                box = Box(det, u, v)
                if not np.any(box.mask):
                    continue
                box_idx = np.flatnonzero(box.mask)
                per_arm = {}
                for arm in arms:
                    sel = arm.select(box, u, v)
                    n_sel = int(np.count_nonzero(sel))
                    if n_sel < args.min_points:
                        per_arm[arm.name] = None          # dropped
                        continue
                    sel_idx = np.flatnonzero(sel)
                    local = deciding_indices(
                        x[sel_idx], z[sel_idx], fusion, ground=arm.ground
                    )
                    keep = sel_idx[local]
                    per_arm[arm.name] = dict(
                        position=(
                            float(np.median(x[keep])),
                            float(np.median(y[keep])),
                            float(np.median(z[keep])),
                        ),
                        points=np.column_stack((x[keep], y[keep], z[keep])),
                        pixels=np.column_stack((u[keep], v[keep])),
                        window=arm_window(arm, box),
                        n_selected=n_sel,
                    )
                results.append(dict(
                    track=det.id,
                    label=det.class_name,
                    box=(box.x_min, box.y_min, box.x_max, box.y_max),
                    n_box_points=int(box_idx.size),
                    arms=per_arm,
                ))
        frames.append(dict(
            proj_ns=proj_ns,
            det_ns=stamp_ns(detections_msg.header.stamp),
            detections=results,
        ))
    return frames, projections


def arm_window(arm, box):
    """The pixel rectangle an arm's mask corresponds to, for drawing on the image."""
    if arm.kind == "box":
        return (box.x_min, box.y_min, box.x_max, box.y_max)
    if arm.kind == "patch":
        p = arm.value
        return (
            max(box.x_min, box.cx - p), max(box.y_min, box.cy - p),
            min(box.x_max, box.cx + p), min(box.y_max, box.cy + p),
        )
    return (box.x_min, box.y_min, box.x_max, box.y_min + arm.value * box.h)


# --------------------------------------------------------------------------------------
# image
# --------------------------------------------------------------------------------------


def debayer_half(msg):
    """bayer_rggb8 -> half-resolution rgb8, in numpy alone.

    One 2x2 Bayer cell becomes one output pixel, which is exactly the information the sensor
    captured and needs no interpolation, no OpenCV (not installed on the host) and no
    cv_bridge. Halving also drops the published image from 3.2MB to 800KB per frame.
    Pixel coordinates therefore need halving too; draw_* below take full-res coordinates and
    scale internally so nothing else has to remember that.
    """
    h, w = msg.height, msg.width
    a = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, msg.step)[:, :w]
    if msg.encoding == "bayer_rggb8":
        r = a[0::2, 0::2]
        g = ((a[0::2, 1::2].astype(np.uint16) + a[1::2, 0::2]) // 2).astype(np.uint8)
        b = a[1::2, 1::2]
        return np.dstack((r, g, b))
    if msg.encoding in ("rgb8", "bgr8"):
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, msg.step // 3, 3)[:, :w]
        img = img[::2, ::2]
        return img if msg.encoding == "rgb8" else img[:, :, ::-1]
    if msg.encoding == "mono8":
        return np.dstack([a[::2, ::2]] * 3)
    raise SystemExit(f"unhandled image encoding {msg.encoding!r}")


def draw_rect(img, x0, y0, x1, y1, colour, thick=2):
    """Outline a full-resolution rectangle onto the half-resolution image."""
    h, w, _ = img.shape
    x0, x1 = sorted((int(x0 / 2), int(x1 / 2)))
    y0, y1 = sorted((int(y0 / 2), int(y1 / 2)))
    x0, x1 = max(0, x0), min(w, max(x1, 1))
    y0, y1 = max(0, y0), min(h, max(y1, 1))
    if x1 <= x0 or y1 <= y0:
        return
    c = np.array([int(255 * v) for v in colour], dtype=np.uint8)
    img[y0:min(y0 + thick, h), x0:x1] = c
    img[max(y1 - thick, 0):y1, x0:x1] = c
    img[y0:y1, x0:min(x0 + thick, w)] = c
    img[y0:y1, max(x1 - thick, 0):x1] = c


def draw_points(img, pixels, colour, radius=3):
    """Mark each deciding return's pixel, so you can see which returns an arm actually used."""
    h, w, _ = img.shape
    c = np.array([int(255 * v) for v in colour], dtype=np.uint8)
    for pu, pv in pixels:
        cu, cv = int(pu / 2), int(pv / 2)
        u0, u1 = max(0, cu - radius), min(w, cu + radius + 1)
        v0, v1 = max(0, cv - radius), min(h, cv + radius + 1)
        if u1 > u0 and v1 > v0:
            img[v0:v1, u0:u1] = c


# --------------------------------------------------------------------------------------
# publishing
# --------------------------------------------------------------------------------------


class Republisher(Node):
    def __init__(self, arms, frame):
        super().__init__("patch_ab_rviz")
        self.arms = arms
        self.frame = frame
        volatile = QoSProfile(
            depth=1, history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        latched = QoSProfile(
            depth=1, history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.clock_pub = self.create_publisher(Clock, "/clock", 10)
        self.cloud_pub = self.create_publisher(PointCloud2, "/lidar_2d_projection", volatile)
        self.image_pub = self.create_publisher(Image, "/ab/image", volatile)
        self.tf_pub = self.create_publisher(TFMessage, TF_STATIC_TOPIC, latched)
        self.marker_pubs = {
            a.name: self.create_publisher(MarkerArray, f"/ab/{topic_safe(a.name)}/markers", volatile)
            for a in arms
        }
        self.point_pubs = {
            a.name: self.create_publisher(PointCloud2, f"/ab/{topic_safe(a.name)}/points", volatile)
            for a in arms
        }

    def publish_clock(self, ns):
        msg = Clock()
        msg.clock = ns_to_time(ns)
        self.clock_pub.publish(msg)

    def publish_frame(self, frame, colours, baseline):
        stamp = ns_to_time(frame["proj_ns"])
        for arm_index, arm in enumerate(self.arms):
            markers = MarkerArray()
            clear = Marker()
            clear.header.stamp = stamp
            clear.header.frame_id = self.frame
            clear.ns = arm.name
            clear.id = 0
            clear.action = Marker.DELETEALL
            markers.markers.append(clear)

            pts = []
            mid = 1
            for det in frame["detections"]:
                res = det["arms"].get(arm.name)
                if res is None:
                    continue
                x, y, z = res["position"]
                pts.extend(res["points"].tolist())

                sphere = Marker()
                sphere.header.stamp = stamp
                sphere.header.frame_id = self.frame
                sphere.ns = arm.name
                sphere.id = mid
                mid += 1
                sphere.type = Marker.SPHERE
                sphere.action = Marker.ADD
                sphere.pose.position.x, sphere.pose.position.y, sphere.pose.position.z = x, y, z
                sphere.pose.orientation.w = 1.0
                sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.9
                set_colour(sphere, colours[arm.name], 0.85)
                markers.markers.append(sphere)

                text = Marker()
                text.header.stamp = stamp
                text.header.frame_id = self.frame
                text.ns = arm.name
                text.id = mid
                mid += 1
                text.type = Marker.TEXT_VIEW_FACING
                text.action = Marker.ADD
                text.pose.position.x, text.pose.position.y = x, y
                # Stacked by arm rather than all at one height: arms that agree place their
                # spheres within centimetres of each other, and co-located TEXT_VIEW_FACING
                # markers render straight through one another into unreadable mush.
                text.pose.position.z = z + 1.2 + 0.9 * arm_index
                text.pose.orientation.w = 1.0
                text.scale.z = 0.9
                # The class is on the label because it decides how to read the picture: a
                # cone standing on the road is *supposed* to be placed at road height, a car
                # is not, and this bag is 83% cones.
                text.text = (
                    f"{det['label']} {arm.name} {x:.1f}m ({res['points'].shape[0]}pt)"
                )
                set_colour(text, colours[arm.name], 1.0)
                markers.markers.append(text)

                # A line back to the baseline turns "these two disagree by 4 metres" from a
                # number into a segment whose length you can see against the point cloud.
                base = det["arms"].get(baseline)
                if arm.name != baseline and base is not None:
                    line = Marker()
                    line.header.stamp = stamp
                    line.header.frame_id = self.frame
                    line.ns = arm.name
                    line.id = mid
                    mid += 1
                    line.type = Marker.LINE_LIST
                    line.action = Marker.ADD
                    line.pose.orientation.w = 1.0
                    line.scale.x = 0.12
                    set_colour(line, colours[arm.name], 0.9)
                    line.points = [xyz_point(*base["position"]), xyz_point(x, y, z)]
                    markers.markers.append(line)

            self.marker_pubs[arm.name].publish(markers)
            header = Header(stamp=stamp, frame_id=self.frame)
            self.point_pubs[arm.name].publish(
                point_cloud2.create_cloud_xyz32(header, pts)
            )


def topic_safe(name):
    return name.replace("+", "_plus_").replace(".", "_")


def set_colour(marker, rgb, alpha):
    marker.color.r, marker.color.g, marker.color.b = rgb
    marker.color.a = alpha


def xyz_point(x, y, z):
    from geometry_msgs.msg import Point
    p = Point()
    p.x, p.y, p.z = float(x), float(y), float(z)
    return p


def ns_to_time(ns):
    t = TimeMsg()
    t.sec = int(ns // 1_000_000_000)
    t.nanosec = int(ns % 1_000_000_000)
    return t


# --------------------------------------------------------------------------------------
# rviz config
# --------------------------------------------------------------------------------------


def write_rviz_config(path, arms, colours, with_image):
    """Emit a config with one display per arm, since the arm set is chosen at runtime."""
    def rgb255(name):
        return "; ".join(str(int(255 * c)) for c in colours[name])

    displays = [
        {"Class": "rviz_default_plugins/Grid", "Name": "Grid", "Enabled": True,
         "Cell Size": 5, "Plane Cell Count": 40, "Color": "80; 80; 80"},
        {"Class": "rviz_default_plugins/PointCloud2", "Name": "projection",
         "Enabled": True, "Topic": {"Value": "/lidar_2d_projection", "Depth": 1,
                                    "Durability Policy": "Volatile",
                                    "Reliability Policy": "Reliable"},
         "Size (Pixels)": 2, "Style": "Points", "Color Transformer": "FlatColor",
         "Color": "130; 130; 130", "Alpha": 0.6, "Decay Time": 0},
    ]
    for arm in arms:
        displays.append({
            "Class": "rviz_default_plugins/PointCloud2", "Name": f"{arm.name} points",
            "Enabled": True,
            "Topic": {"Value": f"/ab/{topic_safe(arm.name)}/points", "Depth": 1,
                      "Durability Policy": "Volatile", "Reliability Policy": "Reliable"},
            "Size (Pixels)": 9, "Style": "Points", "Color Transformer": "FlatColor",
            "Color": rgb255(arm.name), "Alpha": 1.0, "Decay Time": 0,
        })
        displays.append({
            "Class": "rviz_default_plugins/MarkerArray", "Name": f"{arm.name} markers",
            "Enabled": True,
            "Topic": {"Value": f"/ab/{topic_safe(arm.name)}/markers", "Depth": 10,
                      "Durability Policy": "Volatile", "Reliability Policy": "Reliable"},
        })
    if with_image:
        displays.append({
            "Class": "rviz_default_plugins/Image", "Name": "camera + masks", "Enabled": True,
            "Topic": {"Value": "/ab/image", "Depth": 1,
                      "Durability Policy": "Volatile", "Reliability Policy": "Reliable"},
        })

    config = {
        "Panels": [
            {"Class": "rviz_common/Displays", "Name": "Displays", "Property Tree Widget":
             {"Expanded": [], "Splitter Ratio": 0.5}},
            {"Class": "rviz_common/Views", "Name": "Views"},
        ],
        "Visualization Manager": {
            "Class": "",
            "Name": "root",
            "Displays": displays,
            "Global Options": {
                "Background Color": "30; 30; 34",
                "Fixed Frame": "lidar_tc",
                "Frame Rate": 30,
            },
            "Tools": [{"Class": "rviz_default_plugins/MoveCamera"}],
            "Views": {
                "Current": {
                    "Class": "rviz_default_plugins/Orbit",
                    "Name": "Current View",
                    "Distance": 60.0,
                    "Focal Point": {"X": 30.0, "Y": 0.0, "Z": 0.0},
                    "Pitch": 0.35,
                    "Yaw": 3.14,
                    "Target Frame": "lidar_tc",
                },
                "Saved": None,
            },
        },
    }
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    # RViz reads YAML, and JSON is a subset of it -- which avoids taking a PyYAML dump
    # dependency on ordering quirks for a file nothing but RViz ever reads.
    with open(path, "w", encoding="utf-8") as f:
        json.dump(config, f, indent=2)


# --------------------------------------------------------------------------------------
# playback
# --------------------------------------------------------------------------------------


class Controls:
    """ENTER pauses/resumes, 'n' steps one frame, 'q' quits. Reads stdin on a daemon thread."""

    def __init__(self):
        self.paused = False
        self.step = False
        self.quit = False
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        for line in sys.stdin:
            cmd = line.strip().lower()
            if cmd == "q":
                self.quit = True
                return
            if cmd == "n":
                self.step = True
            else:
                self.paused = not self.paused
                print(f"[{'paused' if self.paused else 'running'}]", flush=True)

    def wait(self):
        while self.paused and not self.quit:
            if self.step:
                self.step = False
                return
            time.sleep(0.02)


def image_stream(source_bag):
    """Yield ``(stamp_ns, Image)`` from the source bag, one at a time.

    Streamed rather than buffered: 1272 frames at 3.2MB of Bayer each is ~4GB, which is why
    the projections are the side of the merge held in memory and the images are not.
    """
    reader, types = open_bag(source_bag)
    if IMAGE_TOPIC not in types:
        print(f"[warn] {source_bag} has no {IMAGE_TOPIC}; no image overlay")
        return
    typ = get_message(types[IMAGE_TOPIC])
    reader.set_filter(rosbag2_py.StorageFilter(topics=[IMAGE_TOPIC]))
    while reader.has_next():
        _topic, data, _recv = reader.read_next()
        msg = deserialize_message(data, typ)
        yield stamp_ns(msg.header.stamp), msg


def read_tf_static(source_bag):
    reader, types = open_bag(source_bag)
    if TF_STATIC_TOPIC not in types:
        return None
    typ = get_message(types[TF_STATIC_TOPIC])
    reader.set_filter(rosbag2_py.StorageFilter(topics=[TF_STATIC_TOPIC]))
    merged = TFMessage()
    while reader.has_next():
        _topic, data, _recv = reader.read_next()
        merged.transforms.extend(deserialize_message(data, typ).transforms)
    return merged


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("bag", help="probe bag directory")
    ap.add_argument("--source-bag", default=None,
                    help="original sensor bag, for the camera image and /tf_static")
    ap.add_argument("--arms", nargs="+", default=["A", "F_0.4", "B_20px"],
                    help="arms to publish; the first is the baseline the others draw a line to")
    ap.add_argument("--rate", type=float, default=0.25, help="playback rate (default 0.25)")
    ap.add_argument("--start-offset", type=float, default=0.0, help="skip the first N seconds")
    ap.add_argument("--skip-empty", action="store_true",
                    help="jump over frames where no arm placed anything")
    ap.add_argument("--classes", nargs="*", default=None,
                    help="only show these classes, e.g. --classes car bus. This bag is 83%% "
                         "cones, and a cone standing on the road is correctly placed at road "
                         "height, so the vehicle classes are what the rules should be judged on")
    ap.add_argument("--point-radius", type=int, default=2,
                    help="half-width, in output pixels, of the deciding-point markers")
    ap.add_argument("--min-points", type=int, default=1)
    ap.add_argument("--rviz-config", default=os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "config", "patch_ab.rviz"))
    ap.add_argument("--max-pairing-skew", type=float, default=0.06)
    ap.add_argument("--projection-buffer-duration", type=float, default=2.0)
    ap.add_argument("--projection-stamp-offset", type=float, default=0.0)
    ap.add_argument("--wait-for-newer", type=float, default=0.06)
    args = ap.parse_args()

    fusion = load_fusion_module()
    arms = [arm_from_name(n) for n in args.arms]
    baseline = arms[0].name
    colours = {a.name: PALETTE[i % len(PALETTE)] for i, a in enumerate(arms)}

    print(f"arms      : {', '.join(a.name for a in arms)}  (baseline {baseline})", flush=True)
    print("scoring the probe bag ...", flush=True)
    frames, projections = precompute(args.bag, fusion, arms, args)
    placed = sum(
        1 for f in frames for d in f["detections"] if any(v for v in d["arms"].values())
    )
    print(f"          : {len(frames)} fused frames, {placed} detections placed by at least one arm",
          flush=True)

    write_rviz_config(args.rviz_config, arms, colours, bool(args.source_bag))
    print(f"rviz      : rviz2 -d {args.rviz_config}", flush=True)
    print("controls  : ENTER pause/resume, 'n' step, 'q' quit", flush=True)

    rclpy.init()
    node = Republisher(arms, frame="lidar_tc")

    if args.source_bag:
        tf = read_tf_static(args.source_bag)
        if tf is not None:
            node.tf_pub.publish(tf)

    by_proj = {f["proj_ns"]: f for f in frames}
    by_det = {f["det_ns"]: f for f in frames}

    # Merge the in-memory projections against the streamed images on header stamps. Both
    # bags carry the same stamps for corresponding data -- transform.py copies the sweep
    # header onto its projection, and tracking copies the image header through -- so the two
    # recordings line up even though they were made in different sessions at different
    # wall-clock times.
    events = [(ns, "cloud", msg) for ns, msg in projections]
    images = image_stream(args.source_bag) if args.source_bag else iter(())

    controls = Controls()
    first_ns = min(ns for ns, _, _ in events) if events else 0
    begin_ns = first_ns + int(args.start_offset * 1e9)

    pending_image = next(images, None)
    wall0 = None
    sim0 = None
    published = 0

    for ns, kind, msg in events:
        while pending_image is not None and pending_image[0] <= ns:
            img_ns, img_msg = pending_image
            if img_ns >= begin_ns:
                emit_image(node, img_ns, img_msg, by_det.get(img_ns), arms, colours,
                           args.point_radius)
            pending_image = next(images, None)

        if ns < begin_ns:
            continue
        if controls.quit:
            break
        frame = by_proj.get(ns)
        if args.skip_empty and not (
            frame and any(v for d in frame["detections"] for v in d["arms"].values())
        ):
            continue

        if wall0 is None:
            wall0, sim0 = time.monotonic(), ns
        else:
            target = (ns - sim0) * 1e-9 / max(args.rate, 1e-6)
            delay = target - (time.monotonic() - wall0)
            if delay > 0:
                time.sleep(min(delay, 5.0))

        controls.wait()
        if controls.quit:
            break

        node.publish_clock(ns)
        node.cloud_pub.publish(msg)
        if frame is not None:
            node.publish_frame(frame, colours, baseline)
        published += 1

    print(f"\npublished {published} frames")
    node.destroy_node()
    rclpy.shutdown()


def emit_image(node, ns, img_msg, frame, arms, colours, point_radius=2):
    """Draw the 2D box, each arm's mask window and each arm's deciding pixels, then publish."""
    img = debayer_half(img_msg)
    if frame is not None:
        for det in frame["detections"]:
            x0, y0, x1, y1 = det["box"]
            draw_rect(img, x0, y0, x1, y1, (1.0, 1.0, 1.0), thick=2)
            for arm in arms:
                res = det["arms"].get(arm.name)
                if res is None:
                    continue
                wx0, wy0, wx1, wy1 = res["window"]
                draw_rect(img, wx0, wy0, wx1, wy1, colours[arm.name], thick=2)
                draw_points(img, res["pixels"], colours[arm.name], radius=point_radius)

    out = Image()
    out.header.stamp = ns_to_time(ns)
    out.header.frame_id = img_msg.header.frame_id
    out.height, out.width = img.shape[0], img.shape[1]
    out.encoding = "rgb8"
    out.is_bigendian = 0
    out.step = out.width * 3
    out.data = np.ascontiguousarray(img).tobytes()
    node.image_pub.publish(out)
    node.publish_clock(ns)


if __name__ == "__main__":
    main()
