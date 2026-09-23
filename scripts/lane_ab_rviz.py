#!/usr/bin/env python3
"""Side-by-side RViz replay of the two lane-pair selection arms.

`lane_ab.py` prints a table, and a table can be fooled. This script is how you check the pair by
eye instead of by metric.

An earlier version of this docstring read the low "ego bracketed" score as evidence that the
vehicle spent the bag riding a lane line ~1.8 m left of centre, on the grounds that
`lidar_tc -> base_link` is identity and therefore y=0 is the vehicle centreline. That was wrong
in both halves. Identity is what that transform says because nobody calibrated it: lidar_tc is
yawed ~5.35 deg from the vehicle axis, so y=0 diverges from the ego path by 9.4 cm per metre and
reaches half a lane width by ~19 m. The 1.8 m was that divergence read at the range the score
window samples, and with the ego reference corrected to a ray the vehicle sits 0.44 m from its
lane centre for the whole bag -- ordinary lane-keeping. "Ego bracketed" is a good metric again
once it is measured about the ray, which `lane_ab.metrics` now does.

Both arms are computed by importing `lane_ab`, so the pairing, the pixel match and the arm code
are literally the same objects this tool and the scorer both run; there is no second copy to
drift. Arm A is drawn in red, arm B in green, the projected cloud in grey, and a yellow line
joins the two centrelines wherever they disagree.

Usage:
    ros2 run rmw_zenoh_cpp rmw_zenohd &
    python3 scripts/lane_ab_rviz.py ~/lane_probe_2026-08-25/lane_probe_0.mcap --rate 0.5
    rviz2 -d config/lane_ab.rviz          # written on startup; fixed frame lidar_tc

ENTER pauses/resumes, `n` steps one frame, `q` quits.
"""

import argparse
import json
import os
import sys
import threading
import time

import numpy as np
from scipy.spatial import KDTree

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "src", "perception_common"))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy  # noqa: E402
from builtin_interfaces.msg import Time as TimeMsg  # noqa: E402
from geometry_msgs.msg import Point, TransformStamped  # noqa: E402
from rosgraph_msgs.msg import Clock  # noqa: E402
from sensor_msgs.msg import PointCloud2, PointField  # noqa: E402
from sensor_msgs_py import point_cloud2 as pc2  # noqa: E402
from std_msgs.msg import Header  # noqa: E402
from tf2_msgs.msg import TFMessage  # noqa: E402
from visualization_msgs.msg import Marker, MarkerArray  # noqa: E402

import lane_ab as AB  # noqa: E402
from perception_common.lane_geometry import LanePairSelector  # noqa: E402
from perception_common.stamp_sync import DEFERRED, StampMatchedBuffer  # noqa: E402
from mcap_ros2.reader import read_ros2_messages  # noqa: E402

FRAME = "lidar_tc"

# Distinguishable from each other under the common forms of colour blindness, and both
# distinguishable from the grey projection behind them.
COL_A = (0.95, 0.25, 0.20)      # red   -- baseline, listed first
COL_A_DIM = (0.60, 0.15, 0.12)
COL_B = (0.20, 0.80, 0.35)      # green -- the new arm
COL_B_DIM = (0.10, 0.50, 0.22)
COL_DISAGREE = (1.00, 0.80, 0.10)

FIELDS = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
]


def ns_to_time(ns):
    t = TimeMsg()
    t.sec = int(ns // 1_000_000_000)
    t.nanosec = int(ns % 1_000_000_000)
    return t


def xyz_point(x, y, z):
    p = Point()
    p.x, p.y, p.z = float(x), float(y), float(z)
    return p


def set_colour(marker, rgb, alpha=1.0):
    marker.color.r, marker.color.g, marker.color.b = rgb
    marker.color.a = alpha


# --------------------------------------------------------------------------------------
# rviz config
# --------------------------------------------------------------------------------------


def write_rviz_config(path):
    """Emit the config on startup, so the display set always matches what this tool publishes."""
    def rgb255(c):
        return "; ".join(str(int(255 * v)) for v in c)

    def cloud_display(name, topic, colour, size):
        return {
            "Class": "rviz_default_plugins/PointCloud2", "Name": name, "Enabled": True,
            "Topic": {"Value": topic, "Depth": 1, "Durability Policy": "Volatile",
                      "Reliability Policy": "Reliable"},
            "Size (Pixels)": size, "Style": "Points",
            "Color Transformer": "FlatColor", "Color": rgb255(colour),
            "Alpha": 1.0, "Decay Time": 0,
        }

    displays = [
        {"Class": "rviz_default_plugins/Grid", "Name": "Grid", "Enabled": True,
         "Cell Size": 5, "Plane Cell Count": 40, "Color": "80; 80; 80"},
        cloud_display("projection", "/lidar_2d_projection", (0.51, 0.51, 0.51), 2),
        cloud_display("A left  (HEAD)", "/ab/A/left", COL_A, 8),
        cloud_display("A right (HEAD)", "/ab/A/right", COL_A, 8),
        cloud_display("A centreline", "/ab/A/centerline", COL_A_DIM, 11),
        cloud_display("B left  (new)", "/ab/B/left", COL_B, 8),
        cloud_display("B right (new)", "/ab/B/right", COL_B, 8),
        cloud_display("B centreline", "/ab/B/centerline", COL_B_DIM, 11),
        {"Class": "rviz_default_plugins/MarkerArray", "Name": "annotations", "Enabled": True,
         "Topic": {"Value": "/ab/markers", "Depth": 10, "Durability Policy": "Volatile",
                   "Reliability Policy": "Reliable"}},
    ]
    config = {
        "Panels": [
            {"Class": "rviz_common/Displays", "Name": "Displays",
             "Property Tree Widget": {"Expanded": [], "Splitter Ratio": 0.5}},
            {"Class": "rviz_common/Views", "Name": "Views"},
        ],
        "Visualization Manager": {
            "Class": "", "Name": "root", "Displays": displays,
            "Global Options": {"Background Color": "30; 30; 34",
                               "Fixed Frame": FRAME, "Frame Rate": 30},
            "Tools": [{"Class": "rviz_default_plugins/MoveCamera"}],
            "Views": {
                "Current": {
                    "Class": "rviz_default_plugins/Orbit", "Name": "Current View",
                    # Looking forward down the road from behind and above the sensor: the
                    # lateral disagreement between the arms is what has to be legible.
                    "Distance": 45.0,
                    "Focal Point": {"X": 18.0, "Y": 0.0, "Z": 0.0},
                    "Pitch": 0.55, "Yaw": 3.14, "Target Frame": FRAME,
                },
                "Saved": None,
            },
        },
    }
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    # RViz reads YAML and JSON is a subset of it, which avoids depending on PyYAML's
    # ordering quirks for a file nothing but RViz ever reads.
    with open(path, "w", encoding="utf-8") as f:
        json.dump(config, f, indent=2)


# --------------------------------------------------------------------------------------
# controls
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


# --------------------------------------------------------------------------------------
# pass 1 -- run both arms on the same paired stream lane_ab.py uses
# --------------------------------------------------------------------------------------


def precompute(path, score_max_x, hysteresis_margin, ego_yaw_deg=None):
    lanes_msgs, projections = [], []
    for m in read_ros2_messages(path, topics=["/clrernet/all_lanes", "/lidar_2d_projection"]):
        (lanes_msgs if m.channel.topic == "/clrernet/all_lanes" else projections).append(m.ros_msg)
    print(f"loaded: all_lanes={len(lanes_msgs)} projections={len(projections)}")

    buf = StampMatchedBuffer("projection", buffer_duration=2.0, max_skew=0.08,
                             wait_for_newer=0.06, wrap=lambda msg: msg)
    events = sorted([(AB.stamp_ns(p.header), 0, p) for p in projections]
                    + [(AB.stamp_ns(l.header), 1, l) for l in lanes_msgs],
                    key=lambda e: (e[0], e[1]))
    paired = []
    for ts, kind, msg in events:
        if kind == 0:
            buf.add(msg)
            paired += [(x.value, x.payload) for x in buf.drain(now=ts * 1e-9)
                       if x.value is not None]
        else:
            pr = buf.match(msg.header, now=ts * 1e-9, payload=msg)
            if pr.outcome is not DEFERRED and pr.value is not None:
                paired.append((pr.value, pr.payload))

    sel = LanePairSelector(AB.GRID, score_min_x=6.0, score_max_x=score_max_x,
                           min_window_nodes=8, min_lane_width=2.2, max_lane_width=4.5,
                           incumbent_tol=0.75, hysteresis_margin=hysteresis_margin,
                           switch_debounce=2, memory_timeout=0.5,
                           **({} if ego_yaw_deg is None else {"ego_yaw_deg": ego_yaw_deg}))
    frames, prev_a, prev_b = [], None, None
    for proj, det in paired:
        arr = AB.cloud(proj, ("x", "y", "z", "u", "v"))
        if arr is None or arr.shape[0] < 10:
            continue
        pc_arr = arr[:, :3]
        tree = KDTree(arr[:, 3:5])
        d = {}
        for pt in det.points:
            d.setdefault(pt.lane_id, []).append([pt.x, pt.y])
        if not d:
            continue
        lanes = [np.asarray(p, dtype=np.float64) for _, p in sorted(d.items())]

        La, Ra = AB.arm_a(lanes, tree, pc_arr)
        Lb, Rb, _ = AB.arm_b(lanes, tree, pc_arr, sel, AB.stamp_ns(proj.header) * 1e-9)
        ma, mb = AB.metrics(La, Ra), AB.metrics(Lb, Rb)

        def centre(L, R):
            if len(L) == 0 or len(R) == 0:
                return np.empty((0, 3), dtype=np.float32)
            n = min(len(L), len(R))
            return ((L[:n] + R[:n]) / 2.0).astype(np.float32)

        jump_a = abs(ma["cy"] - prev_a) if (ma and prev_a is not None) else 0.0
        jump_b = abs(mb["cy"] - prev_b) if (mb and prev_b is not None) else 0.0
        if ma:
            prev_a = ma["cy"]
        if mb:
            prev_b = mb["cy"]

        frames.append(dict(
            stamp=AB.stamp_ns(proj.header), pc=pc_arr,
            La=La, Ra=Ra, Ca=centre(La, Ra), Lb=Lb, Rb=Rb, Cb=centre(Lb, Rb),
            ma=ma, mb=mb, jump_a=jump_a, jump_b=jump_b,
            disagree=(ma is not None and mb is not None and abs(ma["cy"] - mb["cy"]) > 0.5),
        ))
    print(f"paired detections: {len(paired)}   frames prepared: {len(frames)}")
    print(f"  arm A jumps >1.75m: {sum(1 for f in frames if f['jump_a'] > 1.75)}")
    print(f"  arm B jumps >1.75m: {sum(1 for f in frames if f['jump_b'] > 1.75)}")
    print(f"  frames where the arms disagree by >0.5m: {sum(1 for f in frames if f['disagree'])}")
    return frames


# --------------------------------------------------------------------------------------
# pass 2 -- replay
# --------------------------------------------------------------------------------------


class Republisher(Node):
    def __init__(self):
        super().__init__("lane_ab_rviz")
        q = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST)
        self.clock_pub = self.create_publisher(Clock, "/clock", 10)
        self.tf_pub = self.create_publisher(TFMessage, "/tf_static",
                                            QoSProfile(depth=1,
                                                       reliability=ReliabilityPolicy.RELIABLE,
                                                       history=HistoryPolicy.KEEP_LAST))
        self.proj_pub = self.create_publisher(PointCloud2, "/lidar_2d_projection", q)
        self.pubs = {}
        for arm in ("A", "B"):
            for part in ("left", "right", "centerline"):
                self.pubs[(arm, part)] = self.create_publisher(
                    PointCloud2, f"/ab/{arm}/{part}", q)
        self.marker_pub = self.create_publisher(MarkerArray, "/ab/markers", 10)

    def publish_clock(self, ns):
        c = Clock()
        c.clock = ns_to_time(ns)
        self.clock_pub.publish(c)

    def publish_tf(self, ns):
        """A single identity map->lidar_tc, so RViz has a tree its fixed frame lives in.

        The probe bag carries no /tf_static of its own -- every cloud here is already in the
        LiDAR frame and nothing is transformed -- but RViz refuses to render against a fixed
        frame it cannot find. Note this really is identity, unlike the vehicle's own
        lidar_tc -> base_link: nothing here claims to be the vehicle frame. The ego path is
        drawn as its own marker rather than assumed to be the y axis.
        """
        t = TransformStamped()
        t.header.stamp = ns_to_time(ns)
        t.header.frame_id = "map"
        t.child_frame_id = FRAME
        t.transform.rotation.w = 1.0
        self.tf_pub.publish(TFMessage(transforms=[t]))

    def publish_cloud(self, pub, pts, ns):
        h = Header()
        h.stamp = ns_to_time(ns)
        h.frame_id = FRAME
        arr = np.asarray(pts, dtype=np.float32).reshape(-1, 3)
        pub.publish(pc2.create_cloud(h, FIELDS, arr))


def build_markers(frame, ns, idx, total):
    ma_, mb_ = frame["ma"], frame["mb"]
    out = MarkerArray()

    def base(mid, mtype):
        m = Marker()
        m.header.frame_id = FRAME
        m.header.stamp = ns_to_time(ns)
        m.ns = "ab"
        m.id = mid
        m.type = mtype
        m.action = Marker.ADD
        return m

    # Ego axis: the y = 0 line the "bracketed" test is measured against.
    axis = base(0, Marker.LINE_STRIP)
    axis.scale.x = 0.06
    set_colour(axis, (0.55, 0.55, 0.62), 0.9)
    axis.points = [xyz_point(x, 0.0, -2.4) for x in (0.0, 60.0)]
    out.markers.append(axis)

    # The disagreement itself: a bar joining the two centrelines at the score window.
    if ma_ and mb_ and frame["disagree"]:
        link = base(1, Marker.LINE_STRIP)
        link.scale.x = 0.14
        set_colour(link, COL_DISAGREE, 1.0)
        link.points = [xyz_point(10.0, ma_["cy"], -2.4), xyz_point(10.0, mb_["cy"], -2.4)]
        out.markers.append(link)

    txt = base(2, Marker.TEXT_VIEW_FACING)
    txt.pose.position = xyz_point(6.0, 9.0, 2.0)
    txt.scale.z = 0.85
    jumped = frame["jump_a"] > 1.75 or frame["jump_b"] > 1.75
    set_colour(txt, COL_DISAGREE if jumped else (0.85, 0.85, 0.90), 1.0)
    fa = f"{ma_['cy']:+.2f}" if ma_ else "  --"
    fb = f"{mb_['cy']:+.2f}" if mb_ else "  --"
    wa = f"{ma_['width']:.2f}" if ma_ else " -- "
    wb = f"{mb_['width']:.2f}" if mb_ else " -- "
    txt.text = (
        f"frame {idx + 1}/{total}\n"
        f"A (HEAD)  centre_y {fa} m   width {wa} m   jump {frame['jump_a']:.2f} m\n"
        f"B (new)   centre_y {fb} m   width {wb} m   jump {frame['jump_b']:.2f} m"
        + ("\n<< LANE JUMP >>" if jumped else "")
    )
    out.markers.append(txt)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag", help="probe bag with /clrernet/all_lanes and /lidar_2d_projection")
    ap.add_argument("--rate", type=float, default=1.0, help="playback rate (default 1.0)")
    ap.add_argument("--start-offset", type=float, default=0.0, help="skip N seconds")
    ap.add_argument("--only-disagreements", action="store_true",
                    help="show only frames where the two arms differ by >0.5 m, or either jumped")
    ap.add_argument("--hold", type=float, default=0.0,
                    help="minimum seconds to dwell on each frame. Bag stamps are only ~0.1 s "
                         "apart and --only-disagreements drops the gaps between them, so "
                         "without this the interesting frames go past too fast to read.")
    ap.add_argument("--loop", action="store_true", help="repeat until 'q'")
    ap.add_argument("--score-max-x", type=float, default=30.0,
                    help="arm B scoring window upper bound (shipped default 30.0)")
    ap.add_argument("--hysteresis-margin", type=float, default=0.75,
                    help="arm B retention margin, metres (shipped default 0.75). The ego ray "
                         "makes the score unambiguous, so 0.0 also holds the lane for the whole "
                         "bag; pass 0.35 with --ego-yaw-deg 0 to replay the original flapping.")
    ap.add_argument("--ego-yaw-deg", type=float, default=None,
                    help="override the ego path's yaw in the LiDAR frame (default %.2f). "
                         "Pass 0.0 to score against y=0 the way the node used to." % AB.DEFAULT_EGO_YAW_DEG)
    ap.add_argument("--rviz-config", default=os.path.join(REPO, "config", "lane_ab.rviz"))
    args = ap.parse_args()

    frames = precompute(args.bag, args.score_max_x, args.hysteresis_margin, args.ego_yaw_deg)
    if not frames:
        print("no frames to show")
        return
    if args.only_disagreements:
        frames = [f for f in frames
                  if f["disagree"] or f["jump_a"] > 1.75 or f["jump_b"] > 1.75]
        print(f"  filtered to {len(frames)} interesting frames")
        if not frames:
            print("  (the arms never disagreed on this bag)")
            return

    write_rviz_config(args.rviz_config)
    print(f"\nwrote {args.rviz_config}\n  rviz2 -d {args.rviz_config}\n"
          "  ENTER pause/resume, 'n' step, 'q' quit\n")

    rclpy.init()
    node = Republisher()
    ctl = Controls()
    t0 = frames[0]["stamp"]
    if args.start_offset > 0:
        frames = [f for f in frames if (f["stamp"] - t0) * 1e-9 >= args.start_offset]

    try:
        pass_no = 0
        while not ctl.quit:
            pass_no += 1
            if args.loop:
                print(f"--- pass {pass_no} ---", flush=True)
            replay_once(node, ctl, frames, args)
            if not args.loop:
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def replay_once(node, ctl, frames, args):
    prev_ns = None
    for i, f in enumerate(frames):
        if ctl.quit:
            break
        ctl.wait()
        if ctl.quit:
            break
        ns = f["stamp"]
        node.publish_clock(ns)
        node.publish_tf(ns)
        node.publish_cloud(node.proj_pub, f["pc"], ns)
        node.publish_cloud(node.pubs[("A", "left")], f["La"], ns)
        node.publish_cloud(node.pubs[("A", "right")], f["Ra"], ns)
        node.publish_cloud(node.pubs[("A", "centerline")], f["Ca"], ns)
        node.publish_cloud(node.pubs[("B", "left")], f["Lb"], ns)
        node.publish_cloud(node.pubs[("B", "right")], f["Rb"], ns)
        node.publish_cloud(node.pubs[("B", "centerline")], f["Cb"], ns)
        node.marker_pub.publish(build_markers(f, ns, i, len(frames)))
        rclpy.spin_once(node, timeout_sec=0.0)

        if f["jump_a"] > 1.75 or f["jump_b"] > 1.75:
            print(f"  frame {i+1}/{len(frames)}  A jump {f['jump_a']:.2f} m   "
                  f"B jump {f['jump_b']:.2f} m", flush=True)

        # Real elapsed bag time between frames, scaled; --only-disagreements leaves big gaps
        # in the stamps, so it is capped rather than sleeping through the skips, and --hold
        # sets a floor because 0.1 s of bag time is not long enough to read a frame.
        dt = args.hold
        if prev_ns is not None and args.rate > 0:
            dt = max(dt, min((ns - prev_ns) * 1e-9 / args.rate, 1.0))
        if dt > 0:
            time.sleep(dt)
        prev_ns = ns


if __name__ == "__main__":
    main()
