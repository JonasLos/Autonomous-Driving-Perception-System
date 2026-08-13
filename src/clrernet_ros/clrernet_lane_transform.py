#!/usr/bin/env python3
# type: ignore

"""Lifts CLRerNet lane polylines into 3D against the LiDAR projection.

A lane detection describes an image captured several hundred milliseconds before the
message arrives (camera transport plus inference), while the projection arrives a few tens
of milliseconds behind its own capture. Pairing each detection with whatever projection is
current at arrival time therefore lifts stale pixels onto fresh geometry, and the lane lands
wherever the vehicle was when the camera saw it.

So the projections are buffered instead, and each detection is fused against the cloud
captured closest to its own header stamp. Skew drops to at most half a LiDAR period,
independent of how far behind the camera pipeline runs. The cost is that a lane is published
~one pipeline latency after the scene it describes; the output carries the matched cloud's
stamp so consumers can account for that. Stamps are only ever compared to each other and
never to the node clock, which keeps this valid under bag replay without any use_sim_time
configuration.
"""

import os
import time as _time

from ament_index_python.packages import get_package_share_directory
import numpy as np
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from sensor_msgs_py import point_cloud2 as pc2
import yaml
from sensor_msgs.msg import PointCloud2, PointField

from clrernet_msgs.msg import LanePoints
from perception_common.stamp_sync import StampMatchedBuffer, apply_bounded_parameters
from perception_common.utils import timer

TOPICS_PATH = os.path.join(
    get_package_share_directory("perception_common"), "topics.yaml"
)
with open(TOPICS_PATH, "r") as f:
    config = yaml.safe_load(f)

CLRERNET_ALL_LANES_TOPIC = config["topics"]["clrernet"]["all_lanes"]
CLRERNET_LEFT_LANE_TOPIC = config["topics"]["clrernet"]["left_lane"]
CLRERNET_RIGHT_LANE_TOPIC = config["topics"]["clrernet"]["right_lane"]
CLRERNET_CENTERLINE_TOPIC = config["topics"]["clrernet"]["centerline"]
LIDAR_2D_PROJ_TOPIC = config["topics"]["transform"]["lidar_2d_projection"]

PIXEL_LIM = 10
LIDAR_ORIGIN = np.array([0.0, 0.0, 0.0])


class Clrernet_Lane_Transform(Node):
    def __init__(self):
        super().__init__("clrernet_lane_transform")

        self.fields = [
            PointField(
                name="x", offset=0, datatype=PointField.FLOAT32, count=1
            ),
            PointField(
                name="y", offset=4, datatype=PointField.FLOAT32, count=1
            ),
            PointField(
                name="z", offset=8, datatype=PointField.FLOAT32, count=1
            ),
            PointField(
                name="intensity", offset=12, datatype=PointField.FLOAT32, count=1
            ),
        ]
        self.prev_centerline = None
        self.ema_alpha = 0.5  # EMA smoothing factor

        # Half a 10Hz LiDAR period is the worst case for a dense buffer; the default
        # leaves headroom for jitter and the occasional dropped scan without ever
        # admitting a cloud from a neighbouring frame.
        max_pairing_skew = float(
            self.declare_parameter("max_pairing_skew", 0.08)
            .get_parameter_value()
            .double_value
        )
        # Must exceed the camera-to-detection latency, or the matching cloud will have
        # been evicted before its lane detection arrives.
        buffer_duration = float(
            self.declare_parameter("projection_buffer_duration", 2.0)
            .get_parameter_value()
            .double_value
        )
        # How long a lane may go unrefreshed before the node publishes empty clouds.
        # Without this a stalled detector leaves the last lane standing in RViz and in
        # any consumer, which reads as valid, current geometry.
        self.lane_timeout = float(
            self.declare_parameter("lane_timeout", 0.5)
            .get_parameter_value()
            .double_value
        )
        # The LiDAR stamp is end-of-sweep, so a point's true capture time is uniform over
        # the preceding sweep. -0.05 centres pairing on the sweep instead of biasing it
        # half a period late. Left at 0.0 until measured against a bag.
        stamp_offset = float(
            self.declare_parameter("projection_stamp_offset", 0.0)
            .get_parameter_value()
            .double_value
        )

        self._projections = StampMatchedBuffer(
            "projection",
            buffer_duration=max(0.0, buffer_duration),
            max_skew=max(0.0, max_pairing_skew),
            stamp_offset=stamp_offset,
        )
        self.lane_timeout = max(0.0, self.lane_timeout)
        self._last_publish = 0.0
        self._last_unmatched_log = 0.0

        self.create_subscription(PointCloud2, LIDAR_2D_PROJ_TOPIC, self._on_projection, 10)
        self.create_subscription(
            LanePoints, CLRERNET_ALL_LANES_TOPIC, self._on_lanes, 10
        )

        # Publishers
        self.left_pcl_pub = self.create_publisher(PointCloud2, CLRERNET_LEFT_LANE_TOPIC, 5)
        self.right_pcl_pub = self.create_publisher(PointCloud2, CLRERNET_RIGHT_LANE_TOPIC, 5)
        self.centerline_pub = self.create_publisher(PointCloud2, CLRERNET_CENTERLINE_TOPIC, 1)

        # Fixed period, so lane_timeout is enforced to within 0.5s of granularity.
        self._watchdog_timer = self.create_timer(0.5, self._watchdog)

        # The pairing bound and the buffer depth both depend on measured pipeline
        # latency, so keep them settable at runtime for calibration against a replaying
        # bag without restarting the node.
        self.add_on_set_parameters_callback(self._on_set_parameters)

        self.get_logger().info(
            f"Clrernet_Lane_Transform ready: {LIDAR_2D_PROJ_TOPIC} + {CLRERNET_ALL_LANES_TOPIC}, "
            f"max_pairing_skew={self._projections.max_skew:.3f}s "
            f"projection_buffer_duration={self._projections.buffer_duration:.3f}s "
            f"lane_timeout={self.lane_timeout:.3f}s (stamp-matched pairing)"
        )

    def _on_set_parameters(self, params) -> SetParametersResult:
        targets = {
            "max_pairing_skew": (self._projections, "max_skew"),
            "projection_buffer_duration": (self._projections, "buffer_duration"),
            "lane_timeout": (self, "lane_timeout"),
        }
        ok, reason, applied = apply_bounded_parameters(params, targets)
        if not ok:
            return SetParametersResult(successful=False, reason=reason)
        for name, value in applied:
            self.get_logger().info(f"{name} set to {value:.3f}s")

        # Unbounded: a negative offset is the expected direction for an end-of-sweep stamp.
        for p in params:
            if p.name == "projection_stamp_offset":
                self._projections.stamp_offset = float(p.value)
                self.get_logger().info(
                    f"projection_stamp_offset set to {float(p.value):.3f}s"
                )
        return SetParametersResult(successful=True)

    def _on_projection(self, msg_proj):
        """Buffer the projection so a later lane detection can be paired against it."""
        self._projections.add(msg_proj)

    def _on_lanes(self, msg_all_lanes):
        entry, skew = self._projections.match(msg_all_lanes.header)
        if entry is None:
            self._log_unmatched(skew)
            return
        self.lanes_callback(entry, msg_all_lanes)

    def _log_unmatched(self, skew):
        """Rate-limited warning so a starved or misaligned pipeline stays visible."""
        now = _time.monotonic()
        if now - self._last_unmatched_log < 1.0:
            return
        self._last_unmatched_log = now
        self.get_logger().warning(
            f"Unmatched lane detection: "
            f"{self._projections.describe_unmatched(skew, 'lane detection')}; "
            f"{self._projections.status()}"
        )

    @timer
    def lanes_callback(self, entry, msg_all_lanes):
        # One parse, one KDTree, shared by the lane pairing and both per-side publishes.
        tree, pc_arr = entry.pixel_tree()
        if tree is None:
            self.get_logger().warning("No points in projected PointCloud2")
            self._publish_empty(entry.header)
            return

        lane_dict = {}
        for pt in msg_all_lanes.points:
            lane_dict.setdefault(pt.lane_id, []).append([pt.x, pt.y])

        lanes = [np.array(pts) for _, pts in sorted(lane_dict.items())]
        if len(lanes) == 0:
            self.get_logger().warning("No lane points detected!")
            self._publish_empty(entry.header)
            return

        left_pts, right_pts, center_pts = self.get_closest_lane_pair_3d(
            lanes, tree, pc_arr, LIDAR_ORIGIN
        )
        if len(left_pts) == 0 or len(right_pts) == 0:
            self.get_logger().warning("Unable to find left or right lane boundaries.")
        if center_pts.shape[0] == 0:
            self.get_logger().warning("Centerline could not be computed.")

        self.publish_3d_lane(left_pts, tree, pc_arr, self.left_pcl_pub, entry.header)
        self.publish_3d_lane(right_pts, tree, pc_arr, self.right_pcl_pub, entry.header)
        self.publish_centerline(center_pts, entry.header)
        self._last_publish = _time.monotonic()

    def get_closest_lane_pair_3d(self, lanes, tree, pc_arr, lidar_origin):
        def transform_lane_to_3d(lane_uv):
            dist, idx = tree.query(lane_uv)  # Vectorized nearest neighbors
            mask = dist < PIXEL_LIM
            matched_pts = pc_arr[idx[mask]]
            if matched_pts.shape[0] == 0:
                self.get_logger().warning("No 3D match found for a lane")
            return matched_pts if matched_pts.shape[0] > 0 else np.empty((0, 3))

        def compute_centerline(pts1, pts2):
            min_len = min(len(pts1), len(pts2))
            if min_len == 0:
                return np.empty((0, 3))
            return (pts1[:min_len] + pts2[:min_len]) / 2.0

        lanes_3d = []
        for lane in lanes:
            lane_3d = transform_lane_to_3d(lane)
            if lane_3d.shape[0] > 0:
                mean_y = np.mean(lane_3d[:, 1])
                lanes_3d.append((lane, lane_3d, mean_y))

        if len(lanes_3d) < 2:
            self.get_logger().warning("Less than 2 valid 3D lanes detected. Cannot compute centerline.")
            return ([], [], np.empty((0, 3)))

        lanes_3d.sort(key=lambda tup: tup[2])
        best_pair = ([], [])
        best_centerline = np.empty((0, 3))
        min_dist = float("inf")

        for i in range(len(lanes_3d) - 1):
            lane1_uv, lane1_3d, _ = lanes_3d[i]
            lane2_uv, lane2_3d, _ = lanes_3d[i + 1]

            centerline_pts = compute_centerline(lane1_3d, lane2_3d)
            if centerline_pts.shape[0] == 0:
                continue

            center_mean = centerline_pts[:10].mean(axis=0)
            dist = np.linalg.norm(center_mean[1] - lidar_origin[1])

            if dist < min_dist:
                min_dist = dist
                if np.mean(lane1_3d[:, 1]) > np.mean(lane2_3d[:, 1]):
                    best_pair = (lane1_uv, lane2_uv)
                else:
                    best_pair = (lane2_uv, lane1_uv)
                best_centerline = centerline_pts

        return best_pair[0], best_pair[1], best_centerline

    def publish_3d_lane(self, lane_uv, tree, pc_arr, publisher, header):
        matched = np.empty((0, 3), dtype=np.float32)
        if len(lane_uv) > 0:
            # One vectorized query rather than one per contour point.
            dist, idx = tree.query(np.asarray(lane_uv))
            hit = idx[dist < PIXEL_LIM]
            if hit.size:
                matched = pc_arr[hit]

        # Published even when empty: an absent lane must read as absent downstream rather
        # than as the last one that happened to work.
        self._publish_points(matched, publisher, header)

    def publish_centerline(self, center_pts, header):
        if center_pts.shape[0] > 0:
            # === Apply Temporal Smoothing ===
            if (
                self.prev_centerline is not None
                and self.prev_centerline.shape == center_pts.shape
            ):
                center_pts = (
                    self.ema_alpha * center_pts
                    + (1 - self.ema_alpha) * self.prev_centerline
                )
            self.prev_centerline = center_pts.copy()

        self._publish_points(center_pts, self.centerline_pub, header)

    def _publish_points(self, points, publisher, header):
        points = np.asarray(points, dtype=np.float32).reshape(-1, 3)
        intensity = np.ones((points.shape[0], 1), dtype=np.float32)
        cloud_data = np.hstack((points, intensity)) if points.shape[0] else np.empty(
            (0, 4), dtype=np.float32
        )
        publisher.publish(pc2.create_cloud(header, self.fields, cloud_data))

    def _publish_empty(self, header):
        empty = np.empty((0, 3), dtype=np.float32)
        for pub in (self.left_pcl_pub, self.right_pcl_pub, self.centerline_pub):
            self._publish_points(empty, pub, header)
        self._last_publish = _time.monotonic()

    def _watchdog(self):
        """Publish empty lanes while the detector is not refreshing them."""
        if self.lane_timeout <= 0.0:
            return
        newest = self._projections.newest()
        if newest is None:
            return
        if _time.monotonic() - self._last_publish > self.lane_timeout:
            # A stalled detector invalidates the smoothing history too, so the next
            # detection starts clean rather than blending across the gap.
            self.prev_centerline = None
            self._publish_empty(newest.header)


def main(args=None):
    rclpy.init(args=args)
    node = Clrernet_Lane_Transform()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
