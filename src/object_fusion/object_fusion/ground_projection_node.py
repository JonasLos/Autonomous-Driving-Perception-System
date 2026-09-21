"""Raw LiDAR sweep -> projection with a per-point ground flag.

Publishes transform.py's (x, y, z, u, v) layout plus a ``ground`` field, on its own topic.
transform.py keeps running untouched and /lidar_2d_projection is unaffected. camera_lidar_detector
consumes this topic when its ``projection_topic`` points here; pointed back at the old topic, the
flag is simply absent and it behaves exactly as before.

FLAGGED, NOT REMOVED. Ground points stay in the cloud with ground=1. Removing them would take the
decision away from the detector, and it needs it: segmentation alone empties 3.7% of boxes, and
the adopted rule falls back to every point for those instead of losing the detection.

WHY THE RAW SWEEP. Patchwork++ estimates ground per concentric zone and wants the whole sweep; a
narrow forward wedge degrades the model. Measured on the reference replay: widening transform.py's
crop moved the far-field error ~0.35 m out of 32, so raw points buy nothing for COVERAGE -- they
buy a ground model.

Configuration, all measured with scripts/ground_ab.py against radar range:
  * backend patchworkpp  -- 4 ms/sweep; the polar-grid fallback is 20 ms and emptied 9.8% of
                            25-40 m detections by fitting the ground plane through vehicles.
  * max_range 120 m      -- the library default of 80 m leaves the 60-80 m band worse than no
                            segmentation (range sd 4.24 -> 5.29); 150 m moves a zone boundary
                            into that band and its jitter p90 reaches 49 m.
  * transform.py's crop  -- kept, so the point set is exactly what was scored.
  * camera_info bounds   -- 2064x1544, not the 1-px-short principal-point estimate.
  * levelling            -- inside 25 m, labels come from a sweep levelled by odometry roll/pitch
                            (LevelledGroundSegmenter; measured with scripts/lean_ab.py and
                            neighbour_ab.py --level odom-near). Without odometry covering the
                            sweep's stamp the labels are exactly the unlevelled ones.
"""

import collections
import time

import numpy as np
import rclpy
from rclpy.node import Node
import yaml
from ament_index_python.packages import get_package_share_directory

from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, PointCloud2, PointField
from sensor_msgs_py import point_cloud2

from perception_common.utils import crop_pointcloud, stamp_to_seconds
from object_fusion.flagged_cloud import decode_fields
from object_fusion.ground_segmentation import (
    LevelledGroundSegmenter, attitude_at, quaternion_roll_pitch_deg,
)
from object_fusion.projection import image_size_from_proj, project_to_pixels

_F32 = PointField.FLOAT32
_FLAGGED = [PointField(name=n, offset=4 * i, datatype=_F32, count=1)
            for i, n in enumerate(("x", "y", "z", "u", "v", "ground"))]
_XYZ = [PointField(name=n, offset=4 * i, datatype=_F32, count=1)
        for i, n in enumerate(("x", "y", "z"))]


class GroundProjectionNode(Node):
    def __init__(self):
        super().__init__("ground_projection")
        cfg = yaml.safe_load(
            open(get_package_share_directory("object_fusion") + "/config/topics.yaml",
                 encoding="utf-8"))
        common = yaml.safe_load(
            open(get_package_share_directory("perception_common") + "/topics.yaml",
                 encoding="utf-8"))
        self._in = str(self.declare_parameter(
            "input_topic", common["topics"]["raw"]["lidar_tc"]).value)
        self._out = cfg["topics"]["measurements"]["projection_ground_flagged"]

        backend = str(self.declare_parameter("ground_backend", "auto").value)
        # 2.37 m above the road. Two independent measurements agree and both disagree with the
        # 2.46 this shipped with: jeep_selfcal_loc's calib/measured_calibration.yaml has
        # 2.366 +- 0.005 (corridor crop, calm sweeps) and a direct median of near-field ground
        # returns here gives 2.394. MEASURED INERT for Patchwork++ -- scripts/lean_ab.py at 2.46
        # and 2.37 produces byte-identical tables, because its adaptive thresholds re-learn the
        # offset -- so this is a correctness fix, not a behaviour change.
        sensor_height = float(self.declare_parameter("sensor_height", 2.37).value)
        max_range = float(self.declare_parameter("ground_max_range", 120.0).value)
        # transform.py's crop, so the published point set matches what ground_ab.py scored.
        self._crop_range = float(self.declare_parameter("crop_max_range", 150.0).value)
        self._crop_lateral = float(self.declare_parameter("crop_lateral_limit", 20.0).value)
        # Full-sweep ground / non-ground clouds for RViz. Off by default: two extra ~45k-point
        # clouds at 10 Hz are real bandwidth on a shared host.
        self._debug = bool(self.declare_parameter("publish_debug_clouds", False).value)
        # The non-ground cloud for CONSUMERS (lidar_cluster_detector), as opposed to the debug
        # pair. Segmentation runs on the whole sweep before transform.py's crop, so this carries
        # the full 360 degrees -- the side and rear the camera never sees.
        self._pub_ng_on = bool(self.declare_parameter("publish_nonground", False).value)

        # Odometry attitude levelling of the near field (see LevelledGroundSegmenter for numbers).
        self._levelling = bool(self.declare_parameter("ground_levelling", True).value)
        near = float(self.declare_parameter("levelling_near_range", 25.0).value)
        odom_topic = str(self.declare_parameter("odom_topic", "/novatel/oem7/odom").value)
        # Odometry runs at ~105 Hz; a sweep with no attitude sample this close is not levelled.
        self._max_odom_gap = float(self.declare_parameter("max_odom_gap_s", 0.2).value)
        self._seg = LevelledGroundSegmenter(backend=backend, sensor_height=sensor_height,
                                            max_range=max_range, near_range=near)
        self._att = collections.deque(maxlen=1000)       # (stamp, roll_deg, pitch_deg), ~10 s
        self._n_levelled = self._n_no_odom = 0
        self._wh = image_size_from_proj()
        self._have_ci = False
        self._pub = self.create_publisher(PointCloud2, self._out, 5)
        if self._pub_ng_on:
            self._pub_nonground = self.create_publisher(
                PointCloud2, cfg["topics"]["measurements"]["nonground"], 5)
        if self._debug:
            self._pub_g = self.create_publisher(PointCloud2, "/perception/ground_debug/ground", 2)
            self._pub_ng = self.create_publisher(PointCloud2, "/perception/ground_debug/nonground", 2)
        self.create_subscription(PointCloud2, self._in, self._cb, 5)
        if self._levelling:
            self.create_subscription(Odometry, odom_topic, self._odom_cb, 50)
        self.create_subscription(CameraInfo, common["topics"]["raw"]["camera_info"],
                                 self._camera_info_cb, 5)
        self._n = self._pts = self._ground = self._pub_pts = 0
        self._ms = []
        self.create_timer(5.0, self._log)

        if self._seg.backend != "patchworkpp":
            self.get_logger().warning(
                "pypatchworkpp is not installed: using the polar-grid FALLBACK, which is 5x "
                "slower and measurably worse (it empties ~10% of 25-40 m detections). "
                "Rebuild docker/Dockerfile.object_fusion to get Patchwork++.")
        self.get_logger().info(
            f"ground_projection: {self._in} -> {self._out} | backend={self._seg.backend} "
            f"max_range={max_range:.0f}m crop={self._crop_range:.0f}/{self._crop_lateral:.0f}m "
            f"debug_clouds={self._debug} levelling={self._levelling} "
            f"(near {near:.0f} m, {odom_topic})")

    def _camera_info_cb(self, msg):
        # Same contract as transform.py: camera_info's size once it arrives. The principal-point
        # estimate is 1 px short in each axis and permanently dropped the last pixel column.
        if msg.width > 0 and msg.height > 0:
            wh = (int(msg.width), int(msg.height))
            if wh != self._wh or not self._have_ci:
                self.get_logger().info(f"image bounds from camera_info: {wh[0]}x{wh[1]}")
            self._wh, self._have_ci = wh, True

    def _odom_cb(self, msg: Odometry):
        t = stamp_to_seconds(msg.header.stamp)
        if self._att and t < self._att[-1][0] - 1.0:
            self._att.clear()                               # bag rewind
        q = msg.pose.pose.orientation
        self._att.append((t, *quaternion_roll_pitch_deg(q.x, q.y, q.z, q.w)))

    def _attitude_at(self, t):
        """(roll, pitch) at ``t``; (None, None) without a sample within max_odom_gap_s."""
        return attitude_at(list(self._att), t, self._max_odom_gap)

    def _cb(self, msg: PointCloud2):
        f = decode_fields(msg, ("x", "y", "z"))
        if f["x"] is None or f["x"].size == 0:
            return
        xyz = np.stack([f["x"], f["y"], f["z"]], axis=1).astype(np.float64)
        xyz = xyz[np.isfinite(xyz).all(axis=1)]

        # Wall-clock timing on purpose: under use_sim_time the node clock does not advance
        # inside a callback, and timing with it reported 0.0 ms for every sweep.
        roll = pitch = None
        if self._levelling:
            roll, pitch = self._attitude_at(stamp_to_seconds(msg.header.stamp))
            if roll is None:
                self._n_no_odom += 1
            else:
                self._n_levelled += 1
        t0 = time.perf_counter()
        ground = self._seg.segment(xyz, roll, pitch)
        self._ms.append((time.perf_counter() - t0) * 1000.0)
        self._n += 1
        self._pts += xyz.shape[0]
        self._ground += int(ground.sum())

        if self._pub_ng_on:
            self._pub_nonground.publish(point_cloud2.create_cloud(
                msg.header, _XYZ, xyz[~ground].astype(np.float32)))
        if self._debug:
            self._pub_g.publish(point_cloud2.create_cloud(
                msg.header, _XYZ, xyz[ground].astype(np.float32)))
            self._pub_ng.publish(point_cloud2.create_cloud(
                msg.header, _XYZ, xyz[~ground].astype(np.float32)))

        tagged = np.column_stack([xyz, ground.astype(np.float64)])
        cropped = np.asarray(crop_pointcloud(tagged, [0.0, self._crop_range],
                                             [-self._crop_lateral, self._crop_lateral],
                                             [-3.5, 1.0]))
        if cropped.size == 0:
            self._pub.publish(point_cloud2.create_cloud(
                msg.header, _FLAGGED, np.empty((0, 6), np.float32)))
            return
        kept, u, v, idx = project_to_pixels(cropped[:, :3], image_wh=self._wh, return_index=True)
        flag = cropped[idx, 3]
        out = np.column_stack([kept, u, v, flag]).astype(np.float32)
        self._pub_pts += out.shape[0]
        self._pub.publish(point_cloud2.create_cloud(msg.header, _FLAGGED, out))

    def _log(self):
        if not self._n:
            return
        ms = np.asarray(self._ms[-200:])
        self.get_logger().info(
            f"sweeps={self._n} backend={self._seg.backend} seg {np.median(ms):.1f} ms "
            f"ground {100 * self._ground / max(self._pts, 1):.0f}% of points | "
            f"projected/sweep {self._pub_pts / self._n:.0f} | levelled {self._n_levelled} "
            f"no-odom {self._n_no_odom}")


def main():
    rclpy.init()
    node = GroundProjectionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
