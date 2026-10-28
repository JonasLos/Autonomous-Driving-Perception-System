"""Obstacle-robust drop-in twins of the SphereFormer and SAM3 road boundaries.

The existing producers publish /sphereformer_{left,right}_boundary and /sam3_{left,right}_boundary
as the outermost extent of whatever they labelled road, which an obstacle near the edge pulls
inward (see :mod:`object_fusion.road_boundary` for the mechanism and the four steps that fix it).
This node re-derives both from the producers' OWN outputs and publishes twins on
/road_boundary/{sphereformer,sam3}/{left,right}: same message type (PointCloud2 xyz), same frame,
same stamp semantics. Nothing upstream is edited or reconfigured, so the raw topics keep flowing
and a consumer switches back with a topic remap.

Inputs, all existing topics:

* SphereFormer: /sphereformer_road_segmentation, the full labelled sweep (class encoded as the
  fixed SemanticKITTI RGB). Road = class 9; obstacles = vehicle and person classes.
* SAM3: the segmentation node's /segmentation/mask_image/compressed (per-pixel mask ids at
  inference resolution), paired by CAPTURE stamp with the ground-flagged projection
  (x, y, z, u, v, ground) through the same ``StampMatchedBuffer`` sam3_mask_transform uses.
  Every projected LiDAR return whose pixel is road becomes a road point -- the whole mask is
  lifted, not a contour, so both sources feed the same 3D rule. Obstacles come from the
  SphereFormer sweep nearest in stamp, when SphereFormer runs.
* /perception/objects (ego frame), carried to each frame's stamp with ego motion -- boxes add
  obstacles neither segmenter labelled.
* Odometry twist, for the tracker's frame motion.

Rollback, finest first: the four ``enable_*`` parameters at runtime; ``enable_sphereformer`` /
``enable_sam3``; not launching the node (``ENABLE_ROAD_BOUNDARY=false``, the default); and on
the consumer side, simply not remapping to these topics.
"""

from __future__ import annotations

import math
import time

import cv2
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, CompressedImage, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import Marker, MarkerArray

from fusion_msgs.msg import FusedObjectArray
from perception_common.stamp_sync import DEFERRED, StampMatchedBuffer

from object_fusion import frames
from object_fusion.flagged_cloud import GroundFlaggedCloud, decode_fields
from object_fusion.projection import image_size_from_proj
from object_fusion.road_boundary import (
    OCCLUDED, SOURCES, STEPS, UNSUPPORTED, BoundaryParams, BoundaryPipeline,
    decode_semkitti_rgb, mask_road_pixels,
)

_XYZ = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
        for i, n in enumerate(("x", "y", "z"))]
#: BoundaryParams fields exposed as ROS parameters (settable at runtime).
_TUNABLE = ("x_min", "x_max", "bin_len", "occl_margin", "edge_evidence_m", "run_gap",
            "max_extrap", "min_support_frac", "default_width", "max_floor_push", "coast_s",
            "gate_m", "output_step", "ego_radius")
#: Changing these moves the tracker stations, so they reset the trackers.
_RESHAPE = {"x_min", "x_max", "bin_len"}


def _stamp(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class RoadBoundaryNode(Node):
    def __init__(self):
        super().__init__("road_boundary")
        cfg = yaml.safe_load(open(
            get_package_share_directory("object_fusion") + "/config/topics.yaml", encoding="utf-8"))
        common = yaml.safe_load(open(
            get_package_share_directory("perception_common") + "/topics.yaml", encoding="utf-8"))
        out = cfg["topics"]["road_boundary"]

        dp = self.declare_parameter
        p = BoundaryParams()
        for k in _TUNABLE:
            p.update(**{k: dp(k, getattr(p, k)).value})
        self._pl = BoundaryPipeline(
            p, ego_yaw_deg=float(dp("ego_yaw_correction_deg", frames.EGO_YAW_IN_LIDAR_DEG).value),
            objects_max_age=float(dp("objects_max_age", 0.5).value),
            obstacle_max_skew=float(dp("obstacle_max_skew", 0.15).value))
        for k in STEPS:
            # Extraction defaults off: see docs/road_boundary.md (it misfires on wide paved lots).
            self._pl.steps[k] = bool(dp(f"enable_{k}", k != "extraction").value)
        self._pl.use_objects = bool(dp("use_objects", True).value)
        self._enabled = {s: bool(dp(f"enable_{s}", True).value) for s in SOURCES}

        seg_topic = str(dp("segmentation_topic",
                           common["topics"]["sphereformer"]["segmentation_mask"]).value)
        mask_topic = str(dp("sam3_mask_topic", "/segmentation/mask_image/compressed").value)
        proj_topic = str(dp("projection_topic",
                            cfg["topics"]["measurements"]["projection_ground_flagged"]).value)
        info_topic = str(dp("camera_info_topic", common["topics"]["raw"]["camera_info"]).value)
        obj_topic = str(dp("objects_topic", cfg["topics"]["object_fusion"]["objects"]).value)
        odom_topic = str(dp("odom_topic", "/novatel/oem7/odom").value)

        self._image_wh = image_size_from_proj()
        self._projections = StampMatchedBuffer(
            "projection", buffer_duration=float(dp("projection_buffer_duration", 2.0).value),
            max_skew=float(dp("max_pairing_skew", 0.08).value),
            wait_for_newer=float(dp("wait_for_newer", 0.06).value), wrap=GroundFlaggedCloud)

        self._pubs = {s: {side: self.create_publisher(PointCloud2, out[s][side], 5)
                          for side in ("left", "right")} for s in SOURCES}
        self._pub_markers = self.create_publisher(MarkerArray, out["markers"], 2)

        self.create_subscription(PointCloud2, seg_topic, self._on_segmentation, 2)
        self.create_subscription(CompressedImage, mask_topic, self._on_mask, 2)
        self.create_subscription(PointCloud2, proj_topic, self._on_projection, 5)
        self.create_subscription(CameraInfo, info_topic, self._on_camera_info, 1)
        self.create_subscription(FusedObjectArray, obj_topic, self._on_objects, 10)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 50)
        self.add_on_set_parameters_callback(self._on_set_parameters)
        self.create_timer(0.02, self._pump)
        self.create_timer(10.0, self._log_stats)

        self._ms = {s: 0.0 for s in SOURCES}
        self._unmatched_masks = 0
        self._warned_frame = set()
        self.get_logger().info(
            f"road_boundary: sources={self._enabled} steps={self._pl.steps} "
            f"objects={self._pl.use_objects} seg={seg_topic} mask={mask_topic} "
            f"proj={proj_topic} objects={obj_topic} odom={odom_topic} "
            f"ego_yaw={self._pl.ego_yaw_deg:+.2f}deg x=[{p.x_min}, {p.x_max}] bin={p.bin_len}")

    # ---- parameters ----------------------------------------------------------------------
    def _on_set_parameters(self, params):
        reshape = False
        for prm in params:
            name = prm.name
            if name.startswith("enable_") and name[7:] in STEPS:
                self._pl.steps[name[7:]] = bool(prm.value)
                if name == "enable_tracking" and not prm.value:
                    self._pl.reset()
            elif name.startswith("enable_") and name[7:] in SOURCES:
                self._enabled[name[7:]] = bool(prm.value)
            elif name == "use_objects":
                self._pl.use_objects = bool(prm.value)
            elif name in _TUNABLE:
                try:
                    self._pl.p.update(**{name: prm.value})
                except (TypeError, ValueError) as e:
                    return SetParametersResult(successful=False, reason=str(e))
                reshape |= name in _RESHAPE
            elif name == "ego_yaw_correction_deg":
                self._pl.ego_yaw_deg = float(prm.value)
            elif name == "objects_max_age":
                self._pl.objects_max_age = float(prm.value)
            elif name == "obstacle_max_skew":
                self._pl.obstacle_max_skew = float(prm.value)
        if reshape:
            self._pl.reset()
        return SetParametersResult(successful=True)

    # ---- inputs --------------------------------------------------------------------------
    def _on_odom(self, msg: Odometry):
        tw = msg.twist.twist
        self._pl.add_odom(_stamp(msg.header), tw.linear.x, tw.linear.y, tw.angular.z)

    def _on_camera_info(self, msg: CameraInfo):
        if msg.width > 0 and msg.height > 0:
            self._image_wh = (int(msg.width), int(msg.height))

    def _on_objects(self, msg: FusedObjectArray):
        objs = msg.objects
        self._pl.add_objects(
            _stamp(msg.header),
            [(o.pose.position.x, o.pose.position.y) for o in objs],
            [(o.size.x, o.size.y) for o in objs],
            [_yaw(o.pose.orientation) for o in objs],
            [(o.velocity.x, o.velocity.y) if o.velocity_valid else (0.0, 0.0) for o in objs])

    @staticmethod
    def _uint8_field(msg: PointCloud2, name: str):
        """A UINT8 field as an array, or None when the cloud does not carry it."""
        for fld in msg.fields:
            if fld.name == name and fld.datatype == PointField.UINT8:
                step = int(msg.point_step)
                n = len(msg.data) // step if step else 0
                a = np.frombuffer(bytes(msg.data), dtype=np.uint8)[: n * step].reshape(n, step)
                return a[:, fld.offset].astype(np.int32)
        return None

    def _on_segmentation(self, msg: PointCloud2):
        f = decode_fields(msg, ("x", "y", "z", "rgb"))
        if f["x"] is None or f["rgb"] is None:
            return
        xyz = np.stack([f["x"], f["y"], f["z"]], axis=1)
        ok = np.isfinite(xyz).all(axis=1)
        # SphereFormer publishes the class id as a `label` field on the upstream branch; the
        # colour decode is the fallback for clouds that only carry rgb.
        label = self._uint8_field(msg, "label")
        cls = label[ok] if label is not None else decode_semkitti_rgb(f["rgb"][ok])
        t0 = time.perf_counter()
        res = self._pl.sphereformer_frame(_stamp(msg.header), xyz[ok], cls)
        # The obstacles are buffered for SAM3 either way; only publishing is gated.
        if self._enabled["sphereformer"]:
            self._publish("sphereformer", msg.header, res, t0)

    def _on_projection(self, msg: PointCloud2):
        self._projections.add(msg)
        self._pump()

    def _on_mask(self, msg: CompressedImage):
        if not self._enabled["sam3"]:
            return
        pairing = self._projections.match(msg.header, now=self._now(), payload=msg)
        if pairing.outcome is not DEFERRED:
            self._complete(pairing)

    def _pump(self):
        for pairing in self._projections.drain(self._now()):
            self._complete(pairing)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _complete(self, pairing):
        if pairing.value is None:
            self._unmatched_masks += 1
            return
        mask = cv2.imdecode(np.frombuffer(bytes(pairing.payload.data), np.uint8),
                            cv2.IMREAD_GRAYSCALE)
        if mask is None:
            return
        xyz, u, v, ground = pairing.value.arrays()
        if xyz.shape[0] == 0:
            return
        t0 = time.perf_counter()
        header = pairing.value.header
        road_px = mask_road_pixels(mask, extraction=self._pl.steps["extraction"])
        res = self._pl.sam3_frame(_stamp(header), xyz, u, v, ground, road_px, self._image_wh)
        self._publish("sam3", header, res, t0)

    # ---- output --------------------------------------------------------------------------
    def _publish(self, src, header, res, t0):
        left, right, m, (yl, yr, z) = res
        if header.frame_id not in self._warned_frame and header.frame_id != "lidar_tc":
            self._warned_frame.add(header.frame_id)
            self.get_logger().warning(
                f"{src}: input frame '{header.frame_id}' is treated as lidar_tc-aligned.")
        # Published even when empty, as sam3_mask_transform does: an absent boundary must read
        # as absent rather than as the last one that worked.
        for side, pts in (("left", left), ("right", right)):
            self._pubs[src][side].publish(
                point_cloud2.create_cloud(header, _XYZ, pts.astype(np.float32)))
        self._ms[src] += 1e3 * (time.perf_counter() - t0)
        if self._pub_markers.get_subscription_count():
            self._publish_markers(src, header, m, yl, yr, z)

    def _publish_markers(self, src, header, m, yl, yr, z):
        arr = MarkerArray()
        colours = {"left": (0.1, 0.9, 0.2), "right": (0.1, 0.5, 1.0)}
        zz = np.where(np.isfinite(z), z, np.nanmedian(z) if np.isfinite(z).any() else 0.0)
        to_lidar = self._pl._to_lidar
        for side, y in (("left", yl), ("right", yr)):
            mk = Marker(header=header, ns=f"{src}/{side}", id=0, type=Marker.LINE_STRIP)
            mk.scale.x = 0.15
            mk.color.r, mk.color.g, mk.color.b = colours[side]
            mk.color.a = 1.0 if src == "sphereformer" else 0.6
            v = np.isfinite(y)
            if v.sum() >= 2:
                xy = to_lidar(np.column_stack((m.x[v], y[v])))
                mk.points = [Point(x=float(a), y=float(b), z=float(c))
                             for (a, b), c in zip(xy, zz[v])]
            else:
                mk.action = Marker.DELETE
            arr.markers.append(mk)
        # Rejected samples where they were: grey = occluded, orange = unsupported.
        for ns, state, rgb in (("occluded", OCCLUDED, (0.6, 0.6, 0.6)),
                               ("unsupported", UNSUPPORTED, (1.0, 0.6, 0.1))):
            mk = Marker(header=header, ns=f"{src}/{ns}", id=0, type=Marker.SPHERE_LIST)
            mk.scale.x = mk.scale.y = mk.scale.z = 0.4
            mk.color.r, mk.color.g, mk.color.b = rgb
            mk.color.a = 0.9
            pts = []
            for raw, st in ((m.raw_left, m.left_state), (m.raw_right, m.right_state)):
                k = (st == state) & np.isfinite(raw)
                if k.any():
                    pts.append(np.column_stack((m.x[k], raw[k], zz[k])))
            if pts:
                pp = np.concatenate(pts)
                xy = to_lidar(pp[:, :2])
                mk.points = [Point(x=float(a), y=float(b), z=float(c))
                             for (a, b), c in zip(xy, pp[:, 2])]
            else:
                mk.action = Marker.DELETE
            arr.markers.append(mk)
        self._pub_markers.publish(arr)

    def _log_stats(self):
        parts = []
        for s, st in self._pl.stats.items():
            if st["frames"]:
                bins = max(st["bins"], 1)
                tr = self._pl.trackers[s]
                parts.append(
                    f"{s}: {st['frames']} frames {self._ms[s] / st['frames']:.1f}ms "
                    f"occluded={100.0 * st['occluded'] / bins:.1f}% "
                    f"unsupported={100.0 * st['unsupported'] / bins:.1f}% "
                    f"empty_sides={st['empty_sides']} no_odom={st['no_odom']} "
                    f"resets={tr.resets} gated={tr.gated} width={tr.width:.2f}")
        if parts or self._unmatched_masks:
            self.get_logger().info(" | ".join(parts) + f" | unmatched_masks={self._unmatched_masks} "
                                   f"rewinds={self._pl.rewinds} odom_starved={self._pl.twist.starved}")


def main(args=None):
    rclpy.init(args=args)
    node = RoadBoundaryNode()
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
