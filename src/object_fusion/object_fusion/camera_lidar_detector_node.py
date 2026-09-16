"""Tracked 2D detections + the LiDAR projection -> Detection3D, with a real extent and yaw.

Consumes the EXISTING pipeline's outputs -- /yolo/tracking and /lidar_2d_projection -- and
re-runs neither YOLO nor the projection. fusion_node keeps running untouched alongside; this
is a parallel consumer, not a replacement, and /fused_bbox is unaffected.

What is new over fusion_node is everything after the point selection: a BEV rectangle fit for
extent and heading instead of a hardcoded 1.5 m cube and identity orientation, an anisotropic
range-dependent covariance instead of none, a surface-to-centroid correction where the fit is
not identifiable, and a camera-only fallback for boxes with no LiDAR return at all (which
fusion_node drops silently). The last three are each behind their own default-off gate.

Pairing is the repo's standard StampMatchedBuffer with two-sided deferral, so a detection is
fused against the cloud captured closest to its OWN stamp rather than whatever is current at
arrival.
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
import yaml
from ament_index_python.packages import get_package_share_directory

from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from yolo_msgs.msg import DetectionArray
from fusion_msgs.msg import Detection3D, Detection3DArray

from perception_common.configs import PROJ
from perception_common.stamp_sync import DEFERRED, StampMatchedBuffer
from perception_common.utils import stamp_to_seconds

from object_fusion.detection_geometry import (
    SHAPE_FIT_MIN_POINTS, YAW_FROM_RAY, YAW_FROM_SHAPE, camera_only_range_variance, choose_yaw,
    fit_rectangle, select_object_points, surface_to_centroid_offset,
)
from object_fusion.flagged_cloud import GroundFlaggedCloud
from object_fusion.class_vote import ClassVote
from object_fusion.measurement_gate import DepthJumpGate
from object_fusion.projection import camera_ground_position
from object_fusion.tracker import sigma_along, sigma_cross

#: Minimum fitted WIDTH for a rectangle fit to be believed, in metres.
#:
#: This replaces the fit's own ``quality`` score, which is worse than useless as a gate: a
#: DEGENERATE single-face fit scores 20.00, the maximum, because every point lies on an edge,
#: while a good but noisy two-face fit scores 12.41. Thresholding on quality would reject the
#: good fits and keep the bad ones, exactly backwards.
#:
#: A single visible face fits as ``L x W = 1.80 x 0.00`` with a yaw 90 deg from truth, so a
#: non-degenerate width is the direct test for the failure. Combined with choose_yaw's derived
#: 40 m range cap, that is what keeps the fit honest. INVENTED.
MIN_FIT_WIDTH_M = 0.6

_YAW_ENUM = {YAW_FROM_SHAPE: Detection3D.YAW_FROM_SHAPE_FIT,
             YAW_FROM_RAY: Detection3D.YAW_FROM_RAY_DEFAULT}


class CameraLidarDetectorNode(Node):
    def __init__(self):
        super().__init__("camera_lidar_detector")
        cfg = yaml.safe_load(
            open(get_package_share_directory("object_fusion") + "/config/topics.yaml",
                 encoding="utf-8"))
        common = yaml.safe_load(
            open(get_package_share_directory("perception_common") + "/topics.yaml",
                 encoding="utf-8"))
        # Switchable so the ground-removed projection can be A/B'd against
        # transform.py's by swapping one parameter.
        self._proj_topic = str(self.declare_parameter(
            "projection_topic",
            common["topics"]["transform"]["lidar_2d_projection"]).value)
        self._out_topic = cfg["topics"]["measurements"]["camera_lidar"]

        # Same bounds fusion_node uses and for the same measured reasons; see its comments.
        skew = float(self.declare_parameter("max_pairing_skew", 0.06).value)
        buf = float(self.declare_parameter("projection_buffer_duration", 2.0).value)
        wait = float(self.declare_parameter("wait_for_newer", 0.06).value)
        pump = float(self.declare_parameter("deferral_pump_period", 0.02,
                                            ParameterDescriptor(read_only=True)).value)
        # 10 m, not fusion_node's 25 m. MEASURED by sweeping this bound over four replays of
        # selfcal_loc_2026-09-08 and scoring /fused_bbox against radar range:
        #
        #   gate      15-25 m err median / sd      0-15 m  >1 m jumps   p90 jump
        #   25 m      -1.68 / 2.24                 25.9%                1.79
        #   15 m      -1.01 / 1.15                 27.2%                1.79
        #   10 m      -0.99 / 1.15                 17.4%                1.53
        #    5 m      -0.99 / 1.15                 16.2%                1.34
        #
        # 15 m captures the whole 15-25 m win -- the error SPREAD halves -- and going below 15
        # is what buys the near-field jump reduction. 10 m takes essentially all of both.
        # 5 m adds about one percentage point more for twice the exposure to the risk this
        # bound exists to manage: with a 0.4 m margin, an object shorter than that above the
        # road is stripped, and a traffic cone stands ~0.5 m. That cost was NOT measured when
        # this was chosen, on the belief that the bag held no cones. It does -- 2386 of its 4519
        # detections are cones (found by scripts/neighbour_ab.py) -- so the cone cost of this
        # bound is now measurable and still unmeasured.
        #
        # Bands at and beyond 25 m are bit-identical across all four arms, as expected: the
        # gate was already active there.
        #
        # fusion_node keeps its own 25 m default and is untouched; /fused_bbox is unaffected.
        self._ground_min_range = float(
            self.declare_parameter("ground_rejection_min_range", 10.0).value)
        self._ground_margin = float(self.declare_parameter("ground_margin", 0.4).value)
        self._ground_min_points = int(self.declare_parameter("ground_min_points", 2).value)
        # Off by default like every other new capability here.
        self._enable_extent = bool(
            self.declare_parameter("enable_extent_estimation", False).value)
        # Push a visible-surface position back along the ray to the centroid, using the class
        # prior, where the rectangle fit is not identifiable. Separate gate: it changes every
        # far-field position and is worth A/B-ing on its own.
        self._enable_centroid = bool(
            self.declare_parameter("enable_centroid_correction", False).value)
        # Emit an object for a 2D box with NO LiDAR return, ranged from box geometry.
        self._enable_camera_only = bool(
            self.declare_parameter("enable_camera_only_fallback", False).value)
        # Camera height above the road, metres. DERIVED: the LiDAR sits 2.37 m above the road
        # (jeep_selfcal_loc measured 2.366 +- 0.005; a direct median of near-field ground returns
        # gives 2.394; the 2.46 this shipped with was wrong) and tf_static puts camera_fl at
        # z = -0.8425 relative to lidar_tc, so 2.37 - 0.8425 = 1.5275.
        self._camera_height = float(
            self.declare_parameter("camera_height_m", 1.5275).value)
        self._skipped_no_points = 0
        # A box whose every LiDAR return segmentation calls ground: publish nothing (false) or
        # fall back to all its points (true, the original rule). See select_object_points: the
        # fallback put missed cones on road rings and caused 27% of the >2 m depth spikes.
        self._empty_fallback = bool(
            self.declare_parameter("segmentation_empty_fallback", False).value)
        # Withhold a one-frame depth outlier per tracker id. MEASURED on by default: spike rate
        # 5.2% -> 1.5% with no loss of range accuracy against radar; see measurement_gate.py.
        self._gate = DepthJumpGate(
            gate_min_m=float(self.declare_parameter("depth_gate_min_m", 1.5).value),
            gate_range_frac=float(self.declare_parameter("depth_gate_range_frac", 0.05).value))
        self._enable_gate = bool(self.declare_parameter("enable_depth_gate", True).value)
        self._n_seg_empty = 0
        # Size and label a box from its tracker id's recent score-weighted majority class, not the
        # latest label: a truck relabelled `train` for 2 frames became a 200 m box. See class_vote.py.
        self._enable_class_vote = bool(self.declare_parameter("enable_class_vote", True).value)
        self._vote = ClassVote(
            window_s=float(self.declare_parameter("class_vote_window_s", 2.0).value))

        # Per-class extents, read-only, from the radar package's salvaged table. Stored there
        # as [y, z, x] = [width, height, length].
        self._class_size = {}
        try:
            raw = yaml.safe_load(open(
                get_package_share_directory("radar_ros") + "/config/class_averages.yaml",
                encoding="utf-8")) or {}
            self._class_size = {str(k): (float(v[2]), float(v[0]), float(v[1]))
                                for k, v in (raw.get("classes") or {}).items()
                                if isinstance(v, (list, tuple)) and len(v) == 3}
        except (OSError, ValueError, KeyError, TypeError) as exc:
            self.get_logger().warning(
                f"no usable class_averages.yaml ({exc}); centroid correction and the "
                "camera-only fallback will fall back to a 1.5 m cube")

        # GroundFlaggedCloud instead of stamp_sync's default ProjectedCloud: identical parsing
        # plus the optional per-point `ground` flag from ground_projection_node. On a topic
        # without that field it yields ground=None and the selection is exactly the old rule.
        self._proj = StampMatchedBuffer("projection", buffer_duration=buf, max_skew=skew,
                                        wait_for_newer=wait, wrap=GroundFlaggedCloud)
        self._pub = self.create_publisher(Detection3DArray, self._out_topic, 10)
        self._markers = self.create_publisher(MarkerArray, self._out_topic + "_markers", 10)
        self._n_boxes = self._n_seg_used = self._n_seg_fallback = 0
        self.create_subscription(DetectionArray, "/yolo/tracking", self._det_cb, 10)
        self.create_subscription(PointCloud2, self._proj_topic, self._proj_cb, 10)
        if pump > 0.0:
            self.create_timer(pump, self._pump)
        self.create_timer(5.0, lambda: self.get_logger().info(
            f"pairing: {self._proj.status()} | boxes with no LiDAR return: "
            f"{self._skipped_no_points} | segmentation used {self._n_seg_used} "
            f"fallback {self._n_seg_fallback} ground-only {self._n_seg_empty} of "
            f"{self._n_boxes} boxes | depth gate dropped {self._gate.dropped} of "
            f"{self._gate.dropped + self._gate.admitted} | class vote overrode "
            f"{self._vote.overridden}"))
        self.get_logger().info(
            f"camera_lidar_detector: /yolo/tracking + {self._proj_topic} -> {self._out_topic}"
            f"  enable_extent_estimation={self._enable_extent}"
            f"  segmentation_empty_fallback={self._empty_fallback}"
            f"  enable_depth_gate={self._enable_gate}"
            f"  enable_class_vote={self._enable_class_vote}")

    def _proj_cb(self, msg):
        self._proj.add(msg)
        self._pump()

    def _det_cb(self, msg):
        p = self._proj.match(msg.header, now=self._now(), payload=msg)
        if p.outcome is not DEFERRED:
            self._complete(p)

    def _pump(self):
        for p in self._proj.drain(self._now()):
            self._complete(p)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _complete(self, pairing):
        if pairing.value is None or pairing.payload is None:
            return
        self._emit(pairing.payload, pairing.value)

    @staticmethod
    def _cov(pos_xy, r, sigma_a, sigma_c):
        """Anisotropic position covariance, built in the ray frame and rotated out."""
        uu = np.array([pos_xy[0], pos_xy[1]]) / max(r, 1e-6)
        nn = np.array([-uu[1], uu[0]])
        B = np.column_stack((uu, nn))
        C = B @ np.diag([sigma_a ** 2, sigma_c ** 2]) @ B.T
        cov = [0.0] * 9
        cov[0], cov[1], cov[3], cov[4] = C[0, 0], C[0, 1], C[1, 0], C[1, 1]
        cov[8] = 1.0
        return cov

    def _camera_only(self, det, cls, cx, cy, box_h, prior, entry, ground_xyz):
        """A box with no usable LiDAR return on the object, ranged from where its bottom edge meets
        the road. ``None`` when disabled, without ground-flagged returns, or without an answer.

        OFF BY DEFAULT, AND MEASURED TO STAY OFF. Used for ground-only cone boxes on the reference
        replay (scripts/neighbour_ab.py, arm DropNF_cam) it made the spike rate worse, 1.5% ->
        2.1% (cones 1.6% -> 2.9%): the camera ground intercept reads ~3.5 m short of radar inside
        40 m and ~9 m short at 40-60 m, landing on the road in front of the object -- a box-bottom
        or ~0.2 deg pitch calibration bias. Enable only after calibrating that.

        Geometry is projection.camera_ground_position: the ray through the box's bottom-centre
        pixel, through the real camera extrinsic (pitched ~2.7 deg down, 1.2 m ahead of the LiDAR),
        against the local height of LiDAR ground returns. The earlier version treated image row cy
        as the horizon and ignored the extrinsic, which put a 30 m cone at ~200 m.

        The covariance grows as r^4 -- the ground intercept's sensitivity is r^2/(f*h) per pixel.
        """
        if not self._enable_camera_only or ground_xyz is None:
            return None
        hit = camera_ground_position(cx, cy + box_h / 2.0, ground_xyz)
        if hit is None:
            return None
        p, _support = hit
        r = float(math.hypot(p[0], p[1]))
        bearing = math.atan2(p[1], p[0])

        d = Detection3D()
        d.header = entry.header
        d.sensor = Detection3D.SENSOR_CAMERA_ONLY
        (d.class_id, d.class_name), d.score = cls, det.score
        d.tracker_id = det.id
        d.point_count = 0
        d.pose.position.x, d.pose.position.y = float(p[0]), float(p[1])
        if prior is not None:
            d.size.x, d.size.y, d.size.z = prior
        else:
            d.size.x = d.size.y = d.size.z = 1.5
        d.pose.position.z = float(p[2] + 0.5 * d.size.z)
        d.pose.orientation.w = 1.0
        d.size_measured = False
        d.yaw = float(bearing)
        d.yaw_source = _YAW_ENUM[YAW_FROM_RAY]
        d.yaw_measured = False
        sa = math.sqrt(camera_only_range_variance(r, float(PROJ[1, 1]), self._camera_height))
        d.position_covariance = self._cov(np.array([p[0], p[1]]), r, sa, sigma_cross(r))
        d.radar_range = d.radar_range_rate = float("nan")
        d.radar_azimuth_deg = d.radar_amplitude = float("nan")
        return d

    def _emit(self, dets: DetectionArray, entry):
        xyz, u, v, ground = entry.arrays()
        out = Detection3DArray()
        out.header = entry.header
        if xyz.shape[0] == 0:
            self._publish(out)
            return
        x, y, z = xyz[:, 0], xyz[:, 1], xyz[:, 2]
        ground_xyz = xyz[ground] if (ground is not None and self._enable_camera_only) else None

        for det in dets.detections:
            cx, cy = float(det.bbox.center.position.x), float(det.bbox.center.position.y)
            w, h = float(det.bbox.size.x), float(det.bbox.size.y)
            mask = ((u >= cx - w / 2) & (u <= cx + w / 2)
                    & (v >= cy - h / 2) & (v <= cy + h / 2))

            cls = ((self._vote.vote(det.id, stamp_to_seconds(entry.header.stamp), det.class_id,
                                    det.class_name, det.score))
                   if self._enable_class_vote else (det.class_id, det.class_name))
            prior = self._class_size.get(cls[1])

            if not np.any(mask):
                self._skipped_no_points += 1
                d = self._camera_only(det, cls, cx, cy, h, prior, entry, ground_xyz)
                if d is not None:
                    out.detections.append(d)
                continue

            # The adopted rule, shared with scripts/ground_ab.py and scripts/neighbour_ab.py so the
            # node runs exactly what was scored: drop segmented ground, then the percentile cut,
            # then the nearest depth cluster.
            px, py, pz, used_seg = select_object_points(
                x[mask], y[mask], z[mask], None if ground is None else ground[mask],
                ground_min_range=self._ground_min_range, margin=self._ground_margin,
                min_points=self._ground_min_points, empty_fallback=self._empty_fallback)
            self._n_boxes += 1
            if ground is not None:
                if used_seg:
                    self._n_seg_used += 1
                else:
                    self._n_seg_fallback += 1
            if px.size == 0:
                # Only road in the box: no LiDAR range to give. Range it from the camera when
                # that fallback is enabled, exactly as for a box with no return at all.
                self._n_seg_empty += 1
                d = self._camera_only(det, cls, cx, cy, h, prior, entry, ground_xyz)
                if d is not None:
                    out.detections.append(d)
                continue

            # Gated on the visible-surface median BEFORE any centroid correction: that is the
            # quantity scripts/neighbour_ab.py scored, and the one the depth jumps are in.
            if self._enable_gate and not self._gate.admit(
                    det.id, stamp_to_seconds(entry.header.stamp),
                    float(np.median(px)), float(np.median(py))):
                continue

            d = Detection3D()
            d.header = entry.header
            d.sensor = Detection3D.SENSOR_CAMERA_LIDAR
            (d.class_id, d.class_name), d.score = cls, det.score
            d.tracker_id = det.id
            d.point_count = int(px.size)

            surf = np.array([float(np.median(px)), float(np.median(py))])
            r = float(math.hypot(surf[0], surf[1]))
            ray = math.atan2(surf[1], surf[0])

            raw_fit = (fit_rectangle(px, py, min_points=SHAPE_FIT_MIN_POINTS)
                       if self._enable_extent else None)
            # A single visible face fits as 1.80 x 0.00 with a yaw 90 deg from truth, so a
            # degenerate width is rejected here before choose_yaw ever sees it.
            fit = raw_fit if (raw_fit is not None and raw_fit[2] >= MIN_FIT_WIDTH_M) else None
            # Routed through choose_yaw, NOT used directly: that is where the derived 40 m cap
            # and the point-count rule live. Calling fit_rectangle straight through bypassed
            # both, and produced confident 90-degree yaws in the far field.
            yaw, yaw_src = choose_yaw(r, px.size, speed=0.0, velocity_heading=0.0,
                                      ray_heading=ray, fit=fit)

            if yaw_src == YAW_FROM_SHAPE and fit is not None:
                _y, length, width, fcx, fcy, _q = fit
                # Where the fit holds, its centre IS the centroid; no bias model is needed.
                pos_xy = np.array([fcx, fcy])
                d.size.x, d.size.y = float(length), float(width)
                d.size.z = float(np.max(pz) - np.min(pz)) if pz.size > 1 else 1.5
                d.size_measured = True
                d.yaw_measured = True
            else:
                pos_xy = surf.copy()
                if prior is not None:
                    d.size.x, d.size.y, d.size.z = prior
                else:
                    d.size.x = d.size.y = d.size.z = 1.5
                d.size_measured = False
                d.yaw_measured = False
                if self._enable_centroid:
                    # The median of visible returns sits on the near FACE. Push it back along
                    # the ray by half the extent in the viewing direction. The object heading
                    # is unknown here, so it is assumed to face the sensor -- least wrong for a
                    # leading vehicle, and the residual is carried in sigma_along below.
                    off = surface_to_centroid_offset(d.size.x, d.size.y, ray, ray)
                    pos_xy = pos_xy + off * np.array([math.cos(ray), math.sin(ray)])

            d.yaw = float(yaw)
            d.yaw_source = _YAW_ENUM[yaw_src]
            d.pose.position.x, d.pose.position.y = float(pos_xy[0]), float(pos_xy[1])
            d.pose.position.z = float(np.median(pz))
            half = 0.5 * d.yaw
            d.pose.orientation.z, d.pose.orientation.w = math.sin(half), math.cos(half)

            rr = float(math.hypot(pos_xy[0], pos_xy[1]))
            sa, sc = sigma_along(rr), sigma_cross(rr)
            if self._enable_centroid and not d.size_measured:
                # Correcting the mean without inflating the covariance makes the filter
                # over-trust a corrected position, which looks converged and is worse than the
                # bias it replaced. Half the class-length spread is the residual.
                sa = float(math.hypot(sa, 0.5 * 0.25 * d.size.x))
            d.position_covariance = self._cov(pos_xy, rr, sa, sc)
            d.radar_range = d.radar_range_rate = float("nan")
            d.radar_azimuth_deg = d.radar_amplitude = float("nan")
            out.detections.append(d)

        self._publish(out)

    def _publish(self, out):
        self._pub.publish(out)
        arr = MarkerArray()
        clear = Marker()
        clear.header = out.header
        clear.ns = "camera_lidar"
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        for i, d in enumerate(out.detections):
            m = Marker()
            m.header = out.header
            m.ns = "camera_lidar"
            m.id = i + 1
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose = d.pose
            m.scale.x = max(float(d.size.x), 0.2)
            m.scale.y = max(float(d.size.y), 0.2)
            m.scale.z = max(float(d.size.z), 0.2)
            # Blue so it reads apart from fusion_node's green /yolo/fused_bbox_markers in RViz.
            m.color.r, m.color.g, m.color.b, m.color.a = 0.20, 0.55, 1.00, 0.55
            m.lifetime.nanosec = 300_000_000
            arr.markers.append(m)
        self._markers.publish(arr)


def main():
    rclpy.init()
    node = CameraLidarDetectorNode()
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
