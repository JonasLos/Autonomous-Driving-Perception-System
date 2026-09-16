"""Attaches Delphi ESR radar range and velocity to the camera+LiDAR fused objects.

The camera+LiDAR path gives position but no velocity at all, and at long range its depth
can latch onto the road surface in front of a vehicle rather than the vehicle. Radar
measures range to ~0.1 m out to 175 m and range rate directly, which is exactly the pair of
gaps it fills.

This node is strictly downstream and strictly additive. It subscribes to ``/fused_bbox``
and never modifies it; the enriched result goes out on ``/tracked_objects`` as a different
message type, so existing subscribers are unaffected whether or not this node runs.

Two independent gates, both defaulting to OFF:

* ``enable_radar_fusion`` -- whether radar may MODIFY an object. False means every object
  is published ``SOURCE_LIDAR_ONLY``: a pure passthrough of ``/fused_bbox`` into the new
  message.
* ``publish_radar_only`` -- whether radar may CREATE an object. Requires the first.

With fusion off the node still associates radar in the background and logs what it would
have done ("shadow mode"), so the evidence needed to justify turning it on accrues without
radar ever touching the output.

Association happens in the RADAR's polar frame, not in Cartesian ``lidar_tc``: the sensor's
range and azimuth errors differ by an order of magnitude, and a single Euclidean gate cannot
be correct at both 10 m and 100 m. See ``radar_geometry.associate``.

Frames: the fused objects arrive in ``lidar_tc`` and are published in ``lidar_tc``
unchanged. Radar is transformed with a real tf2 lookup rather than a hardcoded matrix --
``/tf_static`` carries ``lidar_tc -> delphi_esr_radar``. Note that ``lidar_tc`` is itself
yawed ~5.35 deg from the vehicle axis while ``tf_static`` declares ``lidar_tc -> base_link``
as identity; radar and LiDAR agree with each other in ``lidar_tc``, so association is sound,
but that shear is still there for any consumer reading these as vehicle-frame coordinates.
"""

import math

import numpy as np

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult

from ament_index_python.packages import get_package_share_directory
import yaml

import tf2_ros
from visualization_msgs.msg import Marker, MarkerArray
from yolo_msgs.msg import DetectionArray
from delphi_esr_driver.msg import EsrTrackArray
from perception_msgs.msg import TrackedObject, TrackedObjectArray

from perception_common.stamp_sync import (
    DEFERRED,
    StampMatchedBuffer,
    apply_bounded_parameters,
)

from radar_ros.radar_geometry import (
    associate,
    cartesian_to_polar,
    gate_tracks,
    polar_to_cartesian,
    radial_velocity_vector,
)

NAN = float("nan")


class RadarFrame:
    """One gated radar sweep, already reduced to plain arrays.

    Buffered by StampMatchedBuffer, which only requires ``.header``. Gating happens once
    here rather than per-association so a sweep matched by two consecutive outputs is not
    filtered twice.
    """

    __slots__ = ("header", "range", "azimuth", "range_rate", "amplitude", "track_id")

    def __init__(self, header, rng, az, rr, amp, tid):
        self.header = header
        self.range = rng
        self.azimuth = az
        self.range_rate = rr
        self.amplitude = amp
        self.track_id = tid

    def __len__(self):
        return int(self.range.shape[0])


class RadarFusionNode(Node):
    def __init__(self) -> None:
        super().__init__("radar_fusion_node")

        topics_path = get_package_share_directory("perception_common") + "/topics.yaml"
        with open(topics_path, "r", encoding="utf-8") as f:
            topic_config = yaml.safe_load(f)

        # Per-class extents, salvaged from the retired legacy node. Optional: an object
        # simply keeps the fused 1.5m cube when the class is not listed.
        self._class_sizes = {}
        try:
            with open(
                get_package_share_directory("radar_ros") + "/config/class_averages.yaml",
                "r",
                encoding="utf-8",
            ) as f:
                raw = yaml.safe_load(f) or {}
            # Stored [y, z, x] = [width, height, length]; Vector3 is (x, y, z).
            self._class_sizes = {
                str(name): (float(d[2]), float(d[0]), float(d[1]))
                for name, d in (raw.get("classes") or {}).items()
                if isinstance(d, (list, tuple)) and len(d) == 3
            }
        except (OSError, ValueError, KeyError, TypeError) as exc:
            self.get_logger().warning(
                f"No usable class_averages.yaml ({exc}); every object keeps the fused "
                "1.5m cube."
            )

        self._radar_topic = topic_config["topics"]["raw"]["radar_tracks"]
        self._fused_topic = topic_config["topics"]["yolo"]["fused_bbox"]
        self._out_topic = topic_config["topics"]["radar"]["tracked_objects"]
        self._marker_topic = topic_config["topics"]["radar"]["track_markers"]

        # ---- the two gates, both off ----------------------------------------------
        # Off means radar cannot influence the output at all. Kept runtime-settable so a
        # bag can be A/B'd by flipping it mid-replay rather than by restarting, which is
        # the same live-rollback the pairing parameters have.
        self._enable_fusion = bool(
            self.declare_parameter("enable_radar_fusion", False)
            .get_parameter_value()
            .bool_value
        )
        # Emitting unmatched radar tracks as obstacles is a separate decision from letting
        # radar refine a known one, and it fails differently: the ESR reports guardrails,
        # manhole covers and overhead signs as tracks. Gated behind its own flag and behind
        # enable_radar_fusion, and left off until the shadow-mode false-alarm count is known.
        self._publish_radar_only = bool(
            self.declare_parameter("publish_radar_only", False)
            .get_parameter_value()
            .bool_value
        )

        # ---- pairing ---------------------------------------------------------------
        # Radar runs ~30Hz, so half a period is ~17ms. The driver stamps at CAN-bundle
        # assembly rather than at capture, which adds up to a further ~33ms of jitter, so
        # 0.05 covers the pair. Raise it before suspecting the association if `unmatched`
        # climbs.
        max_pairing_skew = float(
            self.declare_parameter("max_pairing_skew", 0.05)
            .get_parameter_value()
            .double_value
        )
        buffer_duration = float(
            self.declare_parameter("radar_buffer_duration", 2.0)
            .get_parameter_value()
            .double_value
        )
        wait_for_newer = float(
            self.declare_parameter("wait_for_newer", 0.05)
            .get_parameter_value()
            .double_value
        )
        stamp_offset = float(
            self.declare_parameter("radar_stamp_offset", 0.0)
            .get_parameter_value()
            .double_value
        )
        pump_period = float(
            self.declare_parameter(
                "deferral_pump_period", 0.02, ParameterDescriptor(read_only=True)
            )
            .get_parameter_value()
            .double_value
        )
        stats_log_period = float(
            self.declare_parameter(
                "stats_log_period", 5.0, ParameterDescriptor(read_only=True)
            )
            .get_parameter_value()
            .double_value
        )

        # ---- association gates ------------------------------------------------------
        self._assoc_max_range_err = float(
            self.declare_parameter("assoc_max_range_err", 3.0)
            .get_parameter_value()
            .double_value
        )
        self._assoc_max_azimuth_err_deg = float(
            self.declare_parameter("assoc_max_azimuth_err_deg", 3.0)
            .get_parameter_value()
            .double_value
        )

        # ---- track gating -----------------------------------------------------------
        self._min_range = float(
            self.declare_parameter("min_range", 1.0).get_parameter_value().double_value
        )
        self._max_range = float(
            self.declare_parameter("max_range", 175.0).get_parameter_value().double_value
        )
        # Default below the sensor floor, i.e. off. Measured amplitude on the 2026-08-25
        # bag is -10..18 with -10 the most common value (~30% of tracks), so it reads as a
        # floor rather than a weak return: a threshold of 0.0 would silently discard about
        # half of everything the radar reports.
        self._min_amplitude = float(
            self.declare_parameter("min_amplitude", -1e9)
            .get_parameter_value()
            .double_value
        )
        # INERT with the current driver -- update_count is 0 on every track in every sweep
        # of the 2026-08-25 bag, so ANY positive value here drops 100% of tracks. Kept as a
        # parameter only so a future driver that decodes it needs no code change. See
        # radar_geometry.gate_tracks.
        self._min_update_count = int(
            self.declare_parameter("min_update_count", 0)
            .get_parameter_value()
            .integer_value
        )

        # A fused range this far in front of the radar range is the road-return signature
        # described in fusion_node.reject_ground: measured 14.3m median, 22.2m worst.
        self._range_dispute_threshold = float(
            self.declare_parameter("range_dispute_threshold", 5.0)
            .get_parameter_value()
            .double_value
        )

        self._output_timeout = float(
            self.declare_parameter("output_timeout", 0.5)
            .get_parameter_value()
            .double_value
        )

        # The driver never sets frame_id on the track array -- only its marker paths set a
        # frame. Anything empty is assumed to be this.
        self._radar_frame = str(
            self.declare_parameter("radar_frame_override", "delphi_esr_radar")
            .get_parameter_value()
            .string_value
        )

        self._radar = StampMatchedBuffer(
            "radar",
            buffer_duration=max(0.0, buffer_duration),
            max_skew=max(0.0, max_pairing_skew),
            stamp_offset=stamp_offset,
            wait_for_newer=max(0.0, wait_for_newer),
            # Frames are pre-reduced in the callback; nothing left to parse lazily.
            wrap=lambda frame: frame,
        )

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        # (source_frame, target_frame) -> (R 2x2, t 2,). Static, so one successful lookup
        # holds for the life of the node.
        self._tf_cache: dict = {}
        self._tf_warned = False
        self._frame_warned = False

        self._last_publish = None
        self._last_unmatched_log = None
        # Shadow-mode tallies. These are the numbers that decide whether the gates ever
        # get turned on, so they are counted whether or not fusion is enabled.
        self._shadow_matched = 0
        self._shadow_objects = 0
        self._shadow_radar_only = 0
        self._shadow_disputed = 0
        self._range_residuals: list = []
        self._azimuth_residuals: list = []

        self._pub = self.create_publisher(TrackedObjectArray, self._out_topic, 10)
        self._markers_pub = self.create_publisher(MarkerArray, self._marker_topic, 10)

        self.create_subscription(EsrTrackArray, self._radar_topic, self._radar_cb, 10)
        self.create_subscription(DetectionArray, self._fused_topic, self._fused_cb, 10)

        self._watchdog_timer = self.create_timer(0.5, self._watchdog)
        if pump_period > 0.0:
            self._pump_timer = self.create_timer(pump_period, self._pump)
        if stats_log_period > 0.0:
            self._stats_timer = self.create_timer(stats_log_period, self._log_stats)

        self.add_on_set_parameters_callback(self._on_set_parameters)

        self.get_logger().info(
            f"Radar fusion node ready: {self._fused_topic} + {self._radar_topic} "
            f"-> {self._out_topic} | "
            f"enable_radar_fusion={self._enable_fusion} "
            f"(shadow mode {'off' if self._enable_fusion else 'ON'}) "
            f"publish_radar_only={self._publish_radar_only} "
            f"max_pairing_skew={self._radar.max_skew:.3f}s "
            f"wait_for_newer={self._radar.wait_for_newer:.3f}s "
            f"assoc_gate=+-{self._assoc_max_range_err:.1f}m/"
            f"+-{self._assoc_max_azimuth_err_deg:.1f}deg "
            f"radar_frame={self._radar_frame} "
            f"use_sim_time={self.get_parameter('use_sim_time').value}"
        )
        if not self._enable_fusion:
            self.get_logger().info(
                "enable_radar_fusion is FALSE: /tracked_objects is a passthrough of "
                f"{self._fused_topic} with source=LIDAR_ONLY. Radar is associated and "
                "logged but never applied. Set it true to enable."
            )

    # ------------------------------------------------------------------ housekeeping

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_set_parameters(self, params) -> SetParametersResult:
        for p in params:
            if p.name == "min_update_count" and int(p.value) < 0:
                return SetParametersResult(
                    successful=False, reason="min_update_count must be >= 0"
                )
            if p.name == "publish_radar_only" and bool(p.value) and not self._enable_fusion:
                # Silently accepting this would look like it worked while radar-only
                # objects never appeared, because _fuse short-circuits on the master gate.
                return SetParametersResult(
                    successful=False,
                    reason="publish_radar_only requires enable_radar_fusion to be true",
                )

        targets = {
            "max_pairing_skew": (self._radar, "max_skew"),
            "radar_buffer_duration": (self._radar, "buffer_duration"),
            "wait_for_newer": (self._radar, "wait_for_newer"),
            "assoc_max_range_err": (self, "_assoc_max_range_err"),
            "assoc_max_azimuth_err_deg": (self, "_assoc_max_azimuth_err_deg"),
            "min_range": (self, "_min_range"),
            "max_range": (self, "_max_range"),
            "range_dispute_threshold": (self, "_range_dispute_threshold"),
            "output_timeout": (self, "_output_timeout"),
        }
        ok, reason, applied = apply_bounded_parameters(params, targets)
        if not ok:
            return SetParametersResult(successful=False, reason=reason)
        for name, value in applied:
            self.get_logger().info(f"{name} set to {value:.3f}")

        for p in params:
            if p.name == "enable_radar_fusion":
                self._enable_fusion = bool(p.value)
                self.get_logger().warning(
                    f"enable_radar_fusion set to {self._enable_fusion} -- radar "
                    f"{'NOW AFFECTS' if self._enable_fusion else 'no longer affects'} "
                    f"{self._out_topic}"
                )
                if not self._enable_fusion:
                    self._publish_radar_only = False
            elif p.name == "publish_radar_only":
                self._publish_radar_only = bool(p.value)
                self.get_logger().warning(
                    f"publish_radar_only set to {self._publish_radar_only}"
                )
            elif p.name == "min_update_count":
                self._min_update_count = int(p.value)
            elif p.name == "min_amplitude":
                self._min_amplitude = float(p.value)
            elif p.name == "radar_frame_override":
                self._radar_frame = str(p.value)
                self._tf_cache.clear()

        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------------- tf2

    def _lookup(self, target_frame, source_frame):
        """Cached 2D rigid transform source -> target. None until tf is available.

        Reduced to 2D because the radar has no elevation channel: everything it reports
        lies in its own x-y plane, and carrying a z it never measured would imply a
        precision that is not there.
        """
        key = (source_frame, target_frame)
        cached = self._tf_cache.get(key)
        if cached is not None:
            return cached
        try:
            tf = self._tf_buffer.lookup_transform(
                target_frame, source_frame, rclpy.time.Time()
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as exc:
            if not self._tf_warned:
                self._tf_warned = True
                self.get_logger().warning(
                    f"No transform {source_frame} -> {target_frame} yet ({exc}). Radar "
                    "association is disabled until /tf_static arrives; objects still pass "
                    "through. Check that tf_static is being replayed."
                )
            return None

        q = tf.transform.rotation
        # Yaw only. Roll/pitch of a bumper radar against a roof LiDAR are small, and a full
        # 3D rotation would need an elevation the sensor does not provide.
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        c, s = math.cos(yaw), math.sin(yaw)
        R = np.array([[c, -s], [s, c]], dtype=np.float64)
        t = np.array(
            [tf.transform.translation.x, tf.transform.translation.y], dtype=np.float64
        )
        self._tf_cache[key] = (R, t)
        self.get_logger().info(
            f"Cached transform {source_frame} -> {target_frame}: "
            f"yaw={math.degrees(yaw):+.3f}deg t=({t[0]:+.3f}, {t[1]:+.3f})"
        )
        return R, t

    @staticmethod
    def _apply(R, t, xy):
        """xy is (N,2) in the source frame; returns (N,2) in the target frame."""
        if xy.shape[0] == 0:
            return xy
        return xy @ R.T + t

    # -------------------------------------------------------------------- callbacks

    def _radar_cb(self, msg: EsrTrackArray) -> None:
        n = len(msg.tracks)
        if n == 0:
            return

        rng = np.fromiter((t.range for t in msg.tracks), dtype=np.float64, count=n)
        az = np.fromiter((t.angle for t in msg.tracks), dtype=np.float64, count=n)
        rr = np.fromiter((t.range_rate for t in msg.tracks), dtype=np.float64, count=n)
        amp = np.fromiter((t.amplitude for t in msg.tracks), dtype=np.float64, count=n)
        status = np.fromiter((t.track_status for t in msg.tracks), dtype=np.int32, count=n)
        upd = np.fromiter((t.update_count for t in msg.tracks), dtype=np.int32, count=n)
        tid = np.fromiter((t.track_id for t in msg.tracks), dtype=np.int32, count=n)

        keep = gate_tracks(
            rng,
            az,
            amp,
            status,
            upd,
            min_range=self._min_range,
            max_range=self._max_range,
            min_amplitude=self._min_amplitude,
            min_update_count=self._min_update_count,
        )

        header = msg.header
        if not header.frame_id:
            header.frame_id = self._radar_frame
            if not self._frame_warned:
                self._frame_warned = True
                self.get_logger().info(
                    f"Radar messages carry an empty frame_id (the driver only sets one on "
                    f"its markers); assuming '{self._radar_frame}'. Override with "
                    "radar_frame_override."
                )

        self._radar.add(
            RadarFrame(header, rng[keep], az[keep], rr[keep], amp[keep], tid[keep])
        )
        self._pump()

    def _fused_cb(self, msg: DetectionArray) -> None:
        pairing = self._radar.match(msg.header, now=self._now(), payload=msg)
        if pairing.outcome is not DEFERRED:
            self._complete(pairing)

    def _pump(self) -> None:
        for pairing in self._radar.drain(self._now()):
            self._complete(pairing)

    def _complete(self, pairing) -> None:
        if pairing.value is None:
            # No radar for this frame is not an error -- publish the objects anyway. The
            # alternative, withholding output until radar matches, would make the whole
            # obstacle path depend on a sensor that is off by default.
            self._log_unmatched(pairing.skew, pairing.reason)
            if pairing.payload is not None:
                self._emit(pairing.payload, None, [])
            return
        self._fuse(pairing.payload, pairing.value)

    # ------------------------------------------------------------------------ fusion

    def _fuse(self, fused_msg: DetectionArray, radar: RadarFrame) -> None:
        lidar_frame = fused_msg.header.frame_id or "lidar_tc"
        pairs = []

        tf_to_radar = self._lookup(radar.header.frame_id, lidar_frame)
        if tf_to_radar is not None and len(radar) and fused_msg.detections:
            xy = np.array(
                [
                    [d.bbox3d.center.position.x, d.bbox3d.center.position.y]
                    for d in fused_msg.detections
                ],
                dtype=np.float64,
            )
            # Objects into the radar's frame, so both sides are compared in the space the
            # sensor's error model is expressed in.
            R, t = tf_to_radar
            obj_rng, obj_az = cartesian_to_polar(*self._apply(R, t, xy).T)

            pairs = associate(
                obj_rng,
                obj_az,
                radar.range,
                radar.azimuth,
                max_range_err=self._assoc_max_range_err,
                max_azimuth_err_deg=self._assoc_max_azimuth_err_deg,
            )

            # Shadow tallies, kept whether or not the gate is open.
            self._shadow_objects += len(fused_msg.detections)
            self._shadow_matched += len(pairs)
            self._shadow_radar_only += len(radar) - len(pairs)
            for oi, ri in pairs:
                self._range_residuals.append(float(obj_rng[oi] - radar.range[ri]))
                self._azimuth_residuals.append(float(obj_az[oi] - radar.azimuth[ri]))
                if abs(obj_rng[oi] - radar.range[ri]) > self._range_dispute_threshold:
                    self._shadow_disputed += 1

            pairs = [(oi, ri, float(obj_rng[oi])) for oi, ri in pairs]

        self._emit(fused_msg, radar, pairs)

    def _emit(self, fused_msg: DetectionArray, radar, pairs) -> None:
        out = TrackedObjectArray()
        out.header = fused_msg.header

        by_obj = {oi: (ri, orng) for oi, ri, orng in pairs} if self._enable_fusion else {}

        for i, det in enumerate(fused_msg.detections):
            obj = TrackedObject()
            obj.class_id = det.class_id
            obj.class_name = det.class_name
            obj.score = det.score
            obj.id = det.id
            obj.center = det.bbox3d.center
            # fusion_node emits a fixed 1.5m cube for every class. Substituting the class
            # average is strictly better than that, but it is still an assumed extent, not a
            # measured one -- see config/class_averages.yaml.
            dims = self._class_sizes.get(det.class_name)
            if dims is None:
                obj.size = det.bbox3d.size
            else:
                obj.size.x, obj.size.y, obj.size.z = dims
            obj.frame_id = det.bbox3d.frame_id or fused_msg.header.frame_id

            obj.source = TrackedObject.SOURCE_LIDAR_ONLY
            obj.velocity_valid = False
            obj.radar_range = NAN
            obj.radar_range_rate = NAN
            obj.radar_azimuth_deg = NAN
            obj.radar_amplitude = NAN
            obj.range_disputed = False
            obj.range_disagreement = NAN

            hit = by_obj.get(i)
            if hit is not None and radar is not None:
                ri, obj_range = hit
                obj.source = TrackedObject.SOURCE_RADAR_MATCHED
                obj.radar_range = float(radar.range[ri])
                obj.radar_range_rate = float(radar.range_rate[ri])
                obj.radar_azimuth_deg = float(radar.azimuth[ri])
                obj.radar_amplitude = float(radar.amplitude[ri])
                obj.radar_track_id = int(radar.track_id[ri])

                vx, vy = radial_velocity_vector(
                    radar.range_rate[ri], radar.azimuth[ri]
                )
                # Rotate the radial velocity out of the radar frame into the object frame.
                # Rotation only -- a velocity is a free vector, so the translation between
                # the two frames does not apply to it.
                tf_to_lidar = self._lookup(obj.frame_id, radar.header.frame_id)
                if tf_to_lidar is not None:
                    R, _ = tf_to_lidar
                    v = R @ np.array([float(vx), float(vy)], dtype=np.float64)
                    obj.velocity.x, obj.velocity.y = float(v[0]), float(v[1])
                    obj.velocity.z = 0.0
                    obj.velocity_valid = True

                disagreement = float(obj_range - radar.range[ri])
                obj.range_disagreement = disagreement
                obj.range_disputed = abs(disagreement) > self._range_dispute_threshold

            out.objects.append(obj)

        if self._enable_fusion and self._publish_radar_only and radar is not None:
            self._append_radar_only(out, radar, {ri for _, ri, _ in pairs})

        self._publish(out, radar)

    def _append_radar_only(self, out, radar: RadarFrame, matched_radar) -> None:
        """Emit unmatched radar tracks as objects. Off by default -- see the class docstring."""
        tf_to_lidar = self._lookup(out.header.frame_id or "lidar_tc", radar.header.frame_id)
        if tf_to_lidar is None:
            return
        R, t = tf_to_lidar

        for ri in range(len(radar)):
            if ri in matched_radar:
                continue
            x, y = polar_to_cartesian(radar.range[ri], radar.azimuth[ri])
            p = self._apply(R, t, np.array([[float(x), float(y)]]))[0]

            obj = TrackedObject()
            obj.class_id = -1
            obj.class_name = "radar_track"
            obj.score = 0.0
            obj.id = ""
            obj.center.position.x = float(p[0])
            obj.center.position.y = float(p[1])
            # The radar has no elevation channel, so z is unknown rather than zero. Ground
            # level in lidar_tc is about -2.46m; placing the centre of a 1.5m box there puts
            # it on the road, which is the least wrong assumption available.
            obj.center.position.z = -1.7
            obj.center.orientation.w = 1.0
            obj.size.x = obj.size.y = obj.size.z = 1.5
            obj.frame_id = out.header.frame_id or "lidar_tc"

            obj.source = TrackedObject.SOURCE_RADAR_ONLY
            obj.radar_range = float(radar.range[ri])
            obj.radar_range_rate = float(radar.range_rate[ri])
            obj.radar_azimuth_deg = float(radar.azimuth[ri])
            obj.radar_amplitude = float(radar.amplitude[ri])
            obj.radar_track_id = int(radar.track_id[ri])

            vx, vy = radial_velocity_vector(radar.range_rate[ri], radar.azimuth[ri])
            v = R @ np.array([float(vx), float(vy)], dtype=np.float64)
            obj.velocity.x, obj.velocity.y, obj.velocity.z = float(v[0]), float(v[1]), 0.0
            obj.velocity_valid = True

            obj.range_disputed = False
            obj.range_disagreement = NAN
            out.objects.append(obj)

    # ------------------------------------------------------------------------ output

    def _publish(self, msg: TrackedObjectArray, radar) -> None:
        self._pub.publish(msg)
        self._publish_markers(msg)
        self._last_publish = self._now()

    def _publish_markers(self, msg: TrackedObjectArray) -> None:
        arr = MarkerArray()
        delete_all = Marker()
        delete_all.header = msg.header
        delete_all.ns = "tracked_objects"
        delete_all.id = 0
        delete_all.action = Marker.DELETEALL
        arr.markers.append(delete_all)

        for i, obj in enumerate(msg.objects):
            m = Marker()
            m.header = msg.header
            m.ns = "tracked_objects"
            m.id = i
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose = obj.center
            m.scale.x = max(float(obj.size.x), 0.1)
            m.scale.y = max(float(obj.size.y), 0.1)
            m.scale.z = max(float(obj.size.z), 0.1)
            # Colour by provenance so a glance at RViz says where each box came from.
            if obj.source == TrackedObject.SOURCE_RADAR_MATCHED:
                m.color.r, m.color.g, m.color.b = 0.15, 0.55, 0.95
            elif obj.source == TrackedObject.SOURCE_RADAR_ONLY:
                m.color.r, m.color.g, m.color.b = 0.95, 0.65, 0.10
            else:
                m.color.r, m.color.g, m.color.b = 0.15, 0.85, 0.25
            m.color.a = 0.45
            arr.markers.append(m)

        self._markers_pub.publish(arr)

    def _watchdog(self) -> None:
        """Publish an empty result once output stops being refreshed.

        Without this a dead upstream leaves the last objects standing in RViz and in
        planning, which reads as current geometry. The timeout has to exceed the producer
        period or it stops meaning "dead" and starts meaning "slow": /fused_bbox runs 10Hz
        against a 0.5s default, so there is an order of magnitude of headroom.
        """
        if self._output_timeout <= 0.0 or self._last_publish is None:
            return
        if self._now() - self._last_publish > self._output_timeout:
            empty = TrackedObjectArray()
            newest = self._radar.newest()
            if newest is not None:
                empty.header.stamp = newest.header.stamp
            empty.header.frame_id = "lidar_tc"
            self._publish(empty, None)

    # -------------------------------------------------------------------- diagnostics

    def _log_unmatched(self, skew: float, reason=None) -> None:
        now = self._now()
        if self._last_unmatched_log is not None and now - self._last_unmatched_log < 1.0:
            return
        self._last_unmatched_log = now
        self.get_logger().warning(
            f"Unmatched radar: "
            f"{self._radar.describe_unmatched(skew, 'fused frame', reason=reason)}; "
            f"{self._radar.status()}"
        )

    def _log_stats(self) -> None:
        rate = (
            100.0 * self._shadow_matched / self._shadow_objects
            if self._shadow_objects
            else 0.0
        )

        def _stat(vals):
            if not vals:
                return "n/a"
            a = np.asarray(vals)
            return f"median={np.median(a):+.2f} p90={np.percentile(np.abs(a), 90):.2f}"

        self.get_logger().info(
            f"pairing: {self._radar.status()} | "
            f"{'APPLIED' if self._enable_fusion else 'SHADOW'} "
            f"assoc={self._shadow_matched}/{self._shadow_objects} ({rate:.1f}%) "
            f"radar_only_candidates={self._shadow_radar_only} "
            f"range_disputed={self._shadow_disputed} | "
            f"d_range[m] {_stat(self._range_residuals)} | "
            f"d_azimuth[deg] {_stat(self._azimuth_residuals)}"
        )
        # Bounded so a long run cannot grow these without limit.
        self._range_residuals = self._range_residuals[-4096:]
        self._azimuth_residuals = self._azimuth_residuals[-4096:]


def main() -> None:
    rclpy.init()
    node = RadarFusionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        # Both are ordinary shutdowns. Letting ExternalShutdownException escape is what
        # makes transform_node and sphereformer_node exit 1 on a clean Ctrl-C, which makes
        # the exit code useless for telling a crash from a stop.
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
