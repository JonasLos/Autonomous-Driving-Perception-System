"""The track-level aggregator: three measurement streams + ego motion -> /perception/objects.

``publish_mode`` defaults to ``filtered``: the published position is the EKF state, fused from
camera+LiDAR and radar, with velocity and covariance. It started as ``passthrough`` -- the
camera+LiDAR measurements republished with the filter running only INTERNALLY -- which is how the
radar node was introduced too, and it was switched on 2026-09-17 by the user once the filter beat
its input with radar held out (median range error 1.57 -> 1.21 m, jumps > 2 m 4.4% -> 2.9%,
lag +0.04 m) and they had watched it in RViz. ``publish_mode:=passthrough`` is the rollback;
the set of published objects is identical in both modes, only their position differs.

TIME. Measurements are released in CAPTURE order across sensors by a fixed-lag queue, and the
filter predicts to each measurement's own stamp before updating it. This is deliberately NOT
StampMatchedBuffer -- that is a pairwise matcher, one slow reference against one fast stream,
which is the right tool for fusion_node and wrong for three streams. The cost is stated rather
than hidden: output lags real time by ``measurement_lag``. Do not "fix" that by predicting to
wall-clock now before publishing; it would break the invariant that an output describes the
moment it is stamped with, and hide the latency from every consumer.

EGO YAW. The INS twist is body-referenced and the state lives in lidar_tc; those frames differ
by the measured -5.35 deg. Note that this error is NOT observable from inside the filter --
prediction and state carry the same wrong velocity, so the innovation goes to zero and the
track's velocity silently absorbs it. Verify it with scripts/object_ab.py --ego-yaw, which
measures it filter-free, never from this node's own residuals.
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
import yaml
from ament_index_python.packages import get_package_share_directory

import tf2_ros
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from fusion_msgs.msg import Detection3D, Detection3DArray, FusedObject, FusedObjectArray

from perception_common.stamp_sync import apply_bounded_parameters

from object_fusion import frames
from object_fusion.association import apply_sticky_ids, associate_radar, solve_assignment
from object_fusion.ego_motion import EgoTwist, TwistBuffer
from object_fusion.measurement_queue import DEFAULT_LAG_S, Measurement, MeasurementQueue
from object_fusion.tracker import (
    CAMERA_GATE_CHI2, RADAR_GATE_CHI2, gated_update, compensated_range_rate, init_from_radar, kalman_update,
    lidar_measurement, predict, process_noise, radar_R, radar_h_and_H, range_is_trustworthy,
    wrap_deg,
)
from object_fusion.detection_geometry import ExtentFilter
from object_fusion.track_store import (
    CONFIRMED, SENSOR_CAMERA, SENSOR_RADAR, Track, TrackStore, camera_expected,
    radar_expected,
)

NAN = float("nan")


class _Sweep:
    __slots__ = ("range", "azimuth", "range_rate", "amplitude", "track_id")

    def __init__(self, dets):
        self.range = np.array([d.radar_range for d in dets], dtype=np.float64)
        self.azimuth = np.array([d.radar_azimuth_deg for d in dets], dtype=np.float64)
        self.range_rate = np.array([d.radar_range_rate for d in dets], dtype=np.float64)
        self.amplitude = np.array([d.radar_amplitude for d in dets], dtype=np.float64)
        self.track_id = np.array([d.radar_track_id for d in dets], dtype=np.int32)


#: A backwards jump larger than this is a REWIND -- a looping replay or a new bag -- not the
#: ordinary out-of-order arrival the clock rule below absorbs (the camera path is slower than the
#: radar's, so measurements genuinely interleave by tens of milliseconds).
REWIND_S = 1.0


class ObjectAggregatorNode(Node):
    def __init__(self):
        super().__init__("object_aggregator")
        cfg = yaml.safe_load(
            open(get_package_share_directory("object_fusion") + "/config/topics.yaml",
                 encoding="utf-8"))
        common = yaml.safe_load(
            open(get_package_share_directory("perception_common") + "/topics.yaml",
                 encoding="utf-8"))

        m = cfg["topics"]["measurements"]
        self._out_topic = cfg["topics"]["object_fusion"]["objects"]
        self._marker_topic = cfg["topics"]["object_fusion"]["markers"]
        self._ego_frame = cfg["topics"]["frames"]["ego"]

        # ---- gates, all default OFF ----
        # passthrough: publish the RAW camera+LiDAR measurement position, so the filter runs
        # and logs but genuinely cannot reach the output. An earlier version published the
        # filtered state in both modes and only blanked the covariance, which meant the safety
        # gate did not gate anything.
        # filtered is the default since 2026-09-17 (the user's call, after watching it live).
        self._publish_mode = str(self.declare_parameter("publish_mode", "filtered").value)
        self._enable_radar_only_birth = bool(
            self.declare_parameter("enable_radar_only_birth", False).value)
        self._ego_yaw_deg = float(self.declare_parameter(
            "ego_yaw_correction_deg", frames.EGO_YAW_IN_LIDAR_DEG).value)

        self._lag = float(self.declare_parameter("measurement_lag", DEFAULT_LAG_S).value)
        self._sigma_long = float(self.declare_parameter("sigma_long", 2.0).value)
        self._sigma_lat = float(self.declare_parameter("sigma_lat", 1.0).value)
        self._output_timeout = float(self.declare_parameter("output_timeout", 0.5).value)
        # Widest Euclidean gap at which a camera detection may claim an existing track.
        # INVENTED; it is the one association bound here that is not in a sensor's native
        # space, and it exists only to stop a birth-per-frame in a crowded scene.
        #
        # Narrowing it to 4 m was measured and REJECTED. It does recover objects -- orphaned
        # measurements 10.5% -> 9.0% over a full replay loop -- but it pays for them with
        # births beside tracks that are still coasting: two published boxes on ONE measurement
        # went 3.0% -> 5.6%. The merge distance bound below buys the same objects far more
        # cheaply. ASSOC_MAX_DIST=4.0 reaches the measured alternative without a rebuild.
        self._assoc_max_dist = float(self.declare_parameter("assoc_max_dist", 6.0).value)
        # How far apart two tracks may be and still be called one object. The Mahalanobis
        # clause in should_merge cannot answer that on its own -- it measures distance in units
        # of a covariance that grows without limit while a track coasts -- so the reach is
        # bounded in metres here. Set it to inf to get the original unbounded behaviour back.
        self._merge_max_dist = float(self.declare_parameter("merge_max_dist", 2.5).value)
        # Outer bound on a sticky ByteTrack claim. Generous on purpose: the claim is meant to
        # survive the 14 m road-adoption jump that breaks a position-only associator, and only
        # to refuse a RECYCLED id that would teleport a track.
        self._sticky_sanity_dist = float(
            self.declare_parameter("sticky_sanity_dist", 20.0).value)
        # /odom_grid and /odom are BIT-IDENTICAL in every twist component and differ only in
        # pose yaw, by a constant 1.287 deg of UTM grid convergence. This node uses the twist
        # ONLY, so the two are interchangeable here.
        #
        # The default is /odom rather than /odom_grid for one practical reason: topics.yaml's
        # replay_input category lists `ins: /novatel/oem7/odom`, so that is what
        # play_rosbag.sh actually replays. Defaulting to /odom_grid meant the node subscribed
        # to a topic no replay ever published, and starved silently. Set odom_topic to
        # /novatel/oem7/odom_grid on the vehicle if that is what is running there.
        odom_topic = str(self.declare_parameter(
            "odom_topic", "/novatel/oem7/odom").value)
        self._odom_topic = odom_topic
        pump = float(self.declare_parameter("pump_period", 0.02,
                                            ParameterDescriptor(read_only=True)).value)

        self._twist = TwistBuffer(duration=8.0)
        self._queue = MeasurementQueue(lag=self._lag)
        self._store = TrackStore(enable_radar_only_birth=self._enable_radar_only_birth)
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._tf_cache = {}
        self._last_t = None
        self._last_publish = None
        self._cam_rejected = 0
        self._cam_applied = 0
        self._cam_forced = 0
        self._skipped_no_odom = 0
        self._dt_stats, self._dt_nonpos = {}, {}
        self._rewinds = 0
        self._last_odom_warn = None

        self._pub = self.create_publisher(FusedObjectArray, self._out_topic, 10)
        self._markers = self.create_publisher(MarkerArray, self._marker_topic, 10)
        self.create_subscription(Detection3DArray, m["camera_lidar"], self._cam_cb, 10)
        self.create_subscription(Detection3DArray, m["radar"], self._radar_cb, 10)
        self.create_subscription(Odometry, odom_topic, self._odom_cb, 50)
        if pump > 0.0:
            self.create_timer(pump, self._pump)
        self.create_timer(5.0, self._log_stats)
        self.create_timer(0.5, self._watchdog)
        self.add_on_set_parameters_callback(self._on_set_parameters)

        self.get_logger().info(
            f"object_aggregator -> {self._out_topic} (frame {self._ego_frame}) | "
            f"publish_mode={self._publish_mode} "
            f"enable_radar_only_birth={self._enable_radar_only_birth} "
            f"ego_yaw={self._ego_yaw_deg:+.2f}deg lag={self._lag:.3f}s odom={odom_topic}")
        if self._publish_mode == "passthrough":
            self.get_logger().warning(
                "publish_mode=passthrough (ROLLBACK): the filter runs but the OUTPUT is a "
                "passthrough of the camera+LiDAR measurements -- no radar range, no velocity. "
                "The default is publish_mode:=filtered.")


    # ------------------------------------------------------------------ housekeeping
    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def _stamp(header):
        return header.stamp.sec + header.stamp.nanosec * 1e-9

    def _on_set_parameters(self, params):
        for p in params:
            if p.name == "publish_mode" and str(p.value) not in ("passthrough", "filtered"):
                return SetParametersResult(successful=False,
                                           reason="publish_mode must be passthrough|filtered")
        ok, reason, applied = apply_bounded_parameters(params, {
            "measurement_lag": (self._queue, "lag"),
            "output_timeout": (self, "_output_timeout"),
            "sigma_long": (self, "_sigma_long"),
            "sigma_lat": (self, "_sigma_lat"),
        })
        if not ok:
            return SetParametersResult(successful=False, reason=reason)
        for p in params:
            if p.name == "publish_mode":
                self._publish_mode = str(p.value)
                self.get_logger().warning(f"publish_mode -> {self._publish_mode}")
            elif p.name == "enable_radar_only_birth":
                self._enable_radar_only_birth = bool(p.value)
                self._store.enable_radar_only_birth = self._enable_radar_only_birth
                self.get_logger().warning(
                    f"enable_radar_only_birth -> {self._enable_radar_only_birth}")
            elif p.name == "ego_yaw_correction_deg":
                self._ego_yaw_deg = float(p.value)
        return SetParametersResult(successful=True)

    def _accrue(self, tr, sensor, hit, expected, t):
        """Accrue existence for one track from one sensor's outcome.

        ``expected`` is the whole point: a miss counts against a track ONLY where the sensor
        could actually have seen it. Charging a track for being outside the radar's field of
        view, or past transform.py's crop, kills real objects for being invisible -- which is
        how existence logic usually fails silently.
        """
        dt = 0.0 if tr.last_existence_t is None else max(0.0, t - tr.last_existence_t)
        self._store.update_existence(tr, sensor, hit, expected, dt)
        tr.last_existence_t = t

    def _lookup(self, target, source):
        key = (source, target)
        if key in self._tf_cache:
            return self._tf_cache[key]
        try:
            tf = self._tf_buffer.lookup_transform(target, source, rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        c, s = math.cos(yaw), math.sin(yaw)
        out = (np.array([[c, -s], [s, c]]),
               np.array([tf.transform.translation.x, tf.transform.translation.y]))
        self._tf_cache[key] = out
        return out

    # -------------------------------------------------------------------- callbacks
    def _odom_cb(self, msg: Odometry):
        tw = msg.twist.twist
        self._twist.add(EgoTwist(self._stamp(msg.header), tw.linear.x, tw.linear.y,
                                 tw.angular.z))

    def _cam_cb(self, msg: Detection3DArray):
        self._queue.add(Measurement(self._stamp(msg.header), "camera_lidar", msg))
        self._pump()

    def _radar_cb(self, msg: Detection3DArray):
        self._queue.add(Measurement(self._stamp(msg.header), "radar", msg))
        self._pump()

    def _pump(self):
        for meas in self._queue.release(self._now()):
            self._apply(meas)

    # ------------------------------------------------------------------------ filter
    def _ego_in_lidar(self, t):
        tw = self._twist.at(t)
        if tw is None:
            return None, None
        v = frames.ego_to_lidar([[tw.vx, tw.vy]], correction_deg=self._ego_yaw_deg)[0]
        return tw, v

    def _dt_summary(self):
        """Per sensor: how many measurements could not advance the clock, and the median step.

        A stream that is systematically behind the other cannot be predicted to, so the state is
        only moved by its own updates -- which is what a track frozen in range looks like.
        """
        out = []
        for k, v in self._dt_stats.items():
            out.append((k, len(v), 1000.0 * float(np.median(v)) if v else 0.0))
        return out

    def _apply(self, meas: Measurement):
        t = meas.stamp
        if self._last_t is None:
            self._last_t = t
        dt = t - self._last_t
        # A measurement whose stamp does not advance the clock cannot be predicted to, so the
        # state moves only by its own update. Counted per sensor: a stream systematically behind
        # the other is exactly what a track frozen in range looks like.
        key = "camera" if meas.sensor == "camera_lidar" else "radar"
        self._dt_stats.setdefault(key, []).append(dt)
        del self._dt_stats[key][:-2000]
        if dt <= 0.0:
            self._dt_nonpos[key] = self._dt_nonpos.get(key, 0) + 1
        if dt < -REWIND_S:
            # Everything held belongs to a different stretch of road, and since the clock never
            # rewinds (below), nothing would ever be predicted again -- the tracks would crawl
            # along on updates alone, which is exactly how a looping replay looked.
            self.get_logger().warning(
                f"time jumped back {-dt:.1f} s ({self._last_t:.3f} -> {t:.3f}): bag rewind. "
                f"Dropping {len(self._store.tracks)} tracks and the odometry buffer.")
            self._store = TrackStore(enable_radar_only_birth=self._enable_radar_only_birth)
            self._twist = TwistBuffer(duration=8.0)
            self._last_t = t
            self._rewinds += 1
            self._dt_stats.clear()
            self._dt_nonpos.clear()
            return
        if dt > 0.0:
            inc = self._twist.increment(self._last_t, t)
            if inc is None:
                # Odometry cannot cover this step. Stop predicting rather than extrapolate a
                # stale twist across every track at once -- but say so, loudly and rate
                # limited. Silently skipping every measurement is indistinguishable from a
                # quiet scene, which is exactly how this went unnoticed the first time.
                self._skipped_no_odom += 1
                self._last_t = max(self._last_t, t)
                now = self._now()
                if self._last_odom_warn is None or now - self._last_odom_warn > 5.0:
                    self._last_odom_warn = now
                    self.get_logger().warning(
                        f"no odometry covering [{self._last_t:.3f}, {t:.3f}] -- skipped "
                        f"{self._skipped_no_odom} measurements so far. Check that "
                        f"'{self._odom_topic}' is being published; the filter cannot predict "
                        "without it.")
                return
            dpsi, d_body = inc
            d_lidar = frames.ego_to_lidar([d_body], correction_deg=self._ego_yaw_deg)[0]
            Q = process_noise(dt, self._sigma_long, self._sigma_lat)
            for tr in self._store.tracks:
                tr.x, tr.P = predict(tr.x, tr.P, dt, dpsi, d_lidar, Q)
        # NEVER let the prediction clock run backwards. A measurement released out of capture
        # order -- the camera path is slower than the radar's, and `late` counts hundreds of them
        # per drive -- used to rewind _last_t, so the NEXT measurement predicted across an
        # interval that had already been applied and ego motion was counted twice. Every static
        # object then drifted forward at a fraction of ego speed (measured live: +3.8 m/s,
        # ~35% of ego speed) and the box visibly trailed the detection between camera frames.
        self._last_t = max(self._last_t, t)

        if meas.sensor == "camera_lidar":
            self._apply_camera(meas.payload, t)
        else:
            self._apply_radar(meas.payload, t)

        self._store.prune(t)
        self._store.merge_pass(max_merge_dist=self._merge_max_dist)
        self._store.promote()
        self._publish(meas.payload.header, t)

    def _apply_camera(self, msg: Detection3DArray, t):
        """Associate, update and birth from one camera+LiDAR measurement array.

        Association is the tested pair: a GLOBAL assignment over Euclidean cost
        (``solve_assignment``), then ``apply_sticky_ids`` overriding it wherever a ByteTrack id
        already owns a track. Both are imported rather than reimplemented -- an earlier version
        of this method open-coded a greedy nearest-neighbour loop, which meant the code under
        test and the code in the node were different.
        """
        dets = list(msg.detections)
        tracks = self._store.tracks
        pairs = []
        if dets and tracks:
            P = np.array([[d.pose.position.x, d.pose.position.y] for d in dets])
            T = np.array([tr.x[:2] for tr in tracks])
            cost = np.linalg.norm(T[:, None, :] - P[None, :, :], axis=2)
            pairs = solve_assignment(cost, cost <= self._assoc_max_dist)
            # The visual tracker has already solved association in the image with appearance
            # evidence the filter does not have. Its id wins, inside an outer sanity bound
            # that stops a RECYCLED id from teleporting a track across the scene.
            pairs = apply_sticky_ids(
                tracks, dets, pairs,
                sanity_ok=lambda tr, d: float(np.hypot(
                    tr.x[0] - d.pose.position.x, tr.x[1] - d.pose.position.y))
                    <= self._sticky_sanity_dist,
                id_of_track=lambda tr: tr.bytetrack_id,
                id_of_detection=lambda d: d.tracker_id)

        by_det = {di: ti for ti, di in pairs}
        used = {ti for ti, _ in pairs}

        for di, det in enumerate(dets):
            p = np.array([det.pose.position.x, det.pose.position.y])
            ti = by_det.get(di)
            if ti is None:
                tr = Track(np.array([p[0], p[1], 0.0, 0.0]),
                           np.diag([4.0, 4.0, 400.0, 400.0]), t, SENSOR_CAMERA)
                tr.bytetrack_id = det.tracker_id
                tr.vote_class(det.class_name)
                if det.size_measured:
                    tr.size = det.size
                self._store.add(tr)
                continue

            tr = tracks[ti]
            r = float(np.linalg.norm(tr.x[:2]))
            y, H, R = lidar_measurement(tr.x[:2], p,
                                        drop_range=not range_is_trustworthy(r))
            tr.x, tr.P, _, applied, forced = gated_update(
                tr.x, tr.P, y, H, R, gate_chi2=CAMERA_GATE_CHI2,
                consecutive_rejects=tr.consecutive_rejects)
            if applied:
                self._cam_applied += 1
                tr.hits["camera_lidar"] += 1
                tr.last_update = t
                tr.last_camera_update = t
                tr.consecutive_rejects = 0
                if forced:
                    tr.forced_updates += 1
                    self._cam_forced += 1
            else:
                tr.consecutive_rejects += 1
                # Rejected, not absorbed. The along-ray error is a mixture, and an outlier
                # here is the road-adoption population.
                self._cam_rejected += 1
                tr.camera_rejected += 1
            tr.sensors_ever |= SENSOR_CAMERA
            tr.sensors_this_cycle |= SENSOR_CAMERA
            tr.bytetrack_id = det.tracker_id or tr.bytetrack_id
            tr.vote_class(det.class_name)
            tr.last_cam_xy = p.copy()
            if det.size_measured:
                # A measured extent is a LOWER bound -- occlusion only ever makes an object
                # look shorter. Taking a running upper quantile tracks the true extent from
                # below; averaging would bias it low forever.
                if tr.extent_l is None:
                    tr.extent_l = ExtentFilter(float(det.size.x))
                    tr.extent_w = ExtentFilter(float(det.size.y))
                tr.size = Vector3(x=tr.extent_l.update(float(det.size.x)),
                                  y=tr.extent_w.update(float(det.size.y)),
                                  z=float(det.size.z))
                tr.size_measured = True
                tr.yaw, tr.yaw_source = det.yaw, det.yaw_source
            else:
                # A class prior. Keep any genuinely measured extent already on the track
                # rather than overwriting it with a prior.
                if not tr.size_measured:
                    tr.size = det.size
                if tr.yaw_source is None:
                    tr.yaw, tr.yaw_source = det.yaw, det.yaw_source

        # Existence, for EVERY track -- a track the camera did not update is evidence against
        # itself only if the camera could have seen it.
        for k, tr in enumerate(self._store.tracks):
            expected = camera_expected(tr.x[:2])
            if expected:
                tr.opportunities["camera_lidar"] += 1
            self._accrue(tr, "camera_lidar", k in used, expected, t)

    def _apply_radar(self, msg: Detection3DArray, t):
        if not msg.detections:
            return
        lidar_frame = "lidar_tc"
        tf = self._lookup(msg.header.frame_id or "delphi_esr_radar", lidar_frame)
        if tf is None:
            return
        R_sl, t_sl = tf
        tw, v_lidar = self._ego_in_lidar(t)
        if tw is None:
            return
        v_ego_s = R_sl @ v_lidar
        sweep = _Sweep(msg.detections)

        pairs, _info = associate_radar(self._store.tracks, sweep, R_sl, t_sl, v_ego_s,
                                       max_azimuth_err_deg=1.0, max_range_err=60.0)
        matched = {di for _, di in pairs}
        for ti, di in pairs:
            tr = self._store.tracks[ti]
            z_pred, H = radar_h_and_H(tr.x, R_sl, t_sl, v_ego_s)
            y = np.array([sweep.range[di] - z_pred[0],
                          sweep.range_rate[di] - z_pred[1],
                          float(wrap_deg(sweep.azimuth[di] - z_pred[2]))])
            tr.x, tr.P, _, applied = kalman_update(tr.x, tr.P, y, H, radar_R(),
                                                   gate_chi2=RADAR_GATE_CHI2)
            if applied:
                tr.hits["radar"] += 1
                tr.radar_hits += 1
                tr.last_update = t
                tr.last_radar_update = t
                s = float(compensated_range_rate(sweep.range_rate[di], sweep.azimuth[di],
                                                 v_ego_s))
                if abs(s) >= self._store.radar_only_min_speed:
                    tr.moving_hits += 1
            tr.sensors_ever |= SENSOR_RADAR
            tr.sensors_this_cycle |= SENSOR_RADAR
            tr.radar_slot = int(sweep.track_id[di])

        matched_tracks = {ti for ti, _ in pairs}
        for k, tr in enumerate(self._store.tracks):
            p_s = R_sl @ tr.x[:2] + t_sl
            expected = radar_expected(p_s)
            if expected:
                tr.opportunities["radar"] += 1
            if k in matched_tracks:
                s_k = float(compensated_range_rate(
                    sweep.range_rate[dict(pairs)[k]], sweep.azimuth[dict(pairs)[k]], v_ego_s))
                key = ("radar_moving" if abs(s_k) >= self._store.radar_only_min_speed
                       else "radar_static")
                self._accrue(tr, key, True, expected, t)
            else:
                self._accrue(tr, "radar_static", False, expected, t)

        R_ls, t_ls = R_sl.T, -R_sl.T @ t_sl
        for di in range(sweep.range.size):
            if di in matched:
                continue
            s = float(compensated_range_rate(sweep.range_rate[di], sweep.azimuth[di], v_ego_s))
            if not self._store.may_birth_radar_only(s):
                continue
            x0, P0 = init_from_radar(sweep.range[di], sweep.azimuth[di],
                                     sweep.range_rate[di], v_ego_s, R_ls, t_ls)
            tr = Track(x0, P0, t, SENSOR_RADAR)
            tr.radar_slot = int(sweep.track_id[di])
            self._store.add(tr)
            self._store.shadow["radar_only_born"] += 1

    # ------------------------------------------------------------------------ output
    def _publish(self, header, t):
        out = FusedObjectArray()
        out.header.stamp = header.stamp
        out.header.frame_id = self._ego_frame
        filtered = self._publish_mode == "filtered"

        for tr in self._store.tracks:
            if not self._store.may_publish(tr):
                continue
            o = FusedObject()
            o.track_id = tr.id
            o.age = tr.age(t)
            o.update_count = sum(tr.hits.values())
            o.class_name = tr.class_name()
            o.score = float(tr.existence)
            src_xy = tr.x[:2] if filtered else tr.last_cam_xy
            if src_xy is None:
                # Radar-only track in passthrough mode: it has no measurement the old pipeline
                # would have produced, so it is not published at all rather than leaking the
                # filter's estimate through a gate that is supposed to be closed.
                continue
            xy = frames.lidar_to_ego([src_xy], correction_deg=self._ego_yaw_deg)[0]
            vxy = frames.lidar_to_ego([tr.x[2:]], correction_deg=self._ego_yaw_deg)[0]
            o.pose.position.x, o.pose.position.y = float(xy[0]), float(xy[1])
            o.pose.orientation.w = 1.0
            if tr.size is not None:
                o.size = tr.size
            o.size_measured = bool(tr.size_measured)
            o.yaw_source = int(tr.yaw_source if tr.yaw_source is not None
                               else FusedObject.YAW_FROM_RAY_DEFAULT)
            o.covariance = ([float(c) for c in np.asarray(tr.P).ravel()]
                            if filtered else [NAN] * 16)
            o.velocity.x, o.velocity.y = ((float(vxy[0]), float(vxy[1])) if filtered
                                          else (0.0, 0.0))
            o.velocity_covariance = [float(c) for c in np.asarray(tr.P)[2:, 2:].ravel()]
            o.velocity_valid = filtered and self._twist.newest() is not None
            o.existence_probability = float(tr.existence)
            o.track_status = (FusedObject.STATUS_CONFIRMED if tr.status == CONFIRMED
                              else FusedObject.STATUS_TENTATIVE)
            o.contributions = int(tr.sensors_ever)
            o.contributions_this_frame = int(tr.sensors_this_cycle)
            o.camera_range_dropped = not range_is_trustworthy(
                float(np.linalg.norm(tr.x[:2])))
            o.camera_updates_rejected = int(tr.camera_rejected)
            o.frame_id = self._ego_frame
            tr.sensors_this_cycle = 0
            out.objects.append(o)

        self._pub.publish(out)
        self._publish_markers(out)
        self._last_publish = self._now()

    def _publish_markers(self, msg: FusedObjectArray):
        arr = MarkerArray()
        clear = Marker()
        clear.header = msg.header
        clear.ns = "fused_objects"
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        for i, o in enumerate(msg.objects):
            mk = Marker()
            mk.header = msg.header
            mk.ns = "fused_objects"
            mk.id = i
            mk.type = Marker.CUBE
            mk.action = Marker.ADD
            mk.pose = o.pose
            mk.scale.x = max(float(o.size.x), 0.1)
            mk.scale.y = max(float(o.size.y), 0.1)
            mk.scale.z = max(float(o.size.z), 0.1)
            # Colour by provenance, so a glance says where each box came from.
            if o.contributions & FusedObject.CONTRIB_RADAR and \
               o.contributions & FusedObject.CONTRIB_CAMERA:
                mk.color.r, mk.color.g, mk.color.b = 0.15, 0.55, 0.95
            elif o.contributions & FusedObject.CONTRIB_RADAR:
                mk.color.r, mk.color.g, mk.color.b = 0.95, 0.65, 0.10
            else:
                mk.color.r, mk.color.g, mk.color.b = 0.15, 0.85, 0.25
            mk.color.a = 0.25 + 0.5 * float(o.existence_probability)
            arr.markers.append(mk)

            # Velocity, drawn as one second of travel. Only the filtered state has one -- in
            # passthrough the velocity is zeroed, so nothing is drawn and the absence is honest.
            v = (float(o.velocity.x), float(o.velocity.y))
            if o.velocity_valid and math.hypot(*v) > 1.0:
                ar = Marker()
                ar.header = msg.header
                ar.ns = "fused_velocity"
                ar.id = i
                ar.type = Marker.ARROW
                ar.action = Marker.ADD
                ar.points = [Point(x=o.pose.position.x, y=o.pose.position.y, z=o.pose.position.z),
                             Point(x=o.pose.position.x + v[0], y=o.pose.position.y + v[1],
                                   z=o.pose.position.z)]
                ar.scale.x, ar.scale.y, ar.scale.z = 0.12, 0.35, 0.4   # shaft, head width, head len
                ar.color.r, ar.color.g, ar.color.b, ar.color.a = 1.0, 1.0, 1.0, 0.9
                arr.markers.append(ar)
        self._markers.publish(arr)

    def _watchdog(self):
        """Publish an empty result once output stops being refreshed.

        Without this a dead upstream leaves the last objects standing in RViz and in planning,
        which reads as current geometry.
        """
        if self._output_timeout <= 0.0 or self._last_publish is None:
            return
        if self._now() - self._last_publish > self._output_timeout:
            empty = FusedObjectArray()
            empty.header.frame_id = self._ego_frame
            self._pub.publish(empty)
            self._last_publish = self._now()

    def _log_stats(self):
        s = self._store.shadow
        total = self._cam_applied + self._cam_rejected
        rej = 100.0 * self._cam_rejected / total if total else 0.0
        self.get_logger().info(
            f"tracks={len(self._store.tracks)} mode={self._publish_mode} | "
            f"cam applied={self._cam_applied} rejected={self._cam_rejected} ({rej:.1f}%) "
            f"forced={self._cam_forced} | "
            f"radar_only candidates={s['radar_only_candidates']} born={s['radar_only_born']} | "
            f"merged={s['merged']} confirmed={s['confirmed']} | "
            f"queue={len(self._queue)} late={self._queue.late} "
            f"odom_held={self._twist.held} odom_starved={self._twist.starved} "
            f"skipped_no_odom={self._skipped_no_odom} rewinds={self._rewinds} | "
            + " ".join(f"{k} dt<=0 {self._dt_nonpos.get(k, 0)}/{n} med {ms:+.0f}ms"
                       for k, n, ms in self._dt_summary()))


def main():
    rclpy.init()
    node = ObjectAggregatorNode()
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
