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
stamp so consumers can account for that.

Matching is two-sided: a detection with no cloud yet captured at or after it is deferred for
up to wait_for_newer rather than forced backwards onto an older cloud. Deferred detections are
resolved by _pump, which runs after every buffered cloud and on a short timer.

Message stamps are only ever compared to each other, so the pairing itself is valid under bag
replay regardless of use_sim_time. Deferral expiry and the watchdog do read the node clock, so
run with use_sim_time:=true against a bag if you want them to track playback.
"""

import os

from ament_index_python.packages import get_package_share_directory
import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.node import Node
from sensor_msgs_py import point_cloud2 as pc2
import yaml
from sensor_msgs.msg import PointCloud2, PointField

from clrernet_msgs.msg import LanePoints
from perception_common.lane_geometry import (
    DEFAULT_EGO_YAW_DEG,
    GridSmoother,
    LanePairSelector,
    make_grid,
    resample_lane,
    sample_points,
)
from perception_common.stamp_sync import (
    DEFERRED,
    StampMatchedBuffer,
    apply_bounded_parameters,
)
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
# Condensed samples, not pixels: fewer than three distinct returns along a lane is not enough to
# interpolate from. Provisional until the per-lane match density inside the score window is
# measured -- see the plan's open items.
MIN_LANE_SAMPLES = 3


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
        # Legacy-path smoothing state; ema_alpha is set from its parameter below.
        self.prev_centerline = None

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
        # How long a detection may wait for a cloud captured at or after it. Without this,
        # matching can only reach backwards and skew doubles. Set to 0.0 to restore the old
        # one-sided behaviour at runtime, which is the rollback if this ever starves.
        wait_for_newer = float(
            self.declare_parameter("wait_for_newer", 0.06)
            .get_parameter_value()
            .double_value
        )
        # Deferrals are almost always resolved by the _pump that follows each buffered cloud;
        # this timer only bounds the wait when the projection stream stalls. Read-only because
        # a timer period cannot be changed after construction, and silently ignoring a
        # ros2 param set would be worse than rejecting it.
        pump_period = float(
            self.declare_parameter(
                "deferral_pump_period",
                0.02,
                ParameterDescriptor(read_only=True),
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

        # --- Lane geometry ----------------------------------------------------------------
        # Both boundaries of a lane are lifted onto one shared grid of x values before they are
        # compared, so a pair is always read at matched range. Read-only in the same spirit as
        # deferral_pump_period: changing the grid invalidates the selector's anchor and both
        # smoothers, and silently accepting that at runtime would be worse than requiring a
        # relaunch. The road band carries nothing below ~5 m, has a p95 near 70 m and returns
        # out to 148 m; nodes past a lane's own measured span are masked out anyway, so an
        # over-long grid costs a few unused slots while an over-short one would silently
        # truncate the horizon a consumer already receives.
        grid_min_x = float(
            self.declare_parameter(
                "lane_grid_min_x", 5.0, ParameterDescriptor(read_only=True)
            )
            .get_parameter_value()
            .double_value
        )
        grid_max_x = float(
            self.declare_parameter(
                "lane_grid_max_x", 100.0, ParameterDescriptor(read_only=True)
            )
            .get_parameter_value()
            .double_value
        )
        # Natural LiDAR spacing at 100 m is 0.58 m between rings, so a finer step would
        # interpolate rather than report.
        grid_step = float(
            self.declare_parameter(
                "lane_grid_step", 0.5, ParameterDescriptor(read_only=True)
            )
            .get_parameter_value()
            .double_value
        )
        # The window was narrowed to [6,15] because a wider one scored worse. That was the ego
        # reference diverging from the vehicle at 9.4 cm per metre of range, so a shorter lever arm
        # meant a smaller error -- not curvature, which is roughly 10x smaller than the shear was.
        # With the reference corrected to a ray the relationship inverts and more range is strictly
        # more evidence. Re-swept over the same 536-frame bag, 476 selected:
        #
        #   window    switches   p90 |d centre_y|   ego bracketed   TIER_NEAR_EMPTY
        #   [6,15]        0           0.058 m           14.7%            85.3%
        #   [6,30]        0           0.058 m          100.0%             0.0%
        #   [6,40]        0           0.058 m          100.0%             0.0%
        #
        # Stability is already saturated at [6,15]; what [6,30] buys is evidence. Below it the
        # near field rarely holds min_window_nodes, so 85% of frames were scored on the
        # TIER_NEAR_EMPTY fallback -- ranked last, on their own nearest nodes, and never actually
        # containment-tested. At [6,30] that is 0% and every decision is a full one.
        score_min_x = float(
            self.declare_parameter("lane_score_min_x", 6.0)
            .get_parameter_value()
            .double_value
        )
        score_max_x = float(
            self.declare_parameter("lane_score_max_x", 30.0)
            .get_parameter_value()
            .double_value
        )
        # Grid nodes are interpolation artifacts, so this is a coverage test only. The evidence
        # test is MIN_LANE_SAMPLES, which counts condensed returns.
        min_window_nodes = int(
            self.declare_parameter("lane_min_window_nodes", 8)
            .get_parameter_value()
            .integer_value
        )
        # Interpolation draws a straight line across a hole in the returns, and downstream that
        # is indistinguishable from measured road. Nodes further than half of this from a
        # measured sample are dropped instead.
        self.max_interp_gap = float(
            self.declare_parameter("lane_max_interp_gap", 5.0)
            .get_parameter_value()
            .double_value
        )
        # Backward travel in x as a fraction of forward travel, above which a lane is taken to
        # double back and is rejected rather than sorted. See lane_geometry.backtrack_ratio.
        self.max_backtrack = float(
            self.declare_parameter("lane_max_backtrack", 0.25)
            .get_parameter_value()
            .double_value
        )
        # Every pair of lanes is evaluated rather than only sort-adjacent ones, so this gate is
        # what rejects a pair spanning two lanes. Measured width of the accepted pair is p10 3.08
        # / p50 3.49 / p90 3.77 m, so these bounds clear the real distribution comfortably.
        min_lane_width = float(
            self.declare_parameter("lane_min_width", 2.2)
            .get_parameter_value()
            .double_value
        )
        max_lane_width = float(
            self.declare_parameter("lane_max_width", 4.5)
            .get_parameter_value()
            .double_value
        )
        # How far the tracked pair's lateral offset may move in one frame and still be
        # recognised. A per-frame delta, not an absolute: the anchor is rewritten every frame.
        incumbent_tol = float(
            self.declare_parameter("lane_incumbent_tol", 0.75)
            .get_parameter_value()
            .double_value
        )
        # This is a guard against score noise, not what holds the lane. With the ego reference
        # corrected to a ray the score is unambiguous and the selector holds one pair for all 476
        # selected frames with the margin set to *zero* and the window at [6,40]. The table below
        # was measured against the old point reference, where the two candidate clusters sat ~3 m
        # apart with the reference in the empty valley between them and anything under ~1.2 m
        # flapped; that regime no longer exists.
        #
        #   margin   switches (old reference)   switches (ego ray)
        #    0.00              -                        0
        #    0.35             26                        0
        #    0.75              -                        0
        #    1.75              2                        0
        #
        # 1.75 m is now actively harmful: it exceeds half a lane width, so a genuine lane change
        # cannot beat it. Half of it leaves margin against noise and still admits a real manoeuvre.
        # Caveat: adps_2026-08-25 contains no lane change -- its one manoeuvre is the turn onto the
        # road -- so nothing here shows the selector *following* one. That needs a bag with one.
        hysteresis_margin = float(
            self.declare_parameter("lane_hysteresis_margin", 0.75)
            .get_parameter_value()
            .double_value
        )
        # The margin guards against score noise; this guards against the candidate set itself
        # churning as lanes appear and drop, which the margin cannot see. Set to 1 to disable.
        switch_debounce = int(
            self.declare_parameter("lane_switch_debounce", 2)
            .get_parameter_value()
            .integer_value
        )
        # Hysteresis must survive a dropped detection -- three missed frames is 0.3 s and 1.5 m
        # at 5 m/s -- but not a stall. This bounds it below the watchdog's 0.5 s granularity.
        memory_timeout = float(
            self.declare_parameter("lane_memory_timeout", 0.5)
            .get_parameter_value()
            .double_value
        )
        # The ego path in the LiDAR frame, as a ray: y_ego(x) = offset + tan(yaw) * x. The pair
        # score is the centerline's mean deviation from it, so getting it wrong is what made the
        # selector jump lanes -- see lane_geometry's module docstring for the measurement.
        #
        # The intercept. Road returns sit at y p50 of about -0.6 m, which was the yaw below read
        # at the range the window samples, not a mounting offset; with the yaw applied the ego
        # lane's measured centre sits at -0.44 m p50 over the bag, i.e. the vehicle tracks about
        # 0.4 m right of centre. That is driving, not calibration, so this stays at 0.0.
        ego_y_offset = float(
            self.declare_parameter("lane_ego_y_offset", 0.0)
            .get_parameter_value()
            .double_value
        )
        # The slope, and the whole fix. lidar_tc is yawed about 5.35 deg left of the vehicle axis,
        # so y=0 is not the ego path -- it diverges from it by 9.4 cm per metre of range, reaching
        # half a lane width at the ~18 m the score window actually samples. That put the ego lane
        # and the left-adjacent lane at equal distance from the reference and made the choice
        # between them a coin flip: the jumping. /tf_static publishes lidar_tc -> base_link as
        # identity, which is the one transform in that file claiming otherwise; the radar (-5.443),
        # camera_fl's optical axis (-5.061), the lane vanishing point (-4.95) and the detected
        # lanes' own direction over 159 straight-road frames (-5.354) all agree it is not.
        #
        # Set to 0.0 once lidar_tc -> base_link carries a real calibration and /lidar_2d_projection
        # arrives in the vehicle frame; nothing else here needs to change when it does.
        ego_yaw_deg = float(
            self.declare_parameter("lane_ego_yaw_deg", DEFAULT_EGO_YAW_DEG)
            .get_parameter_value()
            .double_value
        )
        # Output shape. False keeps today's contract exactly: the selected boundaries' own
        # matched returns, and the centerline as their elementwise mean. That centerline carries
        # a measured 3.60 m median range mismatch between the points it averages, so it is wrong
        # whenever the two boundaries matched unevenly -- but it is what the out-of-repo planning
        # consumer receives today, and that consumer cannot be inspected from here. True
        # publishes the grid geometry instead: matched range, uniform spacing, smoothing that
        # actually engages. One parameter, no rebuild.
        self.centerline_from_grid = bool(
            self.declare_parameter("centerline_from_grid", False)
            .get_parameter_value()
            .bool_value
        )
        # Applied per grid node under centerline_from_grid, and to the legacy centerline
        # otherwise. The old shape gate meant this never actually ran, so 0.5 has no operating
        # history in either regime.
        ema_alpha = float(
            self.declare_parameter("centerline_ema_alpha", 0.5)
            .get_parameter_value()
            .double_value
        )

        self._projections = StampMatchedBuffer(
            "projection",
            buffer_duration=max(0.0, buffer_duration),
            max_skew=max(0.0, max_pairing_skew),
            stamp_offset=stamp_offset,
            wait_for_newer=max(0.0, wait_for_newer),
        )
        self.lane_timeout = max(0.0, self.lane_timeout)
        self.max_interp_gap = max(0.0, self.max_interp_gap)
        self.max_backtrack = max(0.0, self.max_backtrack)

        self._grid_step = max(1e-3, grid_step)
        self._grid_x = make_grid(grid_min_x, grid_max_x, self._grid_step)
        if self._grid_x.size == 0:
            # Clamping silently would leave the node running with no geometry at all, which
            # surfaces as empty clouds rather than as a configuration error.
            raise ValueError("lane_grid_max_x must exceed lane_grid_min_x")

        self._selector = LanePairSelector(
            self._grid_x,
            score_min_x=max(0.0, score_min_x),
            score_max_x=max(score_min_x + self._grid_step, score_max_x),
            min_window_nodes=max(1, min_window_nodes),
            min_lane_width=max(0.0, min_lane_width),
            max_lane_width=max(min_lane_width + 1e-3, max_lane_width),
            incumbent_tol=max(0.0, incumbent_tol),
            hysteresis_margin=max(0.0, hysteresis_margin),
            switch_debounce=max(1, switch_debounce),
            memory_timeout=max(0.0, memory_timeout),
            ego_y_offset=ego_y_offset,
            ego_yaw_deg=ego_yaw_deg,
        )
        # Clamped rather than rejected, matching how every other bound is treated at
        # construction; a runtime set is validated and refused instead.
        self.ema_alpha = min(1.0, max(1e-3, ema_alpha))
        self._left_smoother = GridSmoother(self.ema_alpha)
        self._right_smoother = GridSmoother(self.ema_alpha)
        # None until the first publish: under sim time the node clock reads 0 until /clock
        # arrives, and 0.0 here would make the first watchdog tick see a ~1.7e9s gap.
        self._last_publish = None
        self._last_unmatched_log = None
        self._last_no_pair_log = None

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
        if pump_period > 0.0:
            self._pump_timer = self.create_timer(pump_period, self._pump)
        if stats_log_period > 0.0:
            self._stats_timer = self.create_timer(stats_log_period, self._log_stats)

        # The pairing bound and the buffer depth both depend on measured pipeline
        # latency, so keep them settable at runtime for calibration against a replaying
        # bag without restarting the node.
        self.add_on_set_parameters_callback(self._on_set_parameters)

        self.get_logger().info(
            f"Clrernet_Lane_Transform ready: {LIDAR_2D_PROJ_TOPIC} + {CLRERNET_ALL_LANES_TOPIC}, "
            f"max_pairing_skew={self._projections.max_skew:.3f}s "
            f"projection_buffer_duration={self._projections.buffer_duration:.3f}s "
            f"wait_for_newer={self._projections.wait_for_newer:.3f}s "
            f"lane_timeout={self.lane_timeout:.3f}s "
            f"lane_grid=[{self._grid_x[0]:.1f}, {self._grid_x[-1]:.1f}]m "
            f"step={self._grid_step:.2f}m nodes={self._grid_x.size} "
            f"lane_score_window=[{self._selector.score_min_x:.1f}, "
            f"{self._selector.score_max_x:.1f}]m "
            f"lane_min_window_nodes={int(self._selector.min_window_nodes)} "
            f"lane_max_interp_gap={self.max_interp_gap:.2f}m "
            f"lane_max_backtrack={self.max_backtrack:.2f} "
            f"lane_width=[{self._selector.min_lane_width:.2f}, "
            f"{self._selector.max_lane_width:.2f}]m "
            f"lane_incumbent_tol={self._selector.incumbent_tol:.2f}m "
            f"lane_hysteresis_margin={self._selector.hysteresis_margin:.2f}m "
            f"lane_switch_debounce={int(self._selector.switch_debounce)} "
            f"lane_memory_timeout={self._selector.memory_timeout:.3f}s "
            f"lane_ego_ray={self._selector.ego_y_offset:.2f}m"
            f"@{self._selector.ego_yaw_deg:+.2f}deg "
            f"centerline_ema_alpha={self.ema_alpha:.2f} "
            f"centerline_from_grid={self.centerline_from_grid} "
            f"use_sim_time={self.get_parameter('use_sim_time').value} "
            f"(stamp-matched pairing, range-indexed lane geometry, ego path as a ray)"
        )

    def _now(self):
        """Seconds on the node clock -- sim time when use_sim_time is set, else wall time."""
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_set_parameters(self, params) -> SetParametersResult:
        # Everything apply_bounded_parameters cannot express is checked first. It applies as it
        # goes and offers no dry run, so a bound it does not know about has to reject the request
        # before it runs, or a bad set leaves the node half-updated.
        prospective = {p.name: p.value for p in params}

        def proposed(name, current):
            return float(prospective.get(name, current))

        alpha = proposed("centerline_ema_alpha", self.ema_alpha)
        if not 0.0 < alpha <= 1.0:
            return SetParametersResult(
                successful=False, reason="centerline_ema_alpha must be in (0, 1]"
            )
        score_min = proposed("lane_score_min_x", self._selector.score_min_x)
        score_max = proposed("lane_score_max_x", self._selector.score_max_x)
        if score_max <= score_min:
            return SetParametersResult(
                successful=False, reason="lane_score_max_x must exceed lane_score_min_x"
            )
        min_width = proposed("lane_min_width", self._selector.min_lane_width)
        max_width = proposed("lane_max_width", self._selector.max_lane_width)
        if max_width <= min_width:
            return SetParametersResult(
                successful=False, reason="lane_max_width must exceed lane_min_width"
            )
        for name in ("lane_min_window_nodes", "lane_switch_debounce"):
            if name in prospective and int(prospective[name]) < 1:
                return SetParametersResult(successful=False, reason=f"{name} must be >= 1")

        targets = {
            "max_pairing_skew": (self._projections, "max_skew"),
            "projection_buffer_duration": (self._projections, "buffer_duration"),
            "lane_timeout": (self, "lane_timeout"),
            "wait_for_newer": (self._projections, "wait_for_newer"),
            "lane_score_min_x": (self._selector, "score_min_x"),
            "lane_score_max_x": (self._selector, "score_max_x"),
            "lane_max_interp_gap": (self, "max_interp_gap"),
            "lane_max_backtrack": (self, "max_backtrack"),
            "lane_min_width": (self._selector, "min_lane_width"),
            "lane_max_width": (self._selector, "max_lane_width"),
            "lane_incumbent_tol": (self._selector, "incumbent_tol"),
            "lane_hysteresis_margin": (self._selector, "hysteresis_margin"),
            "lane_memory_timeout": (self._selector, "memory_timeout"),
        }
        ok, reason, applied = apply_bounded_parameters(params, targets)
        if not ok:
            return SetParametersResult(successful=False, reason=reason)
        # No unit suffix: half of these are metres and half are seconds.
        for name, value in applied:
            self.get_logger().info(f"{name} set to {value:.3f}")

        for p in params:
            # Unbounded: a negative offset is the expected direction for an end-of-sweep stamp.
            if p.name == "projection_stamp_offset":
                self._projections.stamp_offset = float(p.value)
                self.get_logger().info(
                    f"projection_stamp_offset set to {float(p.value):.3f}s"
                )
            # Unbounded: the ego path may sit either side of the LiDAR y axis.
            elif p.name == "lane_ego_y_offset":
                self._selector.ego_y_offset = float(p.value)
                self.get_logger().info(
                    f"lane_ego_y_offset set to {self._selector.ego_y_offset:.3f}m"
                )
            # Unbounded for the same reason, and in degrees rather than metres.
            elif p.name == "lane_ego_yaw_deg":
                self._selector.ego_yaw_deg = float(p.value)
                self.get_logger().info(
                    f"lane_ego_yaw_deg set to {self._selector.ego_yaw_deg:.3f}deg"
                )
            elif p.name == "centerline_ema_alpha":
                self.ema_alpha = alpha
                self._left_smoother.alpha = alpha
                self._right_smoother.alpha = alpha
                self.get_logger().info(f"centerline_ema_alpha set to {alpha:.3f}")
            elif p.name == "lane_min_window_nodes":
                self._selector.min_window_nodes = int(p.value)
                self.get_logger().info(f"lane_min_window_nodes set to {int(p.value)}")
            elif p.name == "lane_switch_debounce":
                self._selector.switch_debounce = int(p.value)
                self.get_logger().info(f"lane_switch_debounce set to {int(p.value)}")
            elif p.name == "centerline_from_grid":
                self.centerline_from_grid = bool(p.value)
                # The two output shapes are not interchangeable frame to frame, so drop the
                # smoothing history rather than blending across the switch.
                self._reset_geometry_state()
                self.get_logger().info(
                    f"centerline_from_grid set to {self.centerline_from_grid}"
                )
        return SetParametersResult(successful=True)

    def _on_projection(self, msg_proj):
        """Buffer the projection, then release any detection that was waiting for it."""
        self._projections.add(msg_proj)
        # This is what resolves nearly every deferral, in the same tick the awaited cloud
        # lands; the pump timer only matters when this stream stalls.
        self._pump()

    def _on_lanes(self, msg_all_lanes):
        pairing = self._projections.match(
            msg_all_lanes.header, now=self._now(), payload=msg_all_lanes
        )
        if pairing.outcome is not DEFERRED:
            self._complete(pairing)

    def _pump(self):
        for pairing in self._projections.drain(self._now()):
            self._complete(pairing)

    def _complete(self, pairing):
        if pairing.value is None:
            self._log_unmatched(pairing.skew, pairing.reason)
            return
        self.lanes_callback(pairing.value, pairing.payload)

    def _log_unmatched(self, skew, reason=None):
        """Rate-limited warning so a starved or misaligned pipeline stays visible."""
        now = self._now()
        if self._last_unmatched_log is not None and now - self._last_unmatched_log < 1.0:
            return
        self._last_unmatched_log = now
        self.get_logger().warning(
            f"Unmatched lane detection: "
            f"{self._projections.describe_unmatched(skew, 'lane detection', reason=reason)}; "
            f"{self._projections.status()}"
        )

    def _log_no_pair(self, n_lanes):
        """Rate-limited on the same 1s budget: a scene with no plausible pair is common."""
        now = self._now()
        if self._last_no_pair_log is not None and now - self._last_no_pair_log < 1.0:
            return
        self._last_no_pair_log = now
        self.get_logger().warning(
            f"No plausible lane pair from {n_lanes} detection(s); {self._selector.status()}"
        )

    def _log_stats(self):
        """Periodic pairing health, so a skew regression is visible without instrumentation."""
        self.get_logger().info(f"pairing: {self._projections.status()}")
        self.get_logger().info(f"lanes: {self._selector.status()}")

    @timer
    def lanes_callback(self, entry, msg_all_lanes):
        # One parse and one KDTree, shared by every lane in the frame.
        tree, pc_arr = entry.pixel_tree()
        if tree is None:
            self.get_logger().warning("No points in projected PointCloud2")
            self._publish_empty(entry.header)
            return

        lane_dict = {}
        for pt in msg_all_lanes.points:
            lane_dict.setdefault(pt.lane_id, []).append([pt.x, pt.y])

        # lane_id is enumerate() output and carries no identity across frames, so this sort only
        # makes the iteration deterministic. Nothing downstream reads the order.
        lanes = [np.asarray(pts, dtype=np.float64) for _, pts in sorted(lane_dict.items())]
        if not lanes:
            self.get_logger().warning("No lane points detected!")
            self._publish_empty(entry.header)
            return

        left_pts, right_pts, center_pts = self.get_closest_lane_pair_3d(
            lanes, tree, pc_arr
        )
        if center_pts.shape[0] == 0:
            self._log_no_pair(len(lanes))
            self._reset_geometry_state()

        self._publish_points(left_pts, self.left_pcl_pub, entry.header)
        self._publish_points(right_pts, self.right_pcl_pub, entry.header)
        self.publish_centerline(center_pts, entry.header)
        self._last_publish = self._now()

    def get_closest_lane_pair_3d(self, lanes, tree, pc_arr):
        """Lift every lane onto the range grid, pick the ego pair, return its three clouds.

        Returns ``(left, right, centre)`` as ``Nx3`` float32 arrays, all empty when no plausible
        pair exists -- the caller publishes that as an empty cloud rather than withholding it,
        because an absent lane must read as absent downstream rather than as the last one that
        happened to work.

        Under ``centerline_from_grid`` all three come off the grid: matched range, uniform
        spacing, smoothed per node. Otherwise the selected boundaries' own matched returns are
        published unchanged and the centerline is their elementwise mean, which is what today's
        consumer receives -- and which carries a measured 3.60 m median range mismatch between
        the points it averages. Only the *selection* is fixed in that mode.
        """
        empty = np.empty((0, 3), dtype=np.float32)
        resampled = []
        matched_by_lane = []
        for lane_uv in lanes:
            # One vectorised query per lane, and the only one: nothing downstream re-derives the
            # matched set from pixels a second time.
            dist, idx = tree.query(lane_uv)
            matched = pc_arr[idx[dist < PIXEL_LIM]]
            lane = resample_lane(
                matched,
                self._grid_x,
                step=self._grid_step,
                max_interp_gap=self.max_interp_gap,
                max_backtrack=self.max_backtrack,
                min_samples=MIN_LANE_SAMPLES,
            )
            if lane.valid.any():
                resampled.append(lane)
                matched_by_lane.append(matched)

        if len(resampled) < 2:
            return empty, empty, empty

        pair, switched = self._selector.select(resampled, now=self._now())
        if pair is None:
            return empty, empty, empty

        if switched:
            # The smoothing history describes a different pair of lanes. Blending across the
            # switch ramps the output over a ~3.5 m step and reads downstream as a real lateral
            # velocity rather than as the discontinuity it is; a step is at least detectable.
            self._reset_geometry_state()
            self.get_logger().info(
                f"lane pair switched: ego offset={pair.offset:+.2f}m "
                f"width={pair.width:.2f}m tier={pair.tier} nodes={pair.nodes}"
            )

        if not self.centerline_from_grid:
            # Identity, not equality: Lane defines no __eq__ and two lanes can resample alike.
            by_id = {id(lane): raw for lane, raw in zip(resampled, matched_by_lane)}
            left_raw = by_id[id(pair.left)]
            right_raw = by_id[id(pair.right)]
            n = min(len(left_raw), len(right_raw))
            centre = (
                ((left_raw[:n] + right_raw[:n]) / 2.0).astype(np.float32)
                if n
                else empty
            )
            return (
                np.asarray(left_raw, dtype=np.float32),
                np.asarray(right_raw, dtype=np.float32),
                centre,
            )

        left_y, left_z = self._left_smoother.update(
            pair.left.y, pair.left.z, pair.left.valid
        )
        right_y, right_z = self._right_smoother.update(
            pair.right.y, pair.right.z, pair.right.valid
        )
        # Derived from the smoothed boundaries rather than smoothed on its own, so the published
        # centre is the mean of the published left and right at every node all three exist at.
        # Smoothing the centerline independently does not guarantee that.
        return (
            sample_points(self._grid_x, left_y, left_z, pair.left.valid),
            sample_points(self._grid_x, right_y, right_z, pair.right.valid),
            sample_points(
                self._grid_x, 0.5 * (left_y + right_y), 0.5 * (left_z + right_z), pair.valid
            ),
        )

    def publish_centerline(self, center_pts, header):
        # Under centerline_from_grid the smoothing already happened per grid node, where the
        # correspondence between frames is explicit and the identity of the selected pair is in
        # scope. Neither is recoverable from a bare point array, which is why the shape test
        # below is the only gate available in the legacy path -- and why it no-ops on exactly the
        # frames that need it, the measured left/right counts differing by a median of 7.
        if not self.centerline_from_grid and center_pts.shape[0] > 0:
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

    def _reset_geometry_state(self):
        """Drop smoothing history. The selector's anchor has its own timeout and survives."""
        self.prev_centerline = None
        self._left_smoother.reset()
        self._right_smoother.reset()

    def _publish_empty(self, header):
        # Any frame that publishes nothing is a gap the smoothers must not blend across. This
        # used to live only in the watchdog, so a single zero-lane frame left the history
        # standing and the next good frame blended over the hole.
        self._reset_geometry_state()
        empty = np.empty((0, 3), dtype=np.float32)
        for pub in (self.left_pcl_pub, self.right_pcl_pub, self.centerline_pub):
            self._publish_points(empty, pub, header)
        self._last_publish = self._now()

    def _watchdog(self):
        """Publish empty lanes while the detector is not refreshing them."""
        if self.lane_timeout <= 0.0:
            return
        newest = self._projections.newest()
        if newest is None or self._last_publish is None:
            return
        if self._now() - self._last_publish > self.lane_timeout:
            # lane_memory_timeout already expires the anchor below this timer's 0.5s
            # granularity; this is the belt-and-braces path for a clock that never advances.
            self._selector.reset()
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
