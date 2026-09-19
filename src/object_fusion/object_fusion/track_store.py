"""Track lifecycle: birth, confirmation, coasting, deletion, merging, existence.

ROS-free. This is the half of the tracker that will consume more tuning time than the
estimator, and it is separated from :mod:`object_fusion.tracker` for exactly that reason --
it needs its own A/B and its own tests.

THE PROBLEM THIS IS SHAPED AROUND. With ``publish_radar_only`` on, 1202 of 1475 objects were
radar-originated on the 2026-08-25 bag: guardrails, manhole covers and overhead signs all
report as ESR tracks. Be precise about what that number is, though -- it is a ratio of
CANDIDATE COUNTS, not a measured false-alarm rate. Nothing in it says how many of the 1202
were real. Turning it into a rate needs labels, and there is a cheap source: in the region
where camera and radar overlap, YOLO+ByteTrack is a decent proxy, so count radar-only
candidates there that never acquire a camera detection over their whole life. Extrapolating
outside the overlap is a leap and should be labelled as one.

M-of-N alone will not fix that clutter, because those returns are PERSISTENT. Neither will
amplitude (-10 is the sensor floor and ~30% of real tracks sit on it), nor update_count
(inert on this driver), nor elevation (the ESR has none).

**The one discriminator that works is stationarity**, and this architecture is the first here
to have it: every member of the dominant clutter class is stationary over the ground, so with
ego velocity known the compensated range rate separates them.

AND THE THRESHOLD IS DERIVED, NOT INVENTED. ``radar_ab.py`` measured the static-target
range-rate residual at median -0.06 m/s, p90 |residual| 0.45 m/s over 16219 observations.
Three times that p90 gives :data:`RADAR_ONLY_MIN_SPEED` = 1.5 m/s.

Two blind spots, which belong in the launch help rather than being quietly patched around:

* A STOPPED or parked vehicle reads as stationary and is suppressed by this gate. Acceptable
  only because radar-only birth is never the sole path to one -- the camera covers exactly
  the region where a stopped vehicle matters. Do not relax this gate to "fix" stopped cars;
  fix them on the camera path.
* A purely CROSSING target has compensated range rate ~0 too. Within the ESR's +/-45 deg
  forward field this is geometrically uncommon and mostly close in. Accept it and log it.
"""

from __future__ import annotations

import math

import numpy as np

__all__ = [
    "TENTATIVE", "CONFIRMED", "COASTING", "DELETED",
    "SENSOR_CAMERA", "SENSOR_LIDAR", "SENSOR_RADAR",
    "RADAR_ONLY_MIN_SPEED", "LOGODDS_HIT", "LOGODDS_MISS",
    "Track", "TrackStore", "radar_expected", "camera_expected", "should_merge",
]

TENTATIVE, CONFIRMED, COASTING, DELETED = 0, 1, 2, 3
SENSOR_CAMERA, SENSOR_LIDAR, SENSOR_RADAR = 1, 2, 4

#: 3x the measured 0.45 m/s p90 range-rate residual. DERIVED.
RADAR_ONLY_MIN_SPEED = 1.5

# Every increment below is INVENTED and measurable from shadow-mode counts once radar-only
# births are labelled against the camera in the overlap region. Note how little a STATIONARY
# radar hit is worth against a camera+LiDAR one -- that ratio is the direct encoding of the
# 81.5% observation.
LOGODDS_HIT = {"camera_lidar": 1.20, "lidar": 0.40, "radar_moving": 0.60, "radar_static": 0.05}
LOGODDS_MISS = {"camera_lidar": -0.80, "lidar": -0.20, "radar": -0.05}
LOGODDS_ID_LOST = -1.50
LOGODDS_DECAY_PER_S = -0.20
LOGODDS_MIN, LOGODDS_MAX = -6.0, 6.0

CONFIRM_LOGODDS = 1.5
RADAR_ONLY_CONFIRM_LOGODDS = 3.0
P_PUBLISH_RADAR_ONLY = 0.7
P_DELETE = 0.2

#: Coast budgets by evidence. MEASURED 2026-09-18, and the method this comment used to prescribe
#: ("take the 95th percentile of re-acquisition gaps") is WRONG. Those gaps are real -- p95 0.7-1.7 s
#: for the camera across four drives, and 13-28% of re-acquisitions arrive after 0.5 s
#: (scripts/coast_budget_ab.py) -- but covering them makes things worse: at 1.5 s orphaned
#: measurements rise 10.6% -> 13.3% and the held-out range error 1.07 -> 1.58 m, because a stale
#: coasting track drifts far enough to lose the detection it was kept alive for
#: (scripts/coast_sweep_ab.py). 1.0 s is within noise of 0.5 s on every metric, so 0.5 s stays.
#: "radar" and "both" were not swept separately beyond the 1.5/2.0 s arms.
MAX_COAST_S = {"both": 1.0, "camera": 0.5, "radar": 0.3}

#: A diverged track must die, not linger with a confident-looking pose. INVENTED.
MAX_POSITION_TRACE = 400.0


def radar_expected(p_s, *, fov_deg=45.0, min_range=1.0, max_range=175.0) -> bool:
    """Whether the radar should have seen a target at ``p_s`` (radar-frame xy)."""
    p = np.asarray(p_s, dtype=np.float64).reshape(2)
    rho = float(math.hypot(p[0], p[1]))
    if not (min_range <= rho <= max_range):
        return False
    return abs(math.degrees(math.atan2(p[1], p[0]))) <= float(fov_deg)


def camera_expected(p_l, uv=None, image_wh=None, *, max_x=150.0, max_y=20.0,
                    z_range=(-3.5, 1.0), z=0.0) -> bool:
    """Whether the camera+LiDAR path could have produced an object at ``p_l`` (lidar-frame).

    This is ``transform.py``'s crop AND the front-left image. An object past x > 150 m or
    |y| > 20 m produces no projected points at all, so ``fusion_node`` cannot emit it and a
    miss there is not evidence of anything -- it is evidence the object left the observable
    volume, which is a DELETION reason, not a decay reason. Decaying it instead leaves it
    lingering at the crop edge with a confident state and no supporting measurement.

    Note the crop is narrower than the image past ~67 m: +/-20 m at 100 m is +/-11.3 deg
    against the camera's 16.6 deg half-FOV.
    """
    p = np.asarray(p_l, dtype=np.float64).reshape(2)
    if not (0.0 <= p[0] <= float(max_x)) or abs(p[1]) > float(max_y):
        return False
    if not (z_range[0] <= float(z) <= z_range[1]):
        return False
    if uv is not None and image_wh is not None:
        u, v = float(uv[0]), float(uv[1])
        w, h = float(image_wh[0]), float(image_wh[1])
        if not (0.0 <= u < w and 0.0 <= v < h):
            return False
    return True


def should_merge(a, b, *, chi2=9.21, max_merge_dist=2.5, max_range_gap=20.0,
                 max_bearing_deg=1.5, split_branch_only=True,
                 min_range_for_gap=60.0) -> bool:
    """Whether two tracks describe one object.

    Merging is REQUIRED, not a nicety. At 80 m the camera branch's position can be 14 m short
    while the radar branch is correct -- far outside any sane association gate -- so the two
    branches WILL each birth a track for the same car. Two boxes for one vehicle, one of them
    14 m early, is worse for a planner than one box 14 m early, because it adds a phantom at
    the true position.

    Hence the second clause, which is deliberately generous in RANGE specifically: same
    bearing, very different range, is the signature of exactly that split.

    AND IT MUST ONLY FIRE ON THAT SPLIT (``split_branch_only``). A line of traffic cones along
    the road edge has the same geometry -- consecutive cones sit within a degree of each other in
    bearing and a few metres apart in range -- so the clause merged them into one track and the
    boxes vanished. Measured on the reference drive: 30.6% of camera measurements ended with no
    published track within 3 m, against 7.8% with merging off; live the same rule lost 29.7%,
    worst at 0-25 m (-31%). The split this exists for is BETWEEN BRANCHES: one track carrying
    camera evidence and one carrying only radar. Two camera-backed tracks are two objects the
    camera actually saw separately, and the Mahalanobis clause above is the only thing entitled
    to merge those.

    It is also bounded in RANGE (``min_range_for_gap``). The split it repairs is a far-field
    effect -- the camera+LiDAR range is biased -7.6 m at 80-100 m and -32 m beyond, but only
    -1.1 m under 80 m -- so a 20 m merge close in cannot be that split, and near the car it is
    a cone being swallowed by a radar return from the guardrail behind it.

    The Mahalanobis clause carries a HARD DISTANCE BOUND (``max_merge_dist``) as well as the
    chi-square one, because on its own it does not bound a distance at all: it bounds a distance
    in units of the covariance, and a coasting track's covariance grows without limit. At 2 m
    sigma the 99% two-DOF value spans 8.6 m -- past the next cone in a line -- so the clause
    that exists to collapse a track onto its own duplicate reached across to the neighbour
    instead. The two objects this has to tell apart are a duplicate (two tracks on one object,
    typically under a metre apart, never more than about two) and the next cone up the line
    (5 m and more on this drive), so 2.5 m separates them with room on both sides.

    Tightening the chi-square instead was tried and is worse. At chi2 4.0 the clause stops
    reaching genuine duplicates too: the node logged ZERO merges over a whole replay, and two
    published boxes on one measurement went 1.3% -> 5.6%. The distance bound keeps the 99%
    chi-square, so duplicates are still merged, and refuses only reaches longer than 2.5 m.

    Measured over a full replay loop of selfcal_loc_2026-09-08, live, `scripts/live_orphans.py`:

        arm                          orphaned   two boxes on one measurement
        unbounded (as first shipped)   15.7%              1.3%
        bounded at 2.5 m               10.5%              3.0%
    """
    xa = np.asarray(a.x, dtype=np.float64)
    xb = np.asarray(b.x, dtype=np.float64)
    d = xa - xb
    S = np.asarray(a.P, dtype=np.float64) + np.asarray(b.P, dtype=np.float64)
    try:
        if (float(np.hypot(d[0], d[1])) <= float(max_merge_dist)
                and float(d @ np.linalg.solve(S, d)) <= chi2):
            return True
    except np.linalg.LinAlgError:
        pass
    if split_branch_only:
        cam = SENSOR_CAMERA | SENSOR_LIDAR
        a_cam = bool(getattr(a, "sensors_ever", 0) & cam)
        b_cam = bool(getattr(b, "sensors_ever", 0) & cam)
        if a_cam and b_cam:
            return False
    ra, rb = float(np.linalg.norm(xa[:2])), float(np.linalg.norm(xb[:2]))
    if max(ra, rb) < float(min_range_for_gap):
        return False
    ba = math.degrees(math.atan2(xa[1], xa[0]))
    bb = math.degrees(math.atan2(xb[1], xb[0]))
    return abs(ra - rb) <= max_range_gap and abs(ba - bb) <= max_bearing_deg


class Track:
    """One tracked object: kinematic state, provenance, and the evidence behind it."""

    _next_id = 1

    __slots__ = ("id", "x", "P", "status", "logodds", "born", "last_update",
                 "sensors_ever", "sensors_this_cycle", "hits", "opportunities",
                 "moving_hits", "radar_hits", "bytetrack_id", "bytetrack_seen",
                 "radar_slot", "class_votes", "size", "yaw", "yaw_source",
                 "last_radar_update", "last_camera_update", "last_existence_t",
                 "camera_rejected", "extent_l", "extent_w", "last_cam_xy",
                 "size_measured", "consecutive_rejects", "forced_updates")

    def __init__(self, x, P, stamp, sensor_mask=0):
        self.id = Track._next_id
        Track._next_id += 1
        self.x = np.asarray(x, dtype=np.float64).reshape(4).copy()
        self.P = np.asarray(P, dtype=np.float64).reshape(4, 4).copy()
        self.status = TENTATIVE
        self.logodds = 0.0
        self.born = float(stamp)
        self.last_update = float(stamp)
        self.sensors_ever = int(sensor_mask)
        self.sensors_this_cycle = 0
        self.hits = {"camera_lidar": 0, "lidar": 0, "radar": 0}
        self.opportunities = {"camera_lidar": 0, "lidar": 0, "radar": 0}
        self.moving_hits = 0
        self.radar_hits = 0
        self.bytetrack_id = ""
        self.bytetrack_seen = float(stamp)
        self.radar_slot = None
        self.class_votes = {}
        self.size = None
        #: Whether `size` came from an observation or from a class prior. Reported verbatim on
        #: the wire, so it must track provenance and not merely whether the field is set.
        self.size_measured = False
        self.yaw = 0.0
        #: Defaults to RAY_DEFAULT, not 0. Enum value 0 is SHAPE_FIT, so a `None or 0` default
        #: claimed every unfitted track had a measured heading.
        self.yaw_source = None
        self.last_radar_update = None
        self.last_camera_update = None
        #: When existence was last accrued. The decay term is per unit TIME, so it must be
        #: applied once per elapsed interval, not once per sensor callback -- radar at 30 Hz
        #: and camera at 10 Hz would otherwise decay the same track at four times the rate.
        self.last_existence_t = None
        #: Camera updates rejected by the innovation gate over this track's life.
        self.camera_rejected = 0
        #: Consecutive gate rejections. Drives the forced-update escape that stops a drifted
        #: track from rejecting its own measurements forever.
        self.consecutive_rejects = 0
        self.forced_updates = 0
        #: One-sided running extent, length and width. Extent from a partial view is only ever
        #: an UNDER-estimate, so a mean is biased low and never recovers.
        self.extent_l = None
        self.extent_w = None
        #: Last raw camera+LiDAR measurement position, unfiltered. This is what passthrough
        #: mode publishes, so the filter genuinely cannot reach the output while it is set.
        self.last_cam_xy = None

    # -- derived ---------------------------------------------------------------
    @property
    def position(self):
        return self.x[:2]

    @property
    def velocity(self):
        return self.x[2:]

    @property
    def speed(self):
        return float(math.hypot(self.x[2], self.x[3]))

    @property
    def existence(self) -> float:
        return 1.0 / (1.0 + math.exp(-self.logodds))

    @property
    def moving_fraction(self) -> float:
        return self.moving_hits / self.radar_hits if self.radar_hits else 0.0

    def age(self, now) -> float:
        return float(now) - self.born

    def published_status(self) -> int:
        """The status to put on the wire: 0 tentative, 1 confirmed, 2 coasting.

        The numbers are FusedObject's STATUS_* constants, which match this module's.

        COASTING must go out as coasting. It used to be folded into TENTATIVE, and that mislabelled
        almost every long-lived cone: `may_confirm` only re-promotes a camera-only track while
        it has had at most 5 camera opportunities, so an old camera-only track that misses one
        cycle stays COASTING for the rest of its life. A consumer that drops tentative tracks --
        the planner bridge does exactly that -- was dropping them.
        """
        return COASTING if self.status == COASTING else (
            CONFIRMED if self.status == CONFIRMED else TENTATIVE)

    def missed_updates(self, now, period=0.1) -> int:
        """Camera periods since any sensor last updated this track, clipped to a uint8.

        The aggregator runs per measurement, not per cycle, so a "cycle" is taken as the 10 Hz
        camera period -- the slowest stream, and the one a missed object shows up in.
        """
        return int(min(255, max(0, math.floor((float(now) - self.last_update) / period + 1e-9))))

    def class_name(self):
        """Majority vote over the track's history, not the latest frame.

        A YOLO class flip (car <-> truck) changes the extent prior and steps the centroid
        correction by ~1 m. Voting stops a single bad frame from moving the published box.
        """
        if not self.class_votes:
            return ""
        return max(self.class_votes.items(), key=lambda kv: kv[1])[0]

    def vote_class(self, name):
        if name:
            self.class_votes[name] = self.class_votes.get(name, 0) + 1


class TrackStore:
    """The collection, and the lifecycle rules over it."""

    def __init__(self, *, confirm_logodds=CONFIRM_LOGODDS,
                 radar_only_confirm_logodds=RADAR_ONLY_CONFIRM_LOGODDS,
                 p_delete=P_DELETE, radar_only_min_speed=RADAR_ONLY_MIN_SPEED,
                 enable_radar_only_birth=False):
        self.tracks: list[Track] = []
        self.confirm_logodds = float(confirm_logodds)
        self.radar_only_confirm_logodds = float(radar_only_confirm_logodds)
        self.p_delete = float(p_delete)
        self.radar_only_min_speed = float(radar_only_min_speed)
        #: Default OFF, like every other new capability here. Shadow counters still accrue.
        self.enable_radar_only_birth = bool(enable_radar_only_birth)
        self.shadow = {"radar_only_candidates": 0, "radar_only_born": 0,
                       "merged": 0, "deleted_left_volume": 0, "deleted_existence": 0,
                       "deleted_diverged": 0, "confirmed": 0}

    # -- evidence --------------------------------------------------------------
    def update_existence(self, tr: Track, sensor: str, hit: bool, expected: bool, dt: float):
        """Accrue or decay existence. A MISS counts only where the sensor could have seen it.

        That predicate is where existence logic usually breaks silently: charging a track for
        a radar miss while it sits outside the radar's field of view kills real objects for
        being invisible.
        """
        tr.logodds += LOGODDS_DECAY_PER_S * float(dt)
        if hit:
            tr.logodds += LOGODDS_HIT.get(sensor, 0.0)
        elif expected:
            tr.logodds += LOGODDS_MISS.get(sensor.split("_")[0] if sensor.startswith("radar")
                                           else sensor, 0.0)
        tr.logodds = float(np.clip(tr.logodds, LOGODDS_MIN, LOGODDS_MAX))
        return tr.existence

    def may_confirm(self, tr: Track) -> bool:
        """Confirmation: fast when corroborated, slow and conditional when radar-only."""
        if tr.hits["camera_lidar"] >= 3 and tr.opportunities["camera_lidar"] <= 5:
            return True
        both = SENSOR_CAMERA | SENSOR_RADAR
        if (tr.sensors_ever & both) == both:
            return tr.logodds >= self.confirm_logodds
        if tr.sensors_ever == SENSOR_RADAR:
            return (tr.hits["radar"] >= 8
                    and tr.opportunities["radar"] <= 12
                    and tr.moving_fraction >= 0.8
                    and tr.radar_slot is not None
                    and tr.logodds >= self.radar_only_confirm_logodds)
        return False

    def may_publish(self, tr: Track) -> bool:
        """Camera-corroborated tracks publish from TENTATIVE; radar-only must earn it.

        Existence rides on the message either way, so a consumer can pick its own threshold --
        this only decides what is worth putting on the wire at all.
        """
        if tr.sensors_ever & (SENSOR_CAMERA | SENSOR_LIDAR):
            return True
        return tr.status == CONFIRMED and tr.existence >= P_PUBLISH_RADAR_ONLY

    # -- lifecycle -------------------------------------------------------------
    def coast_budget(self, tr: Track) -> float:
        has_cam = bool(tr.sensors_ever & (SENSOR_CAMERA | SENSOR_LIDAR))
        has_rad = bool(tr.sensors_ever & SENSOR_RADAR)
        if has_cam and has_rad:
            return MAX_COAST_S["both"]
        return MAX_COAST_S["camera"] if has_cam else MAX_COAST_S["radar"]

    def prune(self, now, in_volume=None):
        """Apply deletion rules. ``in_volume(track) -> bool`` reports observability."""
        kept = []
        for tr in self.tracks:
            trace = float(tr.P[0, 0] + tr.P[1, 1])
            if in_volume is not None and not in_volume(tr):
                self.shadow["deleted_left_volume"] += 1
                continue
            if trace > MAX_POSITION_TRACE:
                self.shadow["deleted_diverged"] += 1
                continue
            if tr.existence < self.p_delete:
                self.shadow["deleted_existence"] += 1
                continue
            if float(now) - tr.last_update > self.coast_budget(tr):
                self.shadow["deleted_existence"] += 1
                continue
            if float(now) - tr.last_update > 1e-9:
                tr.status = COASTING if tr.status == CONFIRMED else tr.status
            kept.append(tr)
        self.tracks = kept

    def merge_pass(self, *, chi2=9.21, max_merge_dist=2.5, max_range_gap=20.0,
                   max_bearing_deg=1.5, min_range_for_gap=60.0):
        """Collapse tracks that describe one object, keeping the better-supported one."""
        out: list[Track] = []
        for tr in sorted(self.tracks, key=lambda t: -t.logodds):
            dup = next((k for k in out
                        if should_merge(k, tr, chi2=chi2, max_merge_dist=max_merge_dist,
                                        max_range_gap=max_range_gap,
                                        max_bearing_deg=max_bearing_deg,
                                        min_range_for_gap=min_range_for_gap)), None)
            if dup is None:
                out.append(tr)
                continue
            dup.sensors_ever |= tr.sensors_ever
            for k in dup.hits:
                dup.hits[k] += tr.hits[k]
            dup.logodds = float(np.clip(max(dup.logodds, tr.logodds), LOGODDS_MIN, LOGODDS_MAX))
            dup.born = min(dup.born, tr.born)
            self.shadow["merged"] += 1
        self.tracks = out

    def promote(self):
        for tr in self.tracks:
            if tr.status in (TENTATIVE, COASTING) and self.may_confirm(tr):
                if tr.status == TENTATIVE:
                    self.shadow["confirmed"] += 1
                tr.status = CONFIRMED

    # -- birth -----------------------------------------------------------------
    def may_birth_radar_only(self, compensated_speed) -> bool:
        """Gate on ground-referenced motion. See the module docstring for why.

        Counted as a candidate whether or not the gate is open, so the evidence to open it
        accrues while radar still cannot create an object.
        """
        self.shadow["radar_only_candidates"] += 1
        if not self.enable_radar_only_birth:
            return False
        return abs(float(compensated_speed)) >= self.radar_only_min_speed

    def add(self, tr: Track):
        self.tracks.append(tr)
        return tr
