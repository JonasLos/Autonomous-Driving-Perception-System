"""Lane-pair selection and centerline geometry on a fixed longitudinal grid.

Three measured defects in the original pairing motivated this module.

*Index pairing.* The centerline was the elementwise mean of the two boundaries' matched LiDAR
returns after truncation to a common length. The pixel-radius match drops a different subset from
each boundary, so index *i* on the left and index *i* on the right sit at different ranges and
their mean is not between the lanes. Measured over 479 frames of a 2026-08-20 replay, the median
``|x_left[i] - x_right[i]|`` was 3.60 m, the p90 8.00 m and the in-frame worst case 12.26 m.
Everything here is indexed by *range* instead: both boundaries are lifted onto one shared grid of
x values, and the centerline exists only at nodes where both were measured.

*Whole-polyline mean y.* Sorting lanes and assigning left/right by the mean y over an entire
polyline uses a lever arm with an RMS of 39.1 m against the measured range distribution, where the
curvature term ``y ~ x**2 / 2R`` reaches 3.82 m at R=200 m -- larger than the measured 3.49 m lane
width, so the sort inverts. It inverts whenever two lanes have unequal x spans, which is the
normal case: a lead vehicle occludes one boundary's far field and not the other's. So there is no
global sort here. Every pair of lanes is evaluated, a width gate rejects the ones that span more
than a lane, and left/right is decided by comparing the two boundaries at the *same* x, where the
curvature term is common to both and cancels exactly.

*The ego reference was a point, not a ray.* The score is the pair's lateral offset from the ego
path, and the ego path was taken to be the line ``y = 0`` in ``lidar_tc``. It is not: the LiDAR is
yawed about 5.35 degrees left of the vehicle axis, so the ego path is ``y = -0.0937 * x`` and the
old reference diverged from the vehicle by 9.4 cm per metre of range. At the range the score window
actually samples -- around 18 m, because the near field is usually empty -- that is 1.75 m, half a
lane width, which put the ego lane and the left-adjacent lane at equal distance from the reference
and made the argmin between them a coin flip. Measured four ways that do not read ``/tf_static``
or agree with it: the detected lanes' own direction over 159 straight-road frames is -5.354 deg
(IQR -5.68..-5.11); the lane vanishing point sits 6.6 px from the principal point, so the camera
axis is parallel to the road to 0.11 deg, and ``T1`` puts that axis at -5.061 deg in lidar frame;
the bumper radar's +x is at -5.443 deg and ``camera_fl``'s optical axis at -5.061 deg, while
``lidar_tc -> base_link`` is published as identity -- which is what a transform looks like when
nobody calibrated it. So :class:`LanePairSelector` scores against a *ray*, ``ego_y_offset +
tan(ego_yaw_deg) * x``. Correcting it takes the frame-to-frame jump rate to zero with hysteresis
switched off entirely; the hysteresis below was never the fix, it was damping this.

*No hysteresis.* A per-frame argmin over an unstable candidate set switches lanes on detector
noise, and ``lane_id`` is ``enumerate()`` output rather than a tracked identity, so nothing carries
across frames on its own. :class:`LanePairSelector` remembers the accepted pair by its signed
lateral offset -- geometry, not identity -- and makes a challenger beat a margin for consecutive
frames before it takes over.

Free of ``rclpy`` and of every ROS message type: numpy arrays and an opaque float clock, so these
rules can be driven from a bag-replay harness or from pytest without a spinning node. That is the
same split that keeps ``stamp_sync`` testable.
"""

import numpy as np

__all__ = [
    "DEFAULT_EGO_YAW_DEG",
    "TIER_CONTAINED",
    "TIER_SCORED",
    "TIER_NEAR_EMPTY",
    "GridSmoother",
    "Lane",
    "LanePair",
    "LanePairSelector",
    "backtrack_ratio",
    "condense_lane",
    "make_grid",
    "resample_lane",
    "sample_points",
]

# Azimuth of the ego path in the LiDAR frame, in degrees. Not a tuning knob: it is the yaw of
# lidar_tc against the vehicle axis, measured at -5.354 deg from the detected lanes themselves over
# 159 straight-road frames and corroborated by the bumper radar (-5.443), camera_fl's optical axis
# (-5.061) and the lane vanishing point (-4.95). Set it to 0.0 once lidar_tc -> base_link carries a
# real calibration instead of identity and the points arrive already in the vehicle frame.
DEFAULT_EGO_YAW_DEG = -5.35

# Ranked ahead of the score, so a pair the near window can reach always beats one it cannot,
# however good the latter's numbers look on its own far-field evidence.
TIER_CONTAINED = 0  # window covered, width plausible, ego between the boundaries
TIER_SCORED = 1  # window covered, width plausible
TIER_NEAR_EMPTY = 2  # nothing matched inside the window; scored on its own nearest nodes


def make_grid(min_x, max_x, step):
    """Ascending grid of x values, inclusive of ``max_x`` when it lands on a step."""
    if step <= 0.0 or max_x <= min_x:
        return np.empty(0)
    n = int(np.floor((max_x - min_x) / step)) + 1
    return min_x + step * np.arange(n, dtype=np.float64)


def condense_lane(lane_xyz, step):
    """Reduce one lane's matched returns to one median ``(x, y, z)`` per ``step``-wide bin.

    Two problems collapse into this single pass. Lane pixels are 1-3 px apart while projected
    returns in the road band are far sparser, so the nearest-neighbour match hands back the *same*
    return many times over. That is duplicate x, which interpolation is not defined on, and it also
    makes any count of "samples" an overcount of the evidence behind them. Separately, a lane pixel
    whose nearest return landed on a pole or a lead vehicle rather than the road puts one sample
    metres out of place. A per-bin median removes the duplicates and outvotes the outlier together,
    and disjoint bins leave the surviving x strictly increasing.

    Returns ``(x, y, z)``, strictly increasing in x.
    """
    pts = np.asarray(lane_xyz, dtype=np.float64).reshape(-1, 3)
    if pts.shape[0] == 0:
        return np.empty(0), np.empty(0), np.empty(0)

    pts = pts[np.argsort(pts[:, 0], kind="stable")]
    bins = np.floor(pts[:, 0] / float(step)).astype(np.int64)
    starts = np.concatenate(([0], np.flatnonzero(np.diff(bins)) + 1, [pts.shape[0]]))

    out = np.empty((starts.size - 1, 3))
    for i in range(out.shape[0]):
        out[i] = np.median(pts[starts[i] : starts[i + 1]], axis=0)
    return out[:, 0], out[:, 1], out[:, 2]


def backtrack_ratio(lane_xyz):
    """Backward travel in x as a fraction of forward travel, in detector order.

    Lane points arrive near to far, so a lane on a road straight enough to be single-valued in x
    walks forward and essentially never back. A lane that doubles back -- a turn tight enough that
    the road's heading rotates past 90 degrees inside the grid, roughly R < 40 m at a 60 m grid,
    and the minimum radius measured on rosbag2_2026_08_20-13_11_07 is 4.1 m -- walks back about as
    far as it walked out. Sorting such a lane by x folds the outbound and return branches together
    and interpolates a lane that is not there. Sorting is also what destroys the evidence, so this
    has to run on the detector-order sequence first.

    The steps are weighed by length rather than counted: consecutive lane pixels often match the
    same or a neighbouring return, so on a perfectly straight road roughly half the small steps are
    already non-positive and counting reversals would reject almost everything.
    """
    x = np.asarray(lane_xyz, dtype=np.float64).reshape(-1, 3)[:, 0]
    if x.size < 2:
        return 0.0
    d = np.diff(x)
    forward = float(d[d > 0.0].sum())
    backward = float(-d[d < 0.0].sum())
    if forward <= 0.0:
        return float("inf") if backward > 0.0 else 0.0
    return backward / forward


class Lane:
    """One lane resampled onto the shared grid."""

    __slots__ = ("y", "z", "valid", "n_source")

    def __init__(self, y, z, valid, n_source):
        self.y = y
        self.z = z
        self.valid = valid
        self.n_source = n_source


def resample_lane(
    lane_xyz,
    grid_x,
    *,
    step=0.5,
    max_interp_gap=5.0,
    max_backtrack=0.25,
    min_samples=3,
):
    """Lift one lane's matched returns onto ``grid_x``.

    ``valid`` is False outside the lane's own measured span -- nothing is ever extrapolated,
    because a boundary that stopped matching at 30 m says nothing about 40 m -- and False at any
    node further than ``max_interp_gap / 2`` from a measured sample. Without that second guard the
    interpolation draws a straight line across a hole in the returns, and downstream the result is
    indistinguishable from geometry that was actually measured. Half the gap, tested against the
    nearest sample on either side, keeps the test symmetric: a node landing exactly on a sample is
    never masked just because the interval bracketing it is wide on one side.

    ``n_source`` counts *condensed* samples, so it measures evidence rather than the pixel count
    that produced it.
    """
    n = int(np.asarray(grid_x).size)
    empty = Lane(np.zeros(n), np.zeros(n), np.zeros(n, dtype=bool), 0)

    pts = np.asarray(lane_xyz, dtype=np.float64).reshape(-1, 3)
    if n == 0 or pts.shape[0] < min_samples:
        return empty
    if max_backtrack >= 0.0 and backtrack_ratio(pts) > max_backtrack:
        return empty

    x, y, z = condense_lane(pts, step)
    if x.size < min_samples:
        return empty

    y_i = np.interp(grid_x, x, y)
    z_i = np.interp(grid_x, x, z)
    valid = (grid_x >= x[0]) & (grid_x <= x[-1])

    if max_interp_gap > 0.0:
        right = np.clip(np.searchsorted(x, grid_x, side="left"), 1, x.size - 1)
        nearest = np.minimum(grid_x - x[right - 1], x[right] - grid_x)
        valid &= nearest <= 0.5 * max_interp_gap

    return Lane(y_i, z_i, valid, int(x.size))


def sample_points(grid_x, y, z, valid):
    """``Nx3`` float32 of the measured nodes only, ascending in range."""
    idx = np.flatnonzero(valid)
    if idx.size == 0:
        return np.empty((0, 3), dtype=np.float32)
    return np.column_stack((grid_x[idx], y[idx], z[idx])).astype(np.float32)


class LanePair:
    """Two lanes evaluated against each other on the shared grid.

    ``offset`` is the *signed* mean lateral deviation of the centerline from the ego path over the
    scoring nodes -- zero for a pair the vehicle is centred in, about +/-3.5 m for a neighbour.
    Signed, because the hysteresis anchor matches on it and an absolute value cannot tell a pair
    3.5 m to the left from one 3.5 m to the right. The ego path is a ray rather than a point, so it
    is subtracted per node here and does not survive into the pair.
    """

    __slots__ = (
        "left",
        "right",
        "valid",
        "center_y",
        "center_z",
        "offset",
        "width",
        "tier",
        "nodes",
    )

    def __init__(
        self, left, right, valid, center_y, center_z, offset, width, tier, nodes
    ):
        self.left = left
        self.right = right
        self.valid = valid
        self.center_y = center_y
        self.center_z = center_z
        self.offset = offset
        self.width = width
        self.tier = tier
        self.nodes = nodes

    @property
    def score(self):
        return abs(self.offset)

    def __repr__(self):  # pragma: no cover - diagnostics only
        return (
            f"LanePair(tier={self.tier}, offset={self.offset:+.2f}m, "
            f"width={self.width:.2f}m, nodes={self.nodes})"
        )


def _evaluate(a, b, grid_x, window, *, min_window_nodes, min_width, max_width, ego_line):
    """Build a :class:`LanePair` from two resampled lanes, or None when implausible."""
    both = a.valid & b.valid
    idx = np.flatnonzero(both)
    if idx.size < 2:
        return None

    # Left/right decided at the *same* x. The curvature term x^2/2R is identical for two
    # boundaries of one road at one range, so it cancels in the difference -- which is exactly
    # what a mean over two unequal x spans could not do.
    if float(np.mean(a.y[idx] - b.y[idx])) >= 0.0:
        left, right = a, b
    else:
        left, right = b, a

    width = float(np.median(left.y[idx] - right.y[idx]))
    if not min_width <= width <= max_width:
        return None

    center_y = 0.5 * (left.y + right.y)
    center_z = 0.5 * (left.z + right.z)
    # Per node, because the ego path is a ray: a scalar reference is only correct at one range,
    # and the range the window samples moves with whichever nodes happen to be valid.
    deviation = center_y - ego_line

    scored = np.flatnonzero(both & window)
    if scored.size >= min_window_nodes:
        offset = float(np.mean(deviation[scored]))
        # Containment is read at the nearest overlap nodes, where the lever arm and therefore the
        # curvature bias are smallest.
        near = idx[: min(4, idx.size)]
        contained = bool(
            np.all(left.y[near] > ego_line[near])
            and np.all(right.y[near] < ego_line[near])
        )
        tier = TIER_CONTAINED if contained else TIER_SCORED
        nodes = int(scored.size)
    else:
        # Nothing matched inside the window: an occluded near field, or a detection that only
        # begins beyond it. Score on this pair's own nearest nodes and rank it behind every pair
        # the window could reach, rather than discarding it -- a consumer expecting a centerline
        # at ~10 Hz must not be starved because the near field went dark.
        near = idx[: min(min_window_nodes, idx.size)]
        offset = float(np.mean(deviation[near]))
        tier = TIER_NEAR_EMPTY
        nodes = int(near.size)

    return LanePair(
        left, right, both, center_y, center_z, offset, width, tier, nodes
    )


class LanePairSelector:
    """Picks the ego lane pair, with hysteresis keyed on geometry rather than on identity.

    ``lane_id`` is ``enumerate()`` output from the detector and carries no identity across frames,
    so the anchor is the accepted pair's signed lateral offset. Retention needs three things the
    strict per-frame argmin it replaces had none of: a *unique* incumbent (the single nearest
    candidate within ``incumbent_tol``, not "any candidate within it", which flip-flops when two
    are inside), a margin the challenger must beat, and a debounce. The margin guards against score
    noise; the debounce guards against the candidate *set* churning as lanes appear and drop, which
    is the failure the margin cannot see. A strictly better tier wins with neither, because a tier
    change is a discrete change of evidence rather than noise -- that is also what stops a degraded
    first frame becoming a sticky anchor.

    ``hysteresis_margin`` guards against score noise, but it is not what holds the lane. With the
    ego reference corrected to a ray the score is unambiguous, and the selector holds one pair for
    all 476 selected frames of the 2026-08-25 bag with the margin set to **zero** and the window
    widened to [6, 40] m. The 1.75 m it used to carry was sized against the old point reference,
    where candidate offsets formed two clusters ~3 m apart with the reference in the empty valley
    between them, and a margin below ~1.2 m let the choice between two *different lanes* flap on
    centimetres of interpolation noise (margin 0.35 -> 26 switches, 15 of them within 5 frames of
    the previous, against 1 real manoeuvre in the bag). That regime is gone, and 1.75 m is now
    actively harmful: it exceeds half a lane width, so it would suppress a genuine lane change.
    The default is half of it. Note that the bag carries **no** lane change -- its one manoeuvre is
    the turn onto the road -- so no setting here has been shown to *follow* one; that needs a bag
    that contains one.

    Every tunable is a plain float attribute so ``apply_bounded_parameters`` can point at it.
    """

    __slots__ = (
        "grid_x",
        "score_min_x",
        "score_max_x",
        "min_window_nodes",
        "min_lane_width",
        "max_lane_width",
        "incumbent_tol",
        "hysteresis_margin",
        "switch_debounce",
        "memory_timeout",
        "ego_y_offset",
        "ego_yaw_deg",
        "_anchor",
        "_anchor_time",
        "_challenger",
        "_challenger_streak",
        "frames",
        "selected",
        "switches",
        "width_rejects",
        "overlap_rejects",
        "near_empty_selections",
        "expiries",
    )

    def __init__(
        self,
        grid_x,
        *,
        score_min_x=6.0,
        score_max_x=30.0,
        min_window_nodes=8,
        min_lane_width=2.2,
        max_lane_width=4.5,
        incumbent_tol=0.75,
        hysteresis_margin=0.75,
        switch_debounce=2,
        memory_timeout=0.5,
        ego_y_offset=0.0,
        ego_yaw_deg=DEFAULT_EGO_YAW_DEG,
    ):
        self.grid_x = grid_x
        self.score_min_x = score_min_x
        self.score_max_x = score_max_x
        self.min_window_nodes = min_window_nodes
        self.min_lane_width = min_lane_width
        self.max_lane_width = max_lane_width
        self.incumbent_tol = incumbent_tol
        self.hysteresis_margin = hysteresis_margin
        self.switch_debounce = switch_debounce
        self.memory_timeout = memory_timeout
        self.ego_y_offset = ego_y_offset
        self.ego_yaw_deg = ego_yaw_deg

        self.frames = 0
        self.selected = 0
        self.switches = 0
        self.width_rejects = 0
        self.overlap_rejects = 0
        self.near_empty_selections = 0
        self.expiries = 0
        self.reset()

    def reset(self):
        """Forget the tracked pair. The next frame acquires from scratch."""
        self._anchor = None
        self._anchor_time = 0.0
        self._challenger = None
        self._challenger_streak = 0

    def select(self, lanes, now=0.0):
        """Pick the ego pair from resampled ``lanes``. Returns ``(pair_or_None, switched)``.

        ``switched`` is True whenever the returned pair is not the one the anchor described,
        including the case where the tracked pair vanished from the candidate set entirely. The
        caller uses it to drop smoothing history that describes a different pair.
        """
        self.frames += 1
        window = (self.grid_x >= self.score_min_x) & (self.grid_x <= self.score_max_x)
        min_nodes = max(1, int(self.min_window_nodes))
        # Rebuilt every frame rather than cached: both terms are plain attributes a ros2 param set can
        # move between frames, and a stale ray is a wrong reference with nothing to reveal it.
        ego_line = self.ego_y_offset + np.tan(np.radians(self.ego_yaw_deg)) * self.grid_x

        candidates = []
        for i in range(len(lanes)):
            for j in range(i + 1, len(lanes)):
                pair = _evaluate(
                    lanes[i],
                    lanes[j],
                    self.grid_x,
                    window,
                    min_window_nodes=min_nodes,
                    min_width=self.min_lane_width,
                    max_width=self.max_lane_width,
                    ego_line=ego_line,
                )
                if pair is None:
                    # Distinguishing the two rejections is what tells a tuning pass whether the
                    # width gate is too tight or the pixel match is too sparse.
                    if np.count_nonzero(lanes[i].valid & lanes[j].valid) < 2:
                        self.overlap_rejects += 1
                    else:
                        self.width_rejects += 1
                    continue
                candidates.append(pair)

        if not candidates:
            return None, False

        if self._anchor is not None and now - self._anchor_time > self.memory_timeout:
            self._anchor = None
            self._challenger = None
            self._challenger_streak = 0
            self.expiries += 1

        candidates.sort(key=lambda p: (p.tier, p.score))
        challenger = candidates[0]

        incumbent = None
        if self._anchor is not None:
            within = [
                p
                for p in candidates
                if abs(p.offset - self._anchor) <= self.incumbent_tol
            ]
            # The single nearest, not "any within tol": two candidates inside the tolerance would
            # otherwise take turns being the incumbent and reintroduce the flapping.
            if within:
                incumbent = min(within, key=lambda p: abs(p.offset - self._anchor))

        if incumbent is None:
            # No anchor at all is acquisition; an anchor with nothing near it means the pair being
            # tracked is gone, which is a switch as far as the smoothers are concerned.
            chosen = challenger
            switched = self._anchor is not None
            self._challenger = None
            self._challenger_streak = 0
        elif challenger is incumbent:
            chosen, switched = incumbent, False
            self._challenger = None
            self._challenger_streak = 0
        else:
            # Only an escape from TIER_NEAR_EMPTY counts as a discrete change of evidence: the
            # near field became visible where it was not. TIER_CONTAINED against TIER_SCORED is
            # the *same* evidence re-thresholded at the ego reference, and when the vehicle
            # straddles a lane line rather than sitting inside a lane -- a lane change halfway
            # through -- that test flips on centimetres of interpolation noise. (This was once
            # believed to describe most of the 2026-08-25 bag. It does not: that reading was the
            # uncorrected point reference, and the ray puts the vehicle 0.44 m from its lane
            # centre for the whole bag. The rule stands on the synthetic case below.) Letting
            # it bypass the margin and the debounce defeats the hysteresis entirely: on
            # synthetic boundaries at -3.5/0.0/+3.0 with 5 cm of jitter it switched on 29 of 30
            # frames. So a better tier still outranks a worse one, but between two pairs with
            # the same class of evidence the score has to clear the margin like anything else.
            evidence_gain = (
                incumbent.tier == TIER_NEAR_EMPTY and challenger.tier != TIER_NEAR_EMPTY
            )
            beats = evidence_gain or (
                challenger.tier <= incumbent.tier
                and challenger.score < incumbent.score - self.hysteresis_margin
            )
            if beats and (evidence_gain or self._advance_streak(challenger)):
                chosen, switched = challenger, True
                self._challenger = None
                self._challenger_streak = 0
            else:
                chosen, switched = incumbent, False
                if not beats:
                    self._challenger = None
                    self._challenger_streak = 0

        self._anchor = chosen.offset
        self._anchor_time = now
        self.selected += 1
        if switched:
            self.switches += 1
        if chosen.tier == TIER_NEAR_EMPTY:
            self.near_empty_selections += 1
        return chosen, switched

    def _advance_streak(self, challenger):
        """True once the *same* challenger has won ``switch_debounce`` frames running."""
        need = max(1, int(self.switch_debounce))
        if (
            self._challenger is None
            or abs(challenger.offset - self._challenger) > self.incumbent_tol
        ):
            self._challenger_streak = 1
        else:
            self._challenger_streak += 1
        self._challenger = challenger.offset
        return self._challenger_streak >= need

    def status(self):
        """One-line health summary for the periodic stats log."""
        anchor = "none" if self._anchor is None else f"{self._anchor:+.2f}m"
        return (
            f"ego_ray={self.ego_y_offset:+.2f}m@{self.ego_yaw_deg:+.2f}deg "
            f"frames={self.frames} selected={self.selected} switches={self.switches} "
            f"width_rejects={self.width_rejects} overlap_rejects={self.overlap_rejects} "
            f"near_empty={self.near_empty_selections} expiries={self.expiries} "
            f"anchor={anchor}"
        )


class GridSmoother:
    """Per-node exponential smoothing on a fixed grid.

    The implementation this replaces blended two point arrays only when their shapes matched, which
    is precisely the condition that fails whenever the geometry changes -- so it no-opped on every
    frame where the matched point count moved, which is to say on every frame where a pair switch
    or an occlusion happened. Measured left and right point counts differ by a median of 7, so it
    was inert most frames. Indexing by grid node makes the correspondence explicit instead: node k
    is the same range in every frame, so a node measured in both frames can be blended and a node
    absent last frame is taken as measured rather than blended against something stale.

    Two caveats the caller owns. The grid is ego-relative and the vehicle covers ~0.5 m per frame
    at 5 m/s, so node k is the same *range* but not the same patch of road; on a curve that costs
    roughly ``dx * x / R`` metres of lag, about 0.1 m at x=20 m and R=100 m, growing with x. And
    the state describes one lane pair only -- :meth:`reset` it when the selector switches, or the
    output ramps smoothly across a ~3.5 m step and reads downstream as a real lateral velocity
    rather than as the discontinuity it is.
    """

    __slots__ = ("alpha", "_y", "_z", "_valid")

    def __init__(self, alpha=0.5):
        self.alpha = alpha
        self.reset()

    def reset(self):
        self._y = None
        self._z = None
        self._valid = None

    def update(self, y, z, valid):
        """Blend ``(y, z)`` against the previous frame at nodes measured in both."""
        a = float(self.alpha)
        y = np.asarray(y, dtype=np.float64)
        z = np.asarray(z, dtype=np.float64)
        valid = np.asarray(valid, dtype=bool)

        out_y, out_z = y.copy(), z.copy()
        # The shape test is a guard against a rebuilt grid, not a gate on the geometry: on a fixed
        # grid it is always true, which is the whole point of the change.
        if (
            self._valid is not None
            and self._valid.shape == valid.shape
            and 0.0 < a < 1.0
        ):
            blend = valid & self._valid
            out_y[blend] = a * y[blend] + (1.0 - a) * self._y[blend]
            out_z[blend] = a * z[blend] + (1.0 - a) * self._z[blend]

        self._y, self._z, self._valid = out_y, out_z, valid
        return out_y, out_z
