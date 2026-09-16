"""Behaviour tests for range-indexed lane geometry.

These assert the properties the 2026-08-28 change was made to achieve, so a regression fails a
test rather than the vehicle. The two defects being fixed are both reproduced directly: the old
index pairing put the centerline where neither boundary was measured, and the old whole-polyline
``mean_y`` called the right lane the left one on a curve. Both are driven from synthetic geometry
because the input sequences that expose them -- a curve tight enough to invert the sort, one
boundary occluded in the far field and not the other -- are exactly the ones a bag replay will not
reproduce on demand.

Run with ``PYTHONPATH=src/perception_common python3 -m pytest src/perception_common/test -q``.
No CI runs these.
"""

import math

import numpy as np
import pytest

from perception_common.lane_geometry import (
    DEFAULT_EGO_YAW_DEG,
    TIER_CONTAINED,
    TIER_NEAR_EMPTY,
    GridSmoother,
    LanePairSelector,
    backtrack_ratio,
    condense_lane,
    make_grid,
    resample_lane,
    sample_points,
)

GRID = make_grid(5.0, 100.0, 0.5)
ROAD_Z = -2.46


def curved_lane(y_offset, x_from, x_to, n=40, radius=None):
    """A boundary at ``y_offset`` metres, bent by ``radius`` (y = offset + x^2/2R)."""
    x = np.linspace(x_from, x_to, n)
    y = np.full_like(x, float(y_offset))
    if radius:
        y = y + x**2 / (2.0 * float(radius))
    return np.column_stack((x, y, np.full_like(x, ROAD_Z)))


def resample(pts, **kw):
    kw.setdefault("max_interp_gap", 5.0)
    return resample_lane(pts, GRID, step=0.5, **kw)


# --------------------------------------------------------------------------------------------
# Defect reproductions
# --------------------------------------------------------------------------------------------


def test_index_pairing_puts_the_centreline_where_neither_lane_was_measured():
    """Defect 1: the old elementwise mean averages points at different ranges."""
    left = curved_lane(1.75, 6.0, 45.0, n=40)
    # The pixel-radius match survives only every fourth sample on the right boundary, which is
    # what the measurement found: left and right point counts differ by a median of 7.
    right = curved_lane(-1.75, 6.0, 45.0, n=40)[::4]

    n = min(len(left), len(right))
    legacy = (left[:n] + right[:n]) / 2.0
    # The old rule pairs left[i] with right[i]; their x differ by metres.
    mismatch = np.abs(left[:n, 0] - right[:n, 0])
    assert np.median(mismatch) > 3.0
    # And the legacy centre stops far short of the range both boundaries actually covered:
    # truncating to the shorter array cuts more than 10 m off the horizon here.
    assert legacy[:, 0].max() < left[:, 0].max() - 10.0

    a, b = resample(left), resample(right)
    both = a.valid & b.valid
    centre_y = 0.5 * (a.y + b.y)
    # On the grid the two sources share an x by construction, so the centre is where it belongs.
    assert np.allclose(centre_y[both], 0.0, atol=0.02)


def test_whole_polyline_mean_y_calls_the_right_lane_the_left_one_on_a_curve():
    """Defect 2: the old sort key inverts when the two boundaries have unequal x spans."""
    # Left boundary occluded past 20 m, right boundary measured to 80 m, R = 150 m left turn.
    left = curved_lane(1.75, 6.0, 20.0, n=30, radius=150.0)
    right = curved_lane(-1.75, 6.0, 80.0, n=90, radius=150.0)

    # The old key: mean y over the whole polyline. It says the right boundary is further left.
    assert np.mean(right[:, 1]) > np.mean(left[:, 1])

    # Compared at matched range, the curvature term cancels and the order is right.
    a, b = resample(left), resample(right)
    both = a.valid & b.valid
    assert both.any()
    assert np.mean(a.y[both] - b.y[both]) > 0.0
    assert np.median(a.y[both] - b.y[both]) == pytest.approx(3.5, abs=0.05)


# --------------------------------------------------------------------------------------------
# Resampling
# --------------------------------------------------------------------------------------------


def test_resample_never_extrapolates_beyond_the_measured_span():
    lane = resample(curved_lane(1.75, 20.0, 40.0, n=40))
    covered = GRID[lane.valid]
    assert covered.min() >= 20.0
    assert covered.max() <= 40.0
    assert not lane.valid[GRID < 20.0].any()
    assert not lane.valid[GRID > 40.0].any()


def test_resample_masks_nodes_that_bridge_a_gap_in_the_returns():
    pts = np.array([[6.0, 1.75, ROAD_Z], [6.5, 1.75, ROAD_Z], [40.0, 1.75, ROAD_Z]])
    tight = resample(pts, max_interp_gap=5.0)
    assert not tight.valid[(GRID > 10.0) & (GRID < 36.0)].any()
    # Raising the gap makes them valid, proving the span test is not what excluded them.
    wide = resample(pts, max_interp_gap=80.0)
    assert wide.valid[(GRID > 10.0) & (GRID < 36.0)].all()


def test_gap_guard_keeps_a_node_that_lands_on_a_measured_sample():
    """The asymmetric-bracket case: a wide interval to the right must not mask x=10."""
    pts = np.array(
        [[6.0, 1.75, ROAD_Z], [8.0, 1.75, ROAD_Z], [10.0, 1.75, ROAD_Z], [40.0, 1.75, ROAD_Z]]
    )
    lane = resample(pts, max_interp_gap=5.0)
    assert lane.valid[np.argmin(np.abs(GRID - 10.0))]


def test_repeated_nearest_neighbour_hits_are_condensed_to_one_sample():
    """1-NN returns the same LiDAR point for adjacent lane pixels; duplicates are the norm."""
    base = curved_lane(1.75, 6.0, 30.0, n=10)
    pts = np.repeat(base, 30, axis=0)
    lane = resample(pts)
    assert lane.n_source <= 10 * 2  # condensed, not 300
    assert np.isfinite(lane.y).all()
    assert lane.valid.any()


def test_condense_outvotes_a_single_outlier_match():
    """A pixel whose nearest return landed on a pole must not move the lane."""
    pts = np.array(
        [
            [10.1, 1.75, ROAD_Z],
            [10.2, 1.75, ROAD_Z],
            [10.3, 1.75, ROAD_Z],
            [10.4, 8.00, 1.0],  # pole
        ]
    )
    x, y, _ = condense_lane(pts, 0.5)
    assert y[0] == pytest.approx(1.75, abs=0.01)


def test_backtrack_ratio_flags_a_lane_that_doubles_back():
    out = np.linspace(6.0, 30.0, 20)
    back = np.linspace(30.0, 8.0, 20)
    folded = np.column_stack(
        (np.concatenate((out, back)), np.zeros(40), np.full(40, ROAD_Z))
    )
    assert backtrack_ratio(folded) > 0.5
    assert not resample(folded, max_backtrack=0.25).valid.any()


def test_duplicate_and_jitter_noise_does_not_look_like_backtracking():
    """Half the small steps are non-positive on a straight road; counting them would reject it."""
    base = curved_lane(1.75, 6.0, 60.0, n=60)
    jittered = np.repeat(base, 3, axis=0)
    rng = np.random.default_rng(0)
    jittered[:, 0] += rng.normal(0.0, 0.05, jittered.shape[0])
    assert backtrack_ratio(jittered) < 0.25
    assert resample(jittered).valid.any()


def test_sample_points_are_ascending_and_measured_only():
    lane = resample(curved_lane(1.75, 10.0, 50.0, n=40))
    pts = sample_points(GRID, lane.y, lane.z, lane.valid)
    assert pts.dtype == np.float32
    assert pts.shape[0] == int(lane.valid.sum())
    assert np.all(np.diff(pts[:, 0]) > 0)


# --------------------------------------------------------------------------------------------
# Pair evaluation and selection
# --------------------------------------------------------------------------------------------


def selector(**kw):
    kw.setdefault("switch_debounce", 1)
    # Every synthetic lane below is built symmetric about y=0, so these exercise the pairing and
    # hysteresis rules against an ego path that is the y axis. The shipped default is a ray at
    # -5.35 deg, which is a fact about where lidar_tc is bolted rather than about this logic;
    # test_ego_ray_* below covers that separately, and is the case the old code fails.
    kw.setdefault("ego_yaw_deg", 0.0)
    return LanePairSelector(GRID, **kw)


@pytest.mark.parametrize("radius", [None, 500.0, 200.0, 100.0, 60.0])
def test_left_right_assignment_is_curvature_free_at_matched_range(radius):
    lanes = [
        resample(curved_lane(1.75, 6.0, 60.0, n=60, radius=radius)),
        resample(curved_lane(-1.75, 6.0, 60.0, n=60, radius=radius)),
    ]
    pair, _ = selector().select(lanes)
    assert pair is not None
    assert pair.width == pytest.approx(3.5, abs=0.02)
    idx = np.flatnonzero(pair.valid)
    assert np.all(pair.left.y[idx] > pair.right.y[idx])


def test_pair_with_implausible_width_is_rejected():
    lanes = [
        resample(curved_lane(1.75, 6.0, 60.0, n=60)),
        resample(curved_lane(-5.25, 6.0, 60.0, n=60)),
    ]
    sel = selector()
    pair, _ = sel.select(lanes)
    assert pair is None
    assert sel.width_rejects == 1


def test_ego_lane_is_preferred_over_the_neighbouring_lane():
    lanes = [
        resample(curved_lane(y, 6.0, 60.0, n=60)) for y in (-5.25, -1.75, 1.75, 5.25)
    ]
    pair, _ = selector().select(lanes)
    assert pair is not None
    assert pair.offset == pytest.approx(0.0, abs=0.05)
    assert pair.tier == TIER_CONTAINED


def test_all_pairs_are_considered_not_only_sort_adjacent_ones():
    """The correct pair is non-adjacent under a whole-polyline sort; it must still win."""
    raw = [
        curved_lane(1.75, 6.0, 20.0, n=30, radius=150.0),
        curved_lane(-1.75, 6.0, 80.0, n=90, radius=150.0),
        curved_lane(-5.25, 6.0, 80.0, n=90, radius=150.0),
    ]
    # Under the old whole-polyline key the ego boundaries do not end up adjacent: the short
    # +1.75 lane sorts next to the long -5.25 one, because the far field dominates both.
    order = np.argsort([np.mean(lane[:, 1]) for lane in raw])
    assert list(order) == [0, 2, 1]

    pair, _ = selector().select([resample(lane) for lane in raw])
    assert pair is not None
    # The true lane centre is genuinely displaced by the curve, so the test is the pair's
    # identity -- width and containment -- not an offset of zero.
    assert pair.width == pytest.approx(3.5, abs=0.05)
    assert pair.tier == TIER_CONTAINED


def test_pair_beyond_the_score_window_is_ranked_behind_a_scoreable_one():
    near = [
        resample(curved_lane(2.75, 6.0, 20.0, n=30)),
        resample(curved_lane(-0.75, 6.0, 20.0, n=30)),
    ]
    far = [
        resample(curved_lane(1.85, 30.0, 60.0, n=40)),
        resample(curved_lane(-1.65, 30.0, 60.0, n=40)),
    ]
    pair, _ = selector().select(near + far)
    assert pair is not None
    # The far pair scores better (offset 0.10 vs 1.00) but cannot be seen in the window.
    assert pair.offset == pytest.approx(1.0, abs=0.05)


def test_pair_beyond_the_score_window_is_still_selected_when_it_is_the_only_one():
    """The rate contract: a dark near field must not starve the consumer."""
    lanes = [
        resample(curved_lane(1.75, 30.0, 60.0, n=40)),
        resample(curved_lane(-1.75, 30.0, 60.0, n=40)),
    ]
    sel = selector()
    pair, _ = sel.select(lanes)
    assert pair is not None
    assert pair.tier == TIER_NEAR_EMPTY
    assert sel.near_empty_selections == 1


def test_fewer_than_two_overlapping_nodes_is_rejected_not_scored():
    lanes = [
        resample(curved_lane(1.75, 6.0, 15.0, n=20)),
        resample(curved_lane(-1.75, 40.0, 60.0, n=20)),
    ]
    sel = selector()
    pair, _ = sel.select(lanes)
    assert pair is None
    assert sel.overlap_rejects == 1


def test_single_lane_and_empty_input_return_none():
    sel = selector()
    assert sel.select([]) == (None, False)
    assert sel.select([resample(curved_lane(1.75, 6.0, 60.0, n=60))]) == (None, False)


# --------------------------------------------------------------------------------------------
# Hysteresis
# --------------------------------------------------------------------------------------------


def ego_pair():
    return [resample(curved_lane(y, 6.0, 60.0, n=60)) for y in (-1.75, 1.75)]


def test_first_frame_acquires_the_best_scoring_pair():
    sel = selector()
    pair, switched = sel.select(ego_pair())
    assert pair is not None
    assert switched is False
    assert sel.switches == 0


def test_incumbent_is_retained_against_a_challenger_inside_the_margin():
    sel = selector(hysteresis_margin=0.35)
    first, _ = sel.select([resample(curved_lane(y, 6.0, 60.0, n=60)) for y in (-1.9, 1.6)])
    assert first.offset == pytest.approx(-0.15, abs=0.02)
    # A new boundary makes a pair 0.10 m better -- inside the margin, so no switch.
    lanes = [resample(curved_lane(y, 6.0, 60.0, n=60)) for y in (-1.9, 1.6, 1.85)]
    pair, switched = sel.select(lanes)
    assert switched is False
    assert pair.offset == pytest.approx(first.offset, abs=1e-9)
    assert sel.switches == 0


def test_challenger_must_win_the_debounce_before_it_takes_over():
    sel = selector(hysteresis_margin=0.1, switch_debounce=2, incumbent_tol=0.75)
    off = [resample(curved_lane(y, 6.0, 60.0, n=60)) for y in (-2.35, 1.15)]
    sel.select(off)  # anchor at -0.60
    both = off + [resample(curved_lane(y, 6.0, 60.0, n=60)) for y in (-1.75, 1.75)]
    _, switched_first = sel.select(both)
    assert switched_first is False  # debounce not yet satisfied
    _, switched_second = sel.select(both)
    assert switched_second is True
    assert sel.switches == 1


def test_a_strictly_better_tier_wins_without_margin_or_debounce():
    """Stops a degraded first frame becoming a sticky anchor."""
    sel = selector(switch_debounce=5, hysteresis_margin=5.0)
    far = [
        resample(curved_lane(1.85, 30.0, 60.0, n=40)),
        resample(curved_lane(-1.65, 30.0, 60.0, n=40)),
    ]
    first, _ = sel.select(far)
    assert first.tier == TIER_NEAR_EMPTY
    pair, switched = sel.select(far + ego_pair())
    assert switched is True
    assert pair.tier == TIER_CONTAINED


def test_two_candidates_inside_incumbent_tol_do_not_oscillate():
    """The anti-flap test: the incumbent must be the nearest match, not any match."""
    sel = selector(incumbent_tol=0.75, hysteresis_margin=0.35, switch_debounce=2)
    lanes = [resample(curved_lane(y, 6.0, 60.0, n=60)) for y in (-1.85, 1.65, 2.15)]
    offsets = []
    for _ in range(20):
        pair, _ = sel.select(lanes)
        offsets.append(pair.offset)
    assert sel.switches == 0
    assert len(set(np.round(offsets, 9))) == 1


def test_two_lanes_straddling_the_ego_reference_do_not_flap():
    """The 2026-08-25 failure: the ego reference falls between two lanes, not inside one.

    Boundaries at -3.5, 0.0 and +3.0 make two candidate pairs whose centres sit at -1.75 m and
    +1.50 m, with the ego reference in the empty gap between them -- the vehicle straddling a
    lane line rather than sitting in a lane, which is what a lane change looks like halfway
    through. Their scores are then within ~0.25 m of each other, so a margin sized in centimetres
    turns the choice between two *different lanes* into a coin flip.

    This configuration was originally believed to describe most of adps_2026-08-25. It does not:
    that reading was the uncorrected ego reference, and with the ray applied the vehicle sits
    0.44 m from its lane centre for the whole bag. The case is still real -- it is a genuine
    straddle -- so the test stands, but it is synthetic rather than a bag reproduction, and it is
    the reason the margin is not driven to zero even though the bag no longer needs one.
    """
    offsets = (-3.5, 0.0, 3.0)
    sel = selector(hysteresis_margin=1.75, switch_debounce=2)
    seen = []
    for k in range(30):
        # A centimetre of interpolation noise, alternating sign, is all it took.
        jitter = 0.05 * (-1) ** k
        lanes = [resample(curved_lane(o + jitter, 6.0, 60.0, n=60)) for o in offsets]
        pair, _ = sel.select(lanes, now=k * 0.1)
        assert pair is not None
        seen.append(pair.offset)
    assert sel.switches == 0
    # It stays on the -1.75 lane rather than crossing to the +1.5 one. The offset still moves
    # by the jitter itself, which is the pair being measured, not the pair changing.
    assert max(seen) - min(seen) < 0.25
    assert np.mean(seen) == pytest.approx(-1.75, abs=0.15)

    # And the same input with the old centimetre-scale margin is what flapped.
    loose = selector(hysteresis_margin=0.35, switch_debounce=1)
    picks = []
    for k in range(30):
        jitter = 0.6 * (-1) ** k
        lanes = [resample(curved_lane(o + jitter, 6.0, 60.0, n=60)) for o in offsets]
        pair, _ = loose.select(lanes, now=k * 0.1)
        picks.append(round(pair.offset, 3))
    assert loose.switches > 0
    assert len(set(picks)) > 1


def test_margin_still_admits_a_clearly_better_pair():
    """Half a lane width damps noise without welding the selection in place.

    Note a vehicle that re-centres after a lane change looks *identical* from the ego frame --
    the tracked pair is still the one around it -- so the case worth testing is not a lane
    change but a tracked pair that is plainly off centre while a better one is available.
    """
    off = [resample(curved_lane(y, 6.0, 60.0, n=60)) for y in (-5.25, -1.75)]
    sel = selector(hysteresis_margin=1.75, switch_debounce=2)
    first, _ = sel.select(off, now=0.0)
    assert first.offset == pytest.approx(-3.5, abs=0.05)

    # Now a pair centred on the vehicle appears: 3.5 m better, well past half a lane width.
    both = off + [resample(curved_lane(1.75, 6.0, 60.0, n=60))]
    switched = False
    for k in range(1, 6):
        pair, sw = sel.select(both, now=k * 0.1)
        switched |= sw
    assert switched
    assert pair.offset == pytest.approx(0.0, abs=0.05)


def test_a_vanished_incumbent_reports_a_switch():
    sel = selector()
    sel.select(ego_pair())
    far = [resample(curved_lane(y, 6.0, 60.0, n=60)) for y in (5.25, 8.75)]
    pair, switched = sel.select(far)
    assert pair is not None
    assert switched is True


def test_anchor_expires_after_memory_timeout():
    sel = selector(memory_timeout=0.5, hysteresis_margin=5.0, switch_debounce=9)
    off = [resample(curved_lane(y, 6.0, 60.0, n=60)) for y in (-2.35, 1.15)]
    sel.select(off, now=0.0)
    both = off + ego_pair()
    # Still inside the timeout: a three-frame detector dropout must not lose the lock.
    held, _ = sel.select(both, now=0.3)
    assert held.offset == pytest.approx(-0.6, abs=0.05)
    assert sel.expiries == 0
    # Past it: the anchor is forgotten and the better pair is acquired.
    fresh, _ = sel.select(both, now=0.9)
    assert fresh.offset == pytest.approx(0.0, abs=0.05)
    assert sel.expiries == 1


def test_reset_clears_the_anchor():
    sel = selector(hysteresis_margin=5.0, switch_debounce=9)
    off = [resample(curved_lane(y, 6.0, 60.0, n=60)) for y in (-2.35, 1.15)]
    sel.select(off)
    sel.reset()
    pair, switched = sel.select(off + ego_pair())
    assert switched is False
    assert pair.offset == pytest.approx(0.0, abs=0.05)


# --------------------------------------------------------------------------------------------
# Smoothing
# --------------------------------------------------------------------------------------------


def test_smoother_blends_a_node_measured_in_both_frames():
    s = GridSmoother(0.5)
    valid = np.ones(4, dtype=bool)
    s.update(np.zeros(4), np.zeros(4), valid)
    y, _ = s.update(np.full(4, 2.0), np.zeros(4), valid)
    assert np.allclose(y, 1.0)


def test_smoother_takes_a_newly_valid_node_as_measured():
    s = GridSmoother(0.5)
    s.update(np.zeros(4), np.zeros(4), np.array([True, True, False, False]))
    y, _ = s.update(np.full(4, 2.0), np.zeros(4), np.ones(4, dtype=bool))
    assert np.allclose(y[:2], 1.0)  # blended
    assert np.allclose(y[2:], 2.0)  # taken as measured, not blended against a stale zero


def test_smoother_is_not_gated_on_the_point_count():
    """Defect 3: the old shape test no-opped whenever the matched count moved."""
    s = GridSmoother(0.5)
    first = np.zeros(20, dtype=bool)
    first[:12] = True
    second = np.zeros(20, dtype=bool)
    second[:19] = True
    # The old rule compared the *packed* point arrays, whose shapes differ here, and skipped.
    assert sample_points(GRID[:20], np.zeros(20), np.zeros(20), first).shape != sample_points(
        GRID[:20], np.zeros(20), np.zeros(20), second
    ).shape

    s.update(np.zeros(20), np.zeros(20), first)
    y, _ = s.update(np.full(20, 2.0), np.zeros(20), second)
    assert np.allclose(y[:12], 1.0)  # blended on all 12 common nodes
    assert np.allclose(y[12:19], 2.0)


def test_smoother_reset_drops_the_history():
    s = GridSmoother(0.5)
    valid = np.ones(4, dtype=bool)
    s.update(np.zeros(4), np.zeros(4), valid)
    s.reset()
    y, _ = s.update(np.full(4, 2.0), np.zeros(4), valid)
    assert np.allclose(y, 2.0)


def test_smoothed_centre_is_the_mean_of_the_smoothed_boundaries():
    """The consumer invariant: centre == mean(left, right) at every published node."""
    left_s, right_s = GridSmoother(0.5), GridSmoother(0.5)
    valid = np.ones(8, dtype=bool)
    left_s.update(np.full(8, 2.0), np.zeros(8), valid)
    right_s.update(np.full(8, -1.0), np.zeros(8), valid)
    ly, _ = left_s.update(np.full(8, 1.0), np.zeros(8), valid)
    ry, _ = right_s.update(np.full(8, -2.0), np.zeros(8), valid)
    assert np.allclose(ly, 1.5)
    assert np.allclose(ry, -1.5)
    # Deriving the centre from the smoothed boundaries is what makes the invariant hold. It is
    # equal to smoothing the raw centre directly only because the two masks agree here; the node
    # derives it so that it holds even when they do not.
    assert np.allclose(0.5 * (ly + ry), 0.0)


# --------------------------------------------------------------------------------------------
# Contract guard
# --------------------------------------------------------------------------------------------


def test_published_centreline_reaches_the_range_the_matches_covered():
    """Fails if the grid is shortened past what the boundaries actually measured."""
    lanes = [resample(curved_lane(y, 6.0, 85.0, n=120)) for y in (-1.75, 1.75)]
    pair, _ = selector().select(lanes)
    assert pair is not None
    centre = sample_points(GRID, pair.center_y, pair.center_z, pair.valid)
    assert centre[:, 0].max() >= 84.5


# --------------------------------------------------------------------------------------------
# The ego reference is a ray, not a point
# --------------------------------------------------------------------------------------------


def sheared_lane(perp_offset, x_from, x_to, n=60, yaw_deg=DEFAULT_EGO_YAW_DEG):
    """A boundary parallel to the ego path at ``perp_offset`` m, seen in the LiDAR frame.

    The LiDAR is yawed against the vehicle, so a lane that is genuinely straight and parallel to
    the direction of travel is a *sloped* line in the frame the selector works in:
    ``y = tan(yaw) * x + d / cos(yaw)``. This is the geometry the vehicle actually produces, and
    the reason the old point reference picked the wrong lane.
    """
    yaw = math.radians(yaw_deg)
    x = np.linspace(x_from, x_to, n)
    y = math.tan(yaw) * x + float(perp_offset) / math.cos(yaw)
    return np.column_stack((x, y, np.full_like(x, ROAD_Z)))


def three_lanes(x_from=20.0, x_to=50.0):
    """Four boundaries: the ego lane at +/-1.75 m and one neighbour either side."""
    return [resample(sheared_lane(d, x_from, x_to)) for d in (-5.25, -1.75, 1.75, 5.25)]


def test_ego_ray_picks_the_lane_the_vehicle_is_in_when_the_near_field_is_dark():
    """The defect, reproduced: a point reference crosses into the neighbouring lane with range.

    Boundaries start at 20 m, which is the normal case rather than a contrived one -- the near
    field is TIER_NEAR_EMPTY on 85% of frames of adps_2026-08-25. By 20 m the ego path has
    diverged from ``y = 0`` by 1.87 m, more than half a lane width, so ``y = 0`` is no longer
    between the ego boundaries at all: it sits inside the *left* lane. A point reference
    therefore reports the wrong lane as the contained one and wins on tier as well as on score.
    """
    lanes = three_lanes()

    sel = LanePairSelector(GRID, switch_debounce=1)
    pair, _ = sel.select(lanes)
    assert pair is not None
    assert pair.left is lanes[2] and pair.right is lanes[1]
    assert pair.offset == pytest.approx(0.0, abs=0.05)
    assert pair.tier == TIER_CONTAINED

    # The same input against the reference this replaces.
    old = LanePairSelector(GRID, switch_debounce=1, ego_yaw_deg=0.0)
    wrong, _ = old.select(lanes)
    assert wrong is not None
    assert wrong.left is lanes[3] and wrong.right is lanes[2]
    assert wrong.tier == TIER_CONTAINED  # confidently, and about the wrong lane


def test_ego_ray_holds_the_same_lane_as_range_grows():
    """The failure is range-dependent, so the fix has to be range-independent."""
    sel = LanePairSelector(GRID, switch_debounce=1)
    for k, x_from in enumerate((6.0, 12.0, 20.0, 30.0, 45.0)):
        lanes = three_lanes(x_from, x_from + 30.0)
        # now advances by a frame, not by the span, so the anchor stays alive and switches means
        # something. The last span clears the score window entirely and falls to TIER_NEAR_EMPTY,
        # which is the point: the reference has to hold there too.
        pair, _ = sel.select(lanes, now=k * 0.1)
        assert pair is not None
        assert pair.left is lanes[2] and pair.right is lanes[1]
        assert pair.offset == pytest.approx(0.0, abs=0.06)
    assert sel.switches == 0


def test_ray_reduces_to_the_old_point_reference_at_zero_yaw():
    """The change is a generalisation: at yaw 0 the score is the mean centreline y as before."""
    lanes = [resample(curved_lane(y, 6.0, 60.0, n=60)) for y in (-0.75, 2.75)]
    pair, _ = LanePairSelector(GRID, switch_debounce=1, ego_yaw_deg=0.0).select(lanes)
    assert pair is not None
    window = pair.valid & (GRID >= 6.0) & (GRID <= 30.0)
    assert pair.offset == pytest.approx(float(np.mean(pair.center_y[window])), abs=1e-9)


def test_ego_y_offset_still_shifts_the_ray_sideways():
    """The intercept keeps working, so a real lateral mounting offset stays expressible."""
    lanes = three_lanes()
    shifted = LanePairSelector(GRID, switch_debounce=1, ego_y_offset=3.5)
    pair, _ = shifted.select(lanes)
    assert pair is not None
    # An ego path 3.5 m to the left makes the left-adjacent lane the one the vehicle is in.
    assert pair.left is lanes[3] and pair.right is lanes[2]
    assert pair.offset == pytest.approx(0.0, abs=0.05)
