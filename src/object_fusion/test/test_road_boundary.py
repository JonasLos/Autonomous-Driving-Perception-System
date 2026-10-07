"""Road boundary rule on synthetic scenes: what an obstacle does to the edge, step by step."""
import math

import numpy as np
import pytest

from object_fusion.road_boundary import (
    EDGE, OCCLUDED, BoundaryParams, BoundaryTracker, ShadowMap, box_footprint_points,
    decode_semkitti_rgb, measure, resample, select_run,
)

HALF = 3.5
Z = -2.37


def road(x0=3.0, x1=60.0, y0=-HALF, y1=HALF, step=0.3):
    gx, gy = np.meshgrid(np.arange(x0, x1, step), np.arange(y0, y1 + 1e-9, step))
    return np.column_stack((gx.ravel(), gy.ravel()))


def carve(road_xy, obstacle_xy):
    """Remove road the sensor cannot see: under the obstacle and in its shadow."""
    sh = ShadowMap(obstacle_xy)
    keep = ~sh.occluded(road_xy)
    return road_xy[keep]


def car(cx, cy, length=4.5, width=1.8):
    return box_footprint_points([[cx, cy]], [[length, width]], [0.0], step=0.3)


def run(road_xy, obs, **flags):
    p = BoundaryParams()
    return measure(road_xy, np.full(len(road_xy), Z), obs, p, **flags)


ALL_OFF = dict(occlusion=False, extraction=False, fit=False)


def test_semkitti_decode_pins_road_and_car():
    packed = np.array([(255 << 16) | 255, (100 << 16) | (150 << 8) | 245, 12345], dtype=np.uint32)
    assert decode_semkitti_rgb(packed).tolist() == [9, 1, -1]
    assert decode_semkitti_rgb(packed.view(np.float32)).tolist() == [9, 1, -1]


def test_clear_road_is_unchanged_by_every_step():
    r = road()
    raw = run(r, np.empty((0, 2)), **ALL_OFF)
    new = run(r, np.empty((0, 2)))
    for m in (raw, new):
        v = np.isfinite(m.left)
        assert v.sum() >= 25
        assert np.allclose(m.left[v], HALF, atol=0.31)
        assert np.allclose(m.right[np.isfinite(m.right)], -HALF, atol=0.31)


def test_car_at_the_edge_no_longer_pulls_the_boundary_in():
    obs = car(22.0, 2.6)                     # straddles the left edge
    r = carve(road(), obs)
    raw = run(r, obs, **ALL_OFF)
    new = run(r, obs)
    # The failure being fixed: raw left edge pulled in by > 1 m behind the car.
    assert np.nanmin(raw.left) < HALF - 1.0
    assert (new.left_state == OCCLUDED).any()
    v = np.isfinite(new.left)
    assert np.allclose(new.left[v], HALF, atol=0.35)


def test_car_in_mid_lane_changes_nothing():
    obs = car(30.0, 0.0)
    r = carve(road(), obs)
    new = run(r, obs)
    near = new.x < 30.0                     # beyond it the shadow cone legitimately widens
    assert (new.left_state[near] == EDGE).all() and (new.right_state[near] == EDGE).all()
    v = np.isfinite(new.left)
    assert np.allclose(new.left[v], HALF, atol=0.35)
    assert np.allclose(new.right[np.isfinite(new.right)], -HALF, atol=0.35)


def test_extraction_keeps_far_road_across_an_obstacle_and_drops_a_detached_patch():
    near, far = road(3, 24), road(28, 60)
    patch = road(20, 30, 15.0, 18.0)
    blocker = box_footprint_points([[26.0, 0.0]], [[3.0, 2 * HALF + 1]], [0.0], step=0.3)
    pts = np.concatenate([near, far, patch])
    raw = run(pts, blocker, **ALL_OFF)
    new = run(pts, blocker)
    assert np.nanmax(raw.left) > 10.0          # raw: the patch becomes the left edge
    assert np.nanmax(new.left) < HALF + 0.5    # patch dropped
    # Far road kept past the blocker. Occlusion off: a blocker this wide would shadow the whole
    # far road from a real sensor, and occlusion is right to call it unseen.
    ext = run(pts, blocker, occlusion=False)
    assert np.isfinite(ext.left[ext.x > 30]).sum() >= 10
    assert np.nanmax(ext.left) < HALF + 0.5


def test_select_run_bridges_shadow_gaps_and_ignores_obstacle_extremes():
    road_y = np.r_[np.linspace(-3.5, -1.1, 9), np.linspace(1.5, 3.3, 7)]
    ref = (-2.0, 2.0)
    # A 2.5 m hole in the middle: separate runs, unless the hole is a shadow.
    assert select_run(road_y, [], ref, 1.5) is not None
    lo, hi = select_run(road_y, [], ref, 1.5, bridged=lambda y: np.ones(y.size, bool))
    assert lo == pytest.approx(-3.5) and hi == pytest.approx(3.3)
    # An obstacle outboard of the road joins the run but is not an extreme.
    lo, hi = select_run(road_y, [4.0, 4.5], ref, 1.5, bridged=lambda y: np.ones(y.size, bool))
    assert hi == pytest.approx(3.3)
    # Sparse rings: no road in the bin at all.
    assert select_run([], [1.0], ref, 1.5) is None


def test_sparse_rings_are_not_a_disconnection():
    rings = [5, 8, 12, 16, 21, 27, 34, 42, 51]
    pts = np.concatenate([road(x, x + 0.4) for x in rings])
    new = run(pts, np.empty((0, 2)))
    assert np.isfinite(new.left).sum() >= 25 and new.kept_fraction == 1.0


def test_one_side_fully_occluded_is_placed_at_the_width():
    # A wall of vehicles along the whole left edge: no left edge is ever seen.
    obs = np.concatenate([car(x, 2.8) for x in np.arange(5.0, 60.0, 4.6)])
    r = carve(road(), obs)
    new = measure(r, np.full(len(r), Z), obs, BoundaryParams(), width_hint=2 * HALF)
    v = np.isfinite(new.left)
    assert v.sum() >= 20
    assert np.allclose(new.left[v], HALF, atol=0.4)
    assert np.allclose(new.sigma_left[v], BoundaryParams().sigma_mirror)


def test_tracker_coasts_then_expires():
    p = BoundaryParams()
    tr = BoundaryTracker(p)
    m = run(road(), np.empty((0, 2)))
    tr.predict(0.0, (0.0, np.zeros(2)))
    tr.update(m)
    empty = run(np.empty((0, 2)), np.empty((0, 2)))
    for k in range(1, 30):                  # 10 Hz, standing still
        t = 0.1 * k
        tr.predict(t, (0.0, np.zeros(2)))
        tr.update(empty)
        y, _, _ = tr.output()
        if t < p.coast_s - 0.05:
            assert np.isfinite(y).sum() >= 20
    tr.predict(p.coast_s + 0.2, (0.0, np.zeros(2)))
    tr.update(empty)
    assert not np.isfinite(tr.output()[0]).any()


def test_tracker_moves_the_boundary_with_ego_motion():
    p = BoundaryParams()
    tr = BoundaryTracker(p)
    tr.predict(0.0, (0.0, np.zeros(2)))
    tr.update(run(road(), np.empty((0, 2))))
    y0 = np.nanmedian(tr.output()[0])        # the held straight edge, before moving
    dpsi = math.radians(3.0)
    tr.predict(0.1, (dpsi, np.array([1.0, 0.0])))
    y, _, _ = tr.output()
    xa = p.anchors()
    i = np.flatnonzero(np.isfinite(y))[10]
    # World line y = y0 seen from a frame advanced 1 m and yawed +3 deg: the old-frame point
    # (X, y0) lands at x' = c(X-1) + s*y0, y' = -s(X-1) + c*y0.
    c, s = math.cos(dpsi), math.sin(dpsi)
    X = (xa[i] - s * y0) / c + 1.0
    expect = -s * (X - 1.0) + c * y0
    assert y[i] == pytest.approx(expect, abs=0.05)


def test_tracker_holds_an_edge_through_a_passing_occlusion():
    p = BoundaryParams()
    tr = BoundaryTracker(p)
    clear = run(road(), np.empty((0, 2)))
    obs = np.concatenate([car(x, 2.8) for x in np.arange(5.0, 60.0, 4.6)])
    r = carve(road(), obs)
    hidden = measure(r, np.full(len(r), Z), obs, p, fit=False)
    tr.predict(0.0, (0.0, np.zeros(2)))
    tr.update(clear)
    for k in range(1, 10):
        tr.predict(0.1 * k, (0.0, np.zeros(2)))
        tr.update(hidden)
    y, _, _ = tr.output()
    v = np.isfinite(y)
    assert v.sum() >= 20
    assert np.allclose(y[v], HALF, atol=0.35)


def test_resample_spacing_and_runs():
    x = np.array([5.0, 7.0, 9.0, 11.0, 13.0])
    y = np.array([1.0, 1.0, np.nan, 2.0, 2.0])
    out = resample(x, y, np.full(5, Z), 1.0)
    assert out[:, 0].tolist() == [5.0, 6.0, 7.0, 11.0, 12.0, 13.0]


def test_pipeline_recovers_odometry_after_switching_to_an_older_bag():
    from object_fusion.road_boundary import BoundaryPipeline
    pl = BoundaryPipeline()
    for k in range(50):
        pl.add_odom(1000.0 + 0.01 * k, 5.0, 0.0, 0.0)
    for k in range(50):                     # an older bag starts playing
        pl.add_odom(10.0 + 0.01 * k, 5.0, 0.0, 0.0)
    assert pl.rewinds == 1
    assert pl.twist.increment(10.1, 10.3) is not None


def test_a_single_stray_obstacle_point_shadows_nothing():
    stray = np.array([[12.0, 1.0]])
    assert not ShadowMap(stray, min_points=3).occluded([[30.0, 2.5]]).any()
    assert ShadowMap(stray, min_points=1).occluded([[30.0, 2.5]]).all()
    dense = stray * np.linspace(1.0, 1.03, 4)[:, None]       # four returns on one bearing
    assert ShadowMap(dense, min_points=3).occluded([[30.0, 2.5]]).all()


def _meas(p, left, right):
    xa = p.anchors()
    from object_fusion.road_boundary import Measurement
    n = xa.size
    L = np.full(n, float(left)) if np.isscalar(left) else np.asarray(left, float)
    Rr = np.full(n, float(right)) if np.isscalar(right) else np.asarray(right, float)
    st = np.zeros(n, np.int8)
    return Measurement(xa, L, Rr, np.full(n, p.sigma_meas), np.full(n, p.sigma_meas),
                       np.full(n, Z), st, st, float("nan"), 1.0, L.copy(), Rr.copy(), {})


def test_a_wild_initialisation_is_replaced_within_a_few_frames():
    p = BoundaryParams()
    tr = BoundaryTracker(p)
    xa = p.anchors()
    wild = np.where(xa > 20, -4.4, HALF)          # left edge crossing to the far right
    tr.predict(0.0, (0.0, np.zeros(2)))
    tr.update(_meas(p, wild, -HALF))
    left, right, _ = tr.output()
    # The crossed stations are dropped at once rather than published.
    assert not np.any(np.isfinite(left) & np.isfinite(right) & (left - right < p.min_width))
    for k in range(1, 6):
        tr.predict(0.1 * k, (0.0, np.zeros(2)))
        tr.update(_meas(p, HALF, -HALF))
    left, _, _ = tr.output()
    assert np.allclose(left[np.isfinite(left)], HALF, atol=0.2) and np.isfinite(left).sum() == xa.size


def test_a_persistently_gated_station_is_reinitialised():
    p = BoundaryParams()
    tr = BoundaryTracker(p)
    tr.predict(0.0, (0.0, np.zeros(2)))
    tr.update(_meas(p, HALF, -HALF))
    xa = p.anchors()
    moved = np.where((xa > 30) & (xa < 40), HALF + 6.0, HALF)   # a change far outside the gate
    for k in range(1, 5):
        tr.predict(0.1 * k, (0.0, np.zeros(2)))
        tr.update(_meas(p, moved, -HALF))
    left, _, _ = tr.output()
    k = (xa > 30) & (xa < 40)
    assert np.allclose(left[k], HALF + 6.0, atol=0.3)
    assert tr.reinits >= k.sum()


def test_measure_drops_crossed_stations():
    from object_fusion.road_boundary import _drop_crossed
    left = np.array([3.5, 1.0, -4.0])
    right = np.array([-3.5, -1.0, -3.0])
    bad = _drop_crossed(left, right, 2.5)
    assert bad.tolist() == [False, True, True]
    assert np.isnan(left[1:]).all() and np.isnan(right[1:]).all()
