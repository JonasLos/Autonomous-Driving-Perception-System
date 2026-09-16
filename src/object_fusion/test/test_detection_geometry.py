"""Extent, heading, the centroid correction, and the camera-only fallback."""
import math

import numpy as np
import pytest

from object_fusion.detection_geometry import (
    SHAPE_FIT_MAX_RANGE_M, YAW_FROM_RAY, YAW_FROM_SHAPE, YAW_FROM_VELOCITY, ExtentFilter,
    camera_only_range, camera_only_range_variance, choose_yaw, fit_rectangle,
    surface_to_centroid_offset,
)


def box_points(cx, cy, length, width, yaw, n=14):
    """Points on the two visible faces of a rectangle -- an L, as the LiDAR would see it."""
    t = np.linspace(-0.5, 0.5, n)
    front = np.stack([np.full(n, 0.5 * length), t * width], axis=1)
    side = np.stack([t * length, np.full(n, -0.5 * width)], axis=1)
    pts = np.vstack([front, side])
    c, s = math.cos(yaw), math.sin(yaw)
    R = np.array([[c, -s], [s, c]])
    return (pts @ R.T) + np.array([cx, cy])


def test_rectangle_fit_recovers_extent_and_heading():
    pts = box_points(20.0, 0.0, 4.5, 1.8, math.radians(20.0))
    fit = fit_rectangle(pts[:, 0], pts[:, 1], step_deg=1.0)
    assert fit is not None
    yaw, length, width, cx, cy, _ = fit
    assert length == pytest.approx(4.5, abs=0.25)
    assert width == pytest.approx(1.8, abs=0.25)
    assert cx == pytest.approx(20.0, abs=0.25)
    # 4-fold symmetry: the fit is only ever asked for the axis, modulo pi.
    assert min(abs(yaw - math.radians(20.0)),
               abs(yaw - math.radians(20.0) - math.pi)) < math.radians(8.0)


def test_fit_declines_when_under_determined():
    assert fit_rectangle([1.0, 2.0], [1.0, 2.0]) is None
    pts = box_points(20.0, 0.0, 4.5, 1.8, 0.0, n=3)
    assert fit_rectangle(pts[:, 0], pts[:, 1], min_points=30) is None


def test_fit_centre_is_the_centroid_not_the_visible_face():
    """Where the fit succeeds, no bias model is needed -- that is why it is preferred."""
    pts = box_points(30.0, 0.0, 4.5, 1.8, 0.0)
    _, _, _, cx, _, _ = fit_rectangle(pts[:, 0], pts[:, 1], step_deg=1.0)
    assert cx < 32.0, "must sit behind the front face at 32.25 m"
    assert cx == pytest.approx(30.0, abs=0.3)


def test_yaw_cascade_reports_its_source():
    fit = fit_rectangle(*box_points(20.0, 0.0, 4.5, 1.8, 0.3).T, step_deg=1.0)
    assert choose_yaw(20.0, 28, 0.0, 0.0, 0.0, fit)[1] == YAW_FROM_SHAPE
    # Past the derived 40 m bound the cloud is a flat plate -- fall through to velocity.
    assert choose_yaw(80.0, 28, 10.0, 1.2, 0.0, fit)[1] == YAW_FROM_VELOCITY
    assert choose_yaw(80.0, 28, 0.0, 1.2, 0.4, fit) == (0.4, YAW_FROM_RAY)
    assert SHAPE_FIT_MAX_RANGE_M == 40.0


def test_surface_to_centroid_pushes_back_by_half_the_viewed_extent():
    # Seen end-on: half the length.
    assert surface_to_centroid_offset(4.5, 1.8, 0.0, 0.0) == pytest.approx(2.25)
    # Seen broadside: half the width.
    assert surface_to_centroid_offset(4.5, 1.8, 0.0, math.pi / 2) == pytest.approx(0.9)
    # Oblique sits between the two.
    oblique = surface_to_centroid_offset(4.5, 1.8, 0.0, math.pi / 4)
    assert 0.9 < oblique < 2.25 + 1e-9


def test_extent_filter_is_one_sided_and_never_averages_down():
    """Occlusion only makes an object look shorter; a mean would be biased low forever."""
    f = ExtentFilter(prior=4.5)
    f.update(4.4)
    for _ in range(10):
        f.update(2.0)          # a run of badly occluded views
    assert f.value > 3.0, "a mean would have collapsed toward 2.0"


def test_extent_filter_clamps_so_a_bad_merge_cannot_inflate_forever():
    f = ExtentFilter(prior=4.5, clamp=1.5)
    for _ in range(20):
        f.update(40.0)
    assert f.value <= 4.5 * 1.5 + 1e-9


def test_camera_only_range_takes_the_nearer_of_two_estimates():
    # fy*h/dv = 1000*1.5/30 = 50 m ; fy*H/h_px = 1000*1.6/40 = 40 m -> take 40.
    r, src = camera_only_range(v_bottom_px=800.0, fy=1000.0, cy=770.0,
                               camera_height_m=1.5, class_height_m=1.6, box_height_px=40.0)
    assert r == pytest.approx(40.0)
    assert src == "class_height"


def test_camera_only_range_returns_none_above_the_horizon():
    assert camera_only_range(v_bottom_px=700.0, fy=1000.0, cy=770.0,
                             camera_height_m=1.5) is None


def test_camera_only_variance_grows_as_r_to_the_fourth():
    """sigma ~ r^2/(f*h) is measured; the variance therefore goes as r^4.

    Emitting these with a flat variance would be worse than dropping them.
    """
    v40 = camera_only_range_variance(40.0, 1000.0, 1.5)
    v80 = camera_only_range_variance(80.0, 1000.0, 1.5)
    assert v80 / v40 == pytest.approx(16.0, rel=1e-9)
    assert math.sqrt(v80) / math.sqrt(v40) == pytest.approx(4.0, rel=1e-9)


# ------------------------------------------------- the degenerate far-field fit
def flat_face_points(cx, width=1.8, n=14):
    """Only the rear face visible -- what the LiDAR actually returns past ~50 m."""
    t = np.linspace(-0.5, 0.5, n)
    return np.stack([np.full(n, cx), t * width], axis=1)


def test_a_single_visible_face_fits_as_a_degenerate_sliver():
    """The failure behind enable_extent_estimation making results worse.

    With one face visible the fit reports the FACE WIDTH as the length, zero width, and a yaw
    90 deg from truth -- confidently. This test pins the behaviour so the gate that rejects it
    cannot be removed by accident.
    """
    p = flat_face_points(80.0)
    yaw, length, width, cx, cy, _q = fit_rectangle(p[:, 0], p[:, 1], step_deg=1.0)
    assert width == pytest.approx(0.0, abs=1e-6), "a flat face has no measurable width"
    assert length == pytest.approx(1.8, abs=0.05), "it measures the face, not the vehicle"
    assert abs(math.degrees(yaw) - 90.0) < 1.0, "and the yaw is orthogonal to the truth"


def test_the_fit_quality_score_must_not_be_used_as_a_gate():
    """Quality is worse than useless here, and this test says why in code.

    A degenerate single-face fit scores the MAXIMUM, because every point lies on an edge,
    while a good but noisy two-face fit scores lower. Thresholding on quality would keep the
    bad fits and reject the good ones. The range cap and a non-degenerate width are the real
    discriminators.
    """
    flat = fit_rectangle(*flat_face_points(80.0).T, step_deg=1.0)
    rng = np.random.default_rng(3)
    good_pts = box_points(25.0, 0.0, 4.5, 1.8, 0.0, n=20) + rng.normal(0, 0.04, (40, 2))
    good = fit_rectangle(good_pts[:, 0], good_pts[:, 1], step_deg=1.0)
    assert flat[5] > good[5], "the degenerate fit scores HIGHER -- do not gate on this"


def test_choose_yaw_refuses_the_shape_fit_beyond_the_derived_range_cap():
    """The cap the detector must route through rather than calling fit_rectangle directly."""
    p = flat_face_points(80.0)
    fit = fit_rectangle(p[:, 0], p[:, 1], step_deg=1.0)
    assert fit is not None, "the fit itself does not decline -- that is the trap"
    _, src = choose_yaw(80.0, p.shape[0], speed=0.0, velocity_heading=0.0,
                        ray_heading=0.0, fit=fit)
    assert src == YAW_FROM_RAY, "past 40 m the shape fit must not be used"
    _, src_near = choose_yaw(20.0, p.shape[0], speed=0.0, velocity_heading=0.0,
                             ray_heading=0.0, fit=fit)
    assert src_near == YAW_FROM_SHAPE


# ------------------------------------------------- adopted selection with segmentation
def test_selection_drops_flagged_ground_before_the_depth_cluster():
    """The road is nearer than the vehicle, so without the flags the depth cluster adopts it."""
    from object_fusion.detection_geometry import select_object_points
    road = np.column_stack([np.linspace(30.0, 34.0, 40), np.zeros(40), np.full(40, -2.45)])
    car = np.column_stack([np.linspace(38.0, 39.0, 40), np.zeros(40), np.linspace(-2.0, -0.9, 40)])
    pts = np.vstack([road, car])
    flags = np.r_[np.ones(40, bool), np.zeros(40, bool)]

    px, _, _, used = select_object_points(pts[:, 0], pts[:, 1], pts[:, 2], flags,
                                          ground_min_range=10.0)
    assert used and px.min() >= 38.0, "only the vehicle survives"

    px0, _, _, used0 = select_object_points(pts[:, 0], pts[:, 1], pts[:, 2], None,
                                            ground_min_range=10.0)
    assert not used0


def test_a_box_holding_only_ground_yields_no_points_by_default():
    """Measured: 165 of the 169 boxes segmentation emptied were cones missed that frame, and
    falling back to every point put them on a road ring metres away (27% of all >2 m spikes)."""
    from object_fusion.detection_geometry import select_object_points
    pts = np.column_stack([np.linspace(20, 21, 10), np.zeros(10), np.linspace(-2.4, -1.0, 10)])
    px, py, pz, used = select_object_points(pts[:, 0], pts[:, 1], pts[:, 2], np.ones(10, bool))
    assert px.size == py.size == pz.size == 0 and used


def test_the_old_empty_box_fallback_is_still_available():
    from object_fusion.detection_geometry import select_object_points
    pts = np.column_stack([np.linspace(20, 21, 10), np.zeros(10), np.linspace(-2.4, -1.0, 10)])
    px, _, _, used = select_object_points(pts[:, 0], pts[:, 1], pts[:, 2], np.ones(10, bool),
                                          empty_fallback=True)
    assert px.size > 0 and not used
