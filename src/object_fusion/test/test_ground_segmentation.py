"""Ground segmentation: the fallback must be usable, and honest about being a stand-in."""
import numpy as np
import pytest

from object_fusion.ground_segmentation import (
    GroundSegmenter, patchworkpp_available, segment_ground_fallback,
)

RNG = np.random.default_rng(0)


def road(n=4000, z=-2.46, x=(2, 60), y=(-10, 10), slope=0.0):
    p = np.column_stack([RNG.uniform(*x, n), RNG.uniform(*y, n), np.zeros(n)])
    p[:, 2] = z + slope * p[:, 0] + RNG.normal(0, 0.02, n)
    return p


def car(n=150, cx=40.0, height=1.5, z_road=-2.46):
    return np.column_stack([RNG.uniform(cx, cx + 4.5, n), RNG.uniform(-1, 1, n),
                            RNG.uniform(z_road + 0.06, z_road + height, n)])


def test_flat_road_is_ground_and_a_vehicle_is_not():
    r, c = road(), car()
    g = segment_ground_fallback(np.vstack([r, c]))
    assert g[:r.shape[0]].mean() > 0.95
    # The lowest slice of a car sits inside the margin by construction; what matters is that
    # the body is kept, not that every wheel point is.
    assert g[r.shape[0]:].mean() < 0.35


def test_it_follows_a_real_road_grade():
    """These roads fall ~1.4 cm per metre. A single flat threshold across the sweep
    misclassifies the far field on any genuine camber, which is why the fit is per-cell."""
    r = road(slope=-0.014)
    g = segment_ground_fallback(r)
    assert g.mean() > 0.9


def test_a_short_object_survives_where_a_blanket_margin_would_strip_it():
    """The whole point of a ground MODEL over a height margin: a traffic cone stands ~0.5 m,
    and fusion_node's 25 m gate exists precisely because its 0.4 m margin would eat it."""
    r = road()
    cone = np.column_stack([RNG.uniform(12.0, 12.4, 40), RNG.uniform(-0.2, 0.2, 40),
                            RNG.uniform(-2.40, -1.96, 40)])
    g = segment_ground_fallback(np.vstack([r, cone]))
    kept = 1.0 - g[r.shape[0]:].mean()
    assert kept > 0.4, f"only {100*kept:.0f}% of the cone survived"


def test_empty_input():
    assert segment_ground_fallback(np.empty((0, 3))).shape == (0,)


def test_backend_selection_is_explicit_and_honest():
    seg = GroundSegmenter(backend="fallback")
    assert seg.backend == "fallback"
    with pytest.raises(ValueError):
        GroundSegmenter(backend="nonsense")
    # auto resolves to whatever is actually installed -- never silently claims patchworkpp
    assert GroundSegmenter(backend="auto").backend == (
        "patchworkpp" if patchworkpp_available() else "fallback")


def test_the_mask_is_index_aligned_with_the_input():
    pts = np.vstack([road(500), car(50)])
    g = segment_ground_fallback(pts)
    assert g.shape == (pts.shape[0],) and g.dtype == bool


# ---- levelling helpers ----------------------------------------------------------------------

def _tilted_road(slope_x, slope_y, n=4000, seed=1, height=-2.46):
    rng = np.random.default_rng(seed)
    r = rng.uniform(3.0, 20.0, n)
    a = rng.uniform(-np.pi, np.pi, n)
    x, y = r * np.cos(a), r * np.sin(a)
    z = height + slope_x * x + slope_y * y + rng.normal(0.0, 0.02, n)
    return np.column_stack([x, y, z])


def test_levelling_rotation_flattens_the_plane_it_is_given():
    from object_fusion.ground_segmentation import levelling_rotation
    sx, sy = 0.02, -0.03
    pts = _tilted_road(sx, sy, n=500)
    level = levelling_rotation(sx, sy)
    flat = pts @ level.T
    coef, *_ = np.linalg.lstsq(np.column_stack([flat[:, 0], flat[:, 1], np.ones(len(flat))]),
                               flat[:, 2], rcond=None)
    assert abs(coef[0]) < 2e-3 and abs(coef[1]) < 2e-3
    assert np.allclose(level @ level.T, np.eye(3), atol=1e-12), "a proper rotation"
    assert np.allclose(levelling_rotation(0.0, 0.0), np.eye(3))


def test_levelling_introduces_no_yaw():
    from object_fusion.ground_segmentation import levelling_rotation
    level = levelling_rotation(0.03, -0.02)
    for az in np.radians([0.0, 45.0, 90.0, 180.0, -120.0]):
        p = np.array([10.0 * np.cos(az), 10.0 * np.sin(az), -2.46]) @ level.T
        assert abs(np.degrees(np.arctan2(p[1], p[0]) - az) % 360.0) < 1.5 or \
            abs(np.degrees(np.arctan2(p[1], p[0]) - az) % 360.0) > 358.5


def test_road_plane_slopes_recovers_a_tilted_road_despite_a_vehicle():
    from object_fusion.ground_segmentation import road_plane_slopes
    road = _tilted_road(0.015, -0.025)
    car = np.column_stack([np.random.default_rng(2).uniform(8, 12, 800),
                           np.random.default_rng(3).uniform(-1, 1, 800),
                           np.random.default_rng(4).uniform(-2.3, -1.6, 800)])
    sl = road_plane_slopes(np.vstack([road, car]))
    assert sl is not None
    assert sl[0] == pytest.approx(0.015, abs=0.003) and sl[1] == pytest.approx(-0.025, abs=0.003)


def test_road_plane_slopes_refuses_without_support_or_when_implausible():
    from object_fusion.ground_segmentation import road_plane_slopes
    assert road_plane_slopes(_tilted_road(0.0, 0.0, n=50)) is None, "too few returns"
    assert road_plane_slopes(_tilted_road(0.2, 0.0)) is None, "an 11 deg 'road' is not levelled by"


def test_quaternion_roll_pitch():
    from object_fusion.ground_segmentation import quaternion_roll_pitch_deg
    def q(roll, pitch):
        cr, sr = np.cos(np.radians(roll) / 2), np.sin(np.radians(roll) / 2)
        cp, sp = np.cos(np.radians(pitch) / 2), np.sin(np.radians(pitch) / 2)
        return sr * cp, cr * sp, -sr * sp, cr * cp        # ZYX with yaw 0: (x, y, z, w)
    r, p = quaternion_roll_pitch_deg(*q(2.0, -1.5))
    assert r == pytest.approx(2.0, abs=1e-6) and p == pytest.approx(-1.5, abs=1e-6)


def test_segment_level_and_intensity_keep_the_mask_index_aligned():
    from object_fusion.ground_segmentation import GroundSegmenter, levelling_rotation
    pts = _tilted_road(0.03, 0.0, n=3000)
    seg = GroundSegmenter(backend="fallback")
    plain = seg.segment(pts)
    levelled = seg.segment(pts, intensity=np.full(len(pts), 0.5),
                           level=levelling_rotation(0.03, 0.0))
    assert plain.shape == levelled.shape == (len(pts),)
    assert levelled.mean() >= plain.mean() - 0.01


def test_unknown_patchwork_parameter_is_rejected():
    from object_fusion.ground_segmentation import GroundSegmenter, patchworkpp_available
    if not patchworkpp_available():
        pytest.skip("pypatchworkpp not installed")
    with pytest.raises(ValueError):
        GroundSegmenter(backend="patchworkpp", patchwork_params={"no_such_param": 1})


def test_slopes_from_attitude_is_linear_with_a_mount_term():
    from object_fusion.ground_segmentation import ODOM_TILT_COEF, slopes_from_attitude
    c = np.asarray(ODOM_TILT_COEF)
    assert slopes_from_attitude(0.0, 0.0) == pytest.approx(tuple(c[2]))
    assert slopes_from_attitude(1.0, 0.0, include_mount=False) == pytest.approx(tuple(c[0]))
    assert slopes_from_attitude(0.0, 2.0, include_mount=False) == pytest.approx(tuple(2 * c[1]))


def test_levelled_segmenter_changes_only_the_near_field():
    from object_fusion.ground_segmentation import GroundSegmenter, LevelledGroundSegmenter
    rng = np.random.default_rng(5)
    r = rng.uniform(3.0, 60.0, 8000)
    a = rng.uniform(-np.pi, np.pi, 8000)
    pts = np.column_stack([r * np.cos(a), r * np.sin(a), -2.46 + 0.03 * r * np.sin(a)])
    hybrid = LevelledGroundSegmenter(backend="fallback", near_range=25.0)
    plain = GroundSegmenter(backend="fallback").segment(pts)
    assert np.array_equal(hybrid.segment(pts), plain), "no attitude: exactly unlevelled"
    out = hybrid.segment(pts, roll_deg=2.0, pitch_deg=-1.0)
    far = r >= 25.0
    assert np.array_equal(out[far], plain[far]), "beyond near_range the labels are untouched"
    assert out.shape == plain.shape
