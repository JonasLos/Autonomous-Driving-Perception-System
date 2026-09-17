"""Lifecycle: what is allowed to become an object, and what is allowed to stay one."""
import numpy as np
import pytest

from object_fusion.track_store import (
    CONFIRMED, RADAR_ONLY_MIN_SPEED, SENSOR_CAMERA, SENSOR_LIDAR, SENSOR_RADAR, TENTATIVE,
    Track, TrackStore, camera_expected, radar_expected, should_merge,
)


def mk(x=(60.0, 0.0, 0.0, 0.0), P=None, stamp=0.0, mask=0):
    return Track(np.asarray(x, float), np.eye(4) if P is None else P, stamp, mask)


# ---------------------------------------------------------------- expected-miss gates
def test_a_miss_outside_the_sensor_field_is_not_evidence():
    """Charging a track for being invisible is how existence logic breaks silently."""
    s = TrackStore()
    tr = mk(mask=SENSOR_CAMERA)
    before = tr.logodds
    s.update_existence(tr, "radar", hit=False, expected=False, dt=0.0)
    assert tr.logodds == before
    s.update_existence(tr, "radar", hit=False, expected=True, dt=0.0)
    assert tr.logodds < before


def test_radar_field_of_view_and_range_bounds():
    assert radar_expected([60.0, 0.0])
    assert not radar_expected([0.5, 0.0])           # inside min range
    assert not radar_expected([200.0, 0.0])         # past max range
    assert not radar_expected([10.0, 30.0])         # ~72 deg, outside the forward field


def test_camera_expectation_follows_transform_crop_not_the_image_alone():
    """Past ~67 m the +/-20 m crop is narrower than the image; that is where objects vanish."""
    assert camera_expected([50.0, 5.0])
    assert not camera_expected([100.0, 25.0]), "outside the lateral crop"
    assert not camera_expected([200.0, 0.0]), "past the 150 m range crop"


def test_leaving_the_observable_volume_deletes_rather_than_decays():
    """Decaying instead leaves a confident pose lingering at the crop edge with no evidence."""
    s = TrackStore()
    s.add(mk(x=(200.0, 0.0, 0.0, 0.0), mask=SENSOR_CAMERA))
    s.tracks[0].logodds = 5.0
    s.prune(now=0.0, in_volume=lambda t: camera_expected(t.position))
    assert s.tracks == []
    assert s.shadow["deleted_left_volume"] == 1


# ------------------------------------------------------------------- confirmation
def test_camera_confirms_quickly_on_three_of_five():
    s = TrackStore()
    tr = mk(mask=SENSOR_CAMERA)
    tr.hits["camera_lidar"] = 3
    tr.opportunities["camera_lidar"] = 5
    assert s.may_confirm(tr)


def test_radar_only_needs_persistence_and_motion_and_slot_continuity():
    s = TrackStore()
    tr = mk(mask=SENSOR_RADAR)
    tr.hits["radar"], tr.opportunities["radar"] = 8, 12
    tr.radar_hits, tr.moving_hits = 10, 10
    tr.logodds = 4.0
    tr.radar_slot = 7
    assert s.may_confirm(tr)
    tr.moving_hits = 2                       # mostly stationary -> infrastructure
    assert not s.may_confirm(tr)
    tr.moving_hits = 10
    tr.radar_slot = None                     # no slot continuity
    assert not s.may_confirm(tr)


def test_a_stationary_radar_hit_is_worth_far_less_than_a_camera_one():
    """The direct encoding of the 81.5%-radar-originated observation."""
    s = TrackStore()
    a, b = mk(), mk()
    s.update_existence(a, "camera_lidar", hit=True, expected=True, dt=0.0)
    s.update_existence(b, "radar_static", hit=True, expected=True, dt=0.0)
    assert a.logodds > 10 * b.logodds


# --------------------------------------------------------------- radar-only birth
def test_radar_only_birth_is_gated_off_by_default_but_still_counted():
    s = TrackStore()
    assert not s.may_birth_radar_only(9.0)
    assert s.shadow["radar_only_candidates"] == 1, "shadow evidence must accrue while off"


def test_radar_only_birth_requires_ground_referenced_motion():
    s = TrackStore(enable_radar_only_birth=True)
    assert not s.may_birth_radar_only(0.2), "a guardrail is stationary over the ground"
    assert s.may_birth_radar_only(9.0)
    assert RADAR_ONLY_MIN_SPEED == 1.5       # 3x the measured 0.45 m/s p90 residual


def test_the_documented_blind_spot_is_real_and_asserted():
    """A stopped vehicle IS suppressed by this gate. Asserted so nobody 'fixes' it here --
    the camera path covers exactly the region where a stopped vehicle matters."""
    s = TrackStore(enable_radar_only_birth=True)
    assert not s.may_birth_radar_only(0.0)


# ------------------------------------------------------------------------- merging
def test_two_branches_of_one_car_merge_despite_a_large_range_gap():
    """The camera branch can sit 14 m short while radar is right -- both birth a track.

    Two boxes for one vehicle, one a phantom at the true position, is worse for a planner
    than one box in the wrong place.
    """
    a = mk(x=(100.0, 0.0, 0.0, 0.0), P=np.eye(4) * 0.5)
    b = mk(x=(86.0, 0.0, 0.0, 0.0), P=np.eye(4) * 0.5)
    assert should_merge(a, b), "same bearing, disputed range -- one object"


def test_genuinely_separate_objects_do_not_merge():
    a = mk(x=(60.0, 0.0, 0.0, 0.0), P=np.eye(4) * 0.5)
    b = mk(x=(60.0, 12.0, 0.0, 0.0), P=np.eye(4) * 0.5)
    assert not should_merge(a, b)


def test_merge_keeps_the_better_supported_track_and_unions_provenance():
    s = TrackStore()
    weak = s.add(mk(x=(86.0, 0.0, 0.0, 0.0), P=np.eye(4) * 0.5, mask=SENSOR_RADAR))
    strong = s.add(mk(x=(100.0, 0.0, 0.0, 0.0), P=np.eye(4) * 0.5, mask=SENSOR_CAMERA))
    strong.logodds, weak.logodds = 4.0, 1.0
    s.merge_pass()
    assert len(s.tracks) == 1
    assert s.tracks[0] is strong
    assert s.tracks[0].sensors_ever == (SENSOR_CAMERA | SENSOR_RADAR)
    assert s.shadow["merged"] == 1


# ---------------------------------------------------------------------- deletion
def test_a_diverged_track_dies_rather_than_lingering_confident():
    s = TrackStore()
    tr = s.add(mk(P=np.diag([500.0, 500.0, 1.0, 1.0]), mask=SENSOR_CAMERA))
    tr.logodds = 5.0
    s.prune(now=0.0)
    assert s.tracks == [] and s.shadow["deleted_diverged"] == 1


def test_coast_budget_scales_with_corroboration():
    s = TrackStore()
    both = mk(mask=SENSOR_CAMERA | SENSOR_RADAR)
    cam = mk(mask=SENSOR_CAMERA)
    rad = mk(mask=SENSOR_RADAR)
    assert s.coast_budget(both) > s.coast_budget(cam) > s.coast_budget(rad)


def test_a_track_past_its_coast_budget_is_deleted():
    s = TrackStore()
    tr = s.add(mk(mask=SENSOR_RADAR, stamp=0.0))
    tr.logodds = 5.0
    s.prune(now=10.0)
    assert s.tracks == []


# ------------------------------------------------------------------ publication
def test_camera_tracks_publish_from_tentative_but_radar_only_must_earn_it():
    s = TrackStore()
    cam = mk(mask=SENSOR_LIDAR)
    assert cam.status == TENTATIVE and s.may_publish(cam)
    rad = mk(mask=SENSOR_RADAR)
    assert not s.may_publish(rad)
    rad.status, rad.logodds = CONFIRMED, 5.0
    assert s.may_publish(rad)


# --------------------------------------------------------------------- classes
def test_class_is_a_majority_vote_not_the_latest_frame():
    """A car<->truck flip changes the extent prior and steps the centroid correction ~1 m."""
    tr = mk()
    for _ in range(5):
        tr.vote_class("car")
    tr.vote_class("truck")
    assert tr.class_name() == "car"


def test_existence_is_bounded_so_a_long_lived_track_can_still_die():
    s = TrackStore()
    tr = mk(mask=SENSOR_CAMERA)
    for _ in range(200):
        s.update_existence(tr, "camera_lidar", hit=True, expected=True, dt=0.0)
    assert tr.logodds <= 6.0
    for _ in range(200):
        s.update_existence(tr, "camera_lidar", hit=False, expected=True, dt=0.1)
    assert tr.existence < 0.2, "must be able to fall back below the delete threshold"


# ------------------------------------------------ existence must actually accrue
def test_existence_moves_off_the_prior_when_accrued():
    """Regression: the node never called update_existence, so every published object read
    exactly 0.5 forever -- a field that looks calibrated and is inert."""
    s = TrackStore()
    tr = mk(mask=SENSOR_CAMERA)
    assert tr.existence == pytest.approx(0.5), "the prior, before any evidence"
    for _ in range(4):
        s.update_existence(tr, "camera_lidar", hit=True, expected=True, dt=0.1)
    assert tr.existence > 0.9


def test_a_track_carries_the_slots_the_node_needs():
    """These were added when existence and passthrough were wired; __slots__ makes a typo
    here an AttributeError at runtime rather than a silently ignored assignment."""
    tr = mk()
    for slot in ("last_existence_t", "camera_rejected", "extent_l", "extent_w",
                 "last_cam_xy"):
        assert hasattr(tr, slot), slot


def test_provenance_fields_default_to_honest_values():
    """`size_measured` and `yaw_source` are read verbatim by consumers.

    A fresh track must not claim a measured extent, and its yaw source must not default to
    enum 0 (SHAPE_FIT) -- `tr.yaw_source or 0` did exactly that, so every unfitted track
    reported a measured heading it never had.
    """
    tr = mk()
    assert tr.size_measured is False
    assert tr.yaw_source is None, "None means 'no source yet', and must map to RAY_DEFAULT"


def _track(x, y, sensors, logodds=2.0):
    from object_fusion.track_store import SENSOR_CAMERA, Track
    tr = Track(np.array([float(x), float(y), 0.0, 0.0]), np.diag([1.0, 1.0, 9.0, 9.0]), 0.0,
               SENSOR_CAMERA)
    tr.sensors_ever = sensors
    tr.logodds = logodds
    return tr


def test_two_cones_in_a_line_are_not_merged():
    """A line of cones sits within a degree of bearing and metres apart in range -- the same
    geometry as the camera/radar split the range clause exists for. Merging them lost 30.6% of
    published objects on the reference drive (7.8% with merging off)."""
    from object_fusion.track_store import SENSOR_CAMERA, SENSOR_RADAR, should_merge
    near = _track(30.0, 2.0, SENSOR_CAMERA)
    far = _track(38.0, 2.5, SENSOR_CAMERA)          # ~0.9 deg apart, 8 m in range
    assert not should_merge(near, far), "two camera-seen objects are two objects"


def test_the_camera_radar_split_still_merges():
    """The case the clause exists for: at long range the camera branch can sit 14 m short of the
    radar branch, on the same bearing. One of those boxes is a phantom."""
    from object_fusion.track_store import SENSOR_CAMERA, SENSOR_RADAR, should_merge
    cam = _track(66.0, 4.0, SENSOR_CAMERA)
    rad = _track(80.0, 4.9, SENSOR_RADAR)
    assert should_merge(cam, rad)


def test_coincident_tracks_still_merge_whatever_their_provenance():
    """The Mahalanobis clause is untouched: tracks on top of each other are one object."""
    from object_fusion.track_store import SENSOR_CAMERA, should_merge
    a = _track(40.0, 1.0, SENSOR_CAMERA)
    b = _track(40.3, 1.1, SENSOR_CAMERA)
    assert should_merge(a, b)


def test_the_range_clause_does_not_fire_near_the_car():
    """Close in the camera range is good to about a metre, so a 20 m merge cannot be the
    camera/radar split -- it is a cone being swallowed by a radar return behind it."""
    from object_fusion.track_store import SENSOR_CAMERA, SENSOR_RADAR, should_merge
    cone = _track(28.0, 3.0, SENSOR_CAMERA)
    guardrail = _track(44.0, 3.6, SENSOR_RADAR)
    assert not should_merge(cone, guardrail)


def test_a_coasting_track_may_not_swallow_the_cone_beside_it():
    """The Mahalanobis clause does not bound a distance on its own, so it carries one.

    A coasted track has a position variance of several m^2, and at 2 m sigma on each of two
    tracks the 99% two-DOF value spans 8.6 m -- past the next cone in a line. The hard bound is
    what stops it. Measured consequence of not having it: 16.0% of camera measurements had no
    published box within 3 m, live, against 8.0% once the reach was bounded.
    """
    from object_fusion.track_store import SENSOR_CAMERA, should_merge
    coasted = _track(30.0, 0.0, SENSOR_CAMERA)
    coasted.P = np.diag([4.0, 4.0, 9.0, 9.0])       # ~2 m sigma, a second of coasting
    cone = _track(37.0, 0.6, SENSOR_CAMERA)         # the next cone up the line
    cone.P = np.diag([4.0, 4.0, 9.0, 9.0])
    assert should_merge(coasted, cone, max_merge_dist=float("inf")), \
        "unbounded, the chi-square alone reaches right across to the neighbour"
    assert not should_merge(coasted, cone), "7 m apart is not a duplicate, whatever P says"


def test_the_duplicate_it_exists_to_collapse_is_still_merged():
    """The counterpart: bounding the reach must not cost the gate its actual job.

    A track that has just been re-born beside its coasting self sits well under a metre away.
    Tightening the chi-square instead of bounding the distance loses exactly this case --
    duplicate published tracks went 2.4% -> 8.6% when it was tried.
    """
    from object_fusion.track_store import SENSOR_CAMERA, should_merge
    coasting = _track(45.0, 1.0, SENSOR_CAMERA)
    reborn = _track(45.8, 1.2, SENSOR_CAMERA)
    assert should_merge(coasting, reborn)


# ------------------------------------------------------------------- published fields
def test_a_coasting_track_is_published_as_coasting_not_tentative():
    """An old camera-only track that misses one cycle can never be re-promoted -- may_confirm
    stops at 5 camera opportunities -- so it coasts for the rest of its life. Folding that into
    TENTATIVE mislabelled nearly every long-lived cone, and a consumer that drops tentative
    tracks (the planner bridge) dropped them."""
    from object_fusion.track_store import COASTING, CONFIRMED, TENTATIVE
    s = TrackStore()
    tr = s.add(mk(stamp=0.0, mask=SENSOR_CAMERA))
    tr.hits["camera_lidar"], tr.opportunities["camera_lidar"] = 30, 30
    tr.status = CONFIRMED
    s.prune(0.05)                                   # not updated by this measurement
    s.promote()
    assert tr.status == COASTING, "the premise: an old camera-only track cannot re-promote"
    assert tr.published_status() == COASTING
    assert TENTATIVE == 0 and CONFIRMED == 1 and COASTING == 2, "must equal FusedObject.STATUS_*"


def test_missed_updates_counts_camera_periods_since_the_last_update():
    tr = mk(stamp=10.0)
    assert tr.missed_updates(10.0) == 0
    assert tr.missed_updates(10.05) == 0
    assert tr.missed_updates(10.3) == 3
    assert tr.missed_updates(100.0) == 255, "uint8 on the wire"
    assert tr.missed_updates(9.0) == 0, "a late stamp is not a negative miss"
